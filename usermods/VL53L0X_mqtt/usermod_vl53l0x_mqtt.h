/*
 * That usermod implements support of simple hand gestures with VL53L0X sensor: on/off and brightness correction.
 * It can be useful for kitchen strips to avoid any touches.
 * - on/off - just swipe a hand below your sensor ("shortPressAction" is called and can be customized through WLED macros)
 * - brightness correction - keep your hand below sensor for 1 second to switch to "brightness" mode.
        Configure brightness by changing distance to the sensor (see parameters below for customization).
 *
 * Enabling this usermod:
 * 1. Attach VL53L0X sensor to i2c pins according to default pins for your board.
 * 2. Add `-D USERMOD_VL53L0X_GESTURES` to your build flags at platformio.ini (plaformio_override.ini) for needed environment.
 * In my case, for example: `build_flags = ${env.build_flags} -D USERMOD_VL53L0X_GESTURES`
 * 3. Add "pololu/VL53L0X" dependency below to `lib_deps` like this:
 * lib_deps = ${env.lib_deps}
 *     pololu/VL53L0X @ ^1.3.0
 */
#pragma once

#include "wled.h"

#include <Wire.h>
#include <VL53L0X.h>

#ifndef VL53L0X_DELAY_MS
#define VL53L0X_DELAY_MS 500 // how often to get data from sensor
#endif

class UsermodVL53L0XMQTT : public Usermod {
  private:
    //Private class members. You can declare variables and functions only accessible to your usermod here
    unsigned long lastTime = 0;
    VL53L0X sensor;
    bool enabled = true;

    bool mqttInitialized = false;
    int lastRange = 0;
    
    unsigned long readingInterval = VL53L0X_DELAY_MS;
    bool automaticallyToggleLights = false;
    int distanceToToggle = 0;
    int secondsToKeepOnFor = 30;
    uint32_t offTimerStart = 0;

  public:

    void setup() {
      PinManagerPinType pins[2] = { { i2c_scl, true }, { i2c_sda, true } };
      if (!pinManager.allocateMultiplePins(pins, 2, PinOwner::HW_I2C)) { enabled = false; return; }
      Wire.begin();

      sensor.setTimeout(500);
      if (!sensor.init())
      {
        DEBUG_PRINTLN(F("Failed to detect and initialize VL53L0X sensor!"));
      } else {
        sensor.setMeasurementTimingBudget(20000); // set high speed mode
      }
    }

    /**
     * switch off the strip if the delay has elapsed 
     */
    bool handleOffTimer() {
      if (offTimerStart > 0 && millis() - offTimerStart > secondsToKeepOnFor * 1000) {
        offTimerStart = 0;
        shortPressAction();
        return true;
      }
      return false;
    }


    void loop() {
      if (!enabled || strip.isUpdating()) return;
      if (millis() - lastTime > readingInterval) {
        lastTime = millis();

        int range = sensor.readRangeSingleMillimeters();
        DEBUG_PRINTF("range: %d, brightness: %d\r\n", range, bri);

        int currentRange = range / 10;
        if (!between(currentRange, lastRange - 1, lastRange + 1)) {
          if(automaticallyToggleLights) {
            if (range < distanceToToggle) {
              offTimerStart = millis();
              if(bri == 0)
                shortPressAction();
            }
          }

          if (WLED_MQTT_CONNECTED) {
            if (!mqttInitialized) {
              _mqttInitialize();
              mqttInitialized = true;
            }
            char subuf[64];
            strcpy(subuf, mqttDeviceTopic);
            strcat_P(subuf, PSTR("/VL53L0X/distance"));
            mqtt->publish(subuf, 0, true, String(range).c_str());
          }
          
        }
        handleOffTimer();
        lastRange = currentRange;
      }
    }
    
    bool between(const int x, const int min, const int max) {
      return x >= min && x <= max;
    }

    
    // set up Home Assistant discovery entries
    void _mqttInitialize()
    {
      char subuf[64];
      strcpy(subuf, mqttDeviceTopic);
      strcat_P(subuf, PSTR("/VL53L0X/distance"));
      
      _createMqttSensor(F("Distance"), subuf, F("Distance"), F("mm"));
    }
  
    void _createMqttSensor(const String &name, const String &topic, const String &deviceClass, const String &unitOfMeasurement)
    {
      String t = String(F("homeassistant/sensor/")) + mqttClientID + F("/") + name + F("/config");
      
      StaticJsonDocument<600> doc;
      
      doc[F("name")] = String(mqttClientID) + " " + name;
      doc[F("state_topic")] = topic;
      doc[F("unique_id")] = String(mqttClientID) + name;
      if (unitOfMeasurement != "")
        doc[F("unit_of_measurement")] = unitOfMeasurement;
      if (deviceClass != "")
        doc[F("device_class")] = deviceClass;
      doc[F("expire_after")] = 1800;

      JsonObject device = doc.createNestedObject(F("device")); // attach the sensor to the same device
      device[F("name")] = serverDescription;
      device[F("identifiers")] = "wled-sensor-" + String(mqttClientID);
      device[F("manufacturer")] = F("WLED");
      device[F("model")] = F("FOSS");
      device[F("sw_version")] = versionString;

      String temp;
      serializeJson(doc, temp);
      DEBUG_PRINTLN(t);
      DEBUG_PRINTLN(temp);

      mqtt->publish(t.c_str(), 0, true, temp.c_str());
    }
    
  /**
   * addToConfig() (called from set.cpp) stores persistent properties to cfg.json
  */
  void addToConfig(JsonObject &root)
  {
    // we add JSON object.
    JsonObject top = root.createNestedObject(FPSTR("vl53l0x-mqtt")); // usermodname
    top[FPSTR("read-interval-ms")] = readingInterval;
    top[FPSTR("automatically-toggle-lgihts")] = automaticallyToggleLights;
    top[FPSTR("distance-to-toggle")] = distanceToToggle;
    top[FPSTR("seconds-to-keep-on-for")] = secondsToKeepOnFor;

    DEBUG_PRINTLN(F("config saved."));
  }

  /**
  * readFromConfig() is called before setup() to populate properties from values stored in cfg.json
  */
  bool readFromConfig(JsonObject &root)
  {
    // we look for JSON object.
    JsonObject top = root[FPSTR("vl53l0x-mqtt")];
    if (top.isNull()) {
      DEBUG_PRINT(FPSTR("vl53l0x-mqtt"));
      DEBUG_PRINTLN(F(": No config found. (Using defaults.)"));
      return false;
    }

    readingInterval  = (top[FPSTR("read-interval-ms")] | readingInterval);
    automaticallyToggleLights  = (top[FPSTR("automatically-toggle-lgihts")] | automaticallyToggleLights);
    distanceToToggle  = (top[FPSTR("distance-to-toggle")] | distanceToToggle);
    secondsToKeepOnFor  = (top[FPSTR("seconds-to-keep-on-for")] | secondsToKeepOnFor);
    
    DEBUG_PRINT(FPSTR("vl53l0x-mqtt"));
    DEBUG_PRINTLN(F(" config (re)loaded."));

    return !top["seconds-to-keep-on-for"].isNull(); //" when updating Usermod with new features
    //return true;
  }

    /*
     * getId() allows you to optionally give your V2 usermod an unique ID (please define it in const.h!).
     * This could be used in the future for the system to determine whether your usermod is installed.
     */
    uint16_t getId()
    {
      return  USERMOD_ID_VL53L0X_MQTT;
    }
};
