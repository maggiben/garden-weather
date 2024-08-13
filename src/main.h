/**
 * @file         : main.h
 * @summary      : Smart Indoor Automation System
 * @version      : 1.0.0
 * @project      : smart-green
 * @description  : A Smart Indoor Automation System
 * @author       : Benjamin Maggi
 * @email        : benjaminmaggi@gmail.com
 * @date         : 23 Apr 2024
 * @license:     : MIT
 *
 * Copyright 2021 Benjamin Maggi <benjaminmaggi@gmail.com>
 *
 *
 * License:
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files
 * (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to permit
 * persons to whom the Software is furnished to do so, subject to the
 * following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 **/

#pragma once
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include <SPI.h>
#include <Wire.h>
#include <FS.h>
#include <SD.h>
#include <EEPROM.h>
#include <time.h>
#include <RTClib.h> // Date and time functions using a DS3231 RTC connected via I2C and Wire lib
#include <Adafruit_BMP280.h> // Pressure Sensor
#include <ArduinoOTA.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include "soc/soc.h"            // For WRITE_PERI_REG
#include "soc/rtc_cntl_reg.h"   // For RTC_CNTL_BROWN_OUT_REG
#include "constants.h"
// #include "settings.h"

// // Settings
// Settings settings = {
//   // Assuming HOSTNAME is defined
//   HOSTNAME,
//   // Assuming id is 0
//   0,
//   // Assuming lastDateTimeSync is 0
//   0,
//   // Assuming updatedOn is 0
//   0,
//   // Assuming reboot on wifi failed is false
//   SETTINGS_REBOOT_ON_WIFIFAIL,
//   // Flow calibration value
//   0, // FLOW_CALIBRATION_FACTOR
//   // Initializing alarms all to 0 (disabled)
//   {{{0}}},
//   // Max Plants
//   SETTINGS_MAX_PLANTS,
//   // Plant settings
//   {{0}},
//   // TaskLog
//   {0},
//   // Attemp to use Graphical Display
//   USE_DISPLAY,
//   USE_RTC,
//   USE_EEPROM,
//   USE_MCP
// };

// i2c Clock
RTC_DS3231 rtc; // Address 0x68

// i2c Pressure and Temperature sensor
Adafruit_BMP280 bmp; // I2C

// Need a WebServer for http access on port 80.
WebServer server(80);

// Need a WebServer for http access on port 80.
#ifndef server
  #define SERVER_RESPONSE_OK(...)  server.send(200, "application/jsont; charset=utf-8", __VA_ARGS__)
  #define SERVER_RESPONSE_SUCCESS()  SERVER_RESPONSE_OK("{\"success\":true}")
  // #define SERVER_RESPONSE_ERROR(code, ...)  server.send(code, "application/jsont; charset=utf-8", __VA_ARGS__)
  #define SERVER_RESPONSE_ERROR(code, error)  server.send(code, "application/json; charset=utf-8", String("{\"error\":\"") + error + "\"}")
#endif

TaskHandle_t webServerTaskHandle;
TaskHandle_t otaTaskHandle;
TaskHandle_t serialTaskHandle;
// Use only core
#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

// Define custom priority levels
#define PRIORITY_LOW       (tskIDLE_PRIORITY + 1)
#define PRIORITY_MEDIUM    (tskIDLE_PRIORITY + 2)
#define PRIORITY_HIGH      (tskIDLE_PRIORITY + 3)
#define PRIORITY_VERY_HIGH (tskIDLE_PRIORITY + 4)

// #ifndef WIFI_ENABLED
//   #define WIFI_ENABLED
// #endif

// #ifndef ENABLE_OTA
//   #define ENABLE_OTA
// #endif

// #ifndef ENABLE_HTTP
//   #define ENABLE_HTTP
// #endif

// #ifndef ENABLE_LOGGING
//   #define ENABLE_LOGGING
// #endif

// #ifndef ENABLE_SERIAL_COMMANDS
//   #define ENABLE_SERIAL_COMMANDS
// #endif

SemaphoreHandle_t i2cMutex;

/**
 * Hardware Setup
 */

/**
 * Wireless functions
 */
bool connectToWiFi(const char* ssid, const char* password, int max_tries = 20, int pause = 500);
// void handleWifiConnectionError(String error, Settings settings, bool restart = false);

/**
 * Time & RTC Functions
 */
void syncRTC();
void setTimezone(String timezone);
void initTime(String timezone);
long int getRtcOffset();

/**
 * API Handlers
 */
void handleSystemInfo();
void handleLogs();
void handleRoot();
void handleNotFound();

/**
 * LCD Display & Serial debug functions
 */
void printLocalTime();
void printRtcTime();
void serialLog(String message);

/**
 * IO
 */
void printI2cDevices(byte* devices = NULL);

/**
 * Threads
 */
void handleOTATask(void * parameter);
void handleWebServerTask(void * parameter);
