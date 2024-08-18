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
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <RTClib.h>                 // Date and time functions using a DS3231 RTC connected via I2C and Wire lib
#include <Adafruit_Sensor.h>
#include <Adafruit_AHTX0.h>         // humidity sensor
#include <Adafruit_ADS1X15.h>       // ADC
#include "soc/soc.h"                // For WRITE_PERI_REG
#include "soc/rtc_cntl_reg.h"       // For RTC_CNTL_BROWN_OUT_REG
#include "constants.h"

// i2c Clock
RTC_DS3231 rtc; // Address 0x68

// i2c Humidity Sensor
Adafruit_AHTX0 aht;

Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */


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

#ifndef ENABLE_SERIAL_COMMANDS
  #define ENABLE_SERIAL_COMMANDS
#endif

SemaphoreHandle_t i2cMutex;

/**
 * IO
 */
void printI2cDevices(byte* devices = NULL);

/**
 * Threads
 */
void handleOTATask(void * parameter);
void handleWebServerTask(void * parameter);


// Define a struct to hold command and function pointer
struct CommandHandler {
    String command;
    void (*handler)(const String&);
};

/**
 * Serial Command
 */
void serialPortHandler(void *pvParameters);
void handleUnknown();
void handleGetSensor(const String& command);
void handleRawSensor(const String& command);

// Array of command handlers
CommandHandler commandHandlers[] = {
    {"get-sensor", handleGetSensor},
    {"raw-sensor", handleRawSensor},
    // Add other commands and handlers here
};

void handleSerialCommand(const String& command);