#include "main.h"

void printI2cDevices(byte* devices) {
  byte error, address;
  int nDevices;
 
  TRACE("Scanning...\n");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      TRACE("I2C device found at address 0x");
      if (devices != NULL) {
        devices[nDevices] = address;
      }
      if (address<16)
        TRACE("0");
      PRINT(address, HEX);
      TRACE("  !\n");
      nDevices++;
    }
    else if (error==4)
    {
      TRACE("Unknown error at address 0x");
      if (address < 16)
        TRACE("0");
      PRINTLN(address, HEX);
    }    
  }
  if (nDevices == 0)
    TRACE("No I2C devices found\n");
  else
    TRACE("done\n");
}

/**
 * Hardware setup
 */
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  while (!Serial) {
    vTaskDelay(100 / portTICK_PERIOD_MS); // wait for serial port to connect. Needed for native USB
  }

  Serial.println("Setup Init\n");

  Wire.begin();
  i2cMutex = xSemaphoreCreateMutex();
  if (i2cMutex == NULL) {
    TRACE("Error insufficient heap memory to create i2cMutex mutex\n");
  }

  printI2cDevices();

  // Check if EEPROM is ready
  Wire.beginTransmission(EEPROM_ADDRESS);
  if (Wire.endTransmission() != 0) {
    TRACE("EEPROM not found or not ready\n");
    ESP.restart();
  } else {
    // Init EEPROM chip in the RTC module
    if (EEPROM.begin(EEPROM_SIZE)) {
      // EEPROM.put(EEPROM_SETTINGS_ADDRESS, settings);
      // EEPROM.commit();
      // vTaskDelay(500 / portTICK_PERIOD_MS);
      // beep(5, 150);
      // Read EEPROM settings
    //   EEPROM.get(EEPROM_SETTINGS_ADDRESS, settings);
      // TRACE("Settings: %s now: %d\n", settings.hostname, rtc.now().unixtime());
    } else {
      TRACE("EEPROM not Working\n");
      ESP.restart();
    };
  }

  if (!rtc.begin()) {
    TRACE("Couldn't find RTC\n");
    Serial.flush();
    abort();
  } else if (rtc.lostPower()) {
    TRACE("RTC lost power, let's set the time!\n");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

  if (!bmp.begin(0x76)) {
    TRACE("Could not find a valid BMP280 sensor, check wiring or try a different address!\n");
    Serial.flush();
    abort();
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

//   TRACE("settings.hostname %s\n", settings.hostname);

  // Create a queue capable of holding 10 strings of up to 100 characters each
    // Serial commander task
// #if defined(ENABLE_SERIAL_COMMANDS)
//   xTaskCreatePinnedToCore(
//     serialPortHandler,     // Task function
//     "SerialPort",          // Name of the task (for debugging)
//     89000,                 // Stack size (in words, not bytes)
//     NULL,                  // Task input parameter
//     PRIORITY_HIGH,         // Priority of the task
//     &serialTaskHandle,     // Task handle
//     1                      // Core where the task should run
//   );
// #endif

// #if defined(WIFI_ENABLED)
//   String ssid = config["network"]["ssid"].isNull() ? WIFI_SSID : config["network"]["ssid"].as<String>();
//   String password = config["network"]["password"].isNull() ? WIFI_PASSWORD : config["network"]["password"].as<String>();
//   bool enabled = config["network"]["enabled"].isNull() ? WIFI_ENABLED : config["network"]["enabled"].as<bool>();
//   TRACE("ssid: %s\n", ssid.c_str());
//   TRACE("password: %s\n", password.c_str());
//   TRACE("config: %s\n", config.as<String>().c_str());
//   if (enabled && connectToWiFi(ssid.c_str(), password.c_str())) {
//     IPAddress ip = WiFi.localIP();
//     TRACE("\n");
//     TRACE("Wifi Connected: IP: %s - Hostname: %s\n", WiFi.localIP().toString().c_str(), WiFi.getHostname());

//     // Ask for the current time using NTP request builtin into ESP firmware.
//     TRACE("Setup ntp...\n");
//     initTime(TIMEZONE);
//     printLocalTime();

//     // Create a task for handling OTA
// #if defined(ENABLE_OTA)
//     xTaskCreatePinnedToCore(
//       handleOTATask,          // Function to implement the task
//       "OtaTask",              // Name of the task
//       46000,                  // Stack size in words
//       NULL,                   // Task input parameter
//       PRIORITY_LOW,           // Priority of the task
//       &otaTaskHandle,         // Task handle
//       1                       // Core where the task should run
//     );
// #endif
//     // Create a task for handling Web Server
// #if defined(ENABLE_HTTP)
//     xTaskCreatePinnedToCore(
//       handleWebServerTask,    // Function to implement the task
//       "WebServerTask",        // Name of the task
//       46000,                  // Stack size in words
//       NULL,                   // Task input parameter
//       PRIORITY_MEDIUM,        // Priority of the task
//       &webServerTaskHandle,   // Task handle
//       1                       // Core where the task should run
//     );
// #endif

//   } else if (config["network"]["enabled"].as<bool>()) {
//     TRACE("Wifi not connected!\n");  
//     beep(2);
//     handleWifiConnectionError("WiFi connection error", settings);
//   } else {
//     TRACE("Wifi disabled!\n");
//   }
// #endif

  // Good To Go!
  vTaskDelay(100 / portTICK_PERIOD_MS);
}

// bool connectToWiFi(const char* ssid, const char* password, int max_tries, int pause) {
//   int i = 0;
//   // allow to address the device by the given name e.g. http://webserver
//   // Set WiFi mode to Station (Client)
//   WiFi.mode(WIFI_STA);
//   // Disconnect any existing WiFi connections
//   WiFi.disconnect();
//   vTaskDelay(1000 / portTICK_PERIOD_MS);
//   // Set the hostname
//   WiFi.setHostname(settings.hostname);

  
//   #if defined(ARDUINO_ARCH_ESP8266)
//     WiFi.forceSleepWake();
//     delay(200);
//   #endif
//   // Begin the WiFi connection
//   WiFi.begin(ssid, password);
//   do {
//     vTaskDelay(pause / portTICK_PERIOD_MS);
//     TRACE(".");
//     i++;
//   } while (!WiFi.status() == WL_CONNECTED && i < max_tries);
//   WiFi.setAutoReconnect(true);
//   WiFi.persistent(true);

//   // Initialize mDNS
//   TRACE("Setting up MDNS responder!\n");
//   i = 0;
//   do {
//     vTaskDelay(pause / portTICK_PERIOD_MS);
//     TRACE(".");
//     i++;
//   } while (!MDNS.begin(settings.hostname) && i < max_tries);

//   return WiFi.status() == WL_CONNECTED;
// }

// void handleWifiConnectionError(String error, Settings settings, bool restart) {
//   TRACE("Error: %s\n", error.c_str());
//   if (settings.rebootOnWifiFail) {
//     TRACE("Rebooting now...\n");
//     vTaskDelay(150 / portTICK_PERIOD_MS);
//     ESP.restart();
//   }
// }

void syncRTC() {
  // Get current time from NTP server
  time_t now = time(nullptr);
  struct tm timeinfo;
  localtime_r(&now, &timeinfo);

  // Set RTC time
  DateTime dateTime = DateTime(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  rtc.adjust(dateTime);
  // EEPROM.put(EEPROM_SETTINGS_ADDRESS, settings);
  // EEPROM.commit();
  TRACE("RTC synced with NTP time\n");
}

void setTimezone(String timezone) {
  TRACE("Setting Timezone to: %s\n", timezone.c_str());
  setenv("TZ", timezone.c_str(),1);  //  Now adjust the TZ.  Clock settings are adjusted to show the new local time
  tzset();
}

void initTime(String timezone) {
  tm timeinfo;
  // Initialize NTP time sync
  TRACE("Setting up time\n");
  configTime(0, 0, "pool.ntp.org");    // First connect to NTP server, with 0 TZ offset
  // Now we can set the real timezone
  setTimezone(timezone);
  if(!getLocalTime(&timeinfo)){
    TRACE("Failed to obtain time\n");
    return;
  }
  TRACE("NTP TIME: %02d:%02d:%02d\n", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  syncRTC();
}

/**
 * Get the offset between the RTC and the Unix time.
 *
 * @return the offset in seconds
 */
long int getRtcOffset() {
  tm timeinfo;
  getLocalTime(&timeinfo);
  time_t now = mktime(&timeinfo);
  return rtc.now().unixtime() - now;
}

void printLocalTime() {
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    TRACE("Failed to obtain time 1\n");
    return;
  }
  PRINTLN(&timeinfo, "Local Time: %A, %B %d %Y %H:%M:%S zone %Z %z ");
}

void printRtcTime() {
  DateTime now = rtc.now();
  TRACE("%04d/%02d/%02d %02d:%02d:%02d\n", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}


// Function to handle serial communication in a FreeRTOS task
void serialPortHandler(void *pvParameters) {
  uint8_t timer = 0;
  Serial.flush();
  while (true) {

    if (Serial.available() > 0) {
      String command = Serial.readStringUntil('\n');
      Serial.flush();
      command.trim();
      
    }
    if (timer >= 100) {
      // Runs once a second
      timer = 0;
    } else {
      timer += 1;
    }
    Serial.flush();
    vTaskDelay(10 / portTICK_PERIOD_MS);  // Small delay to yield task
  }
}


void loop() {
  if (bmp.takeForcedMeasurement()) {
    // can now print out the new measurements
    DateTime now = rtc.now();
    // Read pressure in hPa and temperature in Celsius
    float pressure_hPa = bmp.readPressure() / 100.0F;
    float temperature = bmp.readTemperature();
    TRACE("%04d/%02d/%02d %02d:%02d:%02d -> Pressure: %.2f Temperature: %.2f\n", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second(), pressure_hPa, temperature);
  } else {
    TRACE("Forced measurement failed!\n");
  }
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}