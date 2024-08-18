#include "main.h"


/**
 * Hardware setup
 */
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  while (!Serial) {
    vTaskDelay(100 / portTICK_PERIOD_MS); // wait for serial port to connect. Needed for native USB
  }

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
  // Initialize AHT10
  if (!aht.begin()) {
    TRACE("Could not find a valid AHT10 sensor, check wiring!\n");
    Serial.flush();
    abort();
  }

  // The ADC input range (or gain) can be changed via the following
  // functions, but be careful never to exceed VDD +0.3V max, or to
  // exceed the upper and lower limits if you adjust the input range!
  // Setting these values incorrectly may destroy your ADC!
  //                                                                ADS1015  ADS1115
  //                                                                -------  -------
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

  if (!ads.begin()) {
    TRACE("Could not find a valid ADS1115 sensor, check wiring!\n");
    Serial.flush();
    abort();
  }
  

// Serial commander task
#if defined(ENABLE_SERIAL_COMMANDS)
  xTaskCreatePinnedToCore(
    serialPortHandler,     // Task function
    "SerialPort",          // Name of the task (for debugging)
    89000,                 // Stack size (in words, not bytes)
    NULL,                  // Task input parameter
    PRIORITY_HIGH,         // Priority of the task
    &serialTaskHandle,     // Task handle
    1                      // Core where the task should run
  );
#endif

  // Good To Go!
  vTaskDelay(100 / portTICK_PERIOD_MS);
}

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

// Function to handle serial communication in a FreeRTOS task
void serialPortHandler(void *pvParameters) {
  uint8_t timer = 0;
  Serial.flush();
  while (true) {

    if (Serial.available() > 0) {
      String command = Serial.readStringUntil('\n');
      Serial.flush();
      command.trim();
      handleSerialCommand(command);
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
  // Nothing
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void handleSerialCommand(const String& command) {
  // Number of commands in the array
  const int numCommands = sizeof(commandHandlers) / sizeof(commandHandlers[0]);
  // Trim and identify command type
  String cmd = command;
  cmd.trim();

  // Find command in array
  for (int i = 0; i < numCommands; i++) {
    if (cmd.startsWith(commandHandlers[i].command)) {
      // Call handler with remaining part of command
      commandHandlers[i].handler(cmd.substring(cmd.indexOf(':') + 1));
      return;
    }
  }
  // Handle unknown command
  handleUnknown();
}

void handleUnknown() {
  TRACE("Unknown input\n");
}


void handleGetSensor(const String& command) {
  DateTime now = rtc.now();
  sensors_event_t humidity, temperature;
  aht.getEvent(&humidity, &temperature);

  /* Be sure to update this value based on the IC and the gain settings! */
  float multiplier = 0.1875F; /* ADS1115  @ +/- 6.144V gain (16-bit results) */

  // Constants for mapping ADC values to percentage
  const int16_t min_value = 1500; // ADC value when submerged in water
  const int16_t max_value = 2800; // ADC value when completely dry

  // Read the values from each of the four channels
  int16_t adc0 = ads.readADC_SingleEnded(0) * multiplier;
  int16_t adc1 = ads.readADC_SingleEnded(1) * multiplier;
  int16_t adc2 = ads.readADC_SingleEnded(2) * multiplier;
  int16_t adc3 = ads.readADC_SingleEnded(3) * multiplier;

  // Map ADC values to percentage
  float percentage0 = (1.0 - ((float)(adc0 - min_value) / (max_value - min_value))) * 100.0;
  float percentage1 = (1.0 - ((float)(adc1 - min_value) / (max_value - min_value))) * 100.0;
  float percentage2 = (1.0 - ((float)(adc2 - min_value) / (max_value - min_value))) * 100.0;
  float percentage3 = (1.0 - ((float)(adc3 - min_value) / (max_value - min_value))) * 100.0;

  // Ensure percentage is within 0 to 100%
  percentage0 = constrain(percentage0, 0.0, 100.0);
  percentage1 = constrain(percentage1, 0.0, 100.0);
  percentage2 = constrain(percentage2, 0.0, 100.0);
  percentage3 = constrain(percentage3, 0.0, 100.0);

  // %Y-%m-%dT%H:%M:%S
  TRACE("%04d-%02d-%02dT%02d:%02d:%02d -> humidity: %.2f temperature: %.2f soil_moisture_0: %.2f%% soil_moisture_1: %.2f%% soil_moisture_2: %.2f%% soil_moisture_3: %.2f%%\n", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second(), humidity.relative_humidity, temperature.temperature, percentage0, percentage1, percentage2, percentage3);
}

void handleRawSensor(const String& command) {
  DateTime now = rtc.now();
  sensors_event_t humidity, temperature;
  aht.getEvent(&humidity, &temperature);

  /* Be sure to update this value based on the IC and the gain settings! */
  float multiplier = 0.1875F; /* ADS1115  @ +/- 6.144V gain (16-bit results) */

  // Read the values from each of the four channels
  int16_t adc0 = ads.readADC_SingleEnded(0); //  * multiplier
  int16_t adc1 = ads.readADC_SingleEnded(1); //  * multiplier
  int16_t adc2 = ads.readADC_SingleEnded(2); //  * multiplier
  int16_t adc3 = ads.readADC_SingleEnded(3); //  * multiplier

  TRACE("%04d-%02d-%02dT%02d:%02d:%02d -> humidity: %.2f temperature: %.2f soil_moisture_0: %d soil_moisture_1: %d soil_moisture_2: %d soil_moisture_3: %d\n", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second(), humidity.relative_humidity, temperature.temperature, adc0, adc1, adc2, adc3);
}