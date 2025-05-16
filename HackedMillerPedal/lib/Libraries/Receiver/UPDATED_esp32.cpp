#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include "Adafruit_MCP4725.h"
#define I2C_ADDR 0x08
#define AVERAGE_WINDOW_SIZE 10  // Number of samples to average

unsigned long lastReceivedTime = 0;
const unsigned long timeout = 500; // Timeout in milliseconds
bool dacChanged = false; // Track if we've already set to zero
int mappedValue;

Adafruit_MCP4725 dac;
const int analogOut = A0;
int CurrentPotValue;

// Define struct for received message
typedef struct struct_message {
  int potValue;
} struct_message;

struct_message pedalData;

// Circular buffer for moving average
int buffer[AVERAGE_WINDOW_SIZE] = {0};
int bufferIndex = 0;
int sum = 0;
int count = 0;

int dac8bit;

int average(int newValue) {
  sum -= buffer[bufferIndex];  // Remove the oldest value from sum
  buffer[bufferIndex] = newValue;  // Insert the new value
  sum += newValue;  // Add the new value to sum

  bufferIndex = (bufferIndex + 1) % AVERAGE_WINDOW_SIZE;  // Update index
  if (count < AVERAGE_WINDOW_SIZE) {
    count++;  // Ensure count does not exceed the window size
  }
  return sum / count;

}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {

    memcpy(&pedalData, incomingData, sizeof(pedalData));
    dacChanged = true;  // Reset the flag

    mappedValue = map(pedalData.potValue, 0, 4095, 0, 255);//* 16;  // Map 0-255 to 0-4095
    // mappedValue = constrain(mappedValue, 0, 255);  // Ensure value is within 0-255
    // CurrentPotValue = average(mappedValue);  // Apply smoothing filter
    CurrentPotValue = mappedValue; // No smoothing
    // Wire.beginTransmission((uint8_t)I2C_ADDR);
    // Wire.write(CurrentPotValue);
    // Wire.endTransmission();
    // delay(10);
    lastReceivedTime = millis();  // Update the timestamp when data is received


}

void setup() {
  Wire.begin(D7,D6, 40000UL); // D7 -> SDA, D6 -> SCL
  dac.begin(0x62);
  dac.setVoltage(0, false);
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
}

// }
void loop() {
    if (dacChanged) {
        // Switch case to set the voltage based on dac8bit level
        dac8bit = average(CurrentPotValue);
        dac8bit = map(CurrentPotValue, 0, 255, 0, 4095);
        dac8bit = constrain(dac8bit, 0, 4095);
        dac.setVoltage(dac8bit, false);

        Serial.println("Pot Value: " );

        Serial.print(dac8bit);

        dacChanged = false;

    }
    else if (millis() - lastReceivedTime > timeout) {

        dac.setVoltage(0, false);
    }

    }