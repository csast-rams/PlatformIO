#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>

#define I2C_ADDR 0x08
#define AVERAGE_WINDOW_SIZE 4  // Number of samples to average

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

int average(int newValue) {
  sum -= buffer[bufferIndex];  // Remove the oldest value from sum
  buffer[bufferIndex] = newValue;  // Insert the new value
  sum += newValue;  // Add the new value to sum

  bufferIndex = (bufferIndex + 1) % AVERAGE_WINDOW_SIZE;  // Update index
  if (count < AVERAGE_WINDOW_SIZE) {
    count++;  // Ensure count does not exceed the window size
  }

  return sum / count;  // Return the rolling average
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&pedalData, incomingData, sizeof(pedalData));

  int mappedValue = map(pedalData.potValue, 0, 4095, 0, 255);
  CurrentPotValue = average(mappedValue);  // Apply smoothing filter

  Wire.beginTransmission((uint8_t)I2C_ADDR);
  Wire.write(CurrentPotValue);
  Wire.endTransmission();

  Serial.print("Raw Pot Value: ");
  Serial.print(pedalData.potValue);
  Serial.print(" -> Mapped: ");
  Serial.print(mappedValue);
  Serial.print(" -> Smoothed: ");
  Serial.println(CurrentPotValue);
}

void setup() {
  Wire.begin(D7, D6); // SCL, SDA
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
}