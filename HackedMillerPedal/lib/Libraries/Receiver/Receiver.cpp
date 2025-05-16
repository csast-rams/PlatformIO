//RECEIVER CODE

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>

#define I2C_ADDR 0x08
#define AVERAGE_WINDOW_SIZE 10  // Number of samples to average

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
  sum -= buffer[bufferIndex];          // Remove the oldest value from sum
  buffer[bufferIndex] = newValue;        // Insert the new value
  sum += newValue;                       // Add the new value to sum
  bufferIndex = (bufferIndex + 1) % AVERAGE_WINDOW_SIZE;  // Update index
  
  if (count < AVERAGE_WINDOW_SIZE) {
    count++;  // Increase count until the buffer is full
  }

  return sum / count;  // Return the rolling average
}

int CurrentPotValue;

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  // Check if the received length matches expected size
  // if (len != sizeof(pedalData)) {
  //   Serial.println("Received data length mismatch");
  //   return;
  // }
  
  memcpy(&pedalData, incomingData, sizeof(pedalData));

  int mappedValue = pedalData.potValue; // Use the raw value directly
  
  // Apply the smoothing filter
  CurrentPotValue = average(mappedValue);  
  // CurrentPotValue = map(CurrentPotValue, 0, 255,0,255); // Ensure value is within bounds
  CurrentPotValue = constrain(CurrentPotValue, 0, 4095); // Ensure value is within bounds
  // If you want the smoothed value, remove the following line:
  // CurrentPotValue = mappedValue;  // This line cancels the smoothing filter
  
  // Wire.beginTransmission((uint8_t)I2C_ADDR);
  // Wire.write(CurrentPotValue);
  // Wire.endTransmission();

  

  // Serial.print("Raw Pot Value: ");
  // Serial.print(pedalData.potValue);
  // Serial.print(" -> Mapped: ");
  // Serial.print(mappedValue);
  Serial.print(" -> Smoothed: ");
  Serial.println(CurrentPotValue);
}

void setup() {
  Serial.begin(115200);

  // Initialize I2C on specified pins; ensure these match your hardware
  Wire.begin(D7, D6); // SDA -> D7, SCL -> D6, 40MHz clock

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  // Nothing needed in the loop as data is handled in the callback
}