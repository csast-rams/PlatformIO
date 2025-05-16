
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h> //Using I2C communication level-shifter module to communicate with Arduino nano

/* Keep in mind: i2c level shifter is digital; no analog mapping directly from esp32 -> nano over i2c*/

#define RX_PIN D7
#define TX_PIN D6


// Analog output pin
const int analogOut = A0;
volatile int CurrentPotValue = 0;
// unsigned long lastDataTime = 0; // Timestamp of the last received data
// const unsigned long timeout = 1000; // Timeout period in milliseconds
volatile bool dataReceived = false;

hw_timer_t *timeoutTimer = NULL; // Hardware timer

//Structure for Received message
typedef struct struct_message {
  int potValue;
} struct_message;

struct_message pedalData;


void IRAM_ATTR OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&pedalData, incomingData, sizeof(pedalData));
  // analogWrite(analogOut, pedalData.potValue);
  CurrentPotValue = pedalData.potValue;
  // Serial.print("Pot Value: ");
  Serial.println(CurrentPotValue);
  // Serial1.println(CurrentPotValue);
  dataReceived = true;
  timerWrite(timeoutTimer, 0);
}

void IRAM_ATTR onTimeout() {
  CurrentPotValue = 0;
  dataReceived = true;
}


void setup() {
  analogWriteResolution(12);

  Serial.begin(115200);
  // Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  // CurrentPotValue = 0;
  WiFi.mode(WIFI_STA);
  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  delayMicroseconds(100);
  //Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
  // Configure hardware timer (500ms timeout)
  timeoutTimer = timerBegin(0, 80, true); // Timer 0, prescaler 80 -> 1µs per tick
  timerAttachInterrupt(timeoutTimer, &onTimeout, true);
  timerAlarmWrite(timeoutTimer, 500000, false); // 500ms timeout
  timerAlarmEnable(timeoutTimer);  // Enable timeout timer
}

void loop() {
  if (dataReceived) {  // Only update analog pin if a change occurs
      dataReceived = false;  // Reset flag
      analogWrite(analogOut, CurrentPotValue);
  }
}