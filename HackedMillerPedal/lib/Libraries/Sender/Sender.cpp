////SENDER CODE

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

#define ADC_SEL A0
#define OptoSig D10 // Opto-isolated contactor signal; initiates data transmission

// Use A0 for analog reading
const uint8_t SignalWiper = A0;
// Broadcast MAC address of the receiver
#define MAC_ADDRESS {0x9c, 0x9e, 0x6e, 0xf7, 0x3b, 0x8c}
uint8_t broadcastAddress[] = MAC_ADDRESS;

// Structure of the data being sent to the receiver
typedef struct struct_message {
  int potValue;
} struct_message;
struct_message pedalData;

// EspNow peer info
esp_now_peer_info_t peerInfo;

// Callback for data sent event (for debugging)
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Optionally, add status checking here.
}

// Timing for sending and sampling (non-blocking)
unsigned long lastSendTime = 0;
unsigned long lastSampleTime = 0;
const unsigned long sendIntervalMicros = 50;   // Interval for sending data
const unsigned long sampleIntervalMicros = 50; // Interval for ADC sampling

// Number of ADC samples to average per reading
const int numSamples = 6;

// Global variable for smoothed ADC value
int smoothedValue = 0;

// Simple exponential smoothing function.
// 'alpha' is a percentage (0-100) determining weight of new sample.
int smooth(int newData, int *prevSmooth, int alpha) {
  *prevSmooth = (alpha * newData + (100 - alpha) * (*prevSmooth));
  return *prevSmooth / 100;
}

void setup() {
  Serial.begin(115200);
  
  // Configure the opto signal pin
  pinMode(OptoSig, INPUT);
  
  // Configure ADC attenuation if needed
  analogSetAttenuation(ADC_11db);

  // Set device as a Wi-Fi station and initialize ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  delayMicroseconds(100);

  // Set up peer info
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // Register the send callback
  esp_now_register_send_cb(OnDataSent);
}

void loop() {
  unsigned long now = micros();
  
  // --- ADC Sampling & Smoothing (runs regardless of opto state) ---
  if (now - lastSampleTime >= sampleIntervalMicros) {
    long sum = 0;
    for (int i = 0; i < numSamples; i++) {
      sum += analogRead(SignalWiper);
    }
    int avg = sum / numSamples;
    // Update global smoothed value (using alpha=90 for strong smoothing)
    smoothedValue = smooth(avg, &smoothedValue, 90);
    lastSampleTime = now;
    
    // (Optional) Uncomment for debugging ADC values:
    // Serial.print("ADC Avg: ");
    // Serial.print(avg);
    // Serial.print(" | Smoothed: ");
    // Serial.println(smoothedValue);
  }
  
  // --- Transmission (only when opto signal is active) ---
  bool optoActive = !digitalRead(OptoSig); // Invert if necessary
  if (optoActive && (now - lastSendTime >= sendIntervalMicros)) {
    // Constrain the value and assign to the message struct
    pedalData.potValue = map(smoothedValue, 0, 4094, 0, 255);
    // pedalData.potValue = constrain(smoothedValue, 0, 255);
    
    // Send the smoothed value via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &pedalData, sizeof(pedalData));
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    } else {
      Serial.println("Error sending the data");
    }
    
    Serial.print("Smoothed Pot Value: ");
    Serial.println(pedalData.potValue);
    lastSendTime = now;
  }
  else if (!optoActive) {
    // If opto signal is inactive, reset the last send time
    // lastSendTime = now; // Reset to avoid sending when not needed
    pedalData.potValue = 0; // Optional: Send zero value when inactive
    esp_err_t Result = esp_now_send(broadcastAddress, (uint8_t *) &pedalData, sizeof(pedalData));
    if (Result == ESP_OK) {
      Serial.println("Sent with success");
    } else {
      Serial.println("Error sending the data");
    }

  }
}
