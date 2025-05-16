#include <Arduino.h>
#include <Adafruit_MCP4725.h>

// Pin definitions
#define analogout2 A3         // PWM output pin (used when outmode==1)
#define OutsetPin D8          // Digital pin to select output mode via interrupt
#define FREQ_POT_PIN A0       // Potentiometer for PWM frequency control
#define MAXVOLT_POT_PIN A1    // Potentiometer for maximum PWM duty cycle

// Output mode: 0 = DAC output, 1 = PWM output  
// (volatile because it is updated inside an ISR)
volatile int outmode = 0;

// UART data reception variables
volatile bool newDataReceived = false;
int rawData = 0;    // Raw value received via UART (expected 0–255)

unsigned long lastReceiveTime = 0;
const unsigned long timeoutInterval = 230;  // Timeout (ms)

// Create DAC object (MCP4725 on I²C, default address 0x62)
Adafruit_MCP4725 dac;

// --- Interrupt Service Routine ---
// Update outmode based on OutsetPin state:  
// LOW => DAC mode (0), HIGH => PWM mode (1)
void updateOutmode() {
  if (digitalRead(OutsetPin) == LOW) {
    outmode = 0;
  } else {
    outmode = 1;
  }
}

void setup() {
  // Set up OutsetPin with internal pullup and attach interrupt
  pinMode(OutsetPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(OutsetPin), updateOutmode, CHANGE);

  Wire.setSCL(D5);
  Wire.setSDA(D4);
  Wire.setClock(400000); // Set clock speed to 400kHz

  // Initialize Serial for debugging
  Serial.begin(115200);
  delay(1000); // Allow time for system stabilization

  // Initialize Serial1 for UART data reception (adjust baud rate as needed)
  Serial1.begin(115200);

  // Initialize the MCP4725 DAC (I²C is automatically configured on RP2040)
  dac.begin(0x62);

  // Set output pin for PWM; no further configuration needed here
  pinMode(analogout2, OUTPUT);
}

void loop() {
  // --- UART Reception ---
  // Check if new UART data is available on Serial1.
  // We assume the sender transmits an ASCII integer terminated by newline.
  if (Serial1.available() > 0) {
    String input = Serial1.readStringUntil('\n');
    rawData = input.toInt();  // Convert received string to an integer (0–255)
    newDataReceived = true;
    lastReceiveTime = millis();
  }

  // --- Read Potentiometers for PWM Parameters ---
  // Read the frequency control potentiometer (assumed 10-bit resolution: 0–1023)
  int freqPot = analogRead(FREQ_POT_PIN);
  // Map to a PWM frequency range, e.g., from 100 Hz to 2000 Hz
  int pwmFreq = map(freqPot, 0, 1023, 100, 2000);
  
  // Read the maximum PWM level potentiometer (0–1023)
  int maxVoltPot = analogRead(MAXVOLT_POT_PIN);
  // Map to a maximum duty cycle (0–255)
  int maxPWMVal = map(maxVoltPot, 0, 1023, 0, 255);

  // --- Process New Data ---
  if (newDataReceived) {
    newDataReceived = false;
    
    // For DAC output: map raw value (0–255) to DAC range (0–4095)
    int dacValue = map(rawData, 0, 255, 0, 4095);
    // For PWM output: scale raw value by the maximum PWM level
    int pwmValue = (rawData * maxPWMVal) / 255;

    if (outmode == 0) {
      // DAC output mode: write value to MCP4725 DAC
      dac.setVoltage(dacValue, false);
      Serial.printf("Mode: DAC | Raw: %d, DAC Value: %d\n", rawData, dacValue);
    } else if (outmode == 1) {
      // Set PWM frequency (if supported)
      // This function may not be available on all cores; adjust if needed.
      // analogWriteFrequency(analogout2, pwmFreq);
      
      // PWM output mode: output via analogWrite on the specified pin
      analogWrite(analogout2, pwmValue);
      Serial.printf("Mode: PWM | Raw: %d, PWM: %d, Freq: %d Hz, MaxPWM: %d\n", rawData, pwmValue, pwmFreq, maxPWMVal);
    }
  } 
  else if (millis() - lastReceiveTime > timeoutInterval) {
    // If no new UART data within the timeout, zero the output for PWM mode
    if (outmode == 1) {
      analogWrite(analogout2, 0);
    }
  }
}


// #include <Arduino.h>
// #include <Adafruit_I2CDevice.h>
// #include <Wire.h>
// #include <Adafruit_MCP4725.h>

// #define I2C1_ADDR 0x08    // I2C address for esp-now receiver data
// #define contactorPin D7
// #define analogout2 A3     // Use A3 as PWM output when outmode==1
// #define OutsetPin D8      // Pin used to select output mode

// // Set output mode: 0 = DAC output, 1 = PWM output (volatile since updated in ISR)
// volatile int outmode = 0;

// // Volatile variable used in the I2C receive callback
// volatile bool newDataReceived = false;
// int rawData = 0;    // Raw data received from I2C

// // Last time data was received
// unsigned long lastReceiveTime = 0;
// const unsigned long timeoutInterval = 230;

// // Set Adafruit MCP4725 DAC as I2C object
// Adafruit_MCP4725 dac;

// // I2C receive event callback (non-blocking)
// void receiveEvent(int bytesReceived) {
//   if (Wire1.available()) {
//     rawData = Wire1.read();
//     lastReceiveTime = millis();
//     newDataReceived = true;
//   }
// }

// // Interrupt service routine to update outmode based on OutsetPin state
// // When OutsetPin is LOW, set outmode to 0 (DAC output); when HIGH, set outmode to 1 (PWM output)
// void updateOutmode() {
//   if (digitalRead(OutsetPin) == LOW) {
//     outmode = 0;
//   } else {
//     outmode = 1;
//   }
// }

// void setup() {
//   // Configure OutsetPin with an internal pullup and attach an interrupt to it
//   pinMode(OutsetPin, INPUT_PULLUP);
//   attachInterrupt(digitalPinToInterrupt(OutsetPin), updateOutmode, CHANGE);

//   Serial.begin(115200);
//   delay(1000); // Allow time for the circuit to stabilize

//   // Set up I2C for primary bus (if needed for other sensors)
//   Wire.setSCL(D5);
//   Wire.setSDA(D4);
//   Wire.setClock(400000); // Set clock speed to 400kHz

//   // Set up I2C for secondary bus (Wire1) used for receiving data
//   Wire1.setSCL(D27); // ESP-Now Receiver SCL (as configured on the ESP32 receiver)
//   Wire1.setSDA(D26); // ESP-Now Receiver SDA
//   Wire1.begin((uint8_t)I2C1_ADDR);

//   // Initialize the DAC (using address 0x62)
//   dac.begin(0x62);

//   // Set additional output pins
//   pinMode(contactorPin, OUTPUT);
//   pinMode(analogout2, OUTPUT);

//   // Register the I2C receive callback
//   Wire1.onReceive(receiveEvent);
// }

// void loop() {
//   if (newDataReceived) {
//     newDataReceived = false;

//     // For DAC output: map raw value (0-255) to 0-4095
//     int dacValue = map(rawData, 0, 255, 0, 4095);
//     // For PWM output: use raw value directly (assuming 8-bit resolution, 0-255)
//     int pwmValue = rawData;

//     if (outmode == 0) {
//       dac.setVoltage(dacValue, false);
//       Serial.printf("Output Mode: DAC | Raw Data: %d, DAC Value: %d\n", rawData, dacValue);
//     }
//     else if (outmode == 1) {
//       analogWrite(analogout2, pwmValue);
//       Serial.printf("Output Mode: PWM | Raw Data: %d, PWM Value: %d\n", rawData, pwmValue);
//     }
//   }
//   else if (millis() - lastReceiveTime > timeoutInterval) {
//     // If no new data within the timeout and PWM mode is active, output zero
//     if (outmode == 1) {
//       analogWrite(analogout2, 0);
//     }
//   }
// }

// // #include <Arduino.h>
// // #include <Adafruit_I2CDevice.h>
// // #include <Wire.h>
// // #include <Adafruit_MCP4725.h>

// // #define I2C1_ADDR 0x08    // I2C address for esp-now receiver data
// // #define contactorPin D7
// // #define analogout2 A3     // Use A2 as PWM output when outmode==1

// // // Set output mode: 0 = DAC output, 1 = PWM output
// // int outmode = 1;  // Change to 1 for PWM output

// // // Volatile variable used in the I2C receive callback
// // volatile bool newDataReceived = false;
// // int rawData = 0;    // Raw data received from I2C

// // // Last time data was received
// // unsigned long lastReceiveTime = 0;
// // const unsigned long timeoutInterval = 230;

// // // Set Adafruit MCP4725 DAC as I2C object
// // Adafruit_MCP4725 dac;

// // // I2C receive event callback (non-blocking)
// // void receiveEvent(int bytesReceived) {
// //   if (Wire1.available()) {
// //     rawData = Wire1.read();
// //     lastReceiveTime = millis();
// //     newDataReceived = true;
// //   }
// // }

// // void setup() {
// //   Serial.begin(115200);

// //   // Set up I2C for primary bus (if needed for other sensors)
// //   Wire.setSCL(D5);
// //   Wire.setSDA(D4);
// //   Wire.setClock(400000); // Set clock speed to 400kHz

// //   // Set up I2C for secondary bus (Wire1) used for receiving data
// //   Wire1.setSCL(D27); // ESP-Now Receiver SCL (as configured on the ESP32 receiver)
// //   Wire1.setSDA(D26); // ESP-Now Receiver SDA
// //   Wire1.begin((uint8_t)I2C1_ADDR);

// //   // Initialize the DAC (using address 0x62)
// //   dac.begin(0x62);

// //   // Set output pins
// //   pinMode(contactorPin, OUTPUT);
// //   pinMode(analogout2, OUTPUT);

// //   // Register the I2C receive callback
// //   Wire1.onReceive(receiveEvent);
// // }

// // void loop() {
// //   if (newDataReceived) {
// //     // Clear the flag and capture the raw data
// //     newDataReceived = false;

// //     // For DAC output: map raw value (0-255) to 0-4095
// //     int dacValue = map(rawData, 0, 255, 0, 4095);
// //     // For PWM output: use raw value directly (assumed 0-255 PWM resolution)
// //     int pwmValue = rawData;

// //     if (outmode == 0) {
// //       // DAC output mode: write to the MCP4725
// //       dac.setVoltage(dacValue, false);
// //       Serial.printf("Output Mode: DAC | Raw Data: %d, DAC Value: %d\n", rawData, dacValue);
// //     }
// //     else if (outmode == 1) {
// //       // PWM output mode: write PWM value to analogout2
// //       analogWrite(analogout2, pwmValue);
// //       Serial.printf("Output Mode: PWM | Raw Data: %d, PWM Value: %d\n", rawData, pwmValue);
// //     }
// //   }
// //   else if (millis() - lastReceiveTime > timeoutInterval) {
// //     // If no new data within the timeout, output zero
// //     // if (outmode == 0) {
// //     //   dac.setVoltage(0, false);
// //     // }
// //     if (outmode == 1) {
// //       analogWrite(analogout2, 0);
// //     }
// //   }
// // }

