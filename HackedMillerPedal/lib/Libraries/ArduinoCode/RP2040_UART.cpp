// /* RP2040 */

#include <Arduino.h>
#include <pico/stdlib.h>
#include <Adafruit_MCP4725.h>
#include <RP2040_PWM.h>
// analogWriteFreq()
RP2040_PWM* PWM;

float  Freq;
float  Duty;
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
const unsigned long timeoutInterval = 200;  // Timeout (ms)

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
  analogWriteResolution(8); // Set resolution to 8 bits (0-255)
  // Set up OutsetPin with internal pullup and attach interrupt
  pinMode(OutsetPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(OutsetPin), updateOutmode, CHANGE);

  // Initialize Serial for debugging
  Serial.begin(115200);
  delay(1000); // Allow time for system stabilization

  // Initialize Serial1 for UART data reception (adjust baud rate as needed)
  Serial1.begin(115200);

  Wire.setSCL(D5);
  Wire.setSDA(D4);
  // Wire.setClock(400000); // Set clock speed to 400kHz

  // Initialize the MCP4725 DAC (I²C is automatically configured on RP2040)
  dac.begin(0x62);

  // Set output pin for PWM; no further configuration needed here
  pinMode(analogout2, OUTPUT);
  Freq = 100.0f;   // initial frequency in Hz
  Duty = 0.0f;     // initial duty cycle in percentage
  PWM = new RP2040_PWM(analogout2, Freq, 0);
  if (PWM) {
    PWM->setPWM();  // Initialize the PWM output
  }


}


void loop() {
  float DutyFactor = 0.9; //90% DC
  Freq = 10.0f; // DEBUG Set to "pwmFreq = 100" 100Hz
  // --- UART Reception ---
  // Check if new UART data is available on Serial1.
  // We assume the sender transmits an ASCII integer terminated by newline.
  // if (Serial1.available() >= 2) {
  //   uint8_t data[2];
  //   Serial1.readBytes(data, 2);
  //   rawData = (data[0] << 8) | data[1];
  //   newDataReceived = true;
  //   lastReceiveTime = millis();
  // }

  // --- UART Reception ---
  if (Serial1.available() >= 2) {
    uint8_t data[2];
    Serial1.readBytes(data, 2);

    // The XIAO code sends the low byte first, then the high byte (little-endian).
    // Reconstruct that accordingly:
    rawData = (data[0]) | (data[1] << 8);

    newDataReceived = true;
    lastReceiveTime = millis();
  }
  // --- Read Potentiometers for PWM Parameters ---
  // Read the frequency control potentiometer (assumed 10-bit resolution: 0–1023)


  
  // int freqPot = analogRead(FREQ_POT_PIN);
  /* DEBUG BY UNCOMMENTING BELOW AND COMMENTING OUT analogRead*/

  // int freqPot = 0; // DEBUG Set to "freqpot = 0 
  // Map to a PWM frequency range, e.g., from 1 Hz to 100 Hz
  // int pwmFreq = map(freqPot, 0, 65535, 1, 100);
  // Read the maximum PWM level potentiometer (0–1023)
  // int maxVoltPot = analogRead(MAXVOLT_POT_PIN);
  /* DEBUG BY UNCOMMENTING BELOW AND COMMENTING OUT analogRead*/
  // int maxVoltPot = 1023; // DEBUG Set to "maxVoltPot = 10bit positive integer"
  // Map to a maximum duty cycle (0–255)
  // int maxPWMVal = map(maxVoltPot, 0, 65535, 0, 255);

  // PWM -> setPWMFrequency(pwmFreq); // Set the PWM frequency
  // Set the maximum PWM level (0–255)
  // PWM -> setPWMLevel(maxPWMVal); // Set the maximum PWM level
  // --- Process New Data ---
  if (newDataReceived) {
    newDataReceived = false;
    // For DAC output: map raw value (0–255) to DAC range (0–4095)
    int dacValue = map(rawData, 0, 255, 0, 4095);
    Duty = float(dacValue / DutyFactor); // Map raw value to duty cycle (0-100%)

    // For PWM output: scale raw value by the maximum PWM level
    // int pwmValue = (dacValue * maxPWMVal) / 4095; // Scale to 0-255 range
    int pwmValue = dacValue; // Assuming 8-bit resolution for PWM

    if (outmode == 0) {
      // DAC output mode: write value to MCP4725 DAC
      dac.setVoltage(dacValue, false);
      // Serial.printf("Mode: DAC | Raw: %d, DAC Value: %d, Freq: %.1f Hz, MaxPWM: %.1f\n", rawData, pwmValue, Freq, Duty);
    } else if (outmode == 1) {
      // Set PWM frequency (if supported)
      // // This function may not be available on all cores; adjust if needed.
      // analogWriteFrequency(analogout2, pwmFreq);
      // PWM->setPWM(analogout2, 100, Duty);
      analogWriteFreq(Freq);
      analogWrite(analogout2, dacValue);

      // Serial.printf("Mode: PWM | Raw: %d, PWM: %d, Freq: %.1f Hz, MaxPWM: %.1f\n", rawData, pwmValue, Freq, Duty);
      // Serial.printf("Mode: PWM | Raw: %d, PWM: %d\n", rawData, pwmValue);

    }
  }
  else if (millis() - lastReceiveTime > timeoutInterval) {
    // If no new UART data within the timeout, zero the output for PWM mode
    // if (outmode == 1) {
    //   // analogWrite(analogout2, 0);
    //   PWM->setPWM(analogout2, Freq, 0);
    // }
    // PWM->setPWM(analogout2, Freq, 0);
    analogWrite(analogout2, 0);
  }
}



// /* RP2040 */

// #include <Arduino.h>
// #include <Adafruit_MCP4725.h>
// #include <RP2040_PWM.h>

// RP2040_PWM* PWM;

// // Pin definitions
// #define analogout2 A3         // PWM output pin (used when outmode==1)
// #define OutsetPin D8          // Digital pin to select output mode via interrupt
// #define FREQ_POT_PIN A0       // Potentiometer for PWM frequency control
// #define MAXVOLT_POT_PIN A1    // Potentiometer for maximum PWM duty cycle

// // Output mode: 0 = DAC output, 1 = PWM output  
// // (volatile because it is updated inside an ISR)
// volatile int outmode = 0;

// // UART data reception variables
// volatile bool newDataReceived = false;
// int rawData = 0;    // Raw value received via UART (expected 0–255)

// unsigned long lastReceiveTime = 0;
// const unsigned long timeoutInterval = 230;  // Timeout (ms)

// // Create DAC object (MCP4725 on I²C, default address 0x62)
// Adafruit_MCP4725 dac;

// // --- Interrupt Service Routine ---
// // Update outmode based on OutsetPin state:  
// // LOW => DAC mode (0), HIGH => PWM mode (1)
// void updateOutmode() {
//   if (digitalRead(OutsetPin) == LOW) {
//     outmode = 0;
//   } else {
//     outmode = 1;
//   }
// }

// void setup() {
//   // Set up OutsetPin with internal pullup and attach interrupt
//   pinMode(OutsetPin, INPUT_PULLUP);
//   attachInterrupt(digitalPinToInterrupt(OutsetPin), updateOutmode, CHANGE);

//   // Initialize Serial for debugging
//   Serial.begin(115200);
//   delay(1000); // Allow time for system stabilization

//   // Initialize Serial1 for UART data reception (adjust baud rate as needed)
//   Serial1.begin(115200);

//   Wire.setSCL(D5);
//   Wire.setSDA(D4);
//   // Wire.setClock(400000); // Set clock speed to 400kHz

//   // Initialize the MCP4725 DAC (I²C is automatically configured on RP2040)
//   dac.begin(0x62);

//   // Set output pin for PWM; no further configuration needed here
//   pinMode(analogout2, OUTPUT);
// }

// void loop() {
//   // --- UART Reception ---
//   // Check if new UART data is available on Serial1.
//   // We assume the sender transmits an ASCII integer terminated by newline.
//   // if (Serial1.available() >= 2) {
//   //   uint8_t data[2];
//   //   Serial1.readBytes(data, 2);
//   //   rawData = (data[0] << 8) | data[1];
//   //   newDataReceived = true;
//   //   lastReceiveTime = millis();
//   // }

//   // --- UART Reception ---
//   if (Serial1.available() >= 2) {
//     uint8_t data[2];
//     Serial1.readBytes(data, 2);

//     // The XIAO code sends the low byte first, then the high byte (little-endian).
//     // Reconstruct that accordingly:
//     rawData = (data[0]) | (data[1] << 8);

//     newDataReceived = true;
//     lastReceiveTime = millis();
  

//   // --- Read Potentiometers for PWM Parameters ---
//   // Read the frequency control potentiometer (assumed 10-bit resolution: 0–1023)


  
//   // int freqPot = analogRead(FREQ_POT_PIN);
//   /* DEBUG BY UNCOMMENTING BELOW AND COMMENTING OUT analogRead*/

//   int freqPot = 0; // DEBUG Set to "freqpot = 0 
//   // Map to a PWM frequency range, e.g., from 1 Hz to 100 Hz
//   int pwmFreq = map(freqPot, 0, 65535, 1, 100);

//   // Read the maximum PWM level potentiometer (0–1023)
//   // int maxVoltPot = analogRead(MAXVOLT_POT_PIN);
//   /* DEBUG BY UNCOMMENTING BELOW AND COMMENTING OUT analogRead*/
//   int maxVoltPot = 1023; // DEBUG Set to "maxVoltPot = 10bit positive integer"
//   // Map to a maximum duty cycle (0–255)
//   int maxPWMVal = map(maxVoltPot, 0, 65535, 0, 255);

//   // --- Process New Data ---
//   if (newDataReceived) {
//     newDataReceived = false;
    
//     // For DAC output: map raw value (0–255) to DAC range (0–4095)
//     int dacValue = map(rawData, 0, 255, 0, 4095);
//     // For PWM output: scale raw value by the maximum PWM level
//     int pwmValue = (dacValue * maxPWMVal) / 4095; // Scale to 0-255 range
  
//     // int pwmValue = rawData; // Assuming 8-bit resolution for PWM

//     if (outmode == 0) {
//       // DAC output mode: write value to MCP4725 DAC
//       dac.setVoltage(dacValue, false);
//       // Serial.printf("Mode: DAC | Raw: %d, DAC Value: %d\n", rawData, dacValue);
//     } else if (outmode == 1) {
//       // Set PWM frequency (if supported)
//       // This function may not be available on all cores; adjust if needed.
//       // analogWriteFrequency(analogout2, pwmFreq);
      
//       // PWM output mode: output via analogWrite on the specified pin
//       analogWrite(analogout2, pwmValue);
//       // Serial.printf("Mode: PWM | Raw: %d, PWM: %d, Freq: %d Hz, MaxPWM: %d\n", rawData, pwmValue, pwmFreq, maxPWMVal);
//       // Serial.printf("Mode: PWM | Raw: %d, PWM: %d\n", rawData, pwmValue);

//     }
//   } 
//   else if (millis() - lastReceiveTime > timeoutInterval) {
//     // If no new UART data within the timeout, zero the output for PWM mode
//     if (outmode == 1) {
//       analogWrite(analogout2, 0);
//     }
//   }
// }












// #include <Arduino.h>
// #include <Adafruit_MCP4725.h>


// // Pin definitions
// #define analogout2 A3         // PWM output pin (used when outmode==1)
// #define OutsetPin D8          // Digital pin to select output mode via interrupt
// // #define FREQ_POT_PIN A0       // Potentiometer for PWM frequency control
// // #define MAXVOLT_POT_PIN A1    // Potentiometer for maximum PWM duty cycle

// // Output mode: 0 = DAC output, 1 = PWM output  
// // (volatile because it is updated inside an ISR)
// volatile int outmode = 0;

// // UART data reception variables
// volatile bool newDataReceived = false;
// int rawData = 0;    // Raw value received via UART (expected 0–255)

// unsigned long lastReceiveTime = 0;
// const unsigned long timeoutInterval = 230;  // Timeout (ms)

// // Create DAC object (MCP4725 on I²C, default address 0x62)
// Adafruit_MCP4725 dac;

// // --- Interrupt Service Routine ---
// // Update outmode based on OutsetPin state:  
// // LOW => DAC mode (0), HIGH => PWM mode (1)
// void updateOutmode() {
//   if (digitalRead(OutsetPin) == LOW) {
//     outmode = 0;
//   } else {
//     outmode = 1;
//   }
// }

// void setup() {
//   // Set up OutsetPin with internal pullup and attach interrupt
//   pinMode(OutsetPin, INPUT_PULLUP);
//   attachInterrupt(digitalPinToInterrupt(OutsetPin), updateOutmode, CHANGE);

//   // Initialize Serial for debugging
//   Serial.begin(115200);
//   delay(1000); // Allow time for system stabilization

//   // Initialize Serial1 for UART data reception (adjust baud rate as needed)
//   Serial1.begin(115200);

//   Wire.setSCL(D5);
//   Wire.setSDA(D4);
//   // Wire.setClock(400000); // Set clock speed to 400kHz

//   // Initialize the MCP4725 DAC (I²C is automatically configured on RP2040)
//   dac.begin(0x62);

//   // Set output pin for PWM; no further configuration needed here
//   pinMode(analogout2, OUTPUT);
// }

// void loop() {
//   // --- UART Reception ---
//   // Check if new UART data is available on Serial1.
//   // We assume the sender transmits an ASCII integer terminated by newline.
//   if (Serial1.available() >= 2) {
//     uint8_t data[2];
//     Serial1.readBytes(data, 2);
//     rawData = (data[0] << 8) | data[1];
//     newDataReceived = true;
//     lastReceiveTime = millis();
//   }

//   // --- Read Potentiometers for PWM Parameters ---
//   // Read the frequency control potentiometer (assumed 10-bit resolution: 0–1023)
//   // int freqPot = analogRead(FREQ_POT_PIN);
//   // Map to a PWM frequency range, e.g., from 100 Hz to 2000 Hz
//   // int pwmFreq = map(freqPot, 0, 1023, 100, 2000);
  
//   // Read the maximum PWM level potentiometer (0–1023)
//   // int maxVoltPot = analogRead(MAXVOLT_POT_PIN);
//   // Map to a maximum duty cycle (0–255)
//   // int maxPWMVal = map(maxVoltPot, 0, 1023, 0, 255);

//   // --- Process New Data ---
//   if (newDataReceived) {
//     newDataReceived = false;
    
//     // For DAC output: map raw value (0–255) to DAC range (0–4095)
//     int dacValue = map(rawData, 0, 255, 0, 4095);
//     // For PWM output: scale raw value by the maximum PWM level
//     // int pwmValue = (rawData * 255) / 255;
  
//     int pwmValue = rawData; // Assuming 8-bit resolution for PWM

//     if (outmode == 0) {
//       // DAC output mode: write value to MCP4725 DAC
//       dac.setVoltage(dacValue, false);
//     //   Serial.printf("Mode: DAC | Raw: %d, DAC Value: %d\n", rawData, dacValue);
//     } else if (outmode == 1) {
//       // Set PWM frequency (if supported)
//       // This function may not be available on all cores; adjust if needed.
//     //   analogWriteFrequency(analogout2, pwmFreq);
      
//       // PWM output mode: output via analogWrite on the specified pin
//       analogWrite(analogout2, pwmValue);
//       // Serial.printf("Mode: PWM | Raw: %d, PWM: %d, Freq: %d Hz, MaxPWM: %d\n", rawData, pwmValue, pwmFreq, maxPWMVal);
//     //   Serial.printf("Mode: PWM | Raw: %d, PWM: %d\n", rawData, pwmValue);

//     }
//   } 
//   else if (millis() - lastReceiveTime > timeoutInterval) {
//     // If no new UART data within the timeout, zero the output for PWM mode
//     if (outmode == 1) {
//       analogWrite(analogout2, 0);
//     }
//   }
// }




// #include <Arduino.h>
// #include <Adafruit_MCP4725.h>

// // Pin definitions
// #define analogout2 A3         // PWM output pin (used when outmode==1)
// #define OutsetPin D8          // Digital pin to select output mode via interrupt
// // #define FREQ_POT_PIN A0       // Potentiometer for PWM frequency control
// // #define MAXVOLT_POT_PIN A1    // Potentiometer for maximum PWM duty cycle

// // Output mode: 0 = DAC output, 1 = PWM output  
// // (volatile because it is updated inside an ISR)
// volatile int outmode = 0;

// // UART data reception variables
// volatile bool newDataReceived = false;
// int rawData = 0;    // Raw value received via UART (expected 0–255)

// unsigned long lastReceiveTime = 0;
// const unsigned long timeoutInterval = 230;  // Timeout (ms)

// // Create DAC object (MCP4725 on I²C, default address 0x62)
// Adafruit_MCP4725 dac;

// // --- Interrupt Service Routine ---
// // Update outmode based on OutsetPin state:  
// // LOW => DAC mode (0), HIGH => PWM mode (1)
// void updateOutmode() {
//   if (digitalRead(OutsetPin) == LOW) {
//     outmode = 0;
//   } else {
//     outmode = 1;
//   }
// }

// void setup() {
//   // Set up OutsetPin with internal pullup and attach interrupt
//   pinMode(OutsetPin, INPUT_PULLUP);
//   attachInterrupt(digitalPinToInterrupt(OutsetPin), updateOutmode, CHANGE);

//   // Initialize Serial for debugging
//   Serial.begin(115200);
//   delay(1000); // Allow time for system stabilization

//   // Initialize Serial1 for UART data reception (adjust baud rate as needed)
//   Serial1.begin(115200);

//   Wire.setSCL(D5);
//   Wire.setSDA(D4);
//   // Wire.setClock(400000); // Set clock speed to 400kHz

//   // Initialize the MCP4725 DAC (I²C is automatically configured on RP2040)
//   dac.begin(0x62);

//   // Set output pin for PWM; no further configuration needed here
//   pinMode(analogout2, OUTPUT);
// }

// void loop() {
//   // --- UART Reception ---
//   // Check if new UART data is available on Serial1.
//   // We assume the sender transmits an ASCII integer terminated by newline.
//   if (Serial1.available() >= 2) {
//     uint8_t data[2];
//     Serial1.readBytes(data, 2);
//     rawData = (data[0] << 8) | data[1];
//     newDataReceived = true;
//     lastReceiveTime = millis();
//   }


//   // --- Process New Data ---
//   if (newDataReceived) {
//     newDataReceived = false;
    
//     // For DAC output: map raw value (0–255) to DAC range (0–4095)
//     int dacValue = map(rawData, 0, 255, 0, 4095);
//     // For PWM output: scale raw value by the maximum PWM level
//     // int pwmValue = (rawData * 255) / 255;
  
//     int pwmValue = rawData; // Assuming 8-bit resolution for PWM

//     if (outmode == 0) {
//       // DAC output mode: write value to MCP4725 DAC
//       dac.setVoltage(dacValue, false);
//       Serial.printf("Mode: DAC | Raw: %d, DAC Value: %d\n", rawData, dacValue);
//     } else if (outmode == 1) {
//       // Set PWM frequency (if supported)
//       // This function may not be available on all cores; adjust if needed.
//       // analogWriteFrequency(analogout2, pwmFreq);
      
//       // PWM output mode: output via analogWrite on the specified pin
//       analogWrite(analogout2, pwmValue);
//       // Serial.printf("Mode: PWM | Raw: %d, PWM: %d, Freq: %d Hz, MaxPWM: %d\n", rawData, pwmValue, pwmFreq, maxPWMVal);
//       Serial.printf("Mode: PWM | Raw: %d, PWM: %d\n", rawData, pwmValue);

//     }
//   } 
//   else if (millis() - lastReceiveTime > timeoutInterval) {
//     // If no new UART data within the timeout, zero the output for PWM mode
//     if (outmode == 1) {
//       analogWrite(analogout2, 0);
//     }
//   }
// }

