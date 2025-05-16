#include <Arduino.h>
#include <Adafruit_MCP4725.h>
#include <pico/multicore.h>
#include <RotaryEncoder.h>
#include <hardware/watchdog.h>

// DAC configuration
Adafruit_MCP4725 dac;

// Pin definitions
#define contactorPIN D8
#define HighVoltageAnalogPin A3
#define DAC_SCL_PIN D5
#define DAC_SDA_PIN D4
#define ENCODER_PIN_A D26
#define ENCODER_PIN_B D27
#define ENCODER_BUTTON_PIN D28  // Used as Reset Button

// Modes: 0 = DAC, 1 = PWM
volatile int outmode = 0;

// Encoder setup
RotaryEncoder encoder(ENCODER_PIN_A, ENCODER_PIN_B);
int encoderPos = 0;
int lastEncoderPos = -1;

// Frequency selections: index 0 = DAC mode, 1-9 = PWM frequencies
const float frequencies[] = {0, 1, 2, 3, 4, 5, 10, 20, 50, 100};
const int numFrequencies = sizeof(frequencies) / sizeof(frequencies[0]);

volatile float desiredFrequency = 0.0f;
volatile float desiredDutyCycle = 0.0f;
volatile float DCyclePercent = 95.0f; // Duty cycle percentage (0-100%); Try to keep it below 100%.

// UART data reception variables
volatile bool newDataReceived = false;
volatile int rawData = 0;
unsigned long lastReceiveTime = 0;
const unsigned long timeoutInterval = 0;

// DAC voltage configuration
const float presetBaseVoltage = 0.0f;
const float presetMaxVoltage = 3.3f;

// const int lowDACValue = 2000;
const int lowDACValue = 0;
const int highDACValue = 4095;


// Core1: Pulse Generator
void core1_entry() {
  const unsigned long interval_us = 1000;
  while (true) {
    if (outmode != 1 || desiredFrequency == 0) {
      delayMicroseconds(interval_us);
      continue;
    }
    float period_us = 1000000.0f / desiredFrequency;
    int samplesPerPeriod = period_us / interval_us;
    int highSamples = samplesPerPeriod * (desiredDutyCycle / 100.0f);

    for (int i = 0; i < samplesPerPeriod; i++) {
      if (outmode != 1) break;
      int value = (i < highSamples) ? highDACValue : lowDACValue;
      dac.setVoltage(value, false, 400000);
      delayMicroseconds(interval_us);
    }
  }
}

// UART callback
void serialEvent1() {
  while (Serial1.available() >= 2) {
    uint8_t data[2];
    Serial1.readBytes(data, 2);
    rawData = (data[0]) | (data[1] << 8);
    newDataReceived = true;
    lastReceiveTime = millis();
  }
}

// ISR: Encoder button reset
void encoderButtonISR() {
  watchdog_reboot(0, 0, 10); // Hardware reset RP2040 after 10ms
}

void setup() {
  analogReadResolution(12);

  // Set A3 high to supply contactor
  pinMode(HighVoltageAnalogPin, OUTPUT);
  digitalWrite(HighVoltageAnalogPin, HIGH);

  // Set up contactor pin
  pinMode(contactorPIN, OUTPUT);
  digitalWrite(contactorPIN, LOW); // Set contactor pin to LOW

  pinMode(ENCODER_BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_BUTTON_PIN), encoderButtonISR, FALLING);

  Serial.begin(115200);
  delay(1000);

  Serial1.begin(115200);

  Wire.setSCL(DAC_SCL_PIN);
  Wire.setSDA(DAC_SDA_PIN);
  Wire.setClock(400000); // Set clock speed to 400kHz
  dac.begin(0x62);

  // Ensure DAC flatline at startup
  dac.setVoltage(lowDACValue, false);

  // Start core1 for PWM generation
  multicore_launch_core1(core1_entry);
}

void loop() {
  // Encoder handling
  encoder.tick();
  encoderPos = encoder.getPosition();

  if (encoderPos != lastEncoderPos) {
    if (encoderPos < 0) encoder.setPosition(0);
    if (encoderPos >= numFrequencies) encoder.setPosition(numFrequencies - 1);
    encoderPos = encoder.getPosition();

    desiredFrequency = frequencies[encoderPos];

    // Set mode based on position
    if (encoderPos == 0) {
      outmode = 0;  // DAC flatline
      dac.setVoltage(lowDACValue, false); // Flat at lowest voltage initially
      // Serial.println("DAC Flatline Mode");
    } else {
      outmode = 1;  // PWM mode
      // Serial.printf("PWM Mode: %.1f Hz\n", desiredFrequency);
    }

    lastEncoderPos = encoderPos;
  }

  if (newDataReceived) {
    newDataReceived = false;
    rawData = constrain(rawData, 0, 255);
    int targetDACValue = map(rawData, 0, 255, lowDACValue, highDACValue);
    desiredDutyCycle = ((float)(targetDACValue - lowDACValue) / (highDACValue - lowDACValue)) * DCyclePercent;
    // digitalWrite(contactorPIN, rawData >= 2 ? HIGH : LOW); // Set contactor pin based on rawData
    Serial.printf("Contactor State: %d\n", digitalRead(contactorPIN));

    if (outmode == 0) {
      // if (rawData != 0) {
      //   digitalWrite(contactorPIN, HIGH); // Set contactor pin to HIGH
      // }
      int dacValue = map(rawData, 0, 255, lowDACValue, highDACValue);
      dac.setVoltage(dacValue, false);
    }

  }


  if (rawData == 0) {
    desiredDutyCycle = 0.0f;
    // dac.setVoltage(0, false);
  }

}


// #include <Arduino.h>
// #include <Adafruit_MCP4725.h>
// #include <RP2040_PWM.h>
// #include <pico/stdlib.h>
// #include <pico/multicore.h>

// // Global PWM instance pointer
// RP2040_PWM* pwmInstance = nullptr;

// // Pin definitions (Waveshare RP2040-Zero: A0 = D26, A1 = D27, A2 = D28, A3 = D29)
// #define PWM_OUT_PIN A3         // PWM output pin (used when outmode==1)
// #define OUTSET_PIN D8          // Pin used to select output mode via interrupt
// #define FREQ_POT_PIN A0        // (Not used for testing – preset frequency instead)
// #define BASEVOLT_POT_PIN A1    // (Not used for testing – preset voltage instead)
// #define MAXVOLT_POT_PIN A2     // (Not used for testing – preset voltage instead)

// // Output mode: 0 = DAC (flat voltage), 1 = PWM (pulsed waveform)
// volatile int outmode = 0;  // Always start in DAC mode
// int lastMode = -1;         // For mode change tracking

// // Global flag to prevent mode changes until initialization is complete
// volatile bool initialized = false;

// // UART data reception variables (rawData from UART from ESP32: 0–255)
// volatile bool newDataReceived = false;
// int rawData = 0;

// unsigned long lastReceiveTime = 0;
// const unsigned long timeoutInterval = 500;  // ms

// // Preset parameters for waveform generation (for PWM mode)
// const float presetBaseVoltage = 0.0f; // Volts (DAC low level)
// const float presetMaxVoltage  = 3.3f; // Volts (DAC high level)
// const int lowDACValue  = (int)((presetBaseVoltage / 3.3f) * 4095);
// const int highDACValue = (int)((presetMaxVoltage  / 3.3f) * 4095);

// // Global dynamic PWM parameters (updated by Core0)
// volatile float desiredFrequency = 10.15f;   // Hz (preset, can be updated later)
// volatile float desiredDutyCycle = 0.1f;       // % (computed from rawData)

// // Create DAC object (MCP4725, default I²C address 0x62)
// Adafruit_MCP4725 dac;

// // --------------------
// // Interrupt Service Routine for Mode Switching
// // --------------------
// void updateOutmode() {
//   // Only allow mode changes after initialization is complete.
//   if (!initialized) return;
//   // Read the pin: if HIGH, set PWM mode (1); if LOW, DAC mode (0)
//   if (digitalRead(OUTSET_PIN) == LOW) {
//     outmode = 0;
//   } else {
//     outmode = 1;
//   }
// }

// // --------------------
// // Core1: Pulse Generator (Software PWM via DAC)
// // --------------------
// void core1_entry() {
//   const unsigned long sampleInterval_us = 1000;  // 1 ms update interval
//   while (true) {
//     // Only generate pulse waveform when in PWM mode.
//     if (outmode != 1) {
//       delayMicroseconds(sampleInterval_us);
//       continue;
//     }
//     // Read current desired frequency and duty cycle.
//     float freq = desiredFrequency;
//     float dutyPerc = desiredDutyCycle;
//     // Compute period in microseconds.
//     float period_us = 1000000.0f / freq;
//     int samplesPerPeriod = (int)(period_us / sampleInterval_us + 0.5f);
//     if (samplesPerPeriod < 1) samplesPerPeriod = 1;
//     int highSamples = (int)((dutyPerc / 100.0f) * samplesPerPeriod + 0.5f);
//     // Generate one period of the waveform.
//     for (int i = 0; i < samplesPerPeriod; i++) {
//       // If mode has changed, break out.
//       if (outmode != 1) break;
//       int value = (i < highSamples) ? highDACValue : lowDACValue;
//       dac.setVoltage(value, false);
//       delayMicroseconds(sampleInterval_us);
//     }
//   }
// }

// // --------------------
// // Nonblocking UART read callback
// // --------------------
// void serialEvent1() {
//   while (Serial1.available() >= 2) {
//     uint8_t data[2];
//     Serial1.readBytes(data, 2);
//     // Little-endian: low byte first.
//     rawData = data[0] | (data[1] << 8);
//     newDataReceived = true;
//     lastReceiveTime = millis();
//   }
// }

// // --------------------
// // Setup
// // --------------------
// void setup() {
//   outmode = 0;
//   // Set analog resolution to 12 bits (0–4095)
//   analogReadResolution(12);

//   // Configure mode select pin with pullup and attach interrupt.
//   pinMode(OUTSET_PIN, INPUT_PULLUP);
//   attachInterrupt(digitalPinToInterrupt(OUTSET_PIN), updateOutmode, CHANGE);

//   Serial.begin(115200);
//   delay(1000);  // Allow stabilization

//   Serial1.begin(115200);

//   // Initialize I2C for DAC (using default RP2040 I2C pins)
//   Wire.setSCL(D5);
//   Wire.setSDA(D4);
//   dac.begin(0x62);

//   // Set PWM output pin as output
//   pinMode(PWM_OUT_PIN, OUTPUT);

//   // Always start in DAC mode.
//   // Serial.println("Starting in DAC mode");
  
//   // Launch Core1 for pulse generation (it will be inactive until outmode==1)
//   multicore_launch_core1(core1_entry);

//   // Mark initialization complete (allowing mode changes)
//   initialized = true;
// }

// // --------------------
// // Main Loop (Core0)
// // --------------------
// void loop() {
//   // Process new UART data if available.
//   if (newDataReceived) {
//     newDataReceived = false;
//     Serial.printf("Received rawData = %d\n", rawData);
//     rawData = constrain(rawData, 0, 255);
//     // Map rawData to a target DAC value between lowDACValue and highDACValue.
//     int targetDACValue = map(rawData, 0, 255, lowDACValue, highDACValue);
//     // Compute duty cycle as percentage.
//     desiredDutyCycle = ((float)(targetDACValue - lowDACValue) / (highDACValue - lowDACValue)) * 100.0f;
//     // For testing, update frequency as well (here you can add your own mapping if needed)
//     desiredFrequency = 10.5f; // For example, fixed at 10.15 Hz
//   }

//   // In DAC mode, update DAC output continuously using rawData mapping.
//   if (outmode == 0) {
//     int flatDACValue = map(rawData, 0, 255, 0, 4095);
//     flatDACValue = constrain(flatDACValue, 0, 4095);
//     dac.setVoltage(flatDACValue, false);
//   }
  
//   // If no new UART data for a while in PWM mode, force duty cycle to zero.
//   if ((millis() - lastReceiveTime > timeoutInterval) && (outmode == 1)) {
//     desiredDutyCycle = 0.0f;
//   }
  
//   // delay(50); // Short delay to prevent flooding serial output
// }
// // #include <Arduino.h>
// // #include <Adafruit_MCP4725.h>
// // #include <RP2040_PWM.h>

// // // Global PWM instance pointer
// // RP2040_PWM* pwmInstance = nullptr;

// // // Pin definitions (Waveshare RP2040-Zero: A0 = D26, A1 = D27, A2 = D28, A3 = D29)
// // #define analogout2 A3         // PWM output pin (used when outmode==1)
// // #define OutsetPin D8          // Digital pin to select output mode via interrupt
// // #define FREQ_POT_PIN A0       // Potentiometer for PWM frequency control
// // #define BASEVOLT_POT_PIN A1   // Potentiometer for base voltage level
// // #define MAXVOLT_POT_PIN A2    // Potentiometer for maximum voltage level

// // // Output mode: 0 = DAC output, 1 = PWM output  
// // volatile int outmode = 0;
// // int lastMode = -1;  // To track mode changes

// // // UART data reception variables (rawData from UART from ESP32: 0–255)
// // volatile bool newDataReceived = false;
// // int rawData;

// // unsigned long lastReceiveTime = 0;
// // const unsigned long timeoutInterval = 400;  // ms

// // int freqPot;
// // int basePot;
// // int maxPot;
// // float pwmFreq;
// // float baseVoltage;
// // float maxVoltage;


// // // Create DAC object (MCP4725, default I²C address 0x62)
// // Adafruit_MCP4725 dac;

// // // --- Interrupt Service Routine ---
// // // Toggle outmode based on OutsetPin state: LOW => DAC, HIGH => PWM
// // void updateOutmode() {
// //   if (digitalRead(OutsetPin) == LOW) {
// //     outmode = 0;
// //   } else {
// //     outmode = 1;
// //   }
// // }

// // // --- Nonblocking UART read callback ---
// // // Automatically called when Serial1 has data.
// // void serialEvent1() {
// //   while (Serial1.available() >= 2) {
// //     uint8_t data[2];
// //     Serial1.readBytes(data, 2);
// //     // Little-endian: low byte first
// //     rawData = data[0] | (data[1] << 8);
// //     newDataReceived = true;
// //     lastReceiveTime = millis();
// //   }

// //   freqPot = 10;
// //   // Map freq potentiometer from 0-4095 to desired frequency range, e.g., 1-100 Hz
// //   basePot = 0;//analogRead(BASEVOLT_POT_PIN);
// //   // Map base voltage potentiometer to 0–3300 mV, then convert to volts
// //   maxPot = map(rawData, 0, 4095, 0, 4095);
// //   // Map maximum voltage potentiometer to 0–3300 mV, then to volts
// //   maxPot = constrain(maxPot, 0, 4095);
// //   freqPot = constrain(freqPot, 0, 100);
// //   basePot = constrain(basePot, 0, 4095);
// //   pwmFreq = map(freqPot, 0, 100, 0, 1000);
// //   // pwmFreq = pwmFreq / 1000.0f; // Convert to Hz

// //   // Map basePot and maxPot to corresponding mV
// //   baseVoltage = map(basePot, 0, 4095, 0, 3300);
// //   maxVoltage = map(maxPot, 0, 4095, 0, 3300);

// //   if (baseVoltage > maxVoltage) {
// //     baseVoltage = maxVoltage;
// //   }

// // }

// // void setup() {
// //   // Set analog resolution to 12 bits (0–4095)
// //   analogReadResolution(12);

// //   // Configure OutsetPin with pullup and attach interrupt
// //   pinMode(OutsetPin, INPUT_PULLUP);
// //   attachInterrupt(digitalPinToInterrupt(OutsetPin), updateOutmode, CHANGE);

// //   Serial.begin(115200);
// //   delay(1000);  // Stabilization

// //   // Initialize Serial1 for UART reception at 115200 baud
// //   Serial1.begin(115200);

// //   // Initialize I2C for the DAC (default RP2040 I2C pins)
// //   Wire.setSCL(D5);
// //   Wire.setSDA(D4);
// //   dac.begin(0x62);

// //   // Set PWM output pin as output
// //   pinMode(analogout2, OUTPUT);

// //   // Create the PWM instance on analogout2, channel 0, with initial frequency 100 Hz and 0% duty
// //   float initFreq = 10.0f;
// //   float initDuty = 0.0f;
// //   pwmInstance = new RP2040_PWM(analogout2, initFreq, initDuty);
// //   pwmInstance->setPWM();  // Initialize PWM output

// //   analogWrite(FREQ_POT_PIN, 4095); // Set to full
// //   analogWrite(BASEVOLT_POT_PIN, 4095); // Set to full
// //   analogWrite(MAXVOLT_POT_PIN, 4095); // Set to full
// // }

// // void loop() {
// //   // Detect mode change and disable PWM if switching out of PWM mode.
// //   if (lastMode != outmode) {
// //     if (outmode == 0) {
// //       // Switched to DAC mode, so disable PWM output by setting pin LOW.
// //       digitalWrite(analogout2, LOW);
// //     }
// //     lastMode = outmode;

// //   }



// //   // --- Process New Data ---
// //   if (newDataReceived) {
// //     newDataReceived = false;
// //     Serial.printf("Raw Data (UART): %d\n", rawData);
// //       // --- Read PWM Parameter Potentiometers ---


  
// //     // Convert voltages to duty cycle percentages (assuming 3.3 V supply)
// //     float baseDuty = (baseVoltage);
// //     float maxDuty = (maxVoltage) / 3300.00f;// Convert to percentage
// //     // maxDuty = maxDuty / 2.05f;

// //     // Compute effective duty cycle: rawData (0-255) maps from baseDuty to maxDuty.
// //     // float dutyCycle = baseDuty + ((maxDuty - baseDuty) * (rawData / 65280.0f));
// //     // float dutyCycle = baseDuty + ((maxDuty - baseDuty) * (rawData / 65280.0f));


// //     // For DAC mode: map rawData to DAC range (0–4095)
// //     int dacValue = map(rawData, 0, 255, 0, 4095);
// //     float dutyCycle = baseDuty + ((maxDuty - baseDuty) * (rawData)) / 2.55f;
// //     dutyCycle = dutyCycle * 16.1f; // Scale to 0-100%
// //     if (outmode == 0) {
// //       dac.setVoltage(dacValue, false);
// //       Serial.printf("Mode: DAC | rawData=%d, dacValue=%d\n", rawData, dacValue);
// //     }
// //     else if (outmode == 1) {
// //       // Update PWM with new frequency and duty cycle

// //       pwmInstance->setPWM(analogout2, pwmFreq, dutyCycle);
// //       Serial.printf("Mode: PWM | rawData=%d, Duty=%.1f%%, Freq=%.1f Hz, Base=%.2f V, Max=%.2f V\n",
// //                     rawData, dutyCycle, pwmFreq, baseDuty, maxDuty);
// //     }
// //   }
// //   // else if (millis() - lastReceiveTime > timeoutInterval) {
// //   //   // If no UART data, ensure PWM is off (only if in PWM mode)
// //   //   if (outmode == 1) {
// //   //     pwmInstance->setPWM(analogout2, 0, 0);
// //   //     analogWrite(analogout2, 0);  // Alternatively, set PWM output to 0
// //   //   }
// //   // }
// //   // delay(50); // short delay to prevent flooding
// // }