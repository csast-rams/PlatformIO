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
volatile float DCyclePercent = 80.0f; // Duty cycle percentage (0-100%); Try to keep it below 100%.

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
  // multicore_launch_core1(core1_entry);
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
      dac.setVoltage(lowDACValue, false); // Flat at lowest voltage initially              ..................... Is this necessary?
      // Serial.println("DAC Flatline Mode");
    } else {
      outmode = 0;  // DAC flatline mode; COMMENT OUT and uncomment below to enable PWM mode
      //Uncomment to enable PWM mode ...
      // outmode = 1;  // PWM mode
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
