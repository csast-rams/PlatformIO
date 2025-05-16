#include <Arduino.h>
#include <Wire.h>
#include "pwm.h"
#include <SPI.h>


#define contactorPin 2
#define analogOut A0
#define analogout2 D6
#define I2C_ADDR 0x08
#define freqInterruptPin 3

PwmOut pwm(analogout2);
Display display;

volatile bool newDataReceived = false;
volatile int latestData = 0;
volatile float latestDataf = 0.0f;
unsigned long lastReceiveTime = 0;
const unsigned long timeoutInterval = 300;

volatile bool interruptTriggered = false;

struct WeldSettings {
    float PotValu = 0.0f;
    float DCp = 0.0f;
    int preset = 0;
} weldSettings;

float readFrequency(int analogPin) {
    const float minFreq = 10.0f;
    const float maxFreq = 500.0f;
    int rawADC = analogRead(analogPin);
    return constrain(round(map(rawADC, 0, 4095, minFreq, maxFreq) / 5.0f) * 5.0f, minFreq, maxFreq);
}

float latestData2(int resolutionToDutyCycle, int preset) {
    if (preset == 0)
        return resolutionToDutyCycle / 2.55f;
    else if (preset == 1)
        return constrain(resolutionToDutyCycle / 2.55f, 40.0f, 40.0f);
    else
        return resolutionToDutyCycle / 2.55f;
}

void receiveEvent(int PotValu) {
    if (Wire1.available()) {
        latestData = Wire1.read();
        newDataReceived = true;
        lastReceiveTime = millis();
        latestDataf = latestData2(latestData, 0);
        weldSettings.PotValu = latestData / 25.5f;
        weldSettings.DCp = latestData / 2.55f;
    }
}



void ISR_SetFrequency() {
    interruptTriggered = true;  // Only set flag, no other action in ISR!
}

void setup() {
    Serial.begin(115200);
    analogReadResolution(12);
    display.initTFT();
    display.StrtValues();

    Wire1.begin((uint8_t)I2C_ADDR);
    pinMode(contactorPin, OUTPUT);
    pinMode(analogout2, OUTPUT);
    pinMode(freqInterruptPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(freqInterruptPin), ISR_SetFrequency, FALLING);

    float initialFreq = readFrequency(A3);
    pwm.begin(initialFreq, 0.0f);

    Wire1.onReceive(receiveEvent);
}

void loop() {
    static unsigned long lastUpdate = 0;
    static float currentFreq = readFrequency(A3);

    // Handle frequency update triggered by interrupt
    if (interruptTriggered) {
        noInterrupts();
        interruptTriggered = false;
        interrupts();

        currentFreq = readFrequency(A3);
        // pwm.period(1.0f / currentFreq);
        pwm.period_us((int)(1000000.0f / currentFreq));  // Correct and precise!
        // Serial.print("Updated PWM Frequency: ");
        // Serial.println(currentFreq);
    }

    if (newDataReceived) {
        newDataReceived = false;
        analogWrite(analogOut, latestData);
        pwm.pulse_perc(latestDataf);

        if (millis() - lastUpdate > 100) {
            display.updateProgress(weldSettings.PotValu, weldSettings.DCp, currentFreq);
            lastUpdate = millis();
        }
    } else if (millis() - lastReceiveTime > timeoutInterval) {
        analogWrite(analogOut, 0);
        pwm.pulse_perc(0.0f);
        digitalWrite(contactorPin, HIGH);

        if (millis() - lastUpdate > 100) {
            display.updateProgress(0.00f, 0.00f, currentFreq);
            lastUpdate = millis();
        }
    }
}