#include <Arduino.h>
#include <Wire.h>
#include "pwm.h"
#include <SPI.h>
// #include "../lib/Libraries/Display/Display.h"
#include "lib/Display/Display.h"

#define contactorPin 2
#define analogOut A0
#define analogout2 D6
#define I2C_ADDR 0x08

PwmOut pwm(analogout2);
Display display;

volatile bool newDataReceived = false;
volatile int latestData = 0;
unsigned long lastReceiveTime = 0;
const unsigned long timeoutInterval = 220;
volatile float latestDataf = 0.0f;

char strtVal[3] = {0, 0, 0};
// char values[];
char values[3];


struct WeldSettings {
    volatile int PotValu = 0;
    volatile int preset = 0;
    volatile int DCp = 0;
};

WeldSettings weldSettings;


float latestData2(int resolutionToDutyCycle, int preset) {
    if (preset == 0)
        return resolutionToDutyCycle / 2.55;
    else if (preset == 1)
        return constrain(resolutionToDutyCycle / 2.55, 40.0f, 40.0f);
    else
        return resolutionToDutyCycle / 2.55;
}

void receiveEvent(int PotValu) {
    // display.Flush();
    if (Wire1.available()) {
        latestData = Wire1.read();
        newDataReceived = true;
        lastReceiveTime = millis();
        latestDataf = latestData2(latestData, 0);
        int DCp = latestData / 2.55;
        weldSettings.PotValu = latestData;
        weldSettings.preset = 0;
        weldSettings.DCp = latestData / 2.55;
        // char values[3] = {weldSettings.PotValu, weldSettings.DCp, 0};
    }



}

void setup() {
    Serial.begin(115200);
    display.initTFT();
    // display.drawCharacters(strtVal);
    display.StrtValues();

    Wire1.begin((uint8_t)I2C_ADDR);
    pinMode(contactorPin, OUTPUT);
    pinMode(analogout2, OUTPUT);

    pwm.begin(200.0f, 0.0f);
    analogReadResolution(12);

    Wire1.onReceive(receiveEvent);
}

void loop() {
    if (newDataReceived) {
        newDataReceived = false;
        analogWrite(analogOut, latestData);
        pwm.pulse_perc(latestDataf);
        display.drawCharacters(weldSettings.PotValu, weldSettings.DCp, 0);
        // display.Flush();
        // Serial.print("Aout: ");
        // Serial.println(analogRead(A0));
        // tft.drawCharacters()
    }
    else if (millis() - lastReceiveTime > timeoutInterval) {
        analogWrite(analogOut, 0);
        pwm.pulse_perc(0.0f);
        digitalWrite(contactorPin, HIGH);
        display.Flush();
        // display.drawCharacters(0, 0, 0);
        // display.StrtValues();
        // display* -> display.getTFT();
        // tft.flush();
    }
    // display.Flush();
    // display

}

//--------------------------------- 1st Code ---------------------------------//
// #include <Arduino.h>
// #include <Wire.h>
// #include "pwm.h"
// #include <SPI.h>
// #include "../lib/Libraries/Display/Display.h"



// #define contactorPin 2

// #define analogOut A0 // Use with Arduino UNOR4
// #define analogout2 D6



// #define I2C_ADDR 0x08 //I2C addr of the slave device
// PwmOut pwm(analogout2);
// // Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
// // Display display;

// int incomingData;



// volatile bool newDataReceived = false;
// volatile int latestData = 0;
// volatile int mapped = 0;
// unsigned long lastReceiveTime = 0;
// const unsigned long timeoutInterval = 220; // timeout in milliseconds
// volatile float latestDataf = 0.0f;



// float latestData2(int resolutionToDutyCycle, int preset){
//   if (preset == 0){
//     return (float)resolutionToDutyCycle/2.55;
//   }
//   else if(preset == 1){
//     return constrain((float)resolutionToDutyCycle/2.55,40.0f,40.0f);
//   }
//   else{
//     return (float)resolutionToDutyCycle/2.55;
//   }
// }

// void receiveEvent(int PotValu) {
//     // Keep the ISR as short as possible:
//     if (Wire1.available()) {
//         latestData = Wire1.read();
//         newDataReceived = true;
//         lastReceiveTime = millis(); // Update last receive time
//         latestDataf = latestData2(latestData,0);


//     }

// }


// void setup() {

//   Serial.begin(115200);
// //   display.initTFT();
// //   display.drawCharacters();
//   Wire1.begin((uint8_t)I2C_ADDR);
//   pinMode(contactorPin, OUTPUT);
//   pinMode(analogout2, OUTPUT);

//   pwm.begin((float)200, 0.0f);
//   analogReadResolution(12);
// //   tft.initR(INITR_REDTAB);
// //   tft.fillScreen(ST7735_BLACK);
//   Wire1.onReceive(receiveEvent);

// }



// void loop() {

//     // If new I2C data was received, process it:
//     if (newDataReceived) {
//       newDataReceived = false;  // Clear the flag for this event
//       analogWrite(analogOut, latestData);

//       pwm.pulse_perc(latestDataf);


//       Serial.print("Aout: ");
//       Serial.println(analogRead(A0));

//     }
//     // If no I2C event received within the timeout, reset value to 0.
//     else if (millis() - lastReceiveTime > timeoutInterval) {
//         analogWrite(analogOut, 0);

//         pwm.pulse_perc(0.0f);
//         //use internal resistor to keep the analog output at 0 when no I2C data is received
//         digitalWrite(contactorPin, HIGH);

//     }

//   }

