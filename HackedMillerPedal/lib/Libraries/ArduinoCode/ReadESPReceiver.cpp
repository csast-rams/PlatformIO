#include <Arduino.h>
#include <Wire.h>
// #include <SPI.h>

#define contactorPin 5
#define analogOut 2
// #define SS_PIN 10 //slave select pin for SPI
int address = 0;
int value = 0;

#define I2C_ADDR 0x08 //I2C addr of the slave device

//SPI pins:
  //MISO = CIPO = D12
  //MOSI = COPI = D11
  //SCK = CLK = D13
  //SS = CS = D10

// int pedalValue;
int incomingData;


volatile bool newDataReceived = false;
volatile int latestData = 0;
volatile int mapped = 0;
unsigned long lastReceiveTime = 0;
const unsigned long timeoutInterval = 200; // timeout in milliseconds



// void DigitalPotWrite(int addr, int val){
//     digitalWrite(SS_PIN, LOW);
//     SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0)); //initialises SPI clock speed, mode and bit order
//     digitalWrite(SS_PIN, LOW);
//     SPI.transfer(addr);
//     SPI.transfer(val);
//     digitalWrite(SS_PIN, HIGH);
//     // delay(1);
//   }

void receiveEvent(int PotValu) {
    // Keep the ISR as short as possible:
    if (Wire.available()) {
        latestData = Wire.read();
        newDataReceived = true;
        lastReceiveTime = millis(); // Update last receive time
        // mapped = map(latestData, 0, 255, 0, 255);

    }

}


void setup() {
  Wire.begin((uint8_t)I2C_ADDR);
//   pinMode(SS_PIN, OUTPUT);
  analogReference(EXTERNAL);
  pinMode(contactorPin, OUTPUT);
  pinMode(analogOut, OUTPUT);
  Serial.begin(115200);
//   SPI.begin();
//   DigitalPotWrite(address, 0);
  Wire.onReceive(receiveEvent);
}


void loop() {

    // If new I2C data was received, process it:
    if (newDataReceived) {
      newDataReceived = false;  // Clear the flag for this event
      digitalWrite(contactorPin, LOW);
      analogWrite(analogOut, latestData);
      // Use the captured data for the SPI transaction
    //   DigitalPotWrite(address, latestData);
      Serial.print("Received: ");
      Serial.println(latestData);
      Serial.print("");
      // ADD CODE FOR DAC:
    //   digitalWrite(contactorPin, LOW);

    }
    // If no I2C event received within the timeout, reset SPI value to 0.
    else if (millis() - lastReceiveTime > timeoutInterval) {
        digitalWrite(contactorPin, HIGH);
        analogWrite(analogOut, 0);
        // DigitalPotWrite(address, 0);
        // ADD CODE FOR DAC:
    }
  }

