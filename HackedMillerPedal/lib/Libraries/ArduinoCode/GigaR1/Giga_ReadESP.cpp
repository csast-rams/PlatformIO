// // #include <mbed.h>
// // #include <Arduino.h>
// // #include <Wire.h>

// // #define contactorPin 5
// // #define analogOut A12
// // #define analogout2 D4

// // // I2C Addresses
// // const int I2C0_ADDR = 0x08; //I2C0 SLAVE addr for receiving ESP32 receiver data

// // //Pins for PWM/ADC/DAC
// // PinName pinPwm1 = digitalPinToPinName(D4);
// // // PinName pinSda0 = digitalPinToPinName(D20);
// // // PinName pinScl0 = digitalPinToPinName(D21);
// // mbed::PwmOut pwm(pinPwm1);

// // //Inputs
// // volatile bool newDataReceived = false;
// // volatile int latestData = 0;
// // unsigned long lastReceiveTime = 0;
// // const unsigned long timeoutInterval = 220; // timeout in milliseconds
// // volatile float latestDataf = 0.0f;

// // //Waveform Parameters
// // // volatile int freq = 100;  // in hertz, change accordingly
// // float latestData2(int resolutionToDutyCycle, int preset){
// //   if (preset == 0){
// //     return (float)resolutionToDutyCycle/2.55;
// //   }
// //   else if(preset == 1){
// //     return constrain((float)resolutionToDutyCycle/2.55,40.0f,40.0f);
// //   }
// //   else{
// //     return (float)resolutionToDutyCycle/2.55;
// //   }
// // }

// // void receiveEvent(int PotValu) {
// //     // Keep the ISR as short as possible:
// //     if (Wire.available()) {
// //         latestData = Wire.read();
// //         newDataReceived = true;
// //         lastReceiveTime = millis(); // Update last receive time
// //         latestDataf = latestData2(latestData,0);
// //     }

// // }

// // // the setup function runs once when you press reset or power the board
// // void setup() {
// //   // initialize digital pin LED_BUILTIN as an output.
// //   pwm.period_ms(1); //1kHz
// //   pwm.pulsewidth(0); //0% duty cycle
// // //   pwm->pulsewidth_us(500);
// //   Wire.begin((uint8_t)I2C0_ADDR); //Begin I2C Slave
// //   pinMode(contactorPin, OUTPUT);
// // //   pinMode(analogOut, OUTPUT);
// //   pinMode(analogout2, OUTPUT);
// //   analogReadResolution(12);
// //   Serial.begin(115200);
// //   Wire.onReceive(receiveEvent);
// // }

// // // the loop function runs over and over again forever
// // // void loop() {
// // //   //delay(1);
// // // }



// // /*
// // ----------------------------------------------------------------------------------------------------------------
// // The code above is a simple example of how to use the mbed library in an Arduino sketch.
// // */




// // void loop() {
// //     // If new I2C data was received, process it:
// //     if (newDataReceived) {
// //       newDataReceived = false;  // Clear the flag for this even
// //       digitalWrite(contactorPin, LOW);
// //       analogWrite(analogOut, latestData);
// //       // analogWrite(analogout2, latestData);
// //       pwm.pulsewidth(latestDataf);
// //       // SetPinFrequencySafe(analogout2, latestData);

// //       // wave.begin(freq);
// //       Serial.print("Aout: ");
// //     //   Serial.println(analogRead(A0));
// //       Serial.println(latestData);
// //     }
// //     // If no I2C event received within the timeout, reset value to 0.
// //     else if (millis() - lastReceiveTime > timeoutInterval) {
// //         analogWrite(analogOut, 0);
// //         //use internal resistor to keep the analog output at 0 when no I2C data is received
// //         digitalWrite(contactorPin, HIGH);
// //         pwm.pulsewidth(0.0f);
// //     }
// //     // delay(10);
// //   }

// // #include "AnalogOut.h"
// // #include <ArduinoCode
// #include <Arduino.h>
// #include <Wire.h>

// // #include
// #define contactorPin 5
// #define analogOut A12
// #define analogout2 D4

// // I2C Address
// const int I2C_ADDR = 0x08; // Ensure consistency

// // Inputs
// volatile bool newDataReceived = false;
// volatile int latestData = 0;
// unsigned long lastReceiveTime = 0;
// const unsigned long timeoutInterval = 220; // Timeout in milliseconds
// volatile float latestDataf = 0.0f;

// // Instantiate mbed PWM
// // arduino::mbed::PwmOut* pwm = new mbed::PwmOut pwm();
// PwmOut pwm(D4);

// // Function to process latest data
// float latestData2(int resolutionToDutyCycle, int preset) {
//     if (preset == 0) {
//         return (float)resolutionToDutyCycle / 2.55;
//     } else if (preset == 1) {
//         return constrain((float)resolutionToDutyCycle / 2.55, 40.0f, 40.0f);
//     } else {
//         return (float)resolutionToDutyCycle / 2.55;
//     }
// }

// // I2C Receive Event
// void receiveEvent(int PotValu) {
//     if (Wire.available()) {
//         latestData = Wire.read();
//         newDataReceived = true;
//         lastReceiveTime = millis();
//         latestDataf = latestData2(latestData, 0);
//     }
// }

// // Setup function
// void setup() {
//     Wire.begin((uint8_t)I2C_ADDR); // Begin I2C Slave
//     pinMode(contactorPin, OUTPUT);
//     analogReadResolution(12);
//     // Serial.begin(115200);
//     // Initialize mbed PWM
//     // pwm.begin(latestDataf,0.0f);  // Initialize
//     pwm.
//     pwm.period_us()  // 1 kHz PWM (matches previous behavior)
//     pwm.pulse_perc(0.0f);   // Start with 0% duty cycle
//     Wire.onReceive(receiveEvent);
// }

// // Main loop function
// void loop() {
//     if (newDataReceived) {
//         newDataReceived = false;  // Clear the flag
//         digitalWrite(contactorPin, LOW);
//         // mbed::analogWrite(analogOut, latestData);
//         analogWrite(DAC, latestData);
//         // Convert latestDataf (0-100%) to mbed PWM scale (0.0-1.0)
//         pwm.pulse_perc(latestDataf);

//         // Serial.print("Aout: ");
//         // Serial.println(analogRead(A0));
//     }
//     // Timeout handling
//     else if (millis() - lastReceiveTime > timeoutInterval) {
//         // analogWrite(analogOut, 0);
//         analogWrite(DAC, 0);
//         pwm.pulse_perc(0.0f);  // Set PWM duty cycle to 0
//         digitalWrite(contactorPin, HIGH);
//     }
// }
