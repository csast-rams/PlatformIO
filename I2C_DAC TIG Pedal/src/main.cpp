#include <Arduino.h>
#include <Wire.h>
#include "Aptinex_MCP4725/Aptinex_MCP4725.h"

Aptinex_MCP4725 dac;
void setup(void) {
  Serial.begin(115200);
  Serial.println("***APTINEX MCP4725A DAC MODULE TEST CODE***");
  // For Aptinex MCP4725A1 the address is 0x62 (default) or 0x63 (ADDR pin tied to VCC)
  // For MCP4725A0 the address is 0x60 or 0x61
  // For MCP4725A2 the address is 0x64 or 0x65
  dac.begin(0x62);
  Serial.println("First, Enter H and adjust pot for the maximum value (10v) "); // measure DAC output voltage with multimeter
  Serial.println("Enter H for maximum value"); // DAC output will be set to the maximum value
  Serial.println("Enter M for medium value"); // DAC output will be set to the medium value
  Serial.println("Enter L for lowest value"); // DAC output will be set to the lowest value
}
void loop(void) {
  uint32_t counter;
  char incomingByte;
  if (Serial.available() > 0) {
    // read the oldest byte in the serial buffer:
    incomingByte = Serial.read();
    if (incomingByte == 'H') {
      counter = 0xFFF;
      dac.setVoltage(counter, false);
      Serial.println(counter);
      delay(100);
    }
    else if (incomingByte == 'M') {
      counter = 0x800;
      dac.setVoltage(counter, false);
      Serial.println(counter);
      delay(100);
    }
    else if (incomingByte == 'L') {
      counter = 0x000;
      dac.setVoltage(counter, false);
      Serial.println(counter);
      delay(100);
    }
  }
}
