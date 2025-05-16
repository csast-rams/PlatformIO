#include <Arduino.h>

const int HallPin = A0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(HallPin, INPUT);
  analogReference(EXTERNAL);
}

void loop() {
  // put your main code here, to run repeatedly:
  int sensorValue = analogRead(HallPin);
  Serial.println(sensorValue);
  delay(500);
}
