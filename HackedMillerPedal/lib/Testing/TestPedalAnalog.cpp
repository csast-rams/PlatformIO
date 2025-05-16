#include <Arduino.h>
// #include <esp_now.h>
// #include <WiFi.h>

#define ADC_SEL A0
#define OptoSig D10 //Opto-isolated contactor signal; initiates data transmission
int invertedOpto = 0;
void setup(){
  Serial.begin(115200);
  pinMode(OptoSig, INPUT_PULLUP);
  //set analog read attenuation to 11dB
//   analogSetAttenuation(ADC_11db);
}

void loop(){
  invertedOpto = !digitalRead(OptoSig);
  if (invertedOpto)
  {
    int potValue = analogRead(ADC_SEL);
    Serial.println(potValue);
    delay(100);
  }
  else{
  }
}
