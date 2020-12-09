#include <Wire.h>
#include "ClosedCube_HDC1080.h"

ClosedCube_HDC1080 hdc1080;


void setup() {
    Serial.begin(9600);
    hdc1080.begin(0x40);
}

void loop() {
  read_data();
  delay(5000);
}
float read_data(){
  //=============================================HDC1080 test==============================//
  Serial.println("HDC1080 test: ");
  Serial.print("T:");
  float Temper = hdc1080.readTemperature();
  Serial.print(Temper);
  Serial.println("C");
  
  Serial.print("Rh:");
  float Humid = hdc1080.readHumidity();
  Serial.print(Humid);
  Serial.println("%");
  Serial.println("===============");
  return Temper;
  return Humid;
}
