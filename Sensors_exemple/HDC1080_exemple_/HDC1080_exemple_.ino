#include <Wire.h>
#include "ClosedCube_HDC1080.h"

ClosedCube_HDC1080 hdc1080;


void setup() {
    Serial.begin(9600);
    hdc1080.begin(0x40);
}

void loop() {

//=============================================HDC1080 test==============================//
  Serial.println("HDC1080 test: ");
  Serial.print("T:");
  Serial.print(hdc1080.readTemperature());
  Serial.println("C");
  Serial.print("Rh:");
  Serial.print(hdc1080.readHumidity());
  Serial.println("%");
  Serial.println("===============");
  
  delay(5000);
}
