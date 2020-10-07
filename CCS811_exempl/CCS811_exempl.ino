#include "Adafruit_CCS811.h"

bool stat = false;

Adafruit_CCS811 ccs;
 
void setup() {
  Serial.begin(115200);
  Serial.println("CCS811 test");
 
  if(!ccs.begin()){
    Serial.println("Failed to start sensor! Please check your wiring.");
    while(1);
  }
  //calibrate temperature sensor
  while(!ccs.available());
  float temp = ccs.calculateTemperature();
  ccs.setTempOffset(temp - 25.0);
}
 
void loop() {
  if(ccs.available()){
    send_ccs811();
  }
  delay(500);
}

bool send_ccs811(){
  stat = false;
  float temp = ccs.calculateTemperature();
  if(!ccs.readData()){
    Serial.print("CO2: ");
    Serial.print(ccs.geteCO2());
    Serial.print("ppm, TVOC: ");
    Serial.print(ccs.getTVOC());
    Serial.print("ppb Temp:");
    Serial.println(temp);
    stat = true;
  }
  return stat;
}
