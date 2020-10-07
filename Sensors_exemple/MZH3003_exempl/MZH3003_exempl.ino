#include <PMserial.h>

unsigned int pm2_5 = 0;
unsigned long timing;
bool allow_stream = false;
bool stat = false;
int send_interval = 10000;

SerialPM pms(PMSx003, 17, 16);            // PMSx003, RX, TX

void setup() {
  Serial.begin(115200);
  pms.init();
}

void loop() {
  if(millis() - timing > send_interval){ 
    timing = millis(); 
    send_pms();
  }
}

bool send_pms(){
  stat = false;
  
  pms.read();
  if(allow_stream){
    Serial.print(F("PM10-"));
    Serial.print(pms.pm10);
    Serial.println(F(".")); 
    Serial.print(F("PM2.5-"));
    Serial.print(pms.pm25);
    Serial.println(F("."));
    stat = true;
  } 
  return stat;  
}
