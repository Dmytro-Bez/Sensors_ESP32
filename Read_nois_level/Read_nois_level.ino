#define SOUND_PIN 34

#include"funk_send_nois.h"

void setup() {
  pinMode(SOUND_PIN,INPUT);
}

void loop() {
  read_nois_level();
}
