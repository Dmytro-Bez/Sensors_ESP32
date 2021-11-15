#ifndef funk_h
#define funk_h

#include"lib.h"

void init_wire() {
  Wire.begin();
  if(!accel.begin()){
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }
  accel.setRange(ADXL345_RANGE_2_G);
  Serial.println("Succesfully connect to sensor ADXL345!");
}

int read_accel_x_read(){
  sensors_event_t event;
  accel.getEvent(&event);

  int acc_x = event.acceleration.x;

  return acc_x;
}

int read_accel_y_read(){
  sensors_event_t event;
  accel.getEvent(&event);

  int acc_y = event.acceleration.y;

  return acc_y;
}

int read_accel_z_read(){
  sensors_event_t event;
  accel.getEvent(&event);

  int acc_z = event.acceleration.z;

  return acc_z;
}

#endif
