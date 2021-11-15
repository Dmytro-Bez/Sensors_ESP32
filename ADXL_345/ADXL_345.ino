#include"lib/funk.h"

void setup() {
  Serial.begin(115200);
  init_wire();
}

void loop() {
  read_accel_x_read();
  read_accel_y_read();
  read_accel_z_read();
}
