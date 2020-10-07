#include <Wire.h>

#define D6T_ADDR 0xA                                                                   //addres work registor
#define D6T_CMD 0x4C

int write_status; 
int r_buf[34];
float t_ptat;
int t_data[16]; 
const int nonvalid = -100;
int t_previouse_data[16] = {nonvalid,nonvalid,nonvalid,nonvalid,nonvalid,nonvalid,nonvalid,nonvalid,nonvalid,nonvalid,nonvalid,nonvalid,nonvalid,nonvalid,nonvalid,nonvalid};
bool was_movement= false;
int p_centr;
int p_eng;
int save_temp;
int p_average;
int max_pix_index;

void setup() {
  Serial.begin(115200);

}

void loop() {
  Wire.begin();
  Wire.beginTransmission(D6T_ADDR);
  Wire.write(D6T_CMD);
  Wire.endTransmission();
  write_status = Wire.endTransmission(false);
  int read_status = Wire.requestFrom(D6T_ADDR, 35);
  if (read_status == 35) {
    for(int b = 0; b < 35; b++) r_buf[b] = Wire.read();
      t_ptat = (r_buf[0] + (r_buf[1] << 8)) * 0.1;
      for(int i = 0; i < 16; i++) {
        t_previouse_data[i] = t_data[i];
        t_data[i] = (r_buf[(i*2 + 2)] + (r_buf[(i*2 + 3)] << 8));
        if (abs(t_data[i] - t_previouse_data[i]) > 3 && t_previouse_data[i] != nonvalid) {
          Serial.println("FALSE");
          Serial.println(t_data[i]);
          Serial.println(t_previouse_data[i]);
          was_movement = true;
        } else {
          Serial.println("TRUE");
        }
      }
  }
//  /
    delay(1000);
}
