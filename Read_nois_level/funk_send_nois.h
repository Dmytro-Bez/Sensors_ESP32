#ifndef funk_send_nois_h
#define funk_send_nois_h

int buff_size = 5;
int sound_value = 0;
float sound_result = 0;
float result = 0;

float read_nois_level(){
  for(int i = 0; i < buff_size; i++){
    sound_value += analogRead(SOUND_PIN);
  }
  sound_result = sound_value / buff_size;
  result = (sound_result * 3.3) / 84.0;
  
  Serial.print("Sound result = ");
  Serial.println(sound_result);
  
  return result;
}

#endif
