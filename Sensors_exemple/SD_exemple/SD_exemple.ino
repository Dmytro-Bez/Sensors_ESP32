#include "SD.h"
#include "FS.h"
#include <SPI.h>

#define SD_CS 5

void write_file(fs::FS &fs, const char* path, const char* message);

void setup() {
  Serial.begin(115200);
  SD.begin(SD_CS);
  if (!SD.begin(SD_CS)) {
    Serial.println("Card Mound Failed!");
    return;
  }
  uint8_t cart_type = SD.cardType();
  if(cart_type == CARD_NONE){
    Serial.println("No SD card attached!");
    return;
  }
  Serial.println("Initializing SD card...");
  if(!SD.begin(SD_CS)){
    Serial.println("ERROR!");
    return;
  }
  File file = SD.open("Testy.txt");
  if (!file) {
    Serial.println("File doesn't exist");
    Serial.println("Creating file...");
    write_file(SD, "/T.txt", "HELLO! \t\n");
  } else {
    Serial.println("File already exists");
  }
  file.close();
}

void loop() {
  
}

void write_file(fs::FS &fs, const char* path, const char* message){
  Serial.printf("Writing file: %s\n", path);
  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("File to open file for writing");
    return;
  } 
  if (file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}
