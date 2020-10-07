/*======================LIB====================*/
#include "key.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <MQTTClient.h>
#include <math.h>
#include <string>
#include <ArduinoJson.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "Adafruit_CCS811.h"
#include "ClosedCube_HDC1080.h"
#include "mhz19.h"
#include "SoftwareSerial.h"
#include <EEPROM.h>
#include<jled.h>
/*======================Define====================*/
int EEPROM_SIZE = 1000;       //Size memory
int EEPROM_ZERO_ADD = 0;      // address of memory

#define PIN_RX  16
#define PIN_TX  17
#define ADDRES_SENSOR 0x53                                                             //Address
#define READ_PIN 6
#define D6T_ADDR 0xA
#define D6T_CMD 0x4C
#define DEVICE_NAME "DEV01"                                                            //The name of the device. This MUST match up with the name defined in the AWS console
#define AWS_IOT_ENDPOINT "a6oi0jm53ol8u.iot.ap-southeast-2.amazonaws.com"              //The MQTTT endpoint for the device (unique for each AWS account but shared amongst devices within the account)
#define AWS_IOT_TOPIC "/"+ DEVICE_NAME+ "/comand"                                      //The MQTT topic that this device should publish to
#define AWS_MAX_RECONNECT_TRIES 50  
#define HOME_SERVICE_UUID       "19B10000-E8F2-537E-4F6C-D104768A1214"
#define SSID_CHARACTERISTIC_UUID "ec5397a2-0afe-4bd9-9403-0cd230976fa8"
#define PASSWORD_CHARACTERISTIC_UUID "1fc79774-fb73-453e-af23-61439d87a389"
#define INTERVAL_CHARACTERISTIC_UUID "4b411f8a-fcb8-11ea-adc1-0242ac120002" 
#define SWITCH_CHARACTERISTIC_UUID  "2dc43574-fb73-453e-af23-61439d87a389"
#define SOUND_PIN 34                                                                   //sound pin
#define BUTTON_PIN 4                                                                   //button pin
#define INFO_LED_PIN 2                                                                 //led pin
#define HDC 0x1050

//============================================Error led config====================================//
#define ERROR_CODE_SIZE 4
#define ERROR_CODE_LONG 0
#define ERROR_CODE_SHORT 1

JLed WIFI_ERROR_LED = JLed(INFO_LED_PIN).Blink(500, 500).Repeat(4);
JLed BLE_ERROR_LED1 = JLed(INFO_LED_PIN).Blink(50, 50).Repeat(1);
JLed BLE_ERROR_LED2 = JLed(INFO_LED_PIN).Blink(500, 500).Repeat(2);
JLed BLE_ERROR_LED3 = JLed(INFO_LED_PIN).Blink(50, 50).Repeat(1);
JLed AWS_ERROR_LED1 = JLed(INFO_LED_PIN).Blink(500, 500).Repeat(1);
JLed AWS_ERROR_LED2 = JLed(INFO_LED_PIN).Blink(50, 50).Repeat(2);
JLed AWS_ERROR_LED3 = JLed(INFO_LED_PIN).Blink(500, 500).Repeat(1);
JLed CC_ERROR_LED = JLed(INFO_LED_PIN).Blink(50, 50).Repeat(4);
JLed HDC_ERROR_LED1 = JLed(INFO_LED_PIN).Blink(50, 50).Repeat(1);
JLed HDC_ERROR_LED2 = JLed(INFO_LED_PIN).Blink(500, 500).Repeat(1);
JLed HDC_ERROR_LED3 = JLed(INFO_LED_PIN).Blink(50, 50).Repeat(1);
JLed HDC_ERROR_LED4 = JLed(INFO_LED_PIN).Blink(500, 500).Repeat(1);
JLed EEPROM_ERROR_LED1 = JLed(INFO_LED_PIN).Blink(500, 500).Repeat(2);
JLed EEPROM_ERROR_LED2 = JLed(INFO_LED_PIN).Blink(50, 50).Repeat(2);
JLed DATA_TRANS_LED = JLed(INFO_LED_PIN).Blink(300, 300).Forever();
//error

//=================================Variable==========================================//
byte buff[READ_PIN];  //buff variable accel
//sensors
int sound_value = 0;  //sound
int write_status;     //variable write
int r_buf[35];        //variable read d6t
int t_data[16];       //variable text d6t
int x, y, z;          //accelerometer coordinates
int reg_add = 0x32;   //registor accel
int co2;              //variable co2
int temp;             //variable temperature
short int cnt = 0;    //variable delay
float t_ptat;         //variable d6t
//wifi configuration
const int max_connect_attempts = 10;             //variable repeat connect
int connect_attempts;                            //variable connect
const int conf_par_size = 25;
char conf_wifi_ssid[conf_par_size] = "";                         //Login and password Wifi network
char conf_wifi_password[conf_par_size] = "";
int conf_interval = 10; //interval in seconds default is 10 seconds
//begin app status
typedef enum {
  WIFI_ERROR,                                    //if this error appear then blinking led scheme is ----
  BLE_ERROR,                                     //if this error appear then blinking led scheme is -.--
  AWS_ERROR,                                     //if this error appear then blinking led scheme is --.-
  CC_ERROR,                                      //error CCS811-sencor blinking code is ---.
  HDC_ERROR,                                     //error HDC1080 sensor blinking code is .---
  EEPROM_ERROR,                                  //error EEPROM memory blinking code is ..--
  DATA_TRANSFERING,                              //start data transfer  blinking code is ...-
  BLE_TRANSFERING,                               //start data transfer  blinking code is ....
  BLE_OFF                                        //start data transfer  blinking code is .-.-
  } app_status;

app_status a_status;

//end app status

bool conf_button_pressed = false;
bool timer_enabled = false;
bool interval_passed = false;
unsigned long int latested_timer = 0;
static SoftwareSerial sensor(PIN_RX, PIN_TX);
static MHZ19 mhz19(&sensor);
Adafruit_CCS811 ccs;
ClosedCube_HDC1080 hdc1080;
MQTTClient client = MQTTClient(256);
WiFiClientSecure net = WiFiClientSecure();
//=================================Variables==========================================//
void init_wire();                                                   //Initialization I2C
void read_from(int device, byte address, int num, byte buff[]);     //Read buff sensor
void write_to(int device, byte address, byte val);                  //Write buff sensor
bool connect_to_aws();                                              //Connect to aws
void disconnect_aws();                                              //Disconnect to aws
void send_data_to_aws();                                            //Send data on the aws
void message_handler(String &topic, String &payload);               //Forming a letter to send
void save_creds();                                                  //Save loggin and password
void read_creds();                                                  //Read loggin and password
void init_ble();                                                    //Initialization BLE
void stop_ble();                                                    //Stop BLE
bool connect_to_wifi();                                             //Connect to network
void disconnect_wifi();                                             //Disconnect network
void start_config();                                                //disconnects from AWS and turns of wifi, turns of BLE to start configuration
void start_transfer();                                              //starts wifi and connects to aws to start transfering data
void start_timer();                                                 //starts time for data transfering
void stop_timer();                                                  //stops timer for data transfering
void check_timer();                                                    //timer interuption funtion
void IRAM_ATTR isr();
//led blinker and parameter is array of ERROR_CODE_SIZE items 
//where ERROR_CODE_LONG long light and ERROR_CODE_SHORT is short light

class MyServerCallbacks;                                            //Class connect and disconnect
class MyCallbacksSwitch;                                            //Class switch flag
class MyCallbacksSSID;                                                 //Class formation loggin
class MyCallbacksPass;                                                 //Class formation password
class MyCallbacksInterval;

void setup(){
  Serial.begin(115200);
  if(!EEPROM.begin(EEPROM_SIZE)){
    a_status = EEPROM_ERROR;
  }

  pinMode(BUTTON_PIN,  INPUT_PULLUP);
  pinMode(INFO_LED_PIN,OUTPUT);
  attachInterrupt(BUTTON_PIN, isr, FALLING);

  init_wire();
  
  read_creds();
  
  if (String(conf_wifi_ssid) == "" ||  String(conf_wifi_password) == "") {  //if we don't have credentials to wifi then lets start ble for initialization
    start_config();
  } else {   
    //otherway we can start connecting to wifi
    Serial.print("The interval of data transmition is: ");
    Serial.print(conf_interval);
    Serial.println(" seconds");
    start_transfer();
  }
}

void loop(){
  check_timer();
  //error handler
  switch(a_status){
    case WIFI_ERROR:
      WIFI_ERROR_LED.Update();
      return;
    case BLE_ERROR:
      BLE_ERROR_LED1.Update();
      BLE_ERROR_LED2.Update();
      BLE_ERROR_LED3.Update();
      break;
    case AWS_ERROR:
     AWS_ERROR_LED1.Update();
     AWS_ERROR_LED2.Update();
     AWS_ERROR_LED3.Update();
      break;
    case CC_ERROR:
      CC_ERROR_LED.Update();
      break;
    case HDC_ERROR:
      HDC_ERROR_LED1.Update();
      HDC_ERROR_LED2.Update();
      HDC_ERROR_LED3.Update();
      HDC_ERROR_LED4.Update();
      break;
    case EEPROM_ERROR:
      EEPROM_ERROR_LED1.Update();
      EEPROM_ERROR_LED2.Update();
      break;
    case BLE_OFF:
      start_transfer();
      break;
    case DATA_TRANSFERING:
      //DATA_TRANS_LED.Update();
      break;
    default:
      break;
  }

  // BLE button handler

  if((conf_button_pressed) && (a_status != BLE_TRANSFERING)){//ble activation button pressed
    start_config();
  } 

  if (interval_passed) {
      send_data_to_aws();
  }
}
  
//=================================Classes BLE==========================================//
class MyServerCallbacks: public BLEServerCallbacks{
  void onConnect(BLEServer* pServer){
    Serial.println("Connected BLE.");
  }
  void onDisconnect(BLEServer* pServer){
    Serial.println("Disconnected BLE.");
  }
};

class MyCallbacksInterval: public BLECharacteristicCallbacks{
  void onWrite(BLECharacteristic *pCharacteristic){
    std::string rxValue = pCharacteristic->getValue();
    if(rxValue.length() > 0){
      Serial.print("Period Received: ");
      conf_interval = atoi(rxValue.c_str());
      Serial.print(conf_interval);
      Serial.println();
      Serial.println("*********");
    }
  }
};

class MyCallbacksSwitch: public BLECharacteristicCallbacks{
  void onWrite(BLECharacteristic *pCharacteristic){
    Serial.println("Switch");
    a_status = BLE_OFF;
  }
};

class MyCallbacksSSID: public BLECharacteristicCallbacks{
  void onWrite(BLECharacteristic *pCharacteristic){
    std::string rxValue = pCharacteristic->getValue();
    if(rxValue.length() > 0){
      Serial.print("Login Received: ");
      for(int i = 0; i < rxValue.length(); i++){
        conf_wifi_ssid[i]=rxValue[i];
      }
      conf_wifi_ssid[rxValue.length()] = '\0';
      Serial.print(conf_wifi_ssid);
      Serial.println();
      Serial.println("*********");
    }
  }
};

class MyCallbacksPass: public BLECharacteristicCallbacks{
  void onWrite(BLECharacteristic *pCharacteristic){
    std::string rxValue = pCharacteristic->getValue();
    if(rxValue.length() > 0){
      Serial.print("Password received: ");
      for(int i = 0; i < rxValue.length(); i++){
        conf_wifi_password[i]=rxValue[i];
      }
      conf_wifi_password[rxValue.length()]='\0';
      Serial.print(conf_wifi_password);
      Serial.println();
      Serial.println("*********");
    }
  }
};
//=================================Functions==========================================//
void IRAM_ATTR isr() {
  conf_button_pressed = true;
}

void init_wire() {
  Wire.begin();
  sensor.begin(MHZ19::BIT_RATE);

  if(ccs.begin()){
    Serial.println("Succesfully connect to sensor CCS!");    
  } else {
    a_status = CC_ERROR;
  }
  
  hdc1080.begin(0x40);
  if(hdc1080.readDeviceId() == HDC){
    Serial.println("Succesfully connect to sensor HDC1080!");
  } else {
    a_status = HDC_ERROR;
  }
  //calibrate sensor
  while(!ccs.available());
  Serial.println("Succesfully connect to sensor CCS and calibrated!");
  write_to(ADDRES_SENSOR, 0x2D, 16);
  write_to(ADDRES_SENSOR, 0x2D, 8);
  Serial.println("Succesfully connect to sensor ADXL345!");
}

void write_to(int device, byte address, byte val){
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(val);
  Wire.endTransmission();
}
  
void read_from(int device, byte address, int num, byte buff[]){
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.endTransmission();
  Wire.beginTransmission(device);
  Wire.requestFrom(device, num);
  int a = 0;
  while(Wire.available()){
    buff[a] = Wire.read();
    a++;
  }
  Wire.endTransmission();
}

bool connect_to_aws() {
  bool c_status = false;
  
  net.setCACert(aws_cert_ca);                                                   //Configure WiFiClientSecure to use the AWS certificates we generated
  net.setCertificate(aws_cert_crt);
  net.setPrivateKey(aws_cert_private);
  client.begin(AWS_IOT_ENDPOINT, 8883, net);                                    //Connect to the MQTT broker on the AWS endpoint we defined earlier
  int retries = 0;
  Serial.println("Connecting to AWS IOT");                                      //Try to connect to AWS and count how many times we retried.
  while(!client.connect(DEVICE_NAME) && retries < AWS_MAX_RECONNECT_TRIES && !conf_button_pressed){
    Serial.print(".");
    delay(1000);
    retries++;
  }
  
  if(!client.connected()){                                                      //Make sure that we did indeed successfully connect to the MQTT broker
    Serial.println(" Timeout!");                                                //If not we just end the function and wait for the next loop.
    
  } else {
    Serial.println("!!!Connected!!!");                                            //If we land here, we have successfully connected to AWS!And we can subscribe to topics and send messages.
    client.subscribe("/" DEVICE_NAME "/comm");
    client.onMessage(message_handler);     
    c_status = true;
  }
  
  return c_status;
}

void disconnect_aws() {
  Serial.println("disconnect_aws");
  client.disconnect();
}

void message_handler(String &topic, String &payload){
  Serial.println("incoming: " + topic + " - " + payload);
}

void send_data_to_aws(){
  stop_timer();
  
  Serial.println("Reading data from sensors and preparing for sending to AWS.");
  StaticJsonDocument<512> json_doc;
  JsonObject state_obj = json_doc.createNestedObject("state");
  JsonObject reported_obj = state_obj.createNestedObject("reported");
//======================================Accel=====================================//
  read_from(ADDRES_SENSOR, reg_add, READ_PIN, buff);
  x=(((int)buff[1]) << 8) | buff[0];
  y=(((int)buff[3]) << 8) | buff[2];
  z=(((int)buff[5]) << 8) | buff[4];
  JsonObject Accel_obj = reported_obj.createNestedObject("Accel");
  Accel_obj["x"] = x;
  Accel_obj["y"] = y;
  Accel_obj["z"] = z;
//==============================Temperature and Humidity==========================//
  reported_obj["T"] = hdc1080.readTemperature();
  reported_obj["Rh"] = hdc1080.readHumidity();
//======================================CO2======================================//
  mhz19.readCO2(&co2, &temp);
  reported_obj["co2"] = co2;
//=====================================TVOC=====================================//
  if(ccs.available()){
    if(!ccs.readData()){
      reported_obj["TV0C"] = ccs.getTVOC();
    } else {
        Serial.println("ERROR!");
        while(1);
      }
  }
//=====================================D6T======================================//
  Wire.beginTransmission(D6T_ADDR);
  Wire.write(D6T_CMD);
  Wire.endTransmission();
  write_status = Wire.endTransmission(false);
  Wire.requestFrom(D6T_ADDR, 35);
  for(int b = 0; b < 35; b++){
    r_buf[b] = Wire.read();
  }
  t_ptat = (r_buf[0] + (r_buf[1] << 8)) * 0.1;
  for(int i = 0; i < 16; i++){
    t_data[i] = (r_buf[(i * 2 + 2)] + (r_buf[(i * 2 + 3)] << 8)) * 0.1;
  }
  int cont = 0;
  for(int i = 0; i < 4; i++){
    for(int j = 0; j < 4; j++){
    }
    cont++;
  
    if(t_data[i] + cont <= 28){
      reported_obj["D6T"] = false;
    } else {
      reported_obj["D6T"] = true;
    }
  }
  //sound measurement
  long sound=0;
  long noise = 0;
  uint8_t counter = 1;
  unsigned long noiseMillis = 0; 
  long previousMillis = 0;
  long sampling_interval = 1000;  
  sound = 0;
  while (noiseMillis < 5100){
    noiseMillis = millis();
    if((noiseMillis - previousMillis) > sampling_interval) {
      previousMillis = noiseMillis;   
      noise = analogRead(SOUND_PIN);
      sound = sound+noise;
      counter++;
    }  
  }
  sound_value = sound / counter; 
    
//  sound_value = analogRead(SOUND_PIN);
//  sound_value = (sound_value * 3.3) / 84;
  reported_obj["dB"] = sound_value;
//=================================JSON========================================//
  char json_buffer[512];
  serializeJson(json_doc, json_buffer);
  client.publish("$aws/things/" DEVICE_NAME "/shadow/update", json_buffer);

  start_timer();
  
  Serial.println("Sending data to AWS");
}

void save_creds(){
  EEPROM_ZERO_ADD = 0;
  EEPROM.writeString(EEPROM_ZERO_ADD, conf_wifi_ssid);
  EEPROM_ZERO_ADD += 26;
  EEPROM.writeString(EEPROM_ZERO_ADD, conf_wifi_password);
  EEPROM_ZERO_ADD += 26;
  EEPROM.writeString(EEPROM_ZERO_ADD, String(conf_interval));
  EEPROM_ZERO_ADD += 26;
  EEPROM.commit();
  Serial.println("Credentials saved in EEPROM");
}

void read_creds(){
  EEPROM_ZERO_ADD = 0;
    
  String rxValue = EEPROM.readString(EEPROM_ZERO_ADD);
  EEPROM_ZERO_ADD += 26;
  for(int i = 0; i < rxValue.length(); i++){
    conf_wifi_ssid[i]=rxValue[i];
  }
  rxValue = EEPROM.readString(EEPROM_ZERO_ADD);
  EEPROM_ZERO_ADD += 26;
  for(int i = 0; i < rxValue.length(); i++){
    conf_wifi_password[i]=rxValue[i];
  }
  
  // String conf_interval = EEPROM.readString(EEPROM_ZERO_ADD);
  conf_interval = atoi(EEPROM.readString(EEPROM_ZERO_ADD).c_str());
  EEPROM_ZERO_ADD += 26;
  Serial.println("Credentials read from EEPROM");
  Serial.println(conf_wifi_ssid);
  Serial.println(conf_wifi_password);
  Serial.println(conf_interval);
}

bool connect_to_wifi(){
  bool c_status = false;

  WiFi.mode(WIFI_STA);
  WiFi.begin(conf_wifi_ssid, conf_wifi_password);

  int retries = 0;                                                            //Only try $max_connect_attempts times to connect to the WiFi
  while(WiFi.status() != WL_CONNECTED && retries < max_connect_attempts && !conf_button_pressed) {
    delay(500);
    Serial.print(".");
    retries++;
  }
  Serial.println(".");
  if(WiFi.status() != WL_CONNECTED && !conf_button_pressed) {                                      //If we still couldn't connect to the WiFi, go to deep sleep for a minute and try again.
    esp_sleep_enable_timer_wakeup(1 * 10L * 1000000L);
    esp_deep_sleep_start();
    c_status = false;
  } else if (!conf_button_pressed) {
    c_status = true;
  }
  return c_status;
}

void disconnect_wifi(){
  Serial.println("disconnect_wifi");
  WiFi.disconnect();                                            // cut off the WIFI connection
  WiFi.softAPdisconnect();                                      // we disconnect the access point (if it was)
  WiFi.mode(WIFI_OFF);                                          // Off WIFI
}

void init_ble() {
  BLEDevice::init("Home Sensor");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(HOME_SERVICE_UUID);
  
  BLECharacteristic *pCharacteristicSSID = pService->createCharacteristic(
                                         SSID_CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  pCharacteristicSSID->setCallbacks(new MyCallbacksSSID());
  
  BLECharacteristic *pCharacteristicPass = pService->createCharacteristic(
                                         PASSWORD_CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  pCharacteristicPass->setCallbacks(new MyCallbacksPass());
  
  BLECharacteristic *pCharacteristicInterval = pService->createCharacteristic(
                                         INTERVAL_CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  pCharacteristicInterval->setCallbacks(new MyCallbacksInterval());
  
  BLECharacteristic *pCharacteristicSwich = pService->createCharacteristic(
                                         SWITCH_CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  pCharacteristicSwich->setCallbacks(new MyCallbacksSwitch());
  pService->start();
  
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(HOME_SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::getAdvertising()->start();

  Serial.println("Home Sensor BLE Service successfully create.");
}

void stop_ble(){
  save_creds();
  ESP.restart();    
  Serial.println("BLE mode is OFF now.");
}

void start_ble() {  
  init_ble();
  
  Serial.println("BLE mode is ON now.");
  a_status = BLE_TRANSFERING;
}

void start_config() {
  Serial.println("Start config mode");
  disconnect_aws();
  disconnect_wifi();
  
  stop_timer();
  start_ble();
}

void start_transfer() {  
  conf_button_pressed = false;

  if (!connect_to_wifi()) {
    stop_timer();
    a_status = WIFI_ERROR;
    Serial.println("Failed connecting to wi-fi");
  } else {
    Serial.println("Succesfully connected to wi-fi");

    if (!connect_to_aws()) {
      a_status = AWS_ERROR;
    } else {
      start_timer();
      a_status = DATA_TRANSFERING;
    }    
  }
}


void start_timer() {
  latested_timer = millis();
  timer_enabled = true;
  Serial.println("Timer has started");
}

void stop_timer() {
  timer_enabled = false;
  latested_timer = -1;
  Serial.println("Timer has stoped.");
}

void check_timer() {
  unsigned long current_millis = millis();
  
  if (timer_enabled) {
    if (latested_timer > current_millis) {
      start_timer();
    }
    
    if (current_millis - latested_timer > (conf_interval * 1000)) {
      interval_passed = true;    
      Serial.println("Timer fired");
    } else {
      interval_passed = false;
    }
  }
}
