#include <Arduino.h>
 
#include <LiquidCrystal_I2C.h>
 
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#include <EEPROM.h>
#include "GravityTDS.h"
GravityTDS tds;


//Init WIFI
#include <WiFi.h> 
#include <WiFiUdp.h>

#define WIFI_SSID "Wifi Ruang KKPI"
#define WIFI_PASSWORD ""

//Init NTP
#include <NTPClient.h>
#include "time.h"

hw_timer_t *timer1 = NULL;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

uint timestamp;

// Init FIREBASE
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

#define API_KEY "AIzaSyAXRTy8gMEf2ukx3XLKzWIFruU9pGXTMqc"
#define DATABASE_URL "https://hydroponic-iot-slice2022-default-rtdb.asia-southeast1.firebasedatabase.app/"


//#define samplerate 100
//#define adc_res 4096

//Pin input
#define water_level   17

//Pin Relay
#define solenoid      33
#define ph_pump       32
#define aerator_pump  14
#define nutrisia_pump 27
#define nutrisib_pump 23
#define buzzer        25 

//Pin LCD
#define lcd_btn 18
#define lcd_addr 0x27

//Pin Sensor PH
#define ph_pin 35 //34 ==> 4

//Pin Sensor EC
#define tds_pin 34 //35 ==> 2
 
//Pin DHT 
#define DHTPIN 15  

DHT dht(DHTPIN, DHT11);

LiquidCrystal_I2C lcd(lcd_addr,16,2);

uint32_t delayMS;

String uid;
  
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
FirebaseJson json;
bool signupOK = false;

String sensor_databasePath = "/SensorData/readings";
String sensor_parentPath;
String tempPath = "/temperature";
String humPath = "/humidity";
String ppmPath = "/ppm";
String phPath = "/ph";
String timePath = "/timestamp";

String batas_waktu = "";

float prevtemp = 30,temperature = 28,prevhum = 60,humidity = 60,tds_adc,TeganganPh,tds_data,ph_data,div_ppm,div_ph;

bool CEK_KADAR_AIR = 0;
bool UPDATE_LCD = 0;
bool UPDATE_SETPOINT = 0;
bool SEND_FIREBASE_DATA = 1;

void initWiFi() {
  delay(1000);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);   
  Serial.print("Connecting to WiFi ..");
  digitalWrite(buzzer, LOW);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(200);
  }
  digitalWrite(buzzer, HIGH);
  Serial.println(WiFi.localIP());  
}

unsigned long getTime() {
  timeClient.update();
  unsigned long now = timeClient.getEpochTime();
  return now+25200;
}

void konversiWaktu(){
  struct tm timeinfo;  
  if(!getLocalTime(&timeinfo)){
    Serial.println("konversi waktu failed");
  }
  Serial.println(&timeinfo, "%A, %B, %d, %Y, %H:%M:%S");
  batas_waktu = (timeinfo.tm_hour);
}

void InitFirebase(){
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL; 
  if (Firebase.signUp(&config, &auth, "", "")){
    Serial.println("ok");
    signupOK = true;
  }
  else{
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }
  
  Firebase.reconnectWiFi(true);
  fbdo.setResponseSize(4096);
  config.token_status_callback = tokenStatusCallback;
  config.max_token_generation_retry = 5;
  Firebase.begin(&config, &auth);

  // Getting the user UID might take a few seconds
  Serial.println("Getting User UID");
  while ((auth.token.uid) == "") {
    Serial.print('.');
    delay(1000);
  }
  uid = auth.token.uid.c_str();
  Serial.print("User UID: ");
  Serial.println(uid); 

}

void sendFirebase(){
  timestamp = getTime();
  sensor_parentPath= sensor_databasePath + "/" + String(timestamp);
  json.set(tempPath.c_str(), String(temperature,1));
  json.set(humPath.c_str(), String(humidity,1));
  json.set(ppmPath.c_str(), String(tds_data,1));
  json.set(phPath.c_str(), String(ph_data,1));
  json.set(timePath, String(timestamp));
  Serial.printf("Set json... %s\n", Firebase.RTDB.setJSON(&fbdo, sensor_parentPath.c_str(), &json) ? "ok" : fbdo.errorReason().c_str());
}

uint16_t setp_hari[5] = {0};
uint16_t setp_ppm[5] = {0};
uint16_t setp_ph[5] = {0};

uint16_t sp_ppm = 500;
uint16_t sp_ph = 6.8;
uint16_t sp_hari = 0;
uint16_t day_count = 0;

void readFirebase(){
  setp_hari[0]  = Firebase.RTDB.getInt(&fbdo, F("/Setpoint/1/hari")) ? fbdo.to<int>() : 0;
  setp_ppm[0]   = Firebase.RTDB.getInt(&fbdo, F("/Setpoint/1/ppm")) ? fbdo.to<int>() : 0;
  setp_ph[0]    = Firebase.RTDB.getInt(&fbdo, F("/Setpoint/1/ph")) ? fbdo.to<int>() : 0;
  setp_hari[1]  = Firebase.RTDB.getInt(&fbdo, F("/Setpoint/2/hari")) ? fbdo.to<int>() : 0;
  setp_ppm[1]   = Firebase.RTDB.getInt(&fbdo, F("/Setpoint/2/ppm")) ? fbdo.to<int>() : 0;
  setp_ph[1]    = Firebase.RTDB.getInt(&fbdo, F("/Setpoint/2/ph")) ? fbdo.to<int>() : 0;
  setp_hari[2]  = Firebase.RTDB.getInt(&fbdo, F("/Setpoint/3/hari")) ? fbdo.to<int>() : 0;
  setp_ppm[2]   = Firebase.RTDB.getInt(&fbdo, F("/Setpoint/3/ppm")) ? fbdo.to<int>() : 0;
  setp_ph[2]    = Firebase.RTDB.getInt(&fbdo, F("/Setpoint/3/ph")) ? fbdo.to<int>() : 0;
  setp_hari[3]  = Firebase.RTDB.getInt(&fbdo, F("/Setpoint/4/hari")) ? fbdo.to<int>() : 0;
  setp_ppm[3]   = Firebase.RTDB.getInt(&fbdo, F("/Setpoint/4/ppm")) ? fbdo.to<int>() : 0;
  setp_ph[3]    = Firebase.RTDB.getInt(&fbdo, F("/Setpoint/4/ph")) ? fbdo.to<int>() : 0;
  setp_hari[4]  = Firebase.RTDB.getInt(&fbdo, F("/Setpoint/5/hari")) ? fbdo.to<int>() : 0;
  setp_ppm[4]   = Firebase.RTDB.getInt(&fbdo, F("/Setpoint/5/ppm")) ? fbdo.to<int>() : 0;
  setp_ph[4]    = Firebase.RTDB.getInt(&fbdo, F("/Setpoint/5/ph")) ? fbdo.to<int>() : 0;
  day_count     =  Firebase.RTDB.getInt(&fbdo, F("/Day_Counter")) ? fbdo.to<int>() : 0;
}

void PrintFirebase(){
  Serial.print("Set Point Ke - 1 ==> hari ke ");
  Serial.print(setp_hari[0]);
  Serial.print(", ppm ");
  Serial.print(setp_ppm[0]);
  Serial.print(", pH ");
  Serial.println(setp_ph[0]);

  Serial.print("Set Point Ke - 2 ==> hari ke ");
  Serial.print(setp_hari[1]);
  Serial.print(", ppm ");
  Serial.print(setp_ppm[1]);
  Serial.print(", pH ");
  Serial.println(setp_ph[1]);

  Serial.print("Set Point Ke - 3 ==> hari ke ");
  Serial.print(setp_hari[2]);
  Serial.print(", ppm ");
  Serial.print(setp_ppm[2]);
  Serial.print(", pH ");
  Serial.println(setp_ph[2]);

  Serial.print("Set Point Ke - 4 ==> hari ke ");
  Serial.print(setp_hari[3]);
  Serial.print(", ppm ");
  Serial.print(setp_ppm[3]);
  Serial.print(", pH ");
  Serial.println(setp_ph[3]);

  Serial.print("Set Point Ke - 5 ==> hari ke ");
  Serial.print(setp_hari[4]);
  Serial.print(", ppm ");
  Serial.print(setp_ppm[4]);
  Serial.print(", pH ");
  Serial.println(setp_ph[4]);

  Serial.print("HARI INI ADALAH HARI KE - ");
  Serial.println(day_count);
}

void getSetpoint(){
  for (int i = 0; i < 5; i++){
    if(day_count>=setp_hari[i]){
      sp_ph = setp_ph[i];
      sp_ppm = setp_ppm[i];
      if(sp_ppm>=1200){
        sp_ppm = 1200;
      }
      sp_hari = setp_hari[i];
      break;
    }
  }
  Serial.print("Hari ke ");
  Serial.println(sp_hari);
  Serial.print("sp ppm ");
  Serial.println(sp_ppm);
  Serial.print("sp ph ");
  Serial.println(sp_ph);
}

void IRAM_ATTR timerFirebase(){
  SEND_FIREBASE_DATA = 1;
}

void IRAM_ATTR displayLCD(){
  UPDATE_LCD = 1;
}

void InitDHT(){
  dht.begin();
  // Serial.println(F("DHTxx Unified Sensor Example"));
  // Print temperature sensor details.
  // sensor_t sensor;
  // dht.temperature().getSensor(&sensor);
  // Serial.println(F("------------------------------------"));
  // Serial.println(F("Temperature Sensor"));
  // Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  // Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  // Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  // Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
  // Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
  // Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
  // Serial.println(F("------------------------------------"));
  // // Print humidity sensor details.
  // dht.humidity().getSensor(&sensor);
  // Serial.println(F("Humidity Sensor"));
  // Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  // Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  // Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  // Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
  // Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
  // Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
  // Serial.println(F("------------------------------------"));
  // // Set delay between sensor readings based on sensor details.
  // delayMS = sensor.min_delay / 1000;  
}
void ReadDHT(){
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();
  if(isnan(temperature)){
    Serial.println(F("Error reading temperature!"));
    temperature = prevtemp;
  }
  else{
    Serial.print("Temperature: ");
    Serial.println(temperature);
    prevtemp = temperature;
  }
  if(isnan(humidity)){
    Serial.println(F("Error reading humidity!"));
    humidity = prevhum;
  }
  else{
    Serial.print("Humidity: ");
    Serial.println(humidity);
    prevhum = humidity;
  }
  // delay(delayMS);
  // // Get temperature event and print its value.
  // sensors_event_t event;
  // dht.temperature().getEvent(&event);
  // else {
  //   temperature = event.temperature;  
  //   Serial.print(F("Temperature: "));
  //   //Serial.print(event.temperature);
  //   Serial.print(temperature,1);
  //   Serial.println(F("°C"));
  // }
  // // Get humidity event and print its value.
  // dht.humidity().getEvent(&event);
  // if (isnan(event.relative_humidity)) {
  //   Serial.println(F("Error reading humidity!"));
  // }
  // else {
  //   humidity=event.relative_humidity;
  //   Serial.print(F("Humidity: "));
  //   Serial.print(humidity,1);
  //   Serial.println(F("%"));
  // }
}

void InitTds(){  
  tds.setPin(tds_pin);
  tds.setAref(3.3);  //reference voltage on ADC, default 5.0V on Arduino UNO
  tds.setAdcRange(4096);  //1024 for 10bit ADC;4096 for 12bit ADC
  tds.begin();  //initialization
  // pinMode(tds_pin, INPUT); 
  // analogReadResolution(10);  
}
void ReadTds(){
  //temperature = readTemperature();  //add your temperature sensor and read it
  // gravityTds.setTemperature(25);  // set the temperature and execute temperature compensation
  // gravityTds.update();  //sample and calculate
  // tds_data = gravityTds.gettds_data();  // then get the value
  // Serial.print(tds_data,0);
  // Serial.println("ppm");
  // delay(1000);
}

void InitPH(){
  pinMode(ph_pin, INPUT); 
  analogReadResolution(12);
}
void ReadPh(){
  int pengukuranPh = analogRead(ph_pin);
  Serial.print("Nilai ADC: ");
  Serial.print(pengukuranPh);
  float TeganganPh = (3.3 / 4096.0)* pengukuranPh;
//  Serial.print("|| TeganganPh: ");
//  Serial.print(TeganganPh, 2);
//  Po = 7.00 + ((teganganPh7 - TeganganPh) / PhStep);
//  PhStep = (PH4 - PH7) / 3
  ph_data = 7.00 + ((2.8 - TeganganPh) / 0.17);
 
  Serial.print(" || NilaipH: ");
  Serial.println(ph_data, 2);
  //delay(500);
}

void readSensorAdc(){
  float sum_tds=0, sum_ph=0;

  for(int i=0;i<100;i++){
    TeganganPh = (3.3 / 4096.0)* analogRead(ph_pin);
    sum_ph += ((7.00 + ((2.8 - TeganganPh) / 0.17)) / 100);
  }
  // tds_data = sum_tds;
  ph_data = sum_ph;

  tds.setTemperature(25);  // set the temperature and execute temperature compensation
  tds.update();  //sample and calculate
  tds_data = tds.getTdsValue();  // then get the value
  tds_data = tds_data + (0.2*tds_data);
  
  Serial.print("Nilai pH: ");
  Serial.println(ph_data, 1);

  Serial.print("TDS (ppm)= ");
  Serial.println(tds_data,1);
}

void prosesOtomasi(){
  // Compare nilai PPM dengan sp_ppm, jika PPM < sp_ppm lakukan pengisian nutrisi
  if((ph_data - sp_ph) > 0.5){
    digitalWrite(aerator_pump, 0);
    digitalWrite(ph_pump, 0);
  }
  // Compare nilai pH dengan sp_ph, jika pH < sp_ph lakukan pengisian pH down
  if((tds_data - sp_ppm) < (-80)){
    digitalWrite(aerator_pump, 0);
    digitalWrite(nutrisia_pump, 0);
    digitalWrite(nutrisib_pump, 0);
  }
  delay(10000);
  digitalWrite(ph_pump, 1);
  digitalWrite(nutrisia_pump, 1);
  digitalWrite(nutrisib_pump, 1);
  delay(30000);               
  digitalWrite(aerator_pump, 1);
}

void setup() {
  Serial.begin(9600);
  Wire.setClock(100000);

  // put your setup code here, to run once:
  pinMode(lcd_btn,INPUT);
  pinMode(water_level,INPUT);

  timer1 = timerBegin(1, 80, true);
  attachInterrupt(lcd_btn, displayLCD, FALLING);
  timerAttachInterrupt(timer1, &timerFirebase, true);
  timerAlarmWrite(timer1, 60000000, true);
  timerAlarmEnable(timer1); //Just Enable

  pinMode(solenoid, OUTPUT);
  pinMode(ph_pump, OUTPUT);
  pinMode(aerator_pump, OUTPUT);
  pinMode(nutrisia_pump, OUTPUT);
  pinMode(nutrisib_pump, OUTPUT);  
  pinMode(buzzer, OUTPUT); 

  digitalWrite(solenoid, HIGH);
  digitalWrite(ph_pump, HIGH);
  digitalWrite(aerator_pump, HIGH);
  digitalWrite(nutrisia_pump, HIGH);
  digitalWrite(nutrisib_pump, HIGH);
  digitalWrite(buzzer, HIGH);

  initWiFi();
  timeClient.begin();
  configTime(7, 25200, "pool.ntp.org");
 
  InitFirebase(); 

  InitDHT();
  InitTds();
  InitPH();
  
  delay(1000);

  readFirebase();
  PrintFirebase();
  readSensorAdc();
  getSetpoint();
  
  delay(2000);
}

void loop() {

  if(WiFi.status() != WL_CONNECTED) {
    initWiFi();
    timeClient.begin();
    configTime(7, 25200, "pool.ntp.org");
  }

  for(int j=0;j<100;j++) while(digitalRead(water_level)==0){       // ketika air dalam bak kosong
    digitalWrite(solenoid, LOW);                                  // kran elektrik buka, air mengisi
    digitalWrite(buzzer, LOW);                                    // Buzzer nyala
    delay(1);                                                     //  debouncing filter

    CEK_KADAR_AIR=1;                                              // statement untuk melakukan pengecekan nutrisi dan pH
    SEND_FIREBASE_DATA=0;
  }
  digitalWrite(solenoid, HIGH);                                   // kran elektrik tutup
  digitalWrite(buzzer, HIGH);                                     // Buzzer mati

  // if(tds_data < sp_ppm  || ph_data > sp_ph){                      // pengecekan nilai PPM dan pH dengan nilai setpointnya
  //   CEK_KADAR_AIR=1;                                              // statement untuk melakukan pengecekan nutrisi dan pH
  // }

  if(CEK_KADAR_AIR==1){
    digitalWrite(aerator_pump, 0);
    delay(10000);
    digitalWrite(aerator_pump, 1);
    readSensorAdc();
    prosesOtomasi();
    if(((ph_data - sp_ph) > 0.5) || ((tds_data - sp_ppm) < (-80))){
      CEK_KADAR_AIR = 1;
    }
    else{
      CEK_KADAR_AIR = 0;
    }
  }

  if(SEND_FIREBASE_DATA==1){
    sendFirebase();
    SEND_FIREBASE_DATA = 0;
  }
  
  if(UPDATE_LCD==1){
    Serial.println("LCD");
    lcd.init();
    lcd.backlight();
    lcd.clear();
    
    lcd.setCursor(0, 0);
    lcd.print("pH: ");
    lcd.setCursor(3, 0);
    lcd.print(ph_data, 1); 
    lcd.setCursor(7, 0);
    lcd.print("Suhu:");
    lcd.setCursor(12, 0);
    lcd.print(temperature);
    lcd.setCursor(14, 0);
    lcd.print(" C");

    lcd.setCursor(0, 1);
    lcd.print("EC:");
    lcd.setCursor(3, 1);
    lcd.print(tds_data);
    lcd.setCursor(7, 1);
    lcd.print("PPM ~PENS");
    UPDATE_LCD = 0;
  }

  if(batas_waktu.toInt() >= 6 && batas_waktu.toInt() <= 7 && UPDATE_SETPOINT == 0){
    readFirebase();
    PrintFirebase();
    getSetpoint();
    UPDATE_SETPOINT = 1;
  }
  else if(!(batas_waktu.toInt() >= 6 && batas_waktu.toInt() <= 7) && UPDATE_SETPOINT == 1){
    UPDATE_SETPOINT = 0;
    prosesOtomasi();
  }

  ReadDHT();
  readSensorAdc();
  konversiWaktu();
}
