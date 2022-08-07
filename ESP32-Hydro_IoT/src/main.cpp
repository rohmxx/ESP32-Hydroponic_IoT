#include <WiFi.h>
#include <Wire.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <LiquidCrystal_I2C.h>
#include <Firebase_ESP_Client.h>
#include "DHT.h"

#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

#define adc_res 4096
#define samplerate 30

#define ppm_ph_pin 2
#define ph_pump 13
#define mixed_pump 14
#define plant_pump 27
#define nutrisi_pump 12
#define pilot_lamp 25
#define ppm_ph_switch 26
#define otolevel_radar 33
#define nutrisi_optional 32
#define lcd_addr 0x27

#define DHTPIN 15
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

#define WIFI_SSID "rohmxx-GalaxyA51"
#define WIFI_PASSWORD "hahahihi"
#define API_KEY "AIzaSyAXRTy8gMEf2ukx3XLKzWIFruU9pGXTMqc"
//#define USER_EMAIL "rohmanaditya24@gmail.com"
//#define USER_PASSWORD "24rhmn11adty02"
#define DATABASE_URL "https://hydroponic-iot-slice2022-default-rtdb.asia-southeast1.firebasedatabase.app/"

LiquidCrystal_I2C lcd(lcd_addr,16,2);

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
FirebaseJson json;
bool signupOK = false;

String uid;

String sensor_databasePath = "/SensorData/readings";
String sensor_parentPath;
String tempPath = "/temperature";
String humPath = "/humidity";
String ppmPath = "/ppm";
String phPath = "/ph";
String timePath = "/timestamp";

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");
uint timestamp;

uint8_t i,j,k;
uint8_t sens_mode = 0; // 0 is ppm; 1 is ph;

uint8_t temperature = 0;
uint8_t humidity = 0;
uint16_t ppm_adc = 0;
uint16_t ph_adc = 0;

uint16_t setp_hari[5] = {0};
uint16_t setp_ppm[5] = {0};
uint16_t setp_ph[5] = {0};

uint16_t sp_ppm = 0;
uint16_t sp_ph = 0;

uint16_t day_count = 0;

float ppm_data = 0;
float ph_data = 0;

unsigned long sendDataPrevMillis = 0;
unsigned long readDataPrevMillis = 0;
unsigned long nutripumpMillis = 0;
unsigned long phpumpMillis = 0;
unsigned long mixpumpMillis = 0;

void initWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  lcd.setCursor(0,0);
  lcd.print("Connecting Firebase");
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
  lcd.setCursor(0,1);
  lcd.print(WiFi.localIP());
  Serial.println();
}

unsigned long getTime() {
  timeClient.update();
  unsigned long now = timeClient.getEpochTime();
  return now;
}

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
}

void sendFirebase(){
  timestamp = getTime();
  sensor_parentPath= sensor_databasePath + "/" + String(timestamp);
  json.set(tempPath.c_str(), String(temperature));
  json.set(humPath.c_str(), String(humidity));
  json.set(ppmPath.c_str(), String(ppm_data));
  json.set(phPath.c_str(), String(ph_data));
  json.set(timePath, String(timestamp));
  Serial.printf("Set json... %s\n", Firebase.RTDB.setJSON(&fbdo, sensor_parentPath.c_str(), &json) ? "ok" : fbdo.errorReason().c_str());
}

void read_DHT11(){
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
}

void read_ppm_ph(){
  // read ppm
  if(sens_mode == 0){
    for (i=0;i<samplerate;i++){
      ppm_adc+= analogRead(ppm_ph_pin);
    }
    ppm_data = 7 + ( ( 2.5 - (3.3/adc_res*ppm_adc/samplerate) ) / 0.18 );
    sens_mode = 1;
    digitalWrite(ppm_ph_switch, 1);
  }
  // read ph
  else if(sens_mode == 1){
    for(i=0;i<samplerate;i++){
      ph_adc += analogRead(ppm_ph_pin);
    }
    ph_data = 7 + ( ( 2.5 - (3.3/adc_res*ph_adc/samplerate) ) / 0.18 );
    sens_mode = 0;
    digitalWrite(ppm_ph_switch, 0);
  }
}

void process_ph(){
  // proses campuran ph
}

void process_ppm(){
  // proses campuran ppm
}

void setup()
{
  Serial.begin(115200);
  dht.begin();
  lcd.init();
  lcd.backlight();
  initWiFi();
  timeClient.begin();

  pinMode(32, OUTPUT);
  pinMode(25, OUTPUT);
  pinMode(27, OUTPUT);
  
//  auth.user.email = USER_EMAIL;
//  auth.user.password = USER_PASSWORD;
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Connecting Firebase");
  if(Firebase.signUp(&config, &auth, "", "")){
    Serial.println("ok");
    lcd.setCursor(0,1);
    lcd.print("ok");
    signupOK = true;
  }
  else{
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
    lcd.setCursor(0,1);
    lcd.print("error");
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
  lcd.clear();

  digitalWrite(ppm_ph_switch, 0);
}

void loop()
{
  // Read setpoint and sensor every 1second
  if(Firebase.ready() && (millis() - readDataPrevMillis > 1000 || readDataPrevMillis == 0)){
    sendDataPrevMillis = millis();
    readFirebase();
    read_DHT11();
    read_ppm_ph();
  }

  // Send to database every 1minute
  if(Firebase.ready() && (millis() - sendDataPrevMillis > 60000 || sendDataPrevMillis == 0)){
    sendDataPrevMillis = millis();
    sendFirebase();
  }

  //Automation Process
  for(j = 0; j < 5; i++){  //get setpoint from daycount
    if(day_count>=setp_hari[i]){
      sp_ph = setp_ph[i];
      sp_ppm = setp_ppm[i];
      break;
    }
  }
  if(ph_data<sp_ph){
    process_ph();
  }
  if(ppm_data<sp_ppm){
    process_ppm();
  }
  // if(ppm<setp){
  //   if(state==1){
  //     if((millis() - prevtime) > 5000){
  //       state=0;
  //       prevtime=millis();
  //     }
  //   }
  //   else if(state==0){
  //     if((millis() - prevtime) > 1000){
  //       state=1;
  //       prevtime=millis();
  //     }
  //   }
  //   digitalWrite(32, state);
  //   digitalWrite(25, state);
  //   digitalWrite(27, state);
  // }
}