/* Verbindet sich mit WLAN und MQTT Broker
* Auch ohne Verbindung wird Loop weiter ausgeführt
* Bei Unterbruch von WLAN und oder MQTT, wird autoamtisch neu verbunden
*/
#include <Arduino.h>
//#include <string>
#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <Wire.h>  
#include <Ticker.h>               // https://github.com/esp8266/Arduino/tree/master/libraries/Ticker
#include <AsyncMqttClient.h>      // https://github.com/marvinroger/async-mqtt-client
#include <RtcDS3231.h>            // https://github.com/Makuna/Rtc/wiki/RtcDateTime-object
#include <EEPROM.h>
#include "Adafruit_Si7021.h"      // https://github.com/adafruit/Adafruit_Si7021

// WiFi & MQTT settings
const int MAX_AP = 3; // Try to connect to the following AP's
const char* WIFI_SSID[MAX_AP]  {"AP-SSID1", "AP-SSID2", "AP-SSID3"};      // more the one Wifi-AP to try
const char* WIFI_PASSWORD[MAX_AP] {"password SSID1", "password SSID2", "password SSID3"};    // more the one Wifi-AP to try
IPAddress IP (192,168,xxx,xxx);    // Static IP adress for flash over the air (OTA)
IPAddress GATEWAY (192,168,xxx,xxx);
IPAddress SUBNET(255,255,255,0);
#define MQTT_HOST IPAddress(192, 168, xxx, xxx)	// Mosquito broker on NAS
//const char MQTT_HOST[] = "test.mosquitto.org";
#define MQTT_PORT 1883

AsyncMqttClient mqttClient;
WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker mqttReconnectTimer;
Ticker wifiReconnectTimer;
Ticker refresh;
Ticker refreshTrend;

unsigned long lastMillis, lastSave, lastBlink, lastRGB;
int HourSet = 12, MinuteSet = 30;   // Default settings
int AP = 0;                         // for different wifi SSID's
int oldMinuteL, oldMinuteP;
bool statusBME;                     // Temp / Humid Sensor
bool lightOn, pumpOn, setTempChanged;
bool onboardLEDblink = 0;           // 0 when no WiFi connection or no MQTT connection
float_t actTemperature, actHumidity;

const int onboardLED = D0;
//D1 = SCL, D2 = SDA of I2C
const int oLight = D3;              // Relais for Grwo light
const int oHeater = D5;             // Relais for heater
const int oPump = D6;               // Relais for water pump

// Structure for all retain variables (EEPROM)
struct retainVar{
  int16_t lightOnTime;
  int16_t lightOffTime;
  int16_t pumpOnTime;
  int16_t pumpOffTime;
  int16_t tempSet;
  int16_t tempSetOld;
};
retainVar stRetain;         // Variable with structured data type

Adafruit_Si7021 sensor = Adafruit_Si7021();
RtcDS3231<TwoWire> Rtc(Wire);        // Real time clock

void setup() 
{
  Serial.begin(19200);

  pinMode(oPump, OUTPUT);
  pinMode(oLight, OUTPUT);
  pinMode(oHeater, OUTPUT);
  pinMode(onboardLED, OUTPUT);
  pumpOn = 1;             // On after reboot
  lightOn = 1;            // On after reboot

// Different handler
  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  
  connectToWifi();        // Tray to connect without waiting here

  sensor.begin();         // Temp & Humidity
  Rtc.Enable32kHzPin(false);
  Rtc.SetSquareWavePin(DS3231SquareWavePin_ModeNone); 

  EEPROM.begin(50);
  EEPROM.get(0, stRetain); 

  refresh.attach(5, mqttRefresh);       // Call function "mqttRefresh" every 5 sec.
  refreshTrend.attach(600, mqttTrend);  // Call puplish to database every 10 min.
  ArduinoOTA.begin();
  digitalWrite(oPump,1);
  digitalWrite(oLight,1);
}

void loop()
{
  ArduinoOTA.handle();
  lightCtrl();
  pumpCtrl();
  LEDctrl();

// When setTemp has changed, write to EEPROM after 5 Sec. 
  if (stRetain.tempSet != stRetain.tempSetOld){
    lastSave = millis();
    setTempChanged = 1;
    stRetain.tempSetOld = stRetain.tempSet;
  }
  if (setTempChanged == 1 && millis() > lastSave + 5000){
    save_In_EEPROM();
    setTempChanged = 0;
  }

  if (millis() > lastMillis + 1000){
    lastMillis = millis();
    RtcDateTime now = Rtc.GetDateTime();
    writeTime(now);
    autoLight(now);
    autoPump(now);
  }
 }

// Message from subscribed topic group "Plant/App/#"
void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  //Serial.println("Publish received.");
  // Shorten the payload to its lenght. Because of problems with chars after the payload from library
  char fixedPayload[len+1];   
  fixedPayload[len] = '\0';
  strncpy(fixedPayload,payload,len);

  if (len > 0){                 // is there a payload?
    if (strcmp(topic,"Plant/App/LightOn")==0)  appLight(atoi(fixedPayload));  
    if (strcmp(topic,"Plant/App/PumpOn")==0)   appPump(atoi(fixedPayload));
    if (strcmp(topic,"Plant/App/HourSet")==0)  HourSet = atoi(fixedPayload);
    if (strcmp(topic,"Plant/App/MinuteSet")==0) MinuteSet = atoi(fixedPayload);
    if (strcmp(topic,"Plant/App/TimeSet")==0)  timeSet(fixedPayload);
    if (strcmp(topic,"Plant/App/TempSet")==0)  stRetain.tempSet = atoi(fixedPayload) * 10;
    if (strcmp(topic,"Plant/App/Refresh")==0)  mqttRefresh();
    if (strcmp(topic,"Plant/App/SetTime")==0)  setTime(Rtc.GetDateTime());
  }
}

void autoLight(const RtcDateTime& dt){
  if(dt.Minute() != oldMinuteL){          // check for one time per minute
    oldMinuteL = dt.Minute();
    if (dt.Hour() * 100 + dt.Minute() == stRetain.lightOnTime){
      lightOn = 1;
    }
    if (dt.Hour() * 100 + dt.Minute() == stRetain.lightOffTime){
      lightOn = 0;
    }  
  }
}

void appLight(int payload){
  if (payload==1){
    lightOn = 1;
  }
  if (payload==0){
    lightOn = 0;
  }
}

void lightCtrl(){
  if (lightOn != digitalRead(oLight)){   // Status has changed
    if (lightOn == 1){
        digitalWrite(oLight,HIGH);
    }else{
        digitalWrite(oLight,LOW);
    }
    mqttRefresh();
  }
} 

void autoPump(const RtcDateTime& dt){
  if(dt.Minute() != oldMinuteP){        // check for one time per minute
    oldMinuteP = dt.Minute();
    if (dt.Hour() * 100 + dt.Minute() == stRetain.pumpOnTime){
      pumpOn = 1;
    }
    if (dt.Hour() * 100 + dt.Minute() == stRetain.pumpOffTime){
      pumpOn = 0;
    }  
  }
}

void appPump(int payload){
  if (payload==1){
    pumpOn = 1;
  }
  if (payload==0){
    pumpOn = 0;
  }
}

void pumpCtrl(){
  if (pumpOn != digitalRead(oPump)){   // Status has changed
    if (pumpOn == 1){
        digitalWrite(oPump,HIGH);
    }else{
        digitalWrite(oPump,LOW);
    }
    mqttRefresh();
  }
}

void timeSet(char* payload){
  if (strcmp(payload,"LightOn")==0) stRetain.lightOnTime = HourSet * 100 + MinuteSet;
  if (strcmp(payload,"LightOff")==0) stRetain.lightOffTime = HourSet * 100 + MinuteSet;
  if (strcmp(payload,"PumpOn")==0) stRetain.pumpOnTime = HourSet * 100 + MinuteSet;
  if (strcmp(payload,"PumpOff")==0) stRetain.pumpOffTime = HourSet * 100 + MinuteSet;
  save_In_EEPROM();
  mqttRefresh();
}

void writeTime(const RtcDateTime& dt){
char payload[32];
   sprintf(payload,"  %02u.%02u.%04u     %02u:%02u:%02u",
           dt.Day(),
           dt.Month(),
           dt.Year(),
           dt.Hour(),
           dt.Minute(),
           dt.Second());
  mqttClient.publish("Plant/ESP/ActDateTime", 0, false, payload);        
} 

// Send all 5 sec. actual values
void mqttRefresh(){
  char payload[6];
  readSensor();                                           // Call function and publish actual messurements 
  sprintf(payload, "%.1f", actTemperature);
  mqttClient.publish("Plant/ESP/Temp", 0, false, payload);
  sprintf(payload, "%.1f", actHumidity);
  mqttClient.publish("Plant/ESP/Humi", 0, false, payload);
  tempCtrl();                                               // Call function and publish actual status of Heating 
  itoa(digitalRead(oHeater), payload, 10);
  mqttClient.publish("Plant/ESP/HeaterIsOn", 0, false, payload);
  itoa(HourSet, payload, 10); 
  mqttClient.publish("Plant/ESP/HourSet", 0, false, payload);
  itoa(MinuteSet, payload, 10); 
  mqttClient.publish("Plant/ESP/MinuteSet", 0, false, payload);
  itoa(digitalRead(oLight), payload, 10); 
  mqttClient.publish("Plant/ESP/LightOn", 0, false, payload);
  itoa(digitalRead(oPump), payload, 10); 
  mqttClient.publish("Plant/ESP/PumpOn", 0, false, payload);
  sprintf(payload,"%02u:%02u", stRetain.lightOnTime/100, stRetain.lightOnTime % 100 );    // Stunden, Rest = Minuten
  mqttClient.publish("Plant/ESP/LightOnTime", 0, false, payload);
  sprintf(payload,"%02u:%02u", stRetain.lightOffTime/100, stRetain.lightOffTime % 100 );    // Stunden, Rest = Minuten
  mqttClient.publish("Plant/ESP/LightOffTime", 0, false, payload);
  sprintf(payload,"%02u:%02u", stRetain.pumpOnTime/100, stRetain.pumpOnTime % 100 );    // Stunden, Rest = Minuten
  mqttClient.publish("Plant/ESP/PumpOnTime", 0, false, payload);
  sprintf(payload,"%02u:%02u", stRetain.pumpOffTime/100, stRetain.pumpOffTime % 100 );    // Stunden, Rest = Minuten
  mqttClient.publish("Plant/ESP/PumpOffTime", 0, false, payload);
  itoa(stRetain.tempSet/10, payload, 10); 
  mqttClient.publish("Plant/ESP/TempSet", 0, false, payload);
}

void readSensor(){
  actTemperature = sensor.readTemperature();
  actHumidity = sensor.readHumidity(); 
}

void tempCtrl(){
  int actTemp = actTemperature * 10;
  if (actTemp <= stRetain.tempSet - 5 ){  // Heating starts 1.5 ° below setpoint
    digitalWrite(oHeater,HIGH); 
  }else{ 
    if (actTemp >= stRetain.tempSet + 5){ // Heating stops 1.5° above setpoint
      digitalWrite(oHeater,LOW); 
    }
  }
}

void mqttTrend(){
  char payload[12];
  sprintf(payload, "%.1f,%.1f", actTemperature, actHumidity);
  mqttClient.publish("Plant/Trend", 0, false, payload);                    
  //itoa(digitalRead(oHeater), payload, 10);
  //mqttClient.publish("Plant/ESP/HeaterIsOn", 0, false, payload);
}

void save_In_EEPROM(){
    EEPROM.put(0, stRetain);
    EEPROM.commit();
    Serial.println("In Eeprom gespeichert");
}

void LEDctrl(){
  if (onboardLEDblink == 1){
    if (millis() > lastBlink + 1000){
      lastBlink = millis();
      digitalWrite(onboardLED,!digitalRead(onboardLED));
    }       
  }else{
    digitalWrite(onboardLED, LOW);
  }
}

void setTime(const RtcDateTime& dt){
  RtcDateTime newTime(dt.Year(),dt.Month(),dt.Day(), HourSet, MinuteSet,0);  // Only h & m can set from App       
  //RtcDateTime newTime(2021,04,17,HourSet, MinuteSet,0);         // In case when the hole Date must be set
  Rtc.SetDateTime(newTime);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void onMqttConnect(bool sessionPresent) {
  onboardLEDblink = 1;
  // DW uint16_t packetIdSub = mqttClient.subscribe("Plant/App/#", 0); // Subscribe to all topics "Plant/App.."
  mqttClient.subscribe("Plant/App/#", 0); // Subscribe to all topics "Plant/App.."
  mqttRefresh();
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  onboardLEDblink = 0;
  if (WiFi.isConnected()) {
    mqttReconnectTimer.once(2, connectToMqtt);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.config(IP, GATEWAY, SUBNET);
  WiFi.begin(WIFI_SSID[AP], WIFI_PASSWORD[AP]);
}

void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  Serial.print("Connected to Wi-Fi: ");
  Serial.println(WiFi.SSID());
  connectToMqtt();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  Serial.println("Disconnected from Wi-Fi.");
  onboardLEDblink = 0;
  mqttReconnectTimer.detach(); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
  wifiReconnectTimer.once(2, connectToWifi);
  AP ++;
  if (AP > MAX_AP-1) AP = 0;
}