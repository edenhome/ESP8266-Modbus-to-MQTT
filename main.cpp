#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>


#include <Wire.h>
#include <NTPClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <LiquidCrystal_I2C.h>
#include "config.h"

#include <ModbusMaster.h> 
#define MAX485_DE  0 // D0
#define MAX485_RE  16  // D1
static uint8_t DPMAddr = 0x01; 
ModbusMaster node;  


#include <SoftwareSerial.h>
SoftwareSerial DPMSerial;

unsigned long startTime;


const char *ssid = SSID; //replace this with your wifi  name
const char *password = WIFI_PASSWORD; //replace with your wifi password
const char* mqttServer = MQTT_SERVER; //replace with your mqtt broker IP
const int mqttPort = 1883;
const char* mqttUser = MQTT_USER; //replace with your mqtt user name
const char* mqttPassword = MQTT_PASSWORD; //replace with your mqtt password 
const long utcOffsetInSeconds = 25200; // utc + 7 hour



#define WHOAMI "MainWatermeter"

int lcdColumns = 16;
int lcdRows = 2;
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);


WiFiClient wifiClient;
PubSubClient client(wifiClient);



// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);


void preTransmission(){                                                                              /* transmission program when triggered*/
  digitalWrite(MAX485_RE, 1);                                                                     /* put RE Pin to high*/
  digitalWrite(MAX485_DE, 1);                                                                     /* put DE Pin to high*/
  delay(1);                                                                                       // When both RE and DE Pin are high, converter is allow to transmit communication
  }


void postTransmission(){                                                                                   /* Reception program when triggered*/
  delay(3);                                                                                       // When both RE and DE Pin are low, converter is allow to receive communication
  digitalWrite(MAX485_RE, 0);                                                                     /* put RE Pin to low*/
  digitalWrite(MAX485_DE, 0);                                                                     /* put DE Pin to low*/
  }

void wifiSetup(const char *hostname) {
  // Set WIFI module to STA mode
  WiFi.mode(WIFI_STA);
  WiFi.hostname(hostname);

  // Connect
  WiFi.begin(ssid, password);

  // Wait for connection. ==> We will hang here in RUN mode forever without a WiFi!
  while (WiFi.status() != WL_CONNECTED) {
    delay(50);
  }
}

void reconnect(){
  while (!client.connected()) {
  if ( WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid, password);
      while (WiFi.status() != WL_CONNECTED) {
        lcd.clear();
        lcd.print(" WIFI error");  
        delay(500);
    }
  }

  if ( client.connect("ESP8266 Device", mqttUser, mqttPassword)){

  }
  else {
    lcd.clear();
    lcd.print(" MQTT error");
    delay( 5000 );
  }
  }
}




void OTA(){
  // ArduinoOTA setup
  ArduinoOTA.setHostname(WHOAMI);  // Set OTA host name
  ArduinoOTA.setPassword("XXXX");  // Set OTA password
  ArduinoOTA.begin();               // start OTA scan
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

}




void setup(){
  Wire.begin();
  // Start WiFI
  wifiSetup(WHOAMI);
  OTA();
  timeClient.begin(); // Start Time client
  client.setServer(mqttServer, 1883 ); //default port for mqtt is 1883
  lcd.init(); 
  lcd.backlight();
  lcd.setCursor(0, 0);


  Serial.begin(115200);   
  DPMSerial.begin(9600,SWSERIAL_8N1,12,14);   // R0 = GPIO4 = D2,,,, DI = GPIO0 = D3

  pinMode(MAX485_RE, OUTPUT); 
  pinMode(MAX485_DE, OUTPUT);
  digitalWrite(MAX485_RE, 0);
  digitalWrite(MAX485_DE, 0);
  node.preTransmission(preTransmission); 
  node.postTransmission(postTransmission);
  node.begin(DPMAddr,DPMSerial);  
  
  // Set loop timer
  startTime = millis();
}


// Convert BCD to Integer
uint32_t bcd_to_ui32(uint64_t bcd){
    uint64_t mask = 0x000f;
    uint64_t pwr = 1;
    uint64_t i = (bcd & mask);
    while (bcd = (bcd >> 4))    {
        pwr *= 10;
        i += (bcd & mask) * pwr;
    }
  return (uint32_t)i;
  }

void read(){
  uint16_t result;
  result = node.readHoldingRegisters(0x0000, 4);
  if (result == node.ku8MBSuccess){
    
    float m3 = bcd_to_ui32(node.getResponseBuffer(0)) + (bcd_to_ui32(node.getResponseBuffer(1)) * 0.0001);
    Serial.println(m3);
    //lcd.setCursor(0, 0);
    lcd.clear();
    lcd.print("Water ");
    lcd.print(m3);
    lcd.print("m3");
    float lh = bcd_to_ui32(node.getResponseBuffer(2)) + (bcd_to_ui32(node.getResponseBuffer(3)) * 0.0001);
    
    lcd.setCursor(0, 1);
    lcd.print("Flow ");
    lcd.print(lh);
    lcd.print("m3/h");
    Serial.println(lh);
    
    yield();
    
  } 
  else{
    lcd.clear();
    lcd.print(" Modbus error");
  }
}



void mqttpost(){

  timeClient.update();
  unsigned long epochTime = timeClient.getEpochTime();
  struct tm *ptm = gmtime ((time_t *)&epochTime);
  int currentYear = ptm->tm_year+1900;
  int currentMonth = ptm->tm_mon+1;
  int monthDay = ptm->tm_mday;
  String currentDate = String(currentYear) + "-" + String(currentMonth) + "-" + String(monthDay);

  StaticJsonDocument<200> doc;

  doc["t_stamp"] = currentDate + " " + timeClient.getFormattedTime();
  doc["Consumption"] =  bcd_to_ui32(node.getResponseBuffer(0)) + bcd_to_ui32(node.getResponseBuffer(1)) * 0.0001;
  doc["Flow"] = bcd_to_ui32(node.getResponseBuffer(2)) + bcd_to_ui32(node.getResponseBuffer(3)) * 0.0001;
  doc["local IP"] = WiFi.localIP().toString();
  doc["wifi channel"] = WiFi.channel();
  doc["RSSI"] =WiFi.RSSI();

  String payload = "";
  serializeJson(doc, payload);
  char charBuff[payload.length() + 1];
  payload.toCharArray(charBuff,payload.length() + 1);
            
  String topic = "Watermeter/";
  topic +=  "Main";
  const char* msg = topic.c_str();
           
  client.publish( msg , charBuff);

}



void loop(){
  //Scan();
  ArduinoOTA.handle();

  if(!client.connected()){
    reconnect();
  }
     
  if(millis() - startTime >= 5000){
    read(); // 5 seconds have elapsed. ... do something interesting ...
    mqttpost();
    startTime = millis();
  }
  
  
    
}     




