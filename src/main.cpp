// EmonLibrary examples openenergymonitor.org, Licence GNU GPL V3
// clone to arduino lib C:\Users\hunter\Documents\Arduino\libraries\EmonLib
// https://github.com/Savjee/EmonLib-esp32

#include "EmonLib.h"             // Include Emon Library

#include <WiFi.h>
#include <PubSubClient.h>
#include <WebServer.h>
//#include "HTTPUpdateServer.h"
//using WebServerClass = WebServer;
//using HTTPUpdateServerClass = HTTPUpdateServer;
#include <AutoConnect.h>
#include "PageBuilder.h"
#include <Ticker.h>
#include <ArduinoJson.h>
//#include "pages.h"

#define STACK_SIZE 10240
Ticker mqttTicker;
Ticker emonTicker;
Ticker wdtTicker;

WebServer server;
AutoConnectConfig config;
WiFiClient espClient;
PubSubClient mqtt(espClient);
AutoConnect portal(server);

#define MQTT_MAX_PACKET_SIZE 1280000//redefine it with the new value
#define CONNECT_TIMEOUT_MS 6000
#define PUBLISH_TIMEOUT_MS 6000 //set from 500 to 1000 to see if its better for discons/hr
#define PING_TIMEOUT_MS 6000 //set from 500 to 1000 to see if its better for discons/hr
#define SUBACK_TIMEOUT_MS 6000 //set from 500 to 1000 to see if its better for discons/hr
void msgReceived(char* topic, byte* payload, unsigned int len);
char* mqtt_server = "192.168.3.1";

#define CREDENTIAL_OFFSET 64
uint8_t opcode; // register
int I_HIGH    = 19;
int I_LOW     = 9;
int I_REPEATS = 5;
int I_PIN = 25;


EnergyMonitor emon1;             // Create an instance

double realPower       = 0;
double apparentPower   = 0;
double powerFactor     = 0;
double supplyVoltage   = 0;
double Irms            = 0;

int h_counter         = 0;
int l_counter         = 0;
int b                 = 0;

byte data[10];
bool pin_9            = true;

long lastReconnectAttempt = 0;
StaticJsonDocument<256> doc;


void wdtTickerCheck();
void callback(char* topic, byte* message, unsigned int length);
boolean reconnect();
void emonTick( void * pvParameters );
void mqttTick();

  
void setup(){ 

  Serial.begin(115200);
  Serial.println("Starting...");

  wdtTicker.attach(60, wdtTickerCheck);

   
  pinMode(I_PIN, OUTPUT);
  digitalWrite(I_PIN, pin_9);

  emon1.voltage(32, 213.5, 1.7);  // Voltage: input pin, calibration, phase_shift 283 1.7 835
  emon1.current(33, 32);       // Current: input pin, calibration. 28 130


  config.apid = "POWER-ap";
  config.psk = "1234567890";
  config.ota = AC_OTA_BUILTIN;
  config.title = "POWER Sensor";
  config.hostName = "POWERSensor";
  config.ticker = true;
//  config.tickerPort = LED_BUILTIN;
  config.tickerOn = HIGH;
  config.boundaryOffset = 2000;
  config.autoReconnect = true;
  config.reconnectInterval = 3;   // Seek interval time is 180[s].
  
  portal.config(config);

  Serial.println("portal.begin...");
  portal.begin();

  mqtt.setServer(mqtt_server, 1883);
  mqtt.setCallback(callback);

  lastReconnectAttempt = 0;
  
  mqttTicker.attach(10, mqttTick);
  Serial.println("loop...");

  static uint8_t ucParameterToPass;
  TaskHandle_t xHandle = NULL;
  xTaskCreate( emonTick, "NAME", STACK_SIZE, &ucParameterToPass, tskIDLE_PRIORITY, &xHandle );
 
}


void loop(){


  if (WiFi.status() == WL_CONNECTED) {
    // Here to do when WiFi is connected.
  }
  else {
    Serial.println("wifi disconnected...");
  }

  portal.handleClient();
  
  if (!mqtt.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      reconnect();
    }
  }

  mqtt.loop();

}

void mqttTick(){
  doc["realPower"] = realPower;
  doc["powerFactor"] = powerFactor;
  doc["Irms"] = Irms;
  doc["supplyVoltage"] = supplyVoltage;
  
  doc["pin"] = pin_9;

  realPower = 0;
  powerFactor = 0;
  Irms = 0;

  char buffer[256];
  size_t n = serializeJson(doc, buffer);

  mqtt.publish("POWER-Sensor/output", buffer, n);
  Serial.println("POWER-Sensor/output");
}


void emonTick( void * pvParameters ){
 
  for( ;; ){
    emon1.calcVI(50,2000);         // Calculate all. No.of half wavelengths (crossings), time-out
    emon1.serialprint();           // Print out all variables (realpower, apparent power, Vrms, Irms, power factor)
  
    //apparentPower   = emon1.apparentPower;    //extract Apparent Power into variable
      
    supplyVoltage   = emon1.Vrms;             //extract Vrms into Variable
  
    if(emon1.powerFactor > 0){
      powerFactor = powerFactor + emon1.powerFactor;        //extract Power Factor into Variable
    }
    if(emon1.realPower > 0){
      realPower = realPower + emon1.realPower;        //extract Real Power into variable
    }
    if(emon1.Irms > 0){
      Irms = (Irms + emon1.Irms) / 2;
    }
  
    if(emon1.Irms >= I_HIGH && pin_9 == true){
      h_counter++;
      if(h_counter > I_REPEATS){
        h_counter = 7;
      }
    }else{
      h_counter = 0;
    }
  
    if( (emon1.Irms + I_LOW) < I_HIGH && pin_9 == false){
      l_counter++;
      if(l_counter > I_REPEATS){
        l_counter = 7;
      }
    }else{
      l_counter = 0;
    }
  
    if(h_counter > I_REPEATS && pin_9 == true ){
      pin_9 = false;
      h_counter = 0;
      digitalWrite(I_PIN, pin_9);
    }
  
    if(l_counter > I_REPEATS && pin_9 == false ){
      pin_9 = true;
      l_counter = 0;
      digitalWrite(I_PIN, pin_9);
    }
    vTaskDelay(100);
    Serial.println("emonTick");
  }
}


boolean reconnect() {
    if (mqtt.connect("POWER-Sensor")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      mqtt.publish("POWER-Sensor/connected", "hello world");
      // ... and resubscribe
      mqtt.subscribe("POWER-Sensor/input");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" try again in 5 seconds");
    }
    return mqtt.connected();
}



void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  char myNewArray[length+1];
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    myNewArray[i] = (char)message[i];
  }
  myNewArray[length] = NULL;
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
//  if (String(topic) == "POWER-Sensor/input") {
//    myObject = JSON.parse(myNewArray);
//    Serial.print("Changing output to ");
//    mainText = String(myNewArray);
//    Serial.print((const char*) myObject["pass"]);
//    Serial.println((const char*) myObject["next"]);
//  }
  
}


int wdtTickerCount = 0;
void wdtTickerCheck(){
  if (WiFi.status() != WL_CONNECTED || !mqtt.connected()) {
    wdtTickerCount++;
  }

  if(wdtTickerCount > 5){
    ESP.restart();
  }
}
