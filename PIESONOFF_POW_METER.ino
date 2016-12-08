#include <ESP8266WiFi.h>
#include <MicroGear.h>

#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include <WiFiManager.h>

#include "power.h"

#define LEDWIFI

#ifdef LEDWIFI
  #include <Ticker.h>
  Ticker ticker;
#endif

#define RELAYPIN 12                     // D6
#define LEDPIN 15                       // D8
#define BUTTONPIN 0                     // D3
#define EEPROM_STATE_ADDRESS 128

#define CHIPID "dev"
#define DEVICENAME "sonoff" CHIPID
#define APDEVICENAME "SONOFF-" CHIPID

#define APPID   "<APPID>"       // e.g. "MySwitch"
#define KEY     "<KEY>"      // e.g. "4DPanXKaSdb2VrT"
#define SECRET  "<SECRET>"   // e.g. "ZgrXbHsaVp7TI8xW5oEcAqvY3"
#define ALIAS   "" DEVICENAME
#define TOPIC_STATE  "/" ALIAS "/state"		// ON/OFF Relay
#define TOPIC_DATA  "/" ALIAS "/data"
#define FEEDID  "<FEEDID>"			// e.g. "meter"
#define FEEDKEY  "<FEEDKEY>"			// e.g. "shjgre212h4jh42hjb4"

WiFiClient client;

char state = 0;
char stateOutdated = 0;
char buff[16];

String datafeed;
unsigned long feedMillis = 0;
const long intervalFeed = 15000;

// Button
bool buttonPress = false;
unsigned long timer = 0;
unsigned long timeoutConnection = 0;
int interruptPress = 3000; // millisec.

// Configure WiFi
bool resetMode = false;
int timeoutDisconnection = 90000;

// LED WiFi
unsigned long timestampLED = 0;
int stateLED = HIGH;

WiFiManager wifiManager;
MicroGear microgear(client);

// METER
ESP8266PowerClass powerMeter;
unsigned long previousMillis = 0;
const long interval = 2000;

/*****************************************************************************************/

void sendState() {
  if (state==0)
    microgear.publish(TOPIC_STATE,"0");
  else
    microgear.publish(TOPIC_STATE,"1");
  Serial.println("send state..");
  stateOutdated = 0;
}

void updateIO() {
  if (state >= 1) {
    digitalWrite(RELAYPIN, HIGH);
    #ifdef LEDRELAY
      digitalWrite(LEDPIN, LOW);
    #endif
    
    EEPROM.write(EEPROM_STATE_ADDRESS, 1);
    EEPROM.commit();
  }
  else {
    state = 0;
    digitalWrite(RELAYPIN, LOW);
    #ifdef LEDRELAY
      digitalWrite(LEDPIN, HIGH);
    #endif

    EEPROM.write(EEPROM_STATE_ADDRESS, 0);
    EEPROM.commit();
  }
}

void onFeedInfo(char *attribute, uint8_t* msg, unsigned int msglen) {
    Serial.print("Feed info --> ");
    for (int i=0; i<msglen; i++)
        Serial.print((char)msg[i]);
    Serial.println();
}

void onFeedError(char *attribute, uint8_t* msg, unsigned int msglen) {
    Serial.print("Feed error --> ");
    for (int i=0; i<msglen; i++)
        Serial.print((char)msg[i]);
    Serial.println();
}

void onMsghandler(char *topic, uint8_t* msg, unsigned int msglen) {
  char m = *(char *)msg;
  
  Serial.print("Incoming message --> ");
  msg[msglen] = '\0';
  Serial.println((char *)msg);
  
  if (m == '0' || m == '1') {
    state = m=='0'?0:1;
    EEPROM.write(EEPROM_STATE_ADDRESS, state);
    EEPROM.commit();
    updateIO();
  }
  if (m == '0' || m == '1' || m == '?') stateOutdated = 1;
}

void tick()
{
  #ifdef LEDPIN
    int stateled = digitalRead(LEDPIN);
    digitalWrite(LEDPIN, !stateled);
  #endif
}

void onConnected(char *attribute, uint8_t* msg, unsigned int msglen) {
  Serial.println("Connected to NETPIE...");
  stateOutdated = 1;
  timeoutConnection=0;
  timestampLED = millis();
  #ifdef LEDPIN
    digitalWrite(LEDPIN, HIGH);
  #endif
  #ifdef LEDWIFI
    ticker.detach();  
    digitalWrite(LEDPIN, LOW);
    stateLED = digitalRead(LEDPIN);
  #endif
}

void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("*PM: Entered config mode");
  Serial.print("*PM: ");
  Serial.println(myWiFiManager->getConfigPortalSSID());
  #ifdef LEDWIFI
    ticker.attach(0.25, tick);
  #endif
}

void initWiFi(){
  wifiManager.setTimeout(180);
  if(resetMode) wifiManager.resetSettings();
  wifiManager.setAPCallback(configModeCallback);
  resetMode = false;
  
  if(!wifiManager.autoConnect(APDEVICENAME)) {
    Serial.println("failed to connect and hit timeout");
    delay(5000);
  }
}

void onButtonPress(){
  int buttonState = digitalRead(BUTTONPIN);
  if (buttonState == LOW) {
    if (buttonPress==false) {
      timer = millis();
      buttonPress = true;
      Serial.println("*PM: Button press.");
    }
  } else {
    if ((millis()-timer)>=interruptPress && buttonPress){
      Serial.println("*PM: Device reset config WiFi... ");
      resetMode = true;
      wifiManager.setRestart(true);
    }else if (buttonPress){
      Serial.println("*PM: Device change state... ");
      
      if (state == 1) state = 0;
      else state = 1;
      
      updateIO();
      stateOutdated = 1;
    }
    buttonPress = false;
  }
}

void ledWiFi(){
  if((millis()-timestampLED)>50 && stateLED == LOW){
    timestampLED = millis();
    digitalWrite(LEDPIN, HIGH);
    stateLED = HIGH;
  }else if((millis()-timestampLED)>5000 && stateLED == HIGH){
    timestampLED = millis();
    digitalWrite(LEDPIN, LOW);
    stateLED = LOW;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting...");
  EEPROM.begin(512);
  
  pinMode(BUTTONPIN, INPUT);
  pinMode(RELAYPIN, OUTPUT);
  #ifdef LEDPIN
    pinMode(LEDPIN, OUTPUT);
  #endif
  
  attachInterrupt(BUTTONPIN, onButtonPress, CHANGE);
  
  powerMeter.setPowerParam(12.65801022, 0.0);
  powerMeter.setCurrentParam(19.52, -85.9);
  powerMeter.setVoltageParam(0.45039823, 0.0);

  powerMeter.enableMeasurePower();
  powerMeter.selectMeasureCurrentOrVoltage(VOLTAGE);
  powerMeter.startMeasure();
  
  state = EEPROM.read(EEPROM_STATE_ADDRESS)==1?1:0;
  updateIO();
  
  initWiFi();
  
  microgear.on(MESSAGE,onMsghandler);
  microgear.on(CONNECTED,onConnected);
  microgear.on(INFO,onFeedInfo);
  microgear.on(ERROR,onFeedError);
  
  microgear.init(KEY,SECRET,ALIAS);
  if(WiFi.status()==WL_CONNECTED){
    #ifdef LEDWIFI
      ticker.attach(0.05, tick);
    #endif
    microgear.connect(APPID);
  }
}

void loop() {
  unsigned long currentMillis = millis();

  if(resetMode) initWiFi();
  
  if(WiFi.status()!=WL_CONNECTED){
      Serial.println("disconnect WiFi.");
      initWiFi();
  }else{
    if (microgear.connected()) {
      if (stateOutdated) sendState();
      #ifdef LEDWIFI
        ledWiFi();
      #endif
      microgear.loop();
      
      if(currentMillis - previousMillis >= interval) {
          previousMillis = currentMillis;   
          
          //  W = Watt, V = Volt, A = Amp
          //  P(W) = E(V) x I(A)
          //  E(V) = P(W) / I(A)
          //  I(A) = P(W) / E(V)
          //  PF = W/(VA) 
          double activepower = powerMeter.getPower();
          double voltage = powerMeter.getVoltage();
          double current = (powerMeter.getPower()/powerMeter.getVoltage())*sqrt(3);
          if(isnan(current)) current = 0.00;
          double apparentpower = powerMeter.getPower()*((powerMeter.getPower()/powerMeter.getVoltage())*sqrt(3));
          if(isnan(apparentpower)) apparentpower = 0.00;
          double powerfactor = powerMeter.getPower()/(powerMeter.getVoltage()*(powerMeter.getPower()/powerMeter.getVoltage())*sqrt(3));
          if(isnan(powerfactor)) powerfactor = 0.00;

          datafeed = "{\"activepower\":";
          datafeed += activepower;
          datafeed += ",\"voltage\":";
          datafeed += voltage;
          datafeed += ",\"current\":";
          datafeed += current;
          datafeed += ",\"apparentpower\":";
          datafeed += apparentpower;
          datafeed += ",\"powerfactor\":";
          datafeed += powerfactor;
          if(state==1) datafeed += ",\"state\":1";
          else  datafeed += ",\"state\":0";
          datafeed += "}";

          Serial.print("[HLW] Active Power (W)      : "); Serial.println(activepower);
          Serial.print("[HLW] Voltage (V)           : "); Serial.println(voltage);
          Serial.print("[HLW] Current (A)           : "); Serial.println(current);
          Serial.print("[HLW] Apparent Power (VA)   : "); Serial.println(apparentpower);
          Serial.print("[HLW] Power Factor (%)      : "); Serial.println(powerfactor);

          Serial.println();
          //Serial.println(datafeed);
          //Serial.println();
          //microgear.publish(TOPIC_DATA,datafeed);
      }
      
      if(currentMillis - feedMillis >= intervalFeed) {
          feedMillis = currentMillis;   
          Serial.println("FEED --->");
          Serial.println(datafeed);
          microgear.writeFeed(FEEDID,datafeed,FEEDKEY);
          Serial.println("<---");
      }
      
    }
    else {
      Serial.println("connection lost, reconnect...");
      if(WiFi.status()==WL_CONNECTED){
        #ifdef LEDWIFI
          ticker.attach(0.05, tick);
        #endif
        microgear.connect(APPID);
      }
      if(timeoutConnection==0){
        timeoutConnection = millis();
      }
      if(millis()-timeoutConnection>=timeoutDisconnection){
        if(WiFi.status()==WL_CONNECTED){
          WiFi.disconnect();
        }
        timeoutConnection=0;
        initWiFi();
      }
    }
  }
}

