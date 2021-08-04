#include "SSD1306.h"
#include <SPI.h>
#include <LoRa.h>
#include <ArduinoJson.h>
#include  <Chrono.h>

// set up Chrono
Chrono sendLora(Chrono::SECONDS); 
Chrono forwardIrrigation(Chrono::SECONDS);
Chrono waterFlow(Chrono::SECONDS); 

// Display Variables
String displaymessages[5] = {"","","","",""};

// Display
SSD1306 display(0x3c, 4, 15);

// LoRa Pins
#define SCK     5    // GPIO5  -- SX127x's SCK
#define MISO    19   // GPIO19 -- SX127x's MISO
#define MOSI    27   // GPIO27 -- SX127x's MOSI
#define SS      18   // GPIO18 -- SX127x's CS
#define RST     14   // GPIO14 -- SX127x's RESET
#define DI00    26   // GPIO26 -- SX127x's IRQ(Interrupt Request)
#define BAND    868E6  //you can set band here directly,e.g. 868E6,915E6
#define PABOOST true

// LoRa Variables
typedef struct {
  byte src_addr;
  char msg[255];
  unsigned long count;
 } _l_packet;
_l_packet pkt;
boolean gotpacket;
int rssi;
bool loraSent = false;

// All eight valve Pins: 25, 22, 12, 23 , 13, 2 , 21, 17

// set up Relays
int numValves = 6;
int valve[] = {25,22,12,23,13,2};
//int enabledValves[] = {true,true,true,true,true,false};
//int irrigationLength[] = {120,120,120,120,60,120};
int enabledValves[] = {false,true,true,false,true,false};
int irrigationLength[] = {120,120,120,120,120,120};
int activeValve = numValves -2; // this is a hack to make the first button press work as expected. TODO
bool nextValve = false;
bool irrigate = false;
bool irrigateOld = false;
int currentIrrigationLength = 0;
String valveStateStr;

// set up Water Flow Sensor
#define FLOWSENSOR  36
float calibrationFactor = 7.34;
volatile byte pulseCount;
byte pulse1Sec = 0;
float flowRate;
float previousFlowRate = 0;
unsigned int flowMilliLitres;
unsigned long totalMilliLitres;
long previousMillis = 0;

// set up button
#define BUTTONPIN 0
bool buttonPressed = false;

// Forward declarations
void initDisplay();
void displayMsg(String msg1, String msg2 = "", String msg3 = "", String msg4 = "");
void IRAM_ATTR pulseCounter();
void checkWaterFlow();
void initLora();
void onReceive(int);
void sendLoraMsg();


void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("LoRa Receiver");

  // set up the display
  initDisplay();
  
  // set up LORA
  gotpacket = false;
  pkt.src_addr = 0;
  pkt.count = 0;
  initLora();
  LoRa.receive();

  // set up flow sensor
  pinMode(FLOWSENSOR, INPUT_PULLUP);
  pulseCount = 0;
  flowRate = 0.0;
  flowMilliLitres = 0;
  totalMilliLitres = 0;
  previousMillis = 0;
  attachInterrupt(digitalPinToInterrupt(FLOWSENSOR), pulseCounter, FALLING);

  // set up relays
  for (int i = 0; i < numValves; i++){
    pinMode(valve[i], OUTPUT);
  }

  // initialize the pushbutton pin as an input:
  pinMode(BUTTONPIN, INPUT);
  
  // ready to go
  Serial.println("ready");
  displaymessages[0] = "ready";
  
  // send initial lora message
  sendLoraMsg();  
}

void loop() {

  // handle button press
  if(digitalRead(BUTTONPIN) == false && buttonPressed == false) {
    irrigate = !irrigate;
    if(irrigate) nextValve = true;
    buttonPressed = true;
    delay(200);
  }
  else if(digitalRead(BUTTONPIN) == true && buttonPressed == true) {
    buttonPressed = false;
    delay(200);
  }

  // check for Chrono intervals
  if (waterFlow.hasPassed(1)) {
    waterFlow.restart(); 
    checkWaterFlow();
  }
  if (sendLora.hasPassed(10)) {  
    sendLora.restart();   
    loraSent = false;
  }

  // Irrigation is ON - cycle through enabledValves and switch at intervals set in irrigationLength
  if(irrigate){
    if(nextValve){
      Serial.println("nextValve");
      currentIrrigationLength = 0;
      nextValve = false;
    }
    if(forwardIrrigation.hasPassed(currentIrrigationLength)){
      forwardIrrigation.restart();
      valveStateStr = "valve " + String(activeValve) + " (" + String(valve[activeValve]) + ") closed";       
      Serial.println(valveStateStr); 
      digitalWrite(valve[activeValve], LOW);
      activeValve++;
      if(activeValve>=numValves){
        activeValve = 0;
      }       
      if(enabledValves[activeValve]){
        digitalWrite(valve[activeValve], HIGH);
        valveStateStr = "valve " + String(activeValve) + " (" + String(valve[activeValve]) + ") open";
        Serial.println(valveStateStr); 
        displaymessages[0] = valveStateStr;
        sendLoraMsg();
        currentIrrigationLength = irrigationLength[activeValve];
      }
      else {
        valveStateStr = "valve " + String(activeValve) + " (" + String(valve[activeValve]) + ") disabled";
        Serial.println(valveStateStr); 
        displaymessages[0] = valveStateStr;
        currentIrrigationLength = 0;
      }               
    }  
  }
     
  // try to parse LoRa packet
  if (gotpacket){
    gotpacket = false;    
    // received a packet
    Serial.println("packet:");
    //Serial.println(pkt.msg);

    String packet = "";
    String logMsg = "";
    
    // set up json
    packet = "{" + String(pkt.msg);
    Serial.println(packet);
      
    DynamicJsonDocument recMessage(1024);
    DeserializationError error = deserializeJson(recMessage, packet);
    if (error) {
      Serial.println("error:");
      Serial.println(error.c_str());
      return;
    }
    
    serializeJson(recMessage, Serial);
    irrigate = (bool) recMessage["irrigate"];
    nextValve = (bool) recMessage["nextValve"];
    
    if(recMessage["enabledValves"]){
      int i=0;
      JsonArray array = recMessage["enabledValves"].as<JsonArray>();
      for(JsonVariant v : array) {
          enabledValves[i] = v.as<bool>();
          i++;
          if(i>numValves)Serial.println("ERROR - Too many Valves!!!");
      }      
    }
    // print RSSI of packet
    Serial.print("' with RSSI ");
    rssi = LoRa.packetRssi();
    Serial.println(rssi);
  }

  // send LoRa message when irrigation is stopped
  if(irrigate != irrigateOld){
    Serial.println("irrigatechange");      
    irrigateOld = irrigate;
    valveStateStr = "irrigation ";
    if(irrigate){
      valveStateStr +="started";
      nextValve = true;
    }
    else{
      valveStateStr +="stopped";
      valveStateStr = "valve " + String(activeValve) + " (" + String(valve[activeValve]) + ") closed";       
      Serial.println(valveStateStr); 
      digitalWrite(valve[activeValve], LOW);      
      activeValve--;
      if(activeValve<0) activeValve = numValves-1;
    }
    Serial.println(valveStateStr);
    displaymessages[0] = valveStateStr+"!!!";
    if(!irrigate){sendLoraMsg();}
  }
  statusDisplay();
  yield();
}

// Display Functions
void statusDisplay(){
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  for (int i = 0; i<=3; i++){
    display.drawString(1 , 12*i , displaymessages[i]);
  }
  display.drawStringMaxWidth(0 , 48 , 128, displaymessages[4]);
  display.display();
}
void initDisplay(){
     pinMode(16,OUTPUT);
     pinMode(25,OUTPUT);
     digitalWrite(16, LOW);    // set GPIO16 low to reset OLED
     delay(50); 
     digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 in high
     display.init();
     display.flipScreenVertically();  
     display.setFont(ArialMT_Plain_10);
     delay(1500);
     display.clear();
}

// Water Flow Meter Functions
void IRAM_ATTR pulseCounter(){
  pulseCount++;
}
void checkWaterFlow(){
    
    pulse1Sec = pulseCount;
    pulseCount = 0;

    // Because this loop may not complete in exactly 1 second intervals we calculate
    // the number of milliseconds that have passed since the last execution and use
    // that to scale the output. We also apply the calibrationFactor to scale the output
    // based on the number of pulses per second per units of measure (litres/minute in
    // this case) coming from the sensor.
    flowRate = ((1000.0 / (millis() - previousMillis)) * pulse1Sec) / calibrationFactor;
    previousMillis = millis();

    // Divide the flow rate in litres/minute by 60 to determine how many litres have
    // passed through the sensor in this 1 second interval, then multiply by 1000 to
    // convert to millilitres.
    flowMilliLitres = (flowRate / 60) * 1000;

    // Add the millilitres passed in this second to the cumulative total
    totalMilliLitres += flowMilliLitres;
    
    // Print the flow rate for this second in litres / minute
    String waterFlowMsg = String(flowRate) + "L/min - " + String(totalMilliLitres) + "mL / " + String(totalMilliLitres / 1000) + "L";
    displaymessages[0] = valveStateStr;
    displaymessages[1] = waterFlowMsg;
    displaymessages[2] = "RSSI: "+String(rssi);
    Serial.println(waterFlowMsg);
    if(((flowRate >0 && flowRate != previousFlowRate) || (previousFlowRate > 0 && flowRate == 0)) && loraSent == false){
      // send lora
      sendLoraMsg();
      Serial.println("flow:"+String(flowRate));
      previousFlowRate = flowRate;
  }
}

// LoRa Functions
void initLora(){
  SPI.begin(SCK,MISO,MOSI,SS);
  LoRa.setPins(SS,RST,DI00);
  displaymessages[0] = "Initialising LoRa module....";
  while (!LoRa.begin(BAND)){
  }
  displaymessages[0] = "LoRa Init success!";
  statusDisplay(); 
  delay(1000);
  LoRa.onReceive(onReceive);
}
void onReceive(int packetSize){
  LoRa.readBytes((uint8_t *)&pkt,packetSize); 
  gotpacket = true;
  pkt.msg[packetSize-1]='\0';
  pkt.count++;
  rssi = LoRa.packetRssi(); 
}
void sendLoraMsg(){
    // send lora
    DynamicJsonDocument sendMessage(1024);
    sendMessage["rate"] = String(flowRate);
    sendMessage["total"] = String(totalMilliLitres / 1000);
    sendMessage["rssi"] = rssi;
    sendMessage["id"] = 1;
    sendMessage["activeValve"] = activeValve;
    JsonArray enabledValvesData = sendMessage.createNestedArray("enabledValves");
    JsonArray irrigationLengthData = sendMessage.createNestedArray("irrigationLength");
    for (int i = 0; i < numValves; i++) {
      enabledValvesData.add(enabledValves[i]);
      irrigationLengthData.add(irrigationLength[i]); 
    }
    sendMessage["irrigate"] = irrigate;
    
    // Send Packet
    String sMessage;
    serializeJson(sendMessage, sMessage);
    LoRa.setTxPower(7);
    LoRa.beginPacket();
    LoRa.print(sMessage);
    LoRa.endPacket();
    Serial.println("Lora Sent");
    Serial.println(sMessage);
    loraSent = true;
    LoRa.receive();

}
