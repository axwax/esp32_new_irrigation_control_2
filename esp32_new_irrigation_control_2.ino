#include "SSD1306.h"
#include <SPI.h>
#include <LoRa.h>
#include <ArduinoJson.h>
#include  <Chrono.h>
//Metro sendLora = Metro(10000);
Chrono sendLora(Chrono::SECONDS); 
Chrono forwardIrrigation(Chrono::SECONDS);


// set up OLED
#define OLED_SDA    4
#define OLED_SCL    15
#define OLED_RST    16
SSD1306 display(0x3c, OLED_SDA, OLED_SCL);

// set up LORA
#define LORA_BAND    868
#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     14   // GPIO14 -- SX1278's RESET
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)
int rssi;
bool loraSent = false;

// All eight valve Pins: 25, 22, 12, 23 , 13, 2 , 21, 17

// set up Relays
int numValves = 6;
int valve[] = {25,22,12,23,13,2};
int enabledValves[] = {true,true,true,true,true,false};
int irrigationLength[] = {120,120,120,120,60,120};

int activeValve = 6;
bool irrigate = true;
bool irrigateOld = false;
int currentIrrigationLength = 0;
String valveStateStr;

// set up Water Flow Sensor
#define FLOWSENSOR  36
Chrono waterFlow(Chrono::SECONDS); 
float calibrationFactor = 7.31;
volatile byte pulseCount;
byte pulse1Sec = 0;
float flowRate;
float previousFlowRate = 0;
unsigned int flowMilliLitres;
unsigned long totalMilliLitres;
long previousMillis = 0;

// Forward declarations
void initDisplay();
void displayMsg(String msg1, String msg2 = "", String msg3 = "", String msg4 = "");
void IRAM_ATTR pulseCounter();
void checkWaterFlow();
void axdelay(int delaylength);
void sendLoraMsg();


void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("LoRa Receiver");

  // set up the display
  initDisplay();
  
  // set up LORA
  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DI0);
  if (!LoRa.begin(LORA_BAND * 1E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

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

  // ready to go
  Serial.println("ready");
  displayMsg("ready");  
}

void loop() {

  // check for Chrono intervals
  if (waterFlow.hasPassed(1)) {
    waterFlow.restart(); 
    checkWaterFlow();
  }

  if (sendLora.hasPassed(10)) {  
    sendLora.restart();   
    loraSent = false;
  }


  if(irrigate){
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
        displayMsg(valveStateStr);
        sendLoraMsg();
        currentIrrigationLength = irrigationLength[activeValve];
      }
      else {
        valveStateStr = "valve " + String(activeValve) + " (" + String(valve[activeValve]) + ") disabled";
        Serial.println(valveStateStr); 
        displayMsg(valveStateStr);
        currentIrrigationLength = 0;
      }               
    }  
  }
     

  
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    Serial.println("packet:");
    Serial.println(packetSize);

    String packet = "";
    String logMsg = "";
    
    // set up json
    char json[500];
    json[0] = '\0';

        
    for (int i = 0; i < packetSize; i++) {
      packet += (char)LoRa.read();
      byte hi = strlen(json);
      json[hi] = (char)LoRa.read();
      json[hi + 1] = '\0';      
    }
    Serial.println(packet);
      
    DynamicJsonDocument recMessage(1024);
    DeserializationError error = deserializeJson(recMessage, json);
    if (error) {
      Serial.println("error:");
      Serial.println(error.c_str());
      return;
    }
    
    serializeJson(recMessage, Serial);
    if(recMessage["irrigate"]){
      irrigate = (bool) recMessage["irrigate"];
    }
    
    if(recMessage["enabledValves"]){
      //enabledValves = recMessage["enabledValves"];
    }
    // print RSSI of packet
    Serial.print("' with RSSI ");
    rssi = LoRa.packetRssi();
    Serial.println(rssi);
  }
  if(irrigate != irrigateOld){
    Serial.println("irrigatechange");      
    irrigateOld = irrigate;
    valveStateStr = "irrigation ";
    if(irrigate) valveStateStr +="started";
    else valveStateStr +="stopped";
    Serial.println(valveStateStr);
    displayMsg(valveStateStr+"!!!");
    if(!irrigate){sendLoraMsg();}
  }
  yield();
}

void initDisplay() {
  // Configure OLED by setting the OLED Reset HIGH, LOW, and then back HIGH
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, HIGH);
  delay(100);
  digitalWrite(OLED_RST, LOW);
  delay(100);
  digitalWrite(OLED_RST, HIGH);
    
  display.init();
  display.flipScreenVertically();
  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.setColor(WHITE);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(display.getWidth() / 2, display.getHeight() / 2, "LoRa Receiver");
  display.display();
  delay(1000);
}

void displayMsg(String msg1, String msg2, String msg3, String msg4) {
  digitalWrite(LED_BUILTIN, LOW);
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  
  display.drawString(0, 25, String(msg1).c_str());
  display.drawString(0, 35, String(msg2).c_str());
  display.drawString(0, 45, String(msg3).c_str());
  display.drawString(0, 55, String(msg4).c_str());
  display.display();
  //delay(1000);
}

void IRAM_ATTR pulseCounter()
{
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
    displayMsg(waterFlowMsg, valveStateStr, "RSSI: "+String(rssi));
    Serial.println(waterFlowMsg);
    //if(((previousFlowRate == 0 && flowRate >0) || (previousFlowRate > 0 && flowRate == 0)) && loraSent == false){
    if(((flowRate >0 && flowRate != previousFlowRate) || (previousFlowRate > 0 && flowRate == 0)) && loraSent == false){
        // send lora
      sendLoraMsg();
      Serial.println("flow:"+String(flowRate));
      previousFlowRate = flowRate;
  }
}

void axdelay(int delaylength){
  long startMillis = millis();
  /*
  while (millis() < (startMillis + delaylength *1000)){
    buttonDebouncer1.update();  
    if(buttonDebouncer1.fell()){
      Serial.println("button");
      irrigate = !irrigate;
      if(irrigate)Serial.println("irrigate should now be on");
      else Serial.println("irrigate should now be off");
      Serial.println("irrigateOld:"+String(irrigateOld));
      Serial.println("irrigate:"+String(irrigate));
      break;
    }
    //Serial.println(String(millis()) + "/" +String(startMillis + delaylength *1000));
  }
  */
}

void sendLoraMsg(){
      // send lora
      DynamicJsonDocument sendMessage(1024);
      sendMessage["rate"] = String(flowRate);
      sendMessage["total"] = String(totalMilliLitres / 1000);
      sendMessage["rssi"] = rssi;
      sendMessage["activeValve"] = activeValve;
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
}
