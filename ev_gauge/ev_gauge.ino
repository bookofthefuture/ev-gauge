/* Canbus-powered information gauge for DIY EV. Designed for Adafruit 1.8in screen powered by ST7755 driver board. 
 * Uses SN65HVD canbus transceiver with ESP32 onboard can
 * OTA updating for software in case the driver is buried in your dash
 * Now with added canbus signalling to control analogue gauges and delete error messages in car
 * Added code to read and display information from outlander heater controller - NOT TESTED!!
 * Added dual displays
*/
 
  
//Include libraries for display, OTA and can communications
#include <WiFi.h>
#include <WiFiClient.h>
#include <AsyncTCP.h> //
#include <ESPAsyncWebServer.h>
#include <Adafruit_GFX.h>    // Core graphics library https://github.com/adafruit/Adafruit-GFX-Library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735 https://github.com/adafruit/Adafruit-ST7735-Library
#include <SPI.h>
#include <Arduino.h>
#include <esp32_can.h> // ESP32 native can library
#include <ElegantOTA.h> //Note: uses library in Async mode. Check documentation here: https://docs.elegantota.pro/async-mode/. Modification needed to library for this to work.
#include <SPIFFS.h>
#include <SPIFFS_ImageReader.h> // https://github.com/lucadentella/SPIFFS_ImageReader
#include <TaskScheduler.h> // https://github.com/arkhipenko/TaskScheduler
  
// Image reader
SPIFFS_ImageReader reader;
  
#define DEBUG;
  
// OTA CONFIG
const char* ssid = "gaugedriver";
const char* password = "123456789";
  
unsigned long ota_progress_millis = 0;
  
// GAUGE CONFIG
CAN_FRAME txFrame;
unsigned long lastMillis;
int motorSpeed = 0; // If I can get canbus comms running from inverter, can get revs from here
int clusterStart = 1; // maxes the rev counter dial on start-up
int motorTemp = 0; // need inverter can comms to get this but could use charger temp as proxy for now
int mt;
int revCount;
int counter_329 = 0;
int brakeOn = 0;
unsigned char accelPot = 0x00;
unsigned char ABSMsg = 0x11; // This is recalculated on a timer so no input needed here
  
// HEATER DATA
bool hvPresent = false;
bool heater_enabled = false;
bool heating = false;

unsigned char heater_temp;
unsigned char heater_target;

// Web interface  
AsyncWebServer server(80);
  
// Include fonts for display
#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>
#include <Fonts/ev_diy.h>


// Configure i2c pins for display - 30 pin layout
// Note change of layout to make wiring simpler
  
//BLK (backlight) connect to 3v3 output
//3v3
//GND
#define TFT_RST        25
#define TFT_SDA        26     
#define TFT_SCL        27
#define TFT_DC         33  
#define TFT_1_CS       14 
#define TFT_2_CS       32 
#define TFT_1_BLK      19  
#define TFT_2_BLK      21
  
// PWM for controlling display brightness
const int TFT_FREQ = 5000;
const int TFT_1_BLK_CHAN = 0;
const int TFT_2_BLK_CHAN = 1;
const int RESOLUTION = 8;
  
//TFT CONFIG
Adafruit_ST7735 tft1 = Adafruit_ST7735(TFT_1_CS, TFT_DC, TFT_SDA, TFT_SCL, TFT_RST);
Adafruit_ST7735 tft2 = Adafruit_ST7735(TFT_2_CS, TFT_DC, TFT_SDA, TFT_SCL, -1);

// Configure CAN TX/RX Pins
#define CAN_RX GPIO_NUM_13
#define CAN_TX GPIO_NUM_15
  
// Pi for circle drawing
float p = 3.1415926;
  
// Variables for displayed stats
int soc;
int soc_error_flag;
int delta;
float temp;

// Task Scheduling
void ms10Task();

Task ms10(10, -1, &ms10Task);

Scheduler runner;

void setup() {
   
  #ifdef DEBUG
    Serial.begin(115200);
    Serial.print(millis());
    Serial.print("\t");
    Serial.println("In setup");
  #endif

  // Task scheduler
  runner.init();

  runner.addTask(ms10);
  ms10.enable();

  pinMode(TFT_RST, OUTPUT);
  
  // initialize SPIFFS
  if(!SPIFFS.begin()) {
    Serial.println("SPIFFS initialisation failed!");
    while (1);
  }  

  // Initialise 1.8" TFT screen:
  tft1.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab
  tft2.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab

  Serial.print(millis());
  Serial.print("\t");
  Serial.println("TFT Init complete");

  // Setup backlights and set to black
  ledcSetup(TFT_1_BLK_CHAN, TFT_FREQ, RESOLUTION);  
  ledcSetup(TFT_2_BLK_CHAN, TFT_FREQ, RESOLUTION);
  ledcAttachPin(TFT_1_BLK, TFT_1_BLK_CHAN);
  ledcAttachPin(TFT_2_BLK, TFT_2_BLK_CHAN);
  ledcWrite(TFT_1_BLK_CHAN, 0);
  ledcWrite(TFT_1_BLK_CHAN, 0);

  Serial.print(millis());
  Serial.print("\t");
  Serial.println("Backlight prep complete");
    
  reader.drawBMP("/launch.bmp", tft1, 0, 0);
  reader.drawBMP("/launch.bmp", tft2, 0, 0);

  Serial.print(millis());
  Serial.print("\t");
  Serial.println("Logos drawn");

  backlight_ramp_up();

  Serial.print(millis());
  Serial.print("\t");
  Serial.println("Backlight ramp complete");

  backlight_ramp_down();
  
  tft1.setTextWrap(false);
  tft1.setRotation(2);
  tft1.fillScreen(ST77XX_BLACK);
  #ifdef DEBUG
    Serial.print(millis());
    Serial.print("\t");
    Serial.println("Erased Screen 1");
  #endif

  tft2.setTextWrap(false);
  tft2.setRotation(2);
  tft2.fillScreen(ST77XX_BLACK);
  #ifdef DEBUG
    Serial.print(millis());
    Serial.print("\t");
    Serial.println("Erased Screen 2");
  #endif

  backlight_ramp_up();

  tft1InitialDisplay();
  tft2InitialDisplay();
  
  // Initialise CANBus
  #ifdef DEBUG
    Serial.println("Initializing CANBus...");
  #endif
  CAN0.setCANPins(CAN_RX, CAN_TX);
  CAN0.begin(500000);
    
  // Set up can filters for target IDs
  CAN0.watchFor(0x355, 0xFFF); //setup a special filter to watch for only 0x355 to get SoC
  CAN0.watchFor(0x356, 0xFFF); //setup a special filter to watch for only 0x356 to get module temps
  CAN0.watchFor(0x373, 0xFFF); //setup a special filter to watch for only 0x373 to get cell deltas
  CAN0.watchFor(0x300, 0xFFF); //setup a special filter to watch for only 0x300 to get heater info
  CAN0.watchFor(0x389, 0xFFF); //setup a special filter to watch for only 0x389 to get charger info
    
  //CAN0.watchFor(); //then let everything else through anyway - enable for debugging
  
  // Set callbacks for target IDs to process and update display
  CAN0.setCallback(0, soc_proc); //callback on first filter to trigger function to update display with SoC
  CAN0.setCallback(1, temp_proc); //callback on second filter to trigger function to update display with temp
  CAN0.setCallback(2, delta_proc); //callback on third filter to trigger function to update display with delta
  CAN0.setCallback(3, heater_proc); //callback on third filter to trigger function to update display with heater info
  CAN0.setCallback(4, charger_proc); //callback on third filter to trigger function to update display with charger info

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  Serial.println("");
  
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Gauge Driver OTA Interface");
  });
  
  ElegantOTA.begin(&server);    // Start ElegantOTA
  // ElegantOTA callbacks
  ElegantOTA.onStart(onOTAStart);
  ElegantOTA.onProgress(onOTAProgress);
  ElegantOTA.onEnd(onOTAEnd);
  
  server.begin();
  #ifdef DEBUG
    Serial.println("HTTP server started");
  #endif 
  delay(4000);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  Serial.println("");
  
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Gauge Driver OTA Interface");
  });
  
  ElegantOTA.begin(&server);    // Start ElegantOTA
  // ElegantOTA callbacks
  ElegantOTA.onStart(onOTAStart);
  ElegantOTA.onProgress(onOTAProgress);
  ElegantOTA.onEnd(onOTAEnd);
  
  server.begin();
  #ifdef DEBUG
    Serial.print(millis());
    Serial.print("\t");
    Serial.println("HTTP server started");
  #endif 
  delay(4000);

  #ifdef DEBUG
    Serial.print(millis());
    Serial.print("\t");
    Serial.println("Ready ...!");
  #endif  
}
  
void loop() {
  runner.execute();
  ElegantOTA.loop();
  
  #ifdef DEBUG
//    CAN_FRAME message;
//    if (CAN0.read(message)) {
//      printFrame(&message);
//    }
  #endif
  
  }

void tft1InitialDisplay() {
// Initial display before data arrives

// Select custom icon font
  tft1.setFont(&ev_diy);

// Heater icon
  tft1.drawChar(0,24,128,0x9515,0,1);

// Charge icon
  tft1.drawChar(104,24,131,0x9515,0,1);

// Module temp icon
  tft1.drawChar(0,160,129,0x9515,0,1);

// Module delta icon
  tft1.drawChar(104,160,133,0x9515,0,1);

// Set normal font - could integrate with symbol font to save space
  tft1.setFont(&FreeSansBold9pt7b);
  tft1.setTextColor(0x9515);

// test text
#ifdef DEBUG
//heater
  tft1.setCursor(30, 16);
  tft1.print("30");
// charge
  tft1.setCursor(80, 16);
  tft1.print("8A");
// module temp
  tft1.setCursor(30, 154);
  tft1.print("5");
// module delta
  tft1.setCursor(80, 154);
  tft1.print("9");
#endif

// SoC
  tft1.drawRoundRect(2, 32, 124, 98, 5, ST77XX_WHITE);
  tft1.setTextColor(ST77XX_WHITE);
  tft1.setCursor(10, 70);
  tft1.setTextSize(1);
  tft1.print("Waiting for");
  tft1.setCursor(10, 90);
  tft1.print("CAN...");
  soc_error_flag = 1;
}

void tft2InitialDisplay() {

  // Initial display of SoC before data arrives
  tft2.setCursor(0,12);
  tft2.setFont(&FreeSansBold9pt7b);
  tft2.setTextSize(1);
  tft2.setTextColor(ST77XX_RED);  
  tft2.print("HV");  
  tft2.setCursor(30,15);
  tft2.setFont(&FreeSansBold9pt7b);
  tft2.setTextSize(1);
  tft2.setTextColor(ST77XX_RED);  
  tft2.print("HE");  
  tft2.setCursor(60,15);
  tft2.setFont(&FreeSansBold9pt7b);
  tft2.setTextSize(1);
  tft2.setTextColor(ST77XX_WHITE);  
  tft2.print("TAR");  
  
  //  tft1.setFont(&FreeSansBold24pt7b);
  //  tft1.setCursor(20, 70);
  //  tft1.setTextSize(1);
  tft2.setFont(&FreeSansBold9pt7b);
  tft2.setCursor(10, 70);
  tft2.setTextSize(1);
  tft2.print("Waiting for");
  tft2.setCursor(10, 90);
  tft2.print("CAN...");
     
  // Initial display of max delta before data arrives
  tft2.fillTriangle(68, 154, 74, 135, 80, 154, ST77XX_BLUE);
  tft2.setCursor(84, 153);
  tft2.setFont(&FreeSansBold12pt7b);
  tft2.setTextSize(1);
  tft2.print("N/A");
  
  // Initial display of module temp before data arrives
  tft2.fillCircle(8, 140, 2, ST77XX_RED);
  tft2.fillCircle(8, 150, 4, ST77XX_RED);
  tft2.fillRect(6, 140, 5, 6, ST77XX_RED);
  tft2.setCursor(16, 153);
  tft2.setFont(&FreeSansBold12pt7b);
  tft2.setTextSize(1);
  tft2.print("N/A");   
}

void printFrame(CAN_FRAME *message)
  {
    Serial.print(message->id, HEX);
    if (message->extended) Serial.print(" X ");
    else Serial.print(" S ");   
    Serial.print(message->length, DEC);
    Serial.print(" ");
    for (int i = 0; i < message->length; i++) {
      Serial.print(message->data.byte[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }

void heater_proc(CAN_FRAME *message)  {
  
  int last_temp_displayed; //set to 1 if target and 0 if actual. Target temp only shown when it changes and stays on screen for a couple of seconds
  int temp_display_delay; // allows for target temp to still be shown for a short delay after you stop twiddling the knob to set it
  
  if(message->data.byte[0] == 0) {hvPresent = true;} else {hvPresent = false;} // HV present at heater
  if(message->data.byte[1] > 0) {heater_enabled = true;} else {heater_enabled = false;} // Heater enabled
  if(message->data.byte[2] > 0) {heating = true;} else {heating = false;} // Heating is active
  
  // top row do HV (green if enabled, red if heating) T (target temp) A (actual - green if heating)
  tft1.setCursor(0,12);
  tft1.setFont(&FreeSansBold9pt7b);
  tft1.setTextSize(1);
  if(heater_enabled && !heating){
    tft1.drawChar(0,16,128,ST77XX_WHITE,0,1);
  } else if (heater_enabled && heating){
    tft1.drawChar(0,16,128,0xFA80,0,1);
  } else  {
    tft1.drawChar(0,16,128,0x9515,0,1);
  }

  // if the target temp has changed, temporarily overwrite the temp display with that
  if((message->data.byte[4]) != heater_target || millis() - temp_display_delay < 1000) {
    // set the cursor to the right position
    tft1.setCursor(30, 16);
    //overwrite the old number in black - last_temp_displayed tells you which number
    tft1.setTextColor(ST77XX_BLACK);  
    if(last_temp_displayed == 1) {
      tft1.print(heater_target,1);  
      // set the new number
    } else {
      tft1.print(heater_temp,1);
      temp_display_delay = millis();     
    }

    //write the new number in green for target temp
    tft1.setTextColor(ST77XX_GREEN);  
    tft1.setCursor(30, 16);
    tft1.print(heater_target,1);  

    //set flag to say the last number displayed was the target temp
    last_temp_displayed = 1;

    // or if the target temp hasn't changed, show the actual
  } else if((message->data.byte[3]) != heater_temp) {
    // set the cursor to the right position
    tft1.setCursor(30, 16);
    //overwrite the old number in black - last_temp_displayed tells you which number
    tft1.setTextColor(ST77XX_BLACK);  
    if(last_temp_displayed == 1) {
      tft1.print(heater_target,1);  
    } else {
      tft1.print(heater_temp,1);     
    }
    
    //write the new number in white for actual temp
    heater_temp = message->data.byte[3]; // Water Temp
    tft1.setCursor(30, 16);
    tft1.setTextColor(ST77XX_WHITE);      
    tft1.print(heater_temp,1); 

    //set flag to say the last number displayed was the target temp
    last_temp_displayed = 0;
   
  } else {}

  #ifdef DEBUG
    printFrame(message); 
    Serial.println("Heater Status");
    Serial.print("HV Present: ");
    Serial.print(hvPresent);
    Serial.print(" Heater Active: ");
    Serial.print(heating);
    Serial.print(" Water Temperature: ");
    Serial.print(heater_temp);
    Serial.println("C");
    Serial.println("");
    Serial.println("Settings");
    Serial.print(" Heating: ");
    Serial.print(heating);
    Serial.print(" Desired Water Temperature: ");
    Serial.print(heater_target);
    Serial.println("");
    Serial.println(""); 
  #endif  
  
}

void charger_proc(CAN_FRAME *message) {
  #ifdef DEBUG
    printFrame(message);
  #endif
  int charge_current;
  if(message->data.byte[6] != charge_current){
    // overwrite the last charge current printed in black
    tft1.setTextColor(ST77XX_BLACK);        
    tft1.setFont(&FreeSansBold9pt7b);
    tft1.setTextSize(1);
    tft1.setCursor(80,16);
    tft1.print(charge_current);
    tft1.print("A");      

    //Set the new charge current
    charge_current = message->data.byte[6];

    //if it is greater than 0, write it out
    if(charge_current > 0) { 
      // Print charge current
      tft1.setTextColor(ST77XX_WHITE);        
      tft1.setFont(&FreeSansBold9pt7b);
      tft1.setTextSize(1);
      tft1.setCursor(80,16);
      tft1.print(charge_current);
      tft1.print("A");
      // Update charge icon to be green
      tft1.drawChar(104,24,131,ST77XX_GREEN,0,1);
    } else {    
      // Update the charge icon to be white
      tft1.drawChar(104,24,131,ST77XX_WHITE,0,1);
    }
  }
}

void soc_proc(CAN_FRAME *message) {
  #ifdef DEBUG
    printFrame(message);
  #endif
    tft1.setFont(&FreeSansBold24pt7b);
    tft1.setTextSize(1);

  if((message->data.byte[1] <<8) + (message->data.byte[0]) != soc){

    if(soc_error_flag == 1){
      tft1.drawRect(0,16,128,100,ST77XX_BLACK);
      tft1.fillRect(0,16,128,100,ST77XX_BLACK);
    } else {
      tft1.setTextColor(ST77XX_BLACK);  
      tft1.setCursor(10,70);  
      tft1.print(soc);
      }
    soc = (message->data.byte[1] <<8) + (message->data.byte[0]); 
    if(soc <= 100) {
      tft1.setTextColor(ST77XX_BLACK);  
      tft1.setCursor(10,70);  
      tft1.print(soc);
      tft1.print("%");
      
      #ifdef DEBUG
      printf("SoC: ");
      printf("%d%%", soc);
      printf("\n");
      #endif
      soc_error_flag = 0;
    } else {      
      tft1.print("...");
      #ifdef DEBUG
        printf("SoC error >> SoC: ");
        printf("%d%%", soc);
        printf("/n");
      #endif            
    }
  }
}
  
  void temp_proc(CAN_FRAME *message) {
    tft1.setTextColor(ST77XX_WHITE);  

    printFrame(message);
    if(((message->data.byte[4] + (message->data.byte[5] <<8)))/10 != temp) {
      temp = (message->data.byte[4] + (message->data.byte[5] <<8))/10;  
      // Module Temp
      if(temp > 150) {
        tft1.drawRect(13,120,51,40,ST77XX_BLACK);
        tft1.fillRect(13,120,51,40,ST77XX_BLACK);
        tft1.setCursor(16, 153);
        tft1.setFont(&FreeSansBold9pt7b);
        tft1.setTextSize(1);
        tft1.print("ERR");
        printf("Temp error >> Temp: ");
        printf("%d%%", temp);
        printf("/n");
    } else if(temp) {
        tft1.drawRect(13,120,51,40,ST77XX_BLACK);
        tft1.fillRect(13,120,51,40,ST77XX_BLACK);
        tft1.setCursor(16, 153);
        tft1.setFont(&FreeSansBold9pt7b);
        tft1.setTextSize(1);
        tft1.print(temp);
        printf("Temp: ");
        printf("%d%%", temp);
        printf("/n");
      } else {
        tft1.drawRect(13,120,51,40,ST77XX_BLACK);
        tft1.fillRect(13,120,51,40,ST77XX_BLACK);
        tft1.setCursor(16, 153);
        tft1.setFont(&FreeSansBold9pt7b);
        tft1.setTextSize(1);
        tft1.print("N/A");
      }
    }
  }
  
  void delta_proc(CAN_FRAME *message) {
    printFrame(message);  
    if((message->data.byte[2] + (message->data.byte[3] <<8))-(message->data.byte[0] + (message->data.byte[1] <<8)) != delta) {
      delta = (message->data.byte[2] + (message->data.byte[3] <<8))-(message->data.byte[0] + (message->data.byte[1] <<8));
    // Max Delta
      if(delta < 0) {
        tft1.drawRect(80,120,48,40,ST77XX_BLACK);
        tft1.fillRect(80,120,48,40,ST77XX_BLACK);
        tft1.setCursor(84, 153);
        tft1.setFont(&FreeSansBold9pt7b);
        tft1.setTextSize(1);
        tft1.print("ERR");
        printf("Delta error >> Delta: ");
        printf("%d%%", delta);
        printf("/n");    
      } else if(delta) {
        tft1.drawRect(80,120,48,40,ST77XX_BLACK);
        tft1.fillRect(80,120,48,40,ST77XX_BLACK);
        tft1.setCursor(84, 153);
        tft1.setFont(&FreeSansBold9pt7b);
        tft1.setTextSize(1);
        tft1.print(delta);
  //      tft1.print("mV");
        printf("Delta: ");
        printf("%d%%", delta);
        printf("/n");    
      } else {
        tft1.drawRect(80,120,48,40,ST77XX_BLACK);
        tft1.fillRect(80,120,48,40,ST77XX_BLACK);
        tft1.setCursor(84, 153);
        tft1.setFont(&FreeSansBold9pt7b);
        tft1.setTextSize(1);
        tft1.print("N/A");
      }
    }
  }
  

  
  void eml(){
    txFrame.rtr = 0;  
    txFrame.id = 0x545;
    txFrame.length = 8;
    txFrame.extended = false;
    txFrame.data.uint8[0] = 0;//2-cel 16-eml 
    txFrame.data.uint8[1] = 0x00;
    txFrame.data.uint8[2] = 0x00;
    txFrame.data.uint8[3] = 0;//overheat(8)
    txFrame.data.uint8[4] = 0x7e;
    txFrame.data.uint8[5] = 10;
    txFrame.data.uint8[6] = 0;
    txFrame.data.uint8[7] = 18;
    CAN0.sendFrame(txFrame);
  }
  
  void eng_speed() {
    revCount = map(motorSpeed,0,10000,0 ,44800);
    if (clusterStart == 0) {revCount = 4800;}
    if (revCount <= 4800) {revCount = 4800;}
    if (revCount >= 44800) {revCount = 44800;}
    if (clusterStart == 1) {revCount = 44800; clusterStart = 0;}
  
    txFrame.rtr = 0;
    txFrame.id = 0x316;
    txFrame.length = 8;
    txFrame.extended = false;
    txFrame.data.uint8[0] = 13;//bit 0 should be 1
    txFrame.data.uint8[1] = 0;
    txFrame.data.uint8[2] = lowByte(revCount);//eng speed lsb
    txFrame.data.uint8[3] = highByte(revCount);//eng speed msb
    txFrame.data.uint8[4] = 0;
    txFrame.data.uint8[5] = 0;
    txFrame.data.uint8[6] = 0;
    txFrame.data.uint8[7] = 0;
    CAN0.sendFrame(txFrame);
  }
  
  void asc() {
    if(counter_329 >= 22) {counter_329 = 0;}
    if(counter_329 == 0) { ABSMsg=0x11;}
    if(counter_329 >= 8 && counter_329 < 15) {ABSMsg=0x86;}
    if(counter_329 >= 15) {ABSMsg=0xd9;}
    counter_329++;
  
    mt=map(motorTemp,0,40,90,254);
  
    txFrame.id  = 0x329;
    txFrame.length = 8;
    txFrame.extended = false;
    txFrame.data.uint8[0] = ABSMsg;
    txFrame.data.uint8[1] = mt;//motor temp 48-255 full scale
    txFrame.data.uint8[2] = 0xc5;
    txFrame.data.uint8[3] = 0;//engine status bit4 ,clutch bit0,engine run bit3,ack can bit2
    txFrame.data.uint8[4] = 0;
    txFrame.data.uint8[5] = accelPot;//throttle position 00-FE
    txFrame.data.uint8[6] = brakeOn;//bit 0 brake on
    txFrame.data.uint8[7] = 0x0;
    CAN0.sendFrame(txFrame);
  }
  
  void onOTAStart() {
    // Log when OTA has started
    Serial.println("OTA update started!");
    // <Add your own code here>
  }
  
  void onOTAProgress(size_t current, size_t final) {
    // Log every 1 second
    if (millis() - ota_progress_millis > 1000) {
      ota_progress_millis = millis();
      Serial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
    }
  }
  
  void onOTAEnd(bool success) {
    // Log when OTA has finished
    if (success) {
      Serial.println("OTA update finished successfully!");
    } else {
      Serial.println("There was an error during OTA update!");
    }
    // <Add your own code here>
  }
  
  void backlight_ramp_up() {
    for(int dutyCycle = 0; dutyCycle < 255; dutyCycle++){   
      // changing the LED brightness with PWM
      ledcWrite(TFT_1_BLK_CHAN, dutyCycle);
      ledcWrite(TFT_2_BLK_CHAN, dutyCycle);
      delay(5);
    }
      ledcWrite(TFT_1_BLK_CHAN, 255);
      ledcWrite(TFT_2_BLK_CHAN, 255);
      return;
  }
  
  void backlight_ramp_down() {
    for(int dutyCycle = 255; dutyCycle > 0; dutyCycle--){   
      // changing the LED brightness with PWM
      ledcWrite(TFT_1_BLK_CHAN, dutyCycle);
      ledcWrite(TFT_2_BLK_CHAN, dutyCycle);
      delay(5);
    }
      ledcWrite(TFT_1_BLK_CHAN, 0);
      ledcWrite(TFT_2_BLK_CHAN, 0);
    return;
  }


void ms10Task() {
  eml();
  eng_speed();
  asc(); 
}
