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
  
//#define DEBUG
  
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
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>
#include "ev_diy_font.h"


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
int soc_error_flag = 0;
int delta;
int delta_error_flag = 0;
float temp;
int temp_error_flag = 0;
int temp_display_delay; // allows for target temp to still be shown for a short delay after you stop twiddling the knob to set it
int display_temp;
int charge_current = 0; // Fix: Initialize charge_current globally

// Reliability improvements
unsigned long last_can_soc = 0;
unsigned long last_can_temp = 0;
unsigned long last_can_delta = 0;
unsigned long last_can_heater = 0;
unsigned long last_can_charger = 0;
const unsigned long CAN_TIMEOUT_MS = 5000; // 5 second timeout
bool display_init_success = false;

// Enhanced color system for safety warnings
#define COLOR_SAFE 0x07E0      // Green
#define COLOR_CAUTION 0xFFE0   // Yellow  
#define COLOR_WARNING 0xFD20   // Orange
#define COLOR_CRITICAL 0xF800  // Red
#define COLOR_INACTIVE 0x9515  // Gray


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

  // Initialise 1.8" TFT screen with error checking:
  bool tft1_ok = true;
  bool tft2_ok = true;
  
  try {
    tft1.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab
    tft1.fillScreen(ST77XX_BLACK);   // Test display 1
  } catch (...) {
    tft1_ok = false;
    Serial.println("TFT1 initialization failed!");
  }
  
  try {
    tft2.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab  
    tft2.fillScreen(ST77XX_BLACK);   // Test display 2
  } catch (...) {
    tft2_ok = false;
    Serial.println("TFT2 initialization failed!");
  }
  
  display_init_success = tft1_ok && tft2_ok;
  
  Serial.print(millis());
  Serial.print("\t");
  if (display_init_success) {
    Serial.println("TFT Init complete - Both displays OK");
  } else {
    Serial.println("TFT Init WARNING - Some displays failed");
  }

  // Setup backlights and set to black
  ledcSetup(TFT_1_BLK_CHAN, TFT_FREQ, RESOLUTION);  
  ledcSetup(TFT_2_BLK_CHAN, TFT_FREQ, RESOLUTION);
  ledcAttachPin(TFT_1_BLK, TFT_1_BLK_CHAN);
  ledcAttachPin(TFT_2_BLK, TFT_2_BLK_CHAN);
  ledcWrite(TFT_1_BLK_CHAN, 0);
  ledcWrite(TFT_2_BLK_CHAN, 0); // Fix: Use correct channel for display 2

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
  
  // Initialize timeout counters to current time to prevent false timeouts on startup
  unsigned long startup_time = millis();
  last_can_soc = startup_time;
  last_can_temp = startup_time;
  last_can_delta = startup_time;
  last_can_heater = startup_time;
  last_can_charger = startup_time;

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

///////////////////////////////////////////////// DISPLAY UTILITY FUNCTIONS ////////////////////////////////////////////////////////////

// Get color for temperature value
uint16_t getTempColor(float temperature) {
  if (temperature < 25) return COLOR_SAFE;      // Green: 15-25°C optimal
  if (temperature < 30) return COLOR_CAUTION;   // Yellow: 25-30°C warm
  if (temperature < 35) return COLOR_WARNING;   // Orange: 30-35°C hot
  return COLOR_CRITICAL;                        // Red: >35°C critical
}

// Get color for delta value  
uint16_t getDeltaColor(int delta_mv) {
  if (delta_mv < 20) return COLOR_SAFE;         // Green: 0-20mV balanced
  if (delta_mv < 35) return COLOR_CAUTION;      // Yellow: 20-35mV minor imbalance
  if (delta_mv < 50) return COLOR_WARNING;      // Orange: 35-50mV attention needed  
  return COLOR_CRITICAL;                        // Red: >50mV critical imbalance
}

// Get color for SoC value
uint16_t getSocColor(int soc_percent) {
  if (soc_percent > 40) return COLOR_SAFE;      // Green: >40% normal
  if (soc_percent > 20) return COLOR_CAUTION;   // Yellow: 20-40% plan charging
  if (soc_percent > 10) return COLOR_WARNING;   // Orange: 10-20% find charger
  return COLOR_CRITICAL;                        // Red: <10% emergency
}

///////////////////////////////////////////////// TFT1 IMPROVED DISPLAY ////////////////////////////////////////////////////////////

void tft1InitialDisplay() {
  // Clear screen
  tft1.fillScreen(ST77XX_BLACK);
  
  // Set custom icon font for status icons
  tft1.setFont(&ev_diy_font);
  tft1.setTextSize(1);
  
  // Top status bar - Heater and Charge icons with spacing
  tft1.drawChar(8, 18, 128, COLOR_INACTIVE, 0, 1);   // Heater icon left
  tft1.drawChar(112, 18, 129, COLOR_INACTIVE, 0, 1); // Charge icon right
  
  // Central SoC display - Large and prominent (main fuel gauge)
  tft1.drawRoundRect(8, 40, 112, 70, 8, ST77XX_WHITE);
  tft1.setFont(&FreeSansBold24pt7b);
  tft1.setTextColor(ST77XX_WHITE);
  tft1.setCursor(20, 85);
  tft1.print("---%");
  
  // Bottom status line - Critical safety info (larger than before)
  tft1.setFont(&FreeSansBold12pt7b);
  tft1.setTextColor(COLOR_INACTIVE);
  tft1.setCursor(8, 130);
  tft1.print("TEMP");
  tft1.setCursor(8, 145); 
  tft1.print("--°C");
  
  tft1.setCursor(75, 130);
  tft1.print("DELTA");
  tft1.setCursor(75, 145);
  tft1.print("--mV");
  
  // Status indicator
  tft1.setFont(&ev_diy_font);
  tft1.setTextColor(COLOR_CAUTION);
  tft1.setCursor(45, 155);
  tft1.print("INIT");
  
  soc_error_flag = 1;
}

///////////////////////////////////////////////// TFT2 INITIAL DISPLAY ////////////////////////////////////////////////////////////


void tft2InitialDisplay() {
  // Clear screen
  tft2.fillScreen(ST77XX_BLACK);
  
  tft2.setFont(&FreeSansBold12pt7b);
  tft2.setTextSize(1);
  
  // Systems Status Header
  tft2.setTextColor(ST77XX_WHITE);
  tft2.setCursor(25, 20);
  tft2.print("SYSTEMS");
  
  // Heater Status Section
  tft2.setFont(&ev_diy_font);
  tft2.setTextColor(COLOR_INACTIVE);
  tft2.setCursor(8, 40);
  tft2.print("HEATER:");
  tft2.setCursor(8, 55);
  tft2.print("--°C -> --°C");
  
  // Charging Status Section  
  tft2.setCursor(8, 75);
  tft2.print("CHARGE:");
  tft2.setCursor(8, 90);
  tft2.print("-- A");
  
  // System Health Indicators
  tft2.setCursor(8, 110);
  tft2.print("CAN: ----");
  tft2.setCursor(8, 125);
  tft2.print("WIFI: ---");
  
  // Uptime/Status
  tft2.setCursor(8, 145);
  tft2.print("UPTIME:");
  tft2.setCursor(8, 160);
  tft2.print("--:--");
}

///////////////////////////////////////////////// PRINTFRAME ////////////////////////////////////////////////////////////


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

///////////////////////////////////////////////// HEATER PROC ////////////////////////////////////////////////////////////
  

void heater_proc(CAN_FRAME *message)  {
  #ifdef DEBUG
    printFrame(message);
  #endif
  
  last_can_heater = millis(); // Update last received time      

  // check the heater status
  if(message->data.byte[0] == 0) {hvPresent = true;} else {hvPresent = false;} // HV present at heater
  if(message->data.byte[1] > 0) {heater_enabled = true;} else {heater_enabled = false;} // Heater enabled
  if(message->data.byte[2] > 0) {heating = true;} else {heating = false;} // Heating is active

  // Update TFT1 heater icon based on status
  if(heater_enabled) {
    if (heating){
      tft1.drawChar(8, 18, 128, COLOR_WARNING, 0, 1);  // Orange when heating
    } else {
      tft1.drawChar(8, 18, 128, COLOR_CAUTION, 0, 1);  // Yellow when enabled but not heating
    } 
  } else {
    tft1.drawChar(8, 18, 128, COLOR_INACTIVE, 0, 1);   // Gray when disabled
  }

  // Update heater temperature display on TFT1 (top status bar)
  unsigned char new_heater_temp = message->data.byte[3];
  unsigned char new_heater_target = message->data.byte[4];
  
  if (new_heater_temp != heater_temp || new_heater_target != heater_target) {
    // Clear previous heater temp display on TFT1
    tft1.fillRect(25, 8, 40, 15, ST77XX_BLACK);
    
    heater_temp = new_heater_temp;
    heater_target = new_heater_target;
    
    tft1.setFont(&ev_diy_font);
    tft1.setTextColor(heating ? COLOR_WARNING : ST77XX_WHITE);
    tft1.setCursor(25, 18);
    tft1.print(heater_temp);
    tft1.print("°C");
    
    // Update detailed heater display on TFT2
    tft2.fillRect(8, 50, 120, 15, ST77XX_BLACK);
    tft2.setFont(&ev_diy_font);
    tft2.setTextColor(heater_enabled ? COLOR_SAFE : COLOR_INACTIVE);
    tft2.setCursor(8, 55);
    tft2.print(heater_temp);
    tft2.print("°C -> ");
    tft2.print(heater_target);
    tft2.print("°C");
  }

  #ifdef DEBUG
    printf("Heater: HV=%d, Enabled=%d, Heating=%d, Temp=%d°C, Target=%d°C\n", 
           hvPresent, heater_enabled, heating, heater_temp, heater_target);
  #endif  
}

///////////////////////////////////////////////// CHARGER PROC ////////////////////////////////////////////////////////////


void charger_proc(CAN_FRAME *message) {
  #ifdef DEBUG
    printFrame(message);
  #endif
  
  last_can_charger = millis(); // Update last received time
  
  int new_charge_current = message->data.byte[6];
  
  if(new_charge_current != charge_current){
    // Clear previous charge current display on TFT1
    tft1.fillRect(80, 8, 30, 15, ST77XX_BLACK);
    
    charge_current = new_charge_current;

    // Update TFT1 charge display and icon
    tft1.setFont(&ev_diy_font);
    
    if(charge_current > 0) { 
      // Charging active - show green icon and current
      tft1.drawChar(112, 18, 129, COLOR_SAFE, 0, 1);     // Green charge icon
      tft1.setTextColor(COLOR_SAFE);        
      tft1.setCursor(85, 18);
      tft1.print(charge_current);
      tft1.print("A");
      
    } else {    
      // Not charging - show gray icon
      tft1.drawChar(112, 18, 129, COLOR_INACTIVE, 0, 1); // Gray charge icon
    }
    
    // Update detailed charge display on TFT2
    tft2.fillRect(8, 85, 120, 15, ST77XX_BLACK);
    tft2.setFont(&ev_diy_font);
    
    if(charge_current > 0) {
      tft2.setTextColor(COLOR_SAFE);
      tft2.setCursor(8, 90);
      tft2.print(charge_current);
      tft2.print(" A");
    } else {
      tft2.setTextColor(COLOR_INACTIVE);
      tft2.setCursor(8, 90);
      tft2.print("0 A");
    }
    
    #ifdef DEBUG
      printf("Charge Current: %dA\n", charge_current);
    #endif
  }
}

///////////////////////////////////////////////// SOC PROC ////////////////////////////////////////////////////////////


void soc_proc(CAN_FRAME *message) {
  #ifdef DEBUG
    printFrame(message);
  #endif
  
  last_can_soc = millis(); // Update last received time

  if((message->data.byte[1] <<8) + (message->data.byte[0]) != soc){
    
    // Clear previous SoC display
    tft1.fillRect(10, 50, 108, 60, ST77XX_BLACK);
    
    soc = (message->data.byte[1] <<8) + (message->data.byte[0]); 
    
    tft1.setFont(&FreeSansBold24pt7b);
    
    if(soc < 101) {
      // Use color coding for SoC based on charge level
      uint16_t soc_color = getSocColor(soc);
      tft1.setTextColor(soc_color);
      
      // Center the SoC percentage in the fuel gauge box
      if(soc < 10) {
        tft1.setCursor(45, 85);  // Single digit
      } else {
        tft1.setCursor(30, 85);  // Two/three digits
      }
      
      tft1.print(soc);
      tft1.print("%");
      
      // Clear any previous error state indicator
      if(soc_error_flag == 1) {
        tft1.setFont(&ev_diy_font);
        tft1.fillRect(40, 150, 50, 15, ST77XX_BLACK);
        tft1.setTextColor(COLOR_SAFE);
        tft1.setCursor(45, 155);
        tft1.print("OK");
      }
      
      #ifdef DEBUG
        printf("SoC: %d%%\n", soc);
      #endif
      soc_error_flag = 0;
      
    } else {      
      // Error state - invalid SoC value
      tft1.setTextColor(COLOR_CRITICAL);
      tft1.setCursor(25, 85);
      tft1.print("ERR");
      
      // Show error in status area
      tft1.setFont(&ev_diy_font);
      tft1.setTextColor(COLOR_CRITICAL);
      tft1.setCursor(40, 155);
      tft1.print("SoC-ERR");
      
      soc_error_flag = 1;
      #ifdef DEBUG
        printf("SoC error >> SoC: %d%%\n", soc);
      #endif            
    }
  }
}

///////////////////////////////////////////////// TEMP PROC ///////////////////////////////////////////////////////////


// Module Temp
void temp_proc(CAN_FRAME *message) {
  #ifdef DEBUG
    printFrame(message);
  #endif
  
  last_can_temp = millis(); // Update last received time
  
  float new_temp = ((message->data.byte[4] + (message->data.byte[5] <<8)))/10.0;
  
  if(new_temp != temp) {
    // Clear previous temperature display
    tft1.fillRect(8, 135, 60, 25, ST77XX_BLACK);
    
    temp = new_temp;
    
    // Use enhanced color coding for temperature
    uint16_t temp_color = getTempColor(temp);
    
    tft1.setFont(&FreeSansBold12pt7b);
    tft1.setTextColor(temp_color);
    tft1.setCursor(8, 145);
    
    if(temp < 100 && temp >= 0) {
      tft1.print(temp, 1);
      tft1.print("°C");
      temp_error_flag = 0;
      
      #ifdef DEBUG 
        printf("Temp: %.1f°C\n", temp);
      #endif
      
    } else {
      // Error state - invalid temperature
      tft1.setTextColor(COLOR_CRITICAL);
      tft1.print("T-ERR");
      temp_error_flag = 1;
      
      #ifdef DEBUG        
        printf("Temp error >> Temp: %.1f°C\n", temp);
      #endif
    }
  }
}

///////////////////////////////////////////////// DELTA PROC ////////////////////////////////////////////////////////////

  
void delta_proc(CAN_FRAME *message) {
  #ifdef DEBUG 
    printFrame(message);
  #endif
  
  last_can_delta = millis(); // Update last received time  

  int new_delta = (message->data.byte[2] + (message->data.byte[3] <<8))-(message->data.byte[0] + (message->data.byte[1] <<8));
  
  if(new_delta != delta) {
    // Clear previous delta display
    tft1.fillRect(75, 135, 53, 25, ST77XX_BLACK);
    
    delta = new_delta;
    
    // Use enhanced color coding for delta
    uint16_t delta_color = getDeltaColor(delta);
    
    tft1.setFont(&FreeSansBold12pt7b);
    tft1.setTextColor(delta_color);
    tft1.setCursor(75, 145);

    // Display delta value with appropriate formatting
    if(delta >= 0 && delta < 1000) {
      tft1.print(delta);
      tft1.print("mV");
      delta_error_flag = 0;
      
      #ifdef DEBUG 
        printf("Delta: %dmV\n", delta);
      #endif

    } else {
      // Error state - invalid delta value
      tft1.setTextColor(COLOR_CRITICAL);
      tft1.print("D-ERR");
      delta_error_flag = 1;
      
      #ifdef DEBUG
        printf("Delta error >> Delta: %dmV\n", delta);
      #endif 
    }
  }
}

///////////////////////////////////////////////// EML - TURN OFF ENGINE MANAGEMENT LIGHT ////////////////////////////////////////////////////////////
  
  
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

///////////////////////////////////////////////// ENG_SPEED: TRANSLATE MOTOR SPEED INTO REVS - USE FOR CURRENT LATER  ////////////////////////////////////////////////////////////
  
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

///////////////////////////////////////////////// ASC - BLUFF STABILITY CONTROL SYSTEM  ////////////////////////////////////////////////////////////

  
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

///////////////////////////////////////////////// OTA FUNCTIONS  ////////////////////////////////////////////////////////////

  
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

///////////////////////////////////////////////// BACKLIGHT RAMP UP/DOWN  ////////////////////////////////////////////////////////////

  
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

///////////////////////////////////////////////// TIMER TASK  ////////////////////////////////////////////////////////////


// CAN timeout monitoring and error recovery
void checkCanTimeouts() {
  unsigned long current_time = millis();
  
  // Check SoC timeout
  if (current_time - last_can_soc > CAN_TIMEOUT_MS && !soc_error_flag) {
    soc_error_flag = 1;
    tft1.setFont(&FreeSansBold24pt7b);
    tft1.fillRect(4,36,120,90,ST77XX_BLACK);
    tft1.setTextColor(ST77XX_RED);
    tft1.setCursor(10,80);
    tft1.print("CAN!");
    tft1.setFont(&ev_diy_font);
    Serial.println("SoC CAN timeout detected");
  }
  
  // Check temp timeout
  if (current_time - last_can_temp > CAN_TIMEOUT_MS && !temp_error_flag) {
    temp_error_flag = 1;
    tft1.drawChar(0,160,130,ST77XX_RED,0,1);
    tft1.setTextColor(ST77XX_RED);
    tft1.setCursor(30, 153);
    tft1.print("TO");
    Serial.println("Temperature CAN timeout detected");
  }
  
  // Check delta timeout
  if (current_time - last_can_delta > CAN_TIMEOUT_MS && !delta_error_flag) {
    delta_error_flag = 1;
    tft1.drawChar(104,160,131,ST77XX_RED,0,1);
    tft1.setTextColor(ST77XX_RED);
    tft1.setCursor(78, 153);
    tft1.print("TO");
    Serial.println("Delta CAN timeout detected");
  }
  
  // Check charger timeout - clear display if no data
  if (current_time - last_can_charger > CAN_TIMEOUT_MS && charge_current != 0) {
    tft1.setTextColor(ST77XX_BLACK);        
    tft1.setCursor(76,16);
    tft1.print(charge_current);
    tft1.print("A");
    charge_current = 0;
    tft1.drawChar(104,24,129,0x9515,0,1); // Gray out charge icon
    Serial.println("Charger CAN timeout detected");
  }
}

// Enhanced CAN bus error detection
void checkCanBusHealth() {
  // Check CAN bus status
  static unsigned long last_can_check = 0;
  if (millis() - last_can_check > 1000) { // Check every second
    last_can_check = millis();
    
    // If all CAN data is timing out, try to reinitialize CAN bus
    unsigned long current_time = millis();
    bool all_timeout = (current_time - last_can_soc > CAN_TIMEOUT_MS * 2) &&
                       (current_time - last_can_temp > CAN_TIMEOUT_MS * 2) &&
                       (current_time - last_can_delta > CAN_TIMEOUT_MS * 2);
    
    if (all_timeout) {
      Serial.println("All CAN data timeout - attempting CAN bus recovery");
      
      // Try to reinitialize CAN bus
      CAN0.begin(500000);
      CAN0.watchFor(0x355, 0xFFF);
      CAN0.watchFor(0x356, 0xFFF);
      CAN0.watchFor(0x373, 0xFFF);
      CAN0.watchFor(0x300, 0xFFF);
      CAN0.watchFor(0x389, 0xFFF);
      
      CAN0.setCallback(0, soc_proc);
      CAN0.setCallback(1, temp_proc);
      CAN0.setCallback(2, delta_proc);
      CAN0.setCallback(3, heater_proc);
      CAN0.setCallback(4, charger_proc);
      
      // Reset timeout counters to prevent immediate re-trigger
      last_can_soc = current_time;
      last_can_temp = current_time;
      last_can_delta = current_time;
      last_can_heater = current_time;
      last_can_charger = current_time;
    }
  }
}

void ms10Task() {
  eml();
  eng_speed();
  asc();
  
  // Add reliability monitoring every 100ms (every 10th call)
  static int timeout_check_counter = 0;
  if (++timeout_check_counter >= 10) {
    timeout_check_counter = 0;
    checkCanTimeouts();
    checkCanBusHealth();
  }
}
