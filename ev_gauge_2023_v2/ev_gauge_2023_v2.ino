/* Canbus-powered information gauge for DIY EV. Designed for Adafruit 1.8in screen powered by ST7755 driver board. Uses SN65HVD canbus transceiver with ESP32 onboard can
 * OTA updating for software in case the driver is buried in your dash
 * Now with added canbus signalling to control analogue gauges and delete error messages in car
 * adding code to read and display information from outlander heater controller - NOT TESTED!!
 *  
 * Prototype version for initial board with following pinout
 *
 * DISPLAY
 * GND BLACK
 * VDD WHITE
 * SCL GREY     GPIO4
 * SDA PURPLE   GPIO2
 * RST BLUE     GPIO15
 * DC  GREEN    GPIO16
 * CS  YELLOW   GPIO17
 * BLK ORANGE   GPIO23
 * 
 * CANBUS
 * CAN_RXD      GPIO26
 * CAN_TXD      GPIO25
*/

//Include libraries for display, OTA and can communications
#include <WiFi.h>
#include <WiFiClient.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <SPI.h>
#include <Arduino.h>
#include <esp32_can.h> // ESP32 native can library
#include <ElegantOTA.h> //Note: uses library in Async mode. Check documentation here: https://docs.elegantota.pro/async-mode/. Modification needed to library for this to work.
#include <SPIFFS.h>
#include <SPIFFS_ImageReader.h>
  
#define DEBUG = TRUE

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
bool heating = false;
unsigned char templsb;
unsigned char tempmsb;
unsigned char targetlsb;
unsigned char targetmsb;

// CANbus/Error Status
int message_status = 0;
int canflag = 0;
int cantimer;
int error_status = 0;

AsyncWebServer server(80);

// Include fonts for display
#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>

// Configure i2c pins for display
#define TFT_SDA        2        
#define TFT_SCL        4
#define TFT_CS         17
#define TFT_RST        15
#define TFT_DC         16
#define TFT_BLK        23
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_SDA, TFT_SCL, TFT_RST);

// PWM for controlling display brightness
const int TFT_FREQ = 1000;
const int TFT_BLK_CHAN = 0;
const int RESOLUTION = 8;
int backlight_status;

// Image reader
SPIFFS_ImageReader reader;

// Configure CAN TX/RX Pins
#define CAN_RX GPIO_NUM_19
#define CAN_TX GPIO_NUM_21

// Pi for circle drawing
float p = 3.1415926;

// Variables for displayed stats
int soc;
int delta;
float temp;

void setup() {
  // Setup backlights and set to black
  ledcSetup(TFT_BLK_CHAN, TFT_FREQ, RESOLUTION);
  ledcAttachPin(TFT_BLK, TFT_BLK_CHAN);
  ledcWrite(TFT_BLK_CHAN, 0);
  
  #ifdef DEBUG
    Serial.begin(115200);
  #endif

  // initialize SPIFFS
  if(!SPIFFS.begin()) {
    #ifdef DEBUG
      Serial.println("SPIFFS initialisation failed!");
    #endif
    while (1);
    }

  // Initialise 1.8" TFT screen and draw logo
  pinMode(TFT_RST, OUTPUT);
  tft.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab
  tft.setRotation(2);
  reader.drawBMP("/launch.bmp", tft, 0, 0);
  backlight_ramp_up();

  // start ElegantOTA wifi access point
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  Serial.println("");

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Gauge Driver OTA Interface");
  });
  
  ElegantOTA.setAutoReboot(true);
  ElegantOTA.begin(&server);    // Start ElegantOTA
   // ElegantOTA callbacks
  ElegantOTA.onStart(onOTAStart);
  ElegantOTA.onProgress(onOTAProgress);
  ElegantOTA.onEnd(onOTAEnd);
  server.begin();
  #ifdef DEBUG
   Serial.println("HTTP server started");
  #endif

  // delay while things wake up?
  //  delay(4000);
  
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
  CAN0.watchFor(0x398, 0xFFF); //setup a special filter to watch for only 0x300 to get heater info
  //CAN0.watchFor(); //then let everything else through anyway - enable for debugging

  // Set callbacks for target IDs to process and update display
  CAN0.setCallback(0, soc_proc); //callback on first filter to trigger function to update display with SoC
  CAN0.setCallback(1, temp_proc); //callback on second filter to trigger function to update display with temp
  CAN0.setCallback(2, delta_proc); //callback on third filter to trigger function to update display with delta
  CAN0.setCallback(3, heater_proc); //callback on third filter to trigger function to update display with heater info
}

void loop() {
  ElegantOTA.loop();

  CAN_FRAME message;
  if (CAN0.read(message)) {
    #ifdef DEBUG
      printFrame(&message);
    #endif
  }

// timer to turn keep asc, eml and other bits happy
  if (millis() - lastMillis >= 25) {
   lastMillis = millis();  //get ready for the next iteration
   eml();
   eng_speed();
   asc();
  }

// timer to keep an eye on canbus messages - go to error if not received
  if (millis() - cantimer >= 1000) {
    cantimer = millis();  //get ready for the next iteration
    if(message_status == 0) {
      canflag++;
      #ifdef DEBUG
        Serial.print("Can Flag: ");
        Serial.println(canflag);
      #endif  
    } else {
      canflag = 0;
      error_status = 0;
    }
  }
  if(canflag > 3) {
    #ifdef DEBUG
      Serial.println("canflag trigger");
    #endif
    if(error_status == 0){
      error_display();
      error_status = 1;
    }
 
  message_status = 0;
  }
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

void soc_proc(CAN_FRAME *message) {
  #ifdef DEBUG
    printFrame(message);
  #endif
  if(message_status == 0){
    backlight_ramp_down;
  }
  message_status = 1; // set flag to show a new canbus message has been received
  if((message->data.byte[1] <<8) + (message->data.byte[0]) != soc){
    soc = (message->data.byte[1] <<8) + (message->data.byte[0]); 
    if(soc > 100) {
      tft.drawRect(0,16,128,100,ST77XX_BLACK);
      tft.fillRect(0,16,128,100,ST77XX_BLACK);
      tft.setCursor(10,70);
      tft.setFont(&FreeSansBold24pt7b);
      tft.setTextSize(1);
      tft.print("...");
      printf("SoC error >> SoC: ");
      printf("%d%%", soc);
      printf("/n");            
    } else if(soc) {
      tft.drawRect(0,16,128,100,ST77XX_BLACK);
      tft.fillRect(0,16,128,100,ST77XX_BLACK);
      tft.setCursor(10,70);
      tft.setFont(&FreeSansBold24pt7b);
      tft.setTextSize(1);
      tft.print(soc);
      tft.print("%");
      printf("SoC: ");
      printf("%d%%", soc);
      printf("\n");      
    } else {
      tft.setCursor(18,70);
      tft.setFont(&FreeSansBold24pt7b);
      tft.setTextSize(2);
      tft.print("N/A");
    }  
  }
  if(backlight_status == 0) {
    backlight_ramp_up;
  }
}

void temp_proc(CAN_FRAME *message) {
  message_status = 1;  // set flag to show a new canbus message has been received
  #ifdef DEBUG
    printFrame(message);
  #endif
  if(((message->data.byte[4] + (message->data.byte[5] <<8)))/10 != temp) {
    temp = (message->data.byte[4] + (message->data.byte[5] <<8))/10;  
    // Module Temp
    if(temp > 150) {
      tft.drawRect(13,120,51,40,ST77XX_BLACK);
      tft.fillRect(13,120,51,40,ST77XX_BLACK);
      tft.setCursor(16, 153);
      tft.setFont(&FreeSansBold9pt7b);
      tft.setTextSize(1);
      tft.print("ERR");
      printf("Temp error >> Temp: ");
      printf("%d%%", temp);
      printf("/n");
  } else if(temp) {
      tft.drawRect(13,120,51,40,ST77XX_BLACK);
      tft.fillRect(13,120,51,40,ST77XX_BLACK);
      tft.setCursor(16, 153);
      tft.setFont(&FreeSansBold9pt7b);
      tft.setTextSize(1);
      tft.print(temp);
      printf("Temp: ");
      printf("%d%%", temp);
      printf("/n");
    } else {
      tft.drawRect(13,120,51,40,ST77XX_BLACK);
      tft.fillRect(13,120,51,40,ST77XX_BLACK);
      tft.setCursor(16, 153);
      tft.setFont(&FreeSansBold9pt7b);
      tft.setTextSize(1);
      tft.print("N/A");
    }
  }
}

void delta_proc(CAN_FRAME *message) {
  message_status = 1; // set flag to show a new canbus message has been received
  #ifdef DEBUG
    printFrame(message);  
  #endif
  if((message->data.byte[2] + (message->data.byte[3] <<8))-(message->data.byte[0] + (message->data.byte[1] <<8)) != delta) {
    delta = (message->data.byte[2] + (message->data.byte[3] <<8))-(message->data.byte[0] + (message->data.byte[1] <<8));
  // Max Delta
    if(delta < 0) {
      tft.drawRect(80,120,48,40,ST77XX_BLACK);
      tft.fillRect(80,120,48,40,ST77XX_BLACK);
      tft.setCursor(84, 153);
      tft.setFont(&FreeSansBold9pt7b);
      tft.setTextSize(1);
      tft.print("ERR");
      printf("Delta error >> Delta: ");
      printf("%d%%", delta);
      printf("/n");    
    } else if(delta) {
      tft.drawRect(80,120,48,40,ST77XX_BLACK);
      tft.fillRect(80,120,48,40,ST77XX_BLACK);
      tft.setCursor(84, 153);
      tft.setFont(&FreeSansBold9pt7b);
      tft.setTextSize(1);
      tft.print(delta);
//      tft.print("mV");
      printf("Delta: ");
      printf("%d%%", delta);
      printf("/n");    
    } else {
      tft.drawRect(80,120,48,40,ST77XX_BLACK);
      tft.fillRect(80,120,48,40,ST77XX_BLACK);
      tft.setCursor(84, 153);
      tft.setFont(&FreeSansBold9pt7b);
      tft.setTextSize(1);
      tft.print("N/A");
    }
  }
}

void heater_proc(CAN_FRAME *message)  {
  #ifdef DEBUG
    printFrame(message); 
  #endif
  // if there is any new data, do stuff. If not, do nothing.
  if(message->data.byte[0] != hvPresent || message->data.byte[2] != heating || message->data.byte[3] != templsb || message->data.byte[4] != tempmsb || message->data.byte[5] != targetlsb || message->data.byte[6] != targetmsb) {
    message->data.byte[0] = hvPresent; // HV Present
    message->data.byte[2] = heating; // Heater active
    message->data.byte[3] = templsb; // Water Temp low bit
    message->data.byte[4] = tempmsb; // Water temp high bit
    message->data.byte[5] = targetlsb; // Target temp low bit
    message->data.byte[6] = targetmsb; // Target temp high bit
    int temp = (int)(((unsigned)tempmsb << 8) | templsb ); //test reconstruction - note doesn't handle negative numbers
    int target = (int)(((unsigned)targetmsb << 8) | targetlsb ); //test reconstruction - note doesn't handle negative numbers
    
    #ifdef DEBUG
      Serial.println("Heater Status");
      Serial.print("HV Present: ");
      Serial.print(hvPresent);
      Serial.print(" Heater Active: ");
      Serial.print(heating);
      Serial.print(" Water Temperature: ");
      Serial.print(temp);
      Serial.println("C");
      Serial.println("");
      Serial.println("Settings");
      Serial.print(" Heating: ");
      Serial.print(heating);
      Serial.print(" Desired Water Temperature: ");
      Serial.print(target);
      Serial.println("");
      Serial.println(""); 
    #endif
  // clear screen if new data
  tft.drawRect(0,0,128,10,ST77XX_BLACK);
  tft.fillRect(0,0,128,10,ST77XX_BLACK);
  
  // top row do HV (red or green if enabled) HEAT (red or green if active) TAR (target temp) TMP (actual)
  tft.setCursor(0,9);
  tft.setFont(&FreeSansBold9pt7b);
  tft.setTextSize(1);
  if(hvPresent){
      tft.setTextColor(ST77XX_GREEN);  
  } else {
      tft.setTextColor(ST77XX_RED);  
    }
  tft.print("HV");  
  tft.setCursor(30, 15);
  if(heating){
      tft.setTextColor(ST77XX_GREEN);  
  } else {
      tft.setTextColor(ST77XX_RED);  
    }
  tft.print("HE");  
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(60, 15);  
  tft.print(target);  
  tft.setCursor(90, 15);  
  tft.print(temp);  
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
}

void error_display() {
  // Display error if no can message received
  #ifdef DEBUG
    Serial.println("Error Display");
  #endif
  backlight_ramp_down();
  tft.setTextWrap(false);
  tft.setTextColor(ST77XX_WHITE);
  tft.setRotation(2);
  tft.fillScreen(ST77XX_BLACK);
  tft.setFont(&FreeSansBold9pt7b);
  tft.setCursor(10, 70);
  tft.setTextSize(1);
  tft.print("Error: CAN");
  tft.setCursor(10, 90);
  tft.print("message");
  tft.setCursor(10, 110);
  tft.print("timeout");  
  backlight_ramp_up();
}

void backlight_ramp_up() {
  for(int dutyCycle = 0; dutyCycle <= 255; dutyCycle++){   
  // changing the LED brightness with PWM
  ledcWrite(TFT_BLK_CHAN, dutyCycle);
  delay(15);
  }
  backlight_status = 1;
}
  
void backlight_ramp_down() {  
  for(int dutyCycle = 255; dutyCycle > 0; dutyCycle--){   
  // changing the LED brightness with PWM
  ledcWrite(TFT_BLK_CHAN, dutyCycle);
  delay(15);
  }
  backlight_status = 0;
}
