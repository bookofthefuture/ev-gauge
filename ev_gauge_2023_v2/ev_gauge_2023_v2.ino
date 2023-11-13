// Canbus-powered information gauge for DIY EV. Designed for Adafruit 1.8in screen powered by ST7755 driver board. Uses SN65HVD canbus transceiver with ESP32 onboard can
// OTA updating for software in case the driver is buried in your dash

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

const char* ssid = "gaugedriver";
const char* password = "123456789";

unsigned long ota_progress_millis = 0;

AsyncWebServer server(80);

// Include fonts for display
#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>

//#define ESP38_PIN 1 // Set this to 1 if using a 38 pin esp32 module

#ifdef ESP38_PIN
  // Configure i2c pins for display - 38 pin layout
  #define TFT_SDA        12 // PURPLE
  #define TFT_SCL        13 // GREY       
  #define TFT_CS         26 // YELLOW
  #define TFT_RST        14 // BLUE
  #define TFT_DC         27 // GREEN
  Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_SDA, TFT_SCL, TFT_RST);
  // Configure CAN TX/RX Pins
  #define CAN_TX GPIO_NUM_23
  #define CAN_RX GPIO_NUM_22
#else 
  #define ESP38_PIN 0
  // Configure i2c pins for display - 30 pin layout
  #define TFT_SDA        23 // PURPLE  
  #define TFT_SCL        18 // GREY
  #define TFT_CS         5  // YELLOW
  #define TFT_RST        4  // BLUE
  #define TFT_DC         2  // GREEN
  Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_SDA, TFT_SCL, TFT_RST);

  // Configure CAN TX/RX Pins
  #define CAN_TX GPIO_NUM_25
  #define CAN_RX GPIO_NUM_26
#endif

// Pi for circle drawing
float p = 3.1415926;

// Variables for displayed stats
int soc;
int delta;
float temp;

void setup() {
  Serial.begin(115200);

  if (ESP38_PIN == 0) {
    Serial.println("Configured for 30 Pin ESP32 Dev Module");
    } else if (ESP38_PIN == 1) {
    Serial.println("Configured for 38 Pin ESP32 Dev Module");
    } else {
    Serial.println("Pinout configuration error");     
    }
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
  Serial.println("HTTP server started");
  delay(4000);
  
// Initialise CANBus
  Serial.println("Initializing ...");
  CAN0.setCANPins(CAN_RX, CAN_TX);
  CAN0.begin(500000);
  pinMode(TFT_RST, OUTPUT);

// Initialise 1.8" TFT screen:
  tft.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab
  tft.setTextWrap(false);
  tft.setTextColor(ST77XX_WHITE);
  tft.setRotation(2);
  tft.fillScreen(ST77XX_BLACK);

  Serial.println("Ready ...!");

  // Set up can filters for target IDs
  CAN0.watchFor(0x355, 0xFFF); //setup a special filter to watch for only 0x355 to get SoC
  CAN0.watchFor(0x356, 0xFFF); //setup a special filter to watch for only 0x356 to get module temps
  CAN0.watchFor(0x373, 0xFFF); //setup a special filter to watch for only 0x373 to get cell deltas  
  //CAN0.watchFor(); //then let everything else through anyway - enable for debugging

  // Set callbacks for target IDs to process and update display
  CAN0.setCallback(0, soc_proc); //callback on first filter to trigger function to update display with SoC
  CAN0.setCallback(1, temp_proc); //callback on second filter to trigger function to update display with temp
  CAN0.setCallback(2, delta_proc); //callback on third filter to trigger function to update display with delta
 
  // Initial display of SoC before data arrives
//  tft.setFont(&FreeSansBold24pt7b);
//  tft.setCursor(20, 70);
//  tft.setTextSize(1);
  tft.setFont(&FreeSansBold9pt7b);
  tft.setCursor(10, 70);
  tft.setTextSize(1);
  tft.print("Waiting for");
  tft.setCursor(10, 90);
  tft.print("CAN...");
   
  // Initial display of max delta before data arrives
  tft.fillTriangle(68, 154, 74, 135, 80, 154, ST77XX_BLUE);
  tft.setCursor(84, 153);
  tft.setFont(&FreeSansBold12pt7b);
  tft.setTextSize(1);
  tft.print("N/A");

  // Initial display of module temp before data arrives
  tft.fillCircle(8, 140, 2, ST77XX_RED);
  tft.fillCircle(8, 150, 4, ST77XX_RED);
  tft.fillRect(6, 140, 5, 6, ST77XX_RED);
  tft.setCursor(16, 153);
  tft.setFont(&FreeSansBold12pt7b);
  tft.setTextSize(1);
  tft.print("N/A");
 
}

void loop() {
  ElegantOTA.loop();
  CAN_FRAME message;
  if (CAN0.read(message)) {
    printFrame(&message);
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
  printFrame(message);
  if((message->data.byte[1] <<8) + (message->data.byte[0]) != soc){
    soc = (message->data.byte[1] <<8) + (message->data.byte[0]); 
    if(soc > 100) {
      tft.drawRect(0,0,128,100,ST77XX_BLACK);
      tft.fillRect(0,0,128,100,ST77XX_BLACK);
      tft.setCursor(10,70);
      tft.setFont(&FreeSansBold24pt7b);
      tft.setTextSize(1);
      tft.print("ERR");
      printf("SoC error >> SoC: ");
      printf("%d%%", soc);
      printf("/n");            
    } else if(soc) {
      tft.drawRect(0,0,128,100,ST77XX_BLACK);
      tft.fillRect(0,0,128,100,ST77XX_BLACK);
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
}

void temp_proc(CAN_FRAME *message) {
  printFrame(message);
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
  printFrame(message);  
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
