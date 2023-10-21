
// TFT Connections
// SCL: GPIO12
// SDA: GPIO26
// RST: GPIO27
// RS/DC:  GPIO14
// CS   GPI013
// BLK 3v3 (Backlight)


// CAN_RXD = ESP32 – IO25
// CAN_TXD = ESP32 – IO26

// SIMPBMS CANBUS SIGNALS
// 0x355 BYTE 0 SOC LSB SCALE 1
// 0x355 BYTE 1 SOC MSB SCALE 1
// 0x356 BYTE 4 TEMP LSB SCALE 0.1
// 0x356 BYTE 5 TEMP MSB SCALE 0.1
// 0x373 BYTE 0 Min Cell Voltage LSB
// 0x373 BYTE 1 Min Cell Voltage MSB
// 0x373 BYTE 2 Max Cell Voltage LSB
// 0x373 BYTE 3 Max Cell Voltage MSB

// CONTACTOR STATUS CAN (TBC)
// 0x123 BYTE 0 CON 1 STATUS (0 OPEN, 1 CLOSED)
// 0x123 BYTE 1 CON 2 STATUS (0 OPEN, 1 CLOSED)
// 0x123 BYTE 2 CON 3 STATUS (0 OPEN, 1 CLOSED)
// 0x123 BYTE 3 CON 4 STATUS (0 OPEN, 1 CLOSED)

//Include libraries for display and can communications
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <SPI.h>
#include <Arduino.h>
#include <esp32_can.h> // ESP32 native can library

// Include fonts for display
#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>

// Configure i2c pins for display
#define TFT_SDA        26        
#define TFT_SCL        12
#define TFT_CS         13
#define TFT_RST        27
#define TFT_DC         14
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_SDA, TFT_SCL, TFT_RST);

// Pi for circle drawing
float p = 3.1415926;

// Variables for displayed stats
int soc;
int delta;
float temp;

void setup() {
  Serial.begin(115200);
  
// Initialise CANBus
  Serial.println("Initializing ...");
  CAN0.setCANPins(GPIO_NUM_23, GPIO_NUM_22);
  CAN0.begin(500000);

// Initialise 1.8" TFT screen:
  tft.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab
  tft.setTextWrap(false);
  tft.setTextColor(ST77XX_WHITE);
  tft.setRotation(2);
  tft.fillScreen(ST77XX_BLACK);

  Serial.println("Ready ...!");

  // Set up can filters for target IDs
  CAN0.watchFor(0x355, 0xF00); //setup a special filter to watch for only 0x355 to get SoC
  CAN0.watchFor(0x355, 0xF00); //setup a special filter to watch for only 0x356 to get module temps
  CAN0.watchFor(0x373, 0xF00); //setup a special filter to watch for only 0x373 to get cell deltas  
  CAN0.watchFor(); //then let everything else through anyway - enable for debugging

  // Set callbacks for target IDs to process and update display
  CAN0.setCallback(0, soc_proc); //callback on first filter to trigger function to update display with SoC
  CAN0.setCallback(1, temp_proc); //callback on second filter to trigger function to update display with temp
  CAN0.setCallback(2, delta_proc); //callback on third filter to trigger function to update display with delta
 
  // Initial display of SoC before data arrives
  tft.setFont(&FreeSansBold24pt7b);
  tft.setCursor(20, 70);
  tft.setTextSize(1);
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
  if((message->data.byte[0] + (message->data.byte[1] <<8)) != soc){
    soc = message->data.byte[0] + (message->data.byte[1] <<8); 
    tft.setCursor(20, 40);
    tft.setTextSize(6);
    if(soc) {
      tft.print(soc);
      tft.print("%");
      printf("SoC: ");
      printf("%d%%", soc);
      printf("/n");      
    } else {
      tft.print("N/A");
    }  
  }
}

void temp_proc(CAN_FRAME *message) {
  printFrame(message);
  if((10*(message->data.byte[4] + (message->data.byte[5] <<8))) != temp) {
    temp = 10*(message->data.byte[4] + (message->data.byte[5] <<8));  
    // Module Temp
    tft.fillCircle(10, 140, 2, ST77XX_RED);
    tft.fillCircle(10, 150, 4, ST77XX_RED);
    tft.fillRect(8, 140, 5, 6, ST77XX_RED);
    tft.setCursor(20, 140);
    tft.setTextSize(2);
    if(temp) {
      tft.print(temp);
      printf("Temp: ");
      printf("%d%%", temp);
      printf("/n");
    } else {
      tft.print("N/A");
    }
  }
}

void delta_proc(CAN_FRAME *message) {
  printFrame(message);  
  if((message->data.byte[2] + (message->data.byte[2] <<8))-(message->data.byte[0] + (message->data.byte[1] <<8)) != delta) {
    delta = (message->data.byte[2] + (message->data.byte[2] <<8))-(message->data.byte[0] + (message->data.byte[1] <<8));
  // Max Delta
  tft.fillTriangle(25, 24, 35, 8, 44, 24, ST77XX_BLUE);
  tft.setCursor(50, 10);
  tft.setTextSize(2);
  if(delta) {
    tft.print(delta);
    tft.print("mV");
    printf("Delta: ");
    printf("%d%%", delta);
    printf("/n");    
    } else {
    tft.print("N/A");
    }
  }
}
