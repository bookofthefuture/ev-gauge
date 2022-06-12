
// TFT Connections
// SCL: GPIO18
// SDA: GPIO23
// RS/DC:  GPIO2
// RST: GPIO4
// CS   GPIO5
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

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <SPI.h>
#include <Arduino.h>
#include <ESP32CAN.h>
#include <CAN_config.h>

#define TFT_CS         5
#define TFT_RST        4
#define TFT_DC         2

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
CAN_device_t CAN_cfg;               // CAN Config
unsigned long previousMillis = 0;   // will store last time a CAN Message was send
const int interval = 1000;          // interval at which send CAN Messages (milliseconds)
const int rx_queue_size = 10;       // Receive Queue size

float p = 3.1415926;

int soc;
int delta;
float temp;
int con1 = 0; // neg
int con2 = 0; // pos
int con3 = 0; // pack2neg
int con4 = 0; // pack2pos

void setup(void) {
  Serial.begin(115200);
  Serial.println("Basic Demo - ESP32-Arduino-CAN");
  CAN_cfg.speed = CAN_SPEED_500KBPS;
  CAN_cfg.tx_pin_id = GPIO_NUM_26;
  CAN_cfg.rx_pin_id = GPIO_NUM_25;
  CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
  // Init CAN Module
  ESP32Can.CANInit();
 
  // Use this initializer if using a 1.8" TFT screen:
  tft.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab
  tft.setTextWrap(false);
  tft.setTextColor(ST77XX_WHITE);
  tft.setRotation(2);
}

void loop() {
  CAN_frame_t rx_frame;

  unsigned long currentMillis = millis();

  // Receive next CAN frame from queue
  if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE) {
    switch (rx_frame.MsgID) {
    case 0x355:
          soc_proc(rx_frame);
          break;
    case 0x356:
          temp_proc(rx_frame);
          break;
    case 0x373:
          delta_proc(rx_frame);
          break;
    case 0x123:
          con_proc(rx_frame);
          break;
    default:
          break;      
      }
    }
  tft.fillScreen(ST77XX_BLACK);
  
  //SoC
  tft.setCursor(20, 40);
  tft.setTextSize(6);
  if(soc) {
    tft.print(soc);
    tft.print("%");
  } else {
    tft.print("N/A");
    }

  // Max Delta
  tft.fillTriangle(25, 24, 35, 8, 44, 24, ST77XX_BLUE);
  tft.setCursor(50, 10);
  tft.setTextSize(2);
  if(delta) {
    tft.print(delta);
    tft.print("mV");
  } else {
    tft.print("NA");
  }

  // Module Temp
  tft.fillCircle(35, 98, 2, ST77XX_RED);
  tft.fillCircle(35, 108, 4, ST77XX_RED);
  tft.fillRect(33, 100, 5, 6, ST77XX_RED);
  tft.setCursor(50, 98);
  tft.setTextSize(2);
  if(temp) {
    tft.print(temp);
  } else {
    tft.print("N/A");
  }
  
  // Contactor Status 
  if(con1) {
    tft.fillCircle(22, 140, 11, ST77XX_GREEN);
  } else {
    tft.fillCircle(22, 140, 11, ST77XX_RED);
  }
  if(con2) {
    tft.fillCircle(50, 140, 11, ST77XX_GREEN);
  } else {
    tft.fillCircle(50, 140, 11, ST77XX_RED);   
  }
  if(con3) {
    tft.fillCircle(78, 140, 11, ST77XX_GREEN);
  } else {
    tft.fillCircle(78, 140, 11, ST77XX_RED);
  }
  if(con4) {
    tft.fillCircle(106, 140, 11, ST77XX_GREEN);
  } else {
    tft.fillCircle(106, 140, 11, ST77XX_RED);
     }
  tft.setCursor(18, 133);
  tft.print("1");
  tft.setCursor(46, 133);
  tft.print("2");
  tft.setCursor(74, 133);
  tft.print("3");
  tft.setCursor(102, 133);
  tft.print("4");  

}
  
void soc_proc(CAN_frame_t rx_frame) {
  int soc = rx_frame.data.u8[0] + (rx_frame.data.u8[1] <<8);  
  printf("SoC: ");
  printf("%d%%", soc);
  printf("\n");
}

void temp_proc(CAN_frame_t rx_frame) {
  temp = 10*(rx_frame.data.u8[4] + (rx_frame.data.u8[5] <<8));  
  printf("Temp: ");
  printf("%d%%", temp);
  printf("\n");
}

void delta_proc(CAN_frame_t rx_frame) {
  delta = (rx_frame.data.u8[2] + (rx_frame.data.u8[2] <<8))-(rx_frame.data.u8[0] + (rx_frame.data.u8[1] <<8));  
  printf("%d%%", delta);
  printf("\n");
}
void con_proc(CAN_frame_t rx_frame) {
  con1 = rx_frame.data.u8[0];
  con2 = rx_frame.data.u8[1];
  con3 = rx_frame.data.u8[2];
  con4 = rx_frame.data.u8[3];
  }
