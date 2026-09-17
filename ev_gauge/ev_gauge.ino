/* Canbus-powered information gauge for DIY EV. Designed for Adafruit 1.8in screen powered by ST7735 driver board.
 * Uses SN65HVD canbus transceiver with ESP32 onboard can
 * OTA updating for software in case the driver is buried in your dash
 * Reads SoC, pack temp, cell delta, motor current, motor temp, 12V aux voltage, car mode,
 * error codes and heater status from Zombieverter-mapped CAN signals (see the CanSignal
 * table and CAN_ID_HEATER below - these CAN IDs are placeholders and MUST be filled in with
 * your Zombieverter CAN map before flashing). The heater sits on its own CAN bus on the far
 * side of Zombieverter, so it's only visible here via Zombieverter's mapping too, same as
 * the battery/motor signals.
 * Display layout changes with car mode (On / Running / Charging) - see drawModeLayout().
 * Single physical display (tft1) - a second display was previously wired in software but
 * never actually installed, so that code has been removed.
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
#include <math.h>

// Image reader
SPIFFS_ImageReader reader;

//#define DEBUG

// OTA CONFIG
const char* ssid = "gaugedriver"; // always-on AP, used as a fallback when not on HOME_WIFI_SSID
const char* password = "123456789";
#include "secrets.h" // HOME_WIFI_SSID / HOME_WIFI_PASSWORD - gitignored, see secrets.h.example

unsigned long ota_progress_millis = 0;

// HEATER DATA
bool hvPresent = false;
bool heater_enabled = false;
bool heating = false;

unsigned char heater_temp = 0;
unsigned char heater_target = 0;
unsigned long heaterTargetChangedAt = 0;

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
#define TFT_1_BLK      19

// PWM for controlling display brightness
const int TFT_FREQ = 5000;
const int TFT_1_BLK_CHAN = 0;
const int RESOLUTION = 8;

//TFT CONFIG
Adafruit_ST7735 tft1 = Adafruit_ST7735(TFT_1_CS, TFT_DC, TFT_SDA, TFT_SCL, TFT_RST);

// Configure CAN TX/RX Pins
#define CAN_RX GPIO_NUM_13
#define CAN_TX GPIO_NUM_15

///////////////////////////////////////////////// ZOMBIEVERTER SIGNAL MAP ////////////////////////////////////////////////////////////
// CAN IDs below are PLACEHOLDERS - replace with the real IDs/offsets from your Zombieverter
// CAN map before flashing. Several of these may end up sharing a single frame once the real
// map is known, in which case they can share one CAN0.watchFor()/callback instead of one each.

struct CanSignal {
  uint32_t id;         // CAN ID this signal is mapped to on the Zombieverter - TODO fill in
  uint8_t  byteOffset;  // first data byte of the signal within the frame
  uint8_t  length;      // 1 or 2 bytes
  bool     isSigned;
  float    scale;       // physical value = raw * scale
};

CanSignal SIG_SOC        = { 0x500, 0, 2, false, 1.0 };   // % , 0-100
CanSignal SIG_PACK_TEMP  = { 0x501, 0, 2, true,  0.1 };   // deg C
CanSignal SIG_CELL_DELTA = { 0x502, 0, 2, false, 1.0 };   // mV
CanSignal SIG_MOTOR_AMPS = { 0x503, 0, 2, true,  1.0 };   // A, DC/pack current, signed (+ traction / - regen).
                                                            // Also reused (sign-inverted) for the Charging-mode
                                                            // charge-current readout - there's no separate charger
                                                            // CAN source any more, see drawChargeCurrent().
CanSignal SIG_MOTOR_TEMP = { 0x504, 0, 1, true,  1.0 };   // deg C
CanSignal SIG_AUX_VOLTS  = { 0x505, 0, 1, false, 0.1 };   // V, 12V aux battery
CanSignal SIG_CAR_MODE   = { 0x506, 0, 1, false, 1.0 };   // enum: 0=On 1=Running 2=Charging
CanSignal SIG_ERROR_CODE = { 0x507, 0, 2, false, 1.0 };   // bitmask of active Zombieverter errors

// The heater controller lives on the heater's own CAN bus, on the far side of Zombieverter -
// it's only visible here once Zombieverter forwards/maps it onto an ID on the main bus, so
// this is a placeholder like the signals above, not a direct read. Assumed to keep the same
// 8-byte layout as before (byte0 HV present, byte1 enabled, byte2 heating active, byte3
// actual water temp, byte4 target water temp) - adjust heater_can_proc() if Zombieverter
// remaps the individual fields instead of forwarding the whole frame.
const uint32_t CAN_ID_HEATER  = 0x508; // TODO Zombieverter-mapped ID for the heater status frame

// Thresholds for exception-only alerting while Running - tune once real signal ranges are known
const float PACK_TEMP_WARN_C  = 45.0;
const int   CELL_DELTA_WARN_MV = 50;
const int   MOTOR_TEMP_WARN_C = 90;
const int   MAX_MOTOR_AMPS    = 400; // full-scale for the amps gauge

///////////////////////////////////////////////// CAR MODE / DISPLAY STATE ////////////////////////////////////////////////////////////

enum CarMode { MODE_ON, MODE_RUNNING, MODE_CHARGING };
CarMode currentMode = MODE_ON;
bool modeChanged = true; // force initial layout draw

// Dirty flags - set by CAN callbacks, cleared by the render task once drawn
enum {
  DIRTY_SOC        = 1 << 0,
  DIRTY_PACKTEMP   = 1 << 1,
  DIRTY_DELTA      = 1 << 2,
  DIRTY_AMPS       = 1 << 3,
  DIRTY_MOTORTEMP  = 1 << 4,
  DIRTY_AUX        = 1 << 5,
  DIRTY_ERROR      = 1 << 6,
  DIRTY_HEATER     = 1 << 7,
};
volatile uint16_t dirtyFlags = 0;

// Live values, updated only by CAN callbacks, drawn only by the render task
volatile int   soc        = -1;   // -1 = unknown / not yet received
volatile float packTemp   = NAN;
volatile int   cellDelta  = -1;
volatile int   motorAmps  = 0;
volatile int   motorTempC = 0;
volatile float auxVolts   = 0;
volatile uint16_t errorCode = 0;

// Task Scheduling - drives the render pass so all TFT drawing happens from one
// consistent context instead of from inside CAN RX callbacks.
void renderTask();

Task renderTick(50, -1, &renderTask); // 50ms -> 20Hz, smooth enough for the amps gauge

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

  runner.addTask(renderTick);
  renderTick.enable();

  pinMode(TFT_RST, OUTPUT);

  // initialize SPIFFS
  if(!SPIFFS.begin()) {
    Serial.println("SPIFFS initialisation failed!");
    while (1);
  }

  // Initialise 1.8" TFT screen:
  tft1.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab

  Serial.print(millis());
  Serial.print("\t");
  Serial.println("TFT Init complete");

  // Setup backlight and set to black
  ledcSetup(TFT_1_BLK_CHAN, TFT_FREQ, RESOLUTION);
  ledcAttachPin(TFT_1_BLK, TFT_1_BLK_CHAN);
  ledcWrite(TFT_1_BLK_CHAN, 0);

  Serial.print(millis());
  Serial.print("\t");
  Serial.println("Backlight prep complete");

  reader.drawBMP("/launch.bmp", tft1, 0, 0);

  Serial.print(millis());
  Serial.print("\t");
  Serial.println("Logo drawn");

  backlight_ramp_up();

  Serial.print(millis());
  Serial.print("\t");
  Serial.println("Backlight ramp complete");

  backlight_ramp_down();

  tft1.setTextWrap(false);
  tft1.setRotation(0); // flipped 180 from the original - display sits upside-down relative to this in the car
  tft1.fillScreen(ST77XX_BLACK);
  #ifdef DEBUG
    Serial.print(millis());
    Serial.print("\t");
    Serial.println("Erased Screen");
  #endif

  backlight_ramp_up();

  // Draw the initial (mode = On) layout before any CAN data has arrived
  drawModeLayout(currentMode);
  modeChanged = false;
  dirtyFlags = 0xFFFF;

  // Initialise CANBus
  #ifdef DEBUG
    Serial.println("Initializing CANBus...");
  #endif
  CAN0.setCANPins(CAN_RX, CAN_TX);
  CAN0.begin(500000);

  // Set up can filters for target IDs
  CAN0.watchFor(CAN_ID_HEATER, 0xFFF);
  CAN0.watchFor(SIG_SOC.id, 0xFFF);
  CAN0.watchFor(SIG_PACK_TEMP.id, 0xFFF);
  CAN0.watchFor(SIG_CELL_DELTA.id, 0xFFF);
  CAN0.watchFor(SIG_MOTOR_AMPS.id, 0xFFF);
  CAN0.watchFor(SIG_MOTOR_TEMP.id, 0xFFF);
  CAN0.watchFor(SIG_AUX_VOLTS.id, 0xFFF);
  CAN0.watchFor(SIG_CAR_MODE.id, 0xFFF);
  CAN0.watchFor(SIG_ERROR_CODE.id, 0xFFF);

  //CAN0.watchFor(); //then let everything else through anyway - enable for debugging

  // Set callbacks for target IDs - order must match the watchFor() calls above
  CAN0.setCallback(0, heater_can_proc);
  CAN0.setCallback(1, soc_can_proc);
  CAN0.setCallback(2, packTemp_can_proc);
  CAN0.setCallback(3, cellDelta_can_proc);
  CAN0.setCallback(4, motorAmps_can_proc);
  CAN0.setCallback(5, motorTemp_can_proc);
  CAN0.setCallback(6, auxVolts_can_proc);
  CAN0.setCallback(7, carMode_can_proc);
  CAN0.setCallback(8, error_can_proc);

  // Run AP and station simultaneously: the "gaugedriver" AP is always reachable for OTA,
  // and the ESP32 also tries to join HOME_WIFI_SSID in the background so the gauge is
  // reachable on the home network without needing to connect to the car's own AP first.
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(ssid, password);
  WiFi.begin(HOME_WIFI_SSID, HOME_WIFI_PASSWORD);
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

  #ifdef DEBUG
    Serial.print(millis());
    Serial.print("\t");
    Serial.println("Ready ...!");
  #endif
}

void loop() {
  runner.execute();
  ElegantOTA.loop();
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

///////////////////////////////////////////////// SIGNAL DECODE ////////////////////////////////////////////////////////////

// Generic decoder for a Zombieverter-mapped signal: little-endian, 1 or 2 bytes, optionally
// signed, scaled to a physical value. Replaces the hand-rolled byte-shift decoding that used
// to be duplicated in every CAN callback.
float decodeSignal(const CanSignal &sig, CAN_FRAME *message) {
  long raw = message->data.byte[sig.byteOffset];
  if (sig.length == 2) {
    raw |= ((long)message->data.byte[sig.byteOffset + 1]) << 8;
  }
  if (sig.isSigned) {
    long signBit = 1L << (sig.length * 8 - 1);
    if (raw & signBit) raw -= (signBit << 1);
  }
  return raw * sig.scale;
}

///////////////////////////////////////////////// CAN CALLBACKS (data only, no drawing) //////////////////////////////////

void heater_can_proc(CAN_FRAME *message) {
  #ifdef DEBUG
    printFrame(message);
  #endif

  static bool prevEnabled = false, prevHeating = false;
  bool changed = false;

  hvPresent = (message->data.byte[0] == 0);      // HV present at heater
  heater_enabled = (message->data.byte[1] > 0);  // Heater enabled
  heating = (message->data.byte[2] > 0);         // Heating is active

  if (heater_enabled != prevEnabled || heating != prevHeating) {
    prevEnabled = heater_enabled;
    prevHeating = heating;
    changed = true;
  }

  if (message->data.byte[4] != heater_target) {
    heater_target = message->data.byte[4];
    heaterTargetChangedAt = millis(); // show the new target for a short delay, then fall back to actual temp
    changed = true;
  }

  if (message->data.byte[3] != heater_temp) {
    heater_temp = message->data.byte[3];
    changed = true;
  }

  if (changed) dirtyFlags |= DIRTY_HEATER;
}

void soc_can_proc(CAN_FRAME *message) {
  #ifdef DEBUG
    printFrame(message);
  #endif
  int v = (int)decodeSignal(SIG_SOC, message);
  if (v != soc) { soc = v; dirtyFlags |= DIRTY_SOC; }
}

void packTemp_can_proc(CAN_FRAME *message) {
  #ifdef DEBUG
    printFrame(message);
  #endif
  float v = decodeSignal(SIG_PACK_TEMP, message);
  if (v != packTemp) { packTemp = v; dirtyFlags |= DIRTY_PACKTEMP; }
}

void cellDelta_can_proc(CAN_FRAME *message) {
  #ifdef DEBUG
    printFrame(message);
  #endif
  int v = (int)decodeSignal(SIG_CELL_DELTA, message);
  if (v != cellDelta) { cellDelta = v; dirtyFlags |= DIRTY_DELTA; }
}

void motorAmps_can_proc(CAN_FRAME *message) {
  #ifdef DEBUG
    printFrame(message);
  #endif
  int v = (int)decodeSignal(SIG_MOTOR_AMPS, message);
  if (v != motorAmps) { motorAmps = v; dirtyFlags |= DIRTY_AMPS; }
}

void motorTemp_can_proc(CAN_FRAME *message) {
  #ifdef DEBUG
    printFrame(message);
  #endif
  int v = (int)decodeSignal(SIG_MOTOR_TEMP, message);
  if (v != motorTempC) { motorTempC = v; dirtyFlags |= DIRTY_MOTORTEMP; }
}

void auxVolts_can_proc(CAN_FRAME *message) {
  #ifdef DEBUG
    printFrame(message);
  #endif
  float v = decodeSignal(SIG_AUX_VOLTS, message);
  if (v != auxVolts) { auxVolts = v; dirtyFlags |= DIRTY_AUX; }
}

void carMode_can_proc(CAN_FRAME *message) {
  #ifdef DEBUG
    printFrame(message);
  #endif
  int raw = (int)decodeSignal(SIG_CAR_MODE, message);
  CarMode m = (raw == 2) ? MODE_CHARGING : (raw == 1) ? MODE_RUNNING : MODE_ON;
  if (m != currentMode) {
    currentMode = m;
    modeChanged = true;
  }
}

void error_can_proc(CAN_FRAME *message) {
  #ifdef DEBUG
    printFrame(message);
  #endif
  uint16_t v = (uint16_t)decodeSignal(SIG_ERROR_CODE, message);
  if (v != errorCode) { errorCode = v; dirtyFlags |= DIRTY_ERROR; }
}

///////////////////////////////////////////////// RENDER TASK ////////////////////////////////////////////////////////////
// Runs on a steady timer via the scheduler - this is the only place that draws to tft1.
// CAN callbacks above only ever update state and dirty flags.

void renderTask() {
  if (modeChanged) {
    drawModeLayout(currentMode);
    modeChanged = false;
    dirtyFlags = 0xFFFF; // redraw every field relevant to the new layout
  }

  if (dirtyFlags & DIRTY_SOC) {
    drawSoc();
    dirtyFlags &= ~DIRTY_SOC;
  }

  switch (currentMode) {
    case MODE_RUNNING:
      // Heater is most worth seeing right here - it's the one thing besides SoC/amps that
      // stays permanently on screen; everything else (temps/delta/errors) is exception-only.
      if (dirtyFlags & DIRTY_AMPS) {
        drawAmpsGauge();
        dirtyFlags &= ~DIRTY_AMPS;
      }
      updateHeaterPanel(134);
      dirtyFlags &= ~(DIRTY_PACKTEMP | DIRTY_DELTA | DIRTY_MOTORTEMP | DIRTY_ERROR | DIRTY_AUX);
      drawExceptionBanner(); // only touches the screen when the alert state actually changes
      break;

    case MODE_CHARGING:
      if (dirtyFlags & DIRTY_AMPS) {
        drawChargeCurrent();
        dirtyFlags &= ~DIRTY_AMPS;
      }
      if (dirtyFlags & DIRTY_PACKTEMP) {
        drawPackTemp(128);
        dirtyFlags &= ~DIRTY_PACKTEMP;
      }
      updateHeaterPanel(156);
      dirtyFlags &= ~(DIRTY_DELTA | DIRTY_MOTORTEMP | DIRTY_ERROR | DIRTY_AUX);
      break;

    case MODE_ON:
    default:
      // No heater panel here - HV (and so the heater) is locked out until Running/Charging.
      if (dirtyFlags & DIRTY_AUX) {
        drawAuxVolts();
        dirtyFlags &= ~DIRTY_AUX;
      }
      if (dirtyFlags & DIRTY_PACKTEMP) {
        drawPackTemp(148);
        dirtyFlags &= ~DIRTY_PACKTEMP;
      }
      dirtyFlags &= ~(DIRTY_AMPS | DIRTY_DELTA | DIRTY_MOTORTEMP | DIRTY_ERROR | DIRTY_HEATER);
      break;
  }
}

///////////////////////////////////////////////// MODE LAYOUT ////////////////////////////////////////////////////////////

void drawModeLayout(CarMode mode) {
  tft1.fillScreen(ST77XX_BLACK);

  tft1.setFont();
  tft1.setTextSize(1);
  tft1.setCursor(4, 10);
  switch (mode) {
    case MODE_RUNNING:
      tft1.setTextColor(ST77XX_GREEN);
      tft1.print("RUNNING");
      break;
    case MODE_CHARGING:
      tft1.setTextColor(ST77XX_CYAN);
      tft1.print("CHARGING");
      break;
    case MODE_ON:
    default:
      tft1.setTextColor(ST77XX_WHITE);
      tft1.print("ON");
      break;
  }

  // SoC panel - shown in every mode
  tft1.drawRoundRect(2, 16, 124, 60, 5, ST77XX_WHITE);

  if (mode == MODE_RUNNING) {
    tft1.drawRect(6, 96, 116, 14, ST77XX_WHITE); // amps gauge border, filled by drawAmpsGauge()
    tft1.setFont(&ev_diy_font);
    tft1.drawChar(6, 134, 128, 0x9515, 0, 1); // heater icon, colour set by updateHeaterPanel()
  } else if (mode == MODE_CHARGING) {
    tft1.setFont(&ev_diy_font);
    tft1.drawChar(6, 100, 129, ST77XX_WHITE, 0, 1); // charge icon
    tft1.drawChar(6, 128, 130, ST77XX_WHITE, 0, 1); // pack temp icon
    tft1.drawChar(6, 156, 128, 0x9515, 0, 1); // heater icon, colour set by updateHeaterPanel() - heater only runs while Charging/Running
  } else { // MODE_ON
    tft1.setFont(&ev_diy_font);
    tft1.drawChar(6, 148, 130, ST77XX_WHITE, 0, 1); // pack temp icon
  }

  tft1.setFont(&ev_diy_font);
  tft1.setTextSize(1);
}

///////////////////////////////////////////////// FIELD RENDERERS ////////////////////////////////////////////////////////////

void drawSoc() {
  tft1.fillRect(4, 18, 120, 56, ST77XX_BLACK);
  tft1.setFont(&FreeSansBold24pt7b);
  tft1.setTextColor(ST77XX_WHITE);
  tft1.setCursor(14, 62);
  if (soc >= 0 && soc <= 100) {
    tft1.print(soc);
    tft1.print("%");
  } else {
    tft1.print("--");
  }
  tft1.setFont(&ev_diy_font);
}

void drawAmpsGauge() {
  const int gx = 6, gy = 96, gw = 116, gh = 14;
  const int cx = gx + gw / 2;

  // numeric readout above the bar - sits in the gap between the SoC box (ends y76) and
  // the bar (starts y96), never on top of either
  tft1.fillRect(gx, gy - 18, gw, 16, ST77XX_BLACK);
  tft1.setFont(&FreeSansBold12pt7b);
  tft1.setTextColor(ST77XX_WHITE);
  tft1.setCursor(gx, gy - 4);
  tft1.print(motorAmps);
  tft1.print("A");
  tft1.setFont(&ev_diy_font);

  // bar, centred on 0A - fills right for traction (+), left for regen (-)
  tft1.fillRect(gx + 1, gy + 1, gw - 2, gh - 2, ST77XX_BLACK);
  tft1.drawFastVLine(cx, gy, gh, ST77XX_WHITE);

  int clamped = constrain(motorAmps, -MAX_MOTOR_AMPS, MAX_MOTOR_AMPS);
  int fillW = map(abs(clamped), 0, MAX_MOTOR_AMPS, 0, gw / 2 - 2);
  uint16_t barColor = (clamped >= 0) ? ST77XX_GREEN : ST77XX_BLUE;
  if (fillW > 0) {
    if (clamped >= 0) {
      tft1.fillRect(cx + 1, gy + 2, fillW, gh - 4, barColor);
    } else {
      tft1.fillRect(cx - 1 - fillW, gy + 2, fillW, gh - 4, barColor);
    }
  }
}

void drawChargeCurrent() {
  // motorAmps is + traction / - regen; while charging the same DC current signal runs the
  // other way, so flip it here rather than show a confusing "-8A" while charging.
  int chargeAmps = -motorAmps;

  tft1.fillRect(36, 84, 84, 20, ST77XX_BLACK);
  tft1.setFont(&FreeSansBold12pt7b);
  tft1.setTextColor(chargeAmps > 0 ? ST77XX_GREEN : ST77XX_WHITE);
  tft1.setCursor(36, 100);
  tft1.print(chargeAmps);
  tft1.print("A");
  tft1.setFont(&ev_diy_font);
}

// baseline matches the pack-temp icon's y in drawModeLayout() for the current mode -
// On and Charging show it at different rows now that Charging also has a heater row.
void drawPackTemp(int baseline) {
  tft1.fillRect(36, baseline - 16, 84, 20, ST77XX_BLACK);
  tft1.setFont(&FreeSansBold12pt7b);
  tft1.setTextColor(!isnan(packTemp) && packTemp >= PACK_TEMP_WARN_C ? 0xFA80 : ST77XX_WHITE);
  tft1.setCursor(36, baseline);
  if (!isnan(packTemp)) {
    tft1.print(packTemp, 1);
    tft1.print("C");
  } else {
    tft1.print("--");
  }
  tft1.setFont(&ev_diy_font);
}

void drawAuxVolts() {
  tft1.fillRect(4, 80, 120, 16, ST77XX_BLACK);
  tft1.setFont();
  tft1.setTextSize(1);
  tft1.setTextColor(auxVolts > 0 && auxVolts < 11.5 ? ST77XX_RED : ST77XX_WHITE);
  tft1.setCursor(6, 90);
  tft1.print("12V: ");
  tft1.print(auxVolts, 1);
  tft1.print("V");
  tft1.setFont(&ev_diy_font);
}

// Heater target/actual display: show the new target for a short delay after it changes,
// then fall back to showing the actual water temperature. Evaluated every render tick so
// the delay expiring (not just a new CAN frame) can trigger the switch-over. Called from
// Running and Charging - the heater's HV is locked out while just On, so there's nothing
// to show there.
void updateHeaterPanel(int baseline) {
  static bool showingTarget = false;
  bool showTargetNow = (millis() - heaterTargetChangedAt) < 1000;

  if (!(dirtyFlags & DIRTY_HEATER) && showTargetNow == showingTarget) {
    return; // nothing changed since the last draw
  }
  showingTarget = showTargetNow;

  tft1.setFont(&ev_diy_font);
  uint16_t iconColor;
  if (heater_enabled) {
    iconColor = heating ? 0xFA80 : ST77XX_WHITE;
  } else {
    iconColor = 0x9515;
  }
  tft1.drawChar(6, baseline, 128, iconColor, 0, 1);

  int valueToShow = showTargetNow ? heater_target : heater_temp;
  uint16_t textColor = showTargetNow ? ST77XX_GREEN : ST77XX_WHITE;

  tft1.fillRect(36, baseline - 16, 60, 18, ST77XX_BLACK);
  tft1.setFont(&FreeSansBold12pt7b);
  tft1.setTextColor(textColor);
  tft1.setCursor(36, baseline);
  tft1.print(valueToShow);
  tft1.setFont(&ev_diy_font);

  dirtyFlags &= ~DIRTY_HEATER;
}

// Running mode only: normally blank. Only draws when pack temp, motor temp, cell delta or an
// active Zombieverter error crosses into "needs attention", and only redraws when that state
// actually changes (cheap to call every render tick).
void drawExceptionBanner() {
  static char lastAlertText[24] = "";
  const int alertY = 136, alertH = 22; // the strip left below the amps gauge and heater row

  char text[24] = "";
  uint16_t color = ST77XX_WHITE;

  if (errorCode != 0) {
    snprintf(text, sizeof(text), "ERR 0x%04X", errorCode);
    color = ST77XX_RED;
  } else if (!isnan(packTemp) && packTemp >= PACK_TEMP_WARN_C) {
    snprintf(text, sizeof(text), "PACK TEMP %.0fC", packTemp);
    color = 0xFA80;
  } else if (motorTempC >= MOTOR_TEMP_WARN_C) {
    snprintf(text, sizeof(text), "MOTOR TEMP %dC", motorTempC);
    color = 0xFA80;
  } else if (cellDelta >= CELL_DELTA_WARN_MV) {
    snprintf(text, sizeof(text), "CELL DELTA %d", cellDelta);
    color = 0xFA80;
  }

  if (strcmp(text, lastAlertText) == 0) {
    return;
  }
  strncpy(lastAlertText, text, sizeof(lastAlertText));

  tft1.fillRect(2, alertY, 124, alertH, ST77XX_BLACK);
  if (text[0] != '\0') {
    tft1.drawRect(2, alertY, 124, alertH, color);
    tft1.setFont();
    tft1.setTextSize(1);
    tft1.setTextColor(color);
    tft1.setCursor(6, alertY + 14);
    tft1.print(text);
    tft1.setFont(&ev_diy_font);
  }
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
    delay(5);
  }
    ledcWrite(TFT_1_BLK_CHAN, 255);
    return;
}

void backlight_ramp_down() {
  for(int dutyCycle = 255; dutyCycle > 0; dutyCycle--){
    // changing the LED brightness with PWM
    ledcWrite(TFT_1_BLK_CHAN, dutyCycle);
    delay(5);
  }
    ledcWrite(TFT_1_BLK_CHAN, 0);
  return;
}
