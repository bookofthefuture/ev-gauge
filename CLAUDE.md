# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an ESP32-based CAN bus gauge system for DIY electric vehicles that displays battery state of charge (SoC), temperature, and cell delta values. The system reads data from SimpBMS over CAN bus and displays it on dual 1.8" TFT screens with ST7735 drivers.

## Build and Development Commands

This project uses the Arduino IDE environment for ESP32. No package.json, Makefile, or other standard build tools were found. The main firmware file is `ev_gauge/ev_gauge.ino` which should be compiled and uploaded through the Arduino IDE or PlatformIO.

## Core Architecture

### Main Components

- **Hardware Target**: ESP32 with dual ST7735 TFT displays (1.8")
- **CAN Communication**: Uses SN65HVD transceiver with ESP32's built-in CAN controller
- **Display System**: Dual screen setup (TFT1 and TFT2) with PWM backlight control
- **OTA Updates**: ElegantOTA library for wireless firmware updates
- **File System**: SPIFFS for storing bitmap assets

### Key Files

- `ev_gauge/ev_gauge.ino` - Main firmware file containing all application logic
- `ev_gauge/ev_diy_font.h` - Custom font definition for display icons and characters
- `Font/ev_diy.h` - Alternative font file (appears to be duplicate)
- `ev_gauge/data/launch.bmp` - Startup logo bitmap displayed on boot

### CAN Bus Architecture

The system monitors specific CAN IDs from SimpBMS:
- `0x355` - Battery SoC data
- `0x356` - Module temperature data  
- `0x373` - Cell voltage delta data
- `0x300` - Heater controller information
- `0x389` - Charger status and current

Each CAN ID has dedicated callback functions:
- `soc_proc()` - Updates SoC display (ev_gauge.ino:522)
- `temp_proc()` - Updates temperature display (ev_gauge.ino:572)
- `delta_proc()` - Updates cell delta display (ev_gauge.ino:617)
- `heater_proc()` - Updates heater status (ev_gauge.ino:388)
- `charger_proc()` - Updates charging current (ev_gauge.ino:488)

### Display System

**TFT1 (Primary Display)**:
- SoC percentage (large font)
- Heater temperature/status
- Charging current
- Module temperature
- Cell voltage delta

**TFT2 (Secondary Display)**:
- HV status indicators
- Heater enable/target temperature
- Additional status information

Both displays use:
- Custom icon font for status symbols
- Color coding for warnings (red, orange, white, green)
- Error state handling with "!" indicators

### Key Functions

- `setup()` - Initialize displays, CAN bus, WiFi AP, and OTA (ev_gauge.ino:117)
- `ms10Task()` - 10ms timer task for CAN transmission (ev_gauge.ino:799)
- `backlight_ramp_up()/backlight_ramp_down()` - Smooth display transitions (ev_gauge.ino:772)
- Display initialization functions: `tft1InitialDisplay()`, `tft2InitialDisplay()`

### Pin Configuration

```
TFT_RST: 25, TFT_SDA: 26, TFT_SCL: 27, TFT_DC: 33
TFT_1_CS: 14, TFT_2_CS: 32
TFT_1_BLK: 19, TFT_2_BLK: 21
CAN_RX: 13, CAN_TX: 15
```

### OTA Configuration

- WiFi AP: "gaugedriver" / "123456789"
- ElegantOTA web interface on port 80
- Callback functions for OTA progress tracking

## Development Notes

- Uses `#ifdef DEBUG` preprocessor blocks for serial debugging
- Task scheduler library manages periodic CAN transmission
- Color values are 16-bit RGB565 format
- Custom error handling with visual indicators on displays
- All CAN filters are set to exact match (0xFFF mask)