# ev-gauge
SoC, motor amps, motor/pack temp, 12V aux voltage, car mode and Zombieverter error
data over CANbus, on a single mode-aware display. For ESP32.

Copy `ev_gauge/secrets.h.example` to `ev_gauge/secrets.h` and fill in your home WiFi
SSID/password before flashing - the gauge runs its own `gaugedriver` AP and joins that
network at the same time, so OTA updates work both standalone and from your desk.
