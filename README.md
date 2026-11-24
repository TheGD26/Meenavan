# Meenavan boat device — run in VS Code with Wokwi

## One-time setup
1. In VS Code, install two extensions: **PlatformIO IDE** and **Wokwi Simulator**.
2. Press Cmd+Shift+P → **Wokwi: Request a New License** (free; signs you in through the browser). Skip if you already did this for RouteVolt.

## Run it
1. File → Open Folder… → choose this `meenavan-device` folder. Wait for PlatformIO to finish setting up (first time downloads the ESP32 tools and the two Adafruit libraries).
2. Build: click the ✓ (Build) icon in the PlatformIO bar at the bottom, or run `pio run` in the terminal.
3. Open `diagram.json`. The circuit appears; press the green ▶ to start the simulation.
   (Or Cmd+Shift+P → **Wokwi: Start Simulator**.)
4. The serial monitor opens in the terminal panel. Type `SOS`, `STATUS` or `RX <hex>` there.

After any code change, build again (step 2), then restart the simulation.

## Files
- `src/main.cpp` — the firmware (same as the wokwi.com sketch, with `#include <Arduino.h>` added)
- `platformio.ini` — ESP32 board and the two libraries
- `diagram.json` — the circuit: ESP32, OLED, TX/RX/BLE LEDs, buzzer, SOS button, battery knob
- `wokwi.toml` — tells Wokwi where the built firmware is (`.pio/build/esp32dev/`)

## What you'll see
Ship list every 5 s (green + blue LEDs), relays and own reports (red LED), WATCH at about 5 s and CLOSE at about 30 s with buzzer beeps, SOS on the red button, LOW when the battery knob is below 11.5 V. Time runs 10x faster than real life. Ships and positions are simulated sample data; the serial monitor stands in for the VHF radio.
