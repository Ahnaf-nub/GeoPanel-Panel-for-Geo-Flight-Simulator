Real-time flight telemetry display for GeoFS flight simulator using a Raspberry Pi Pico with TFT display, rotary encoder, and RGB LED indicators.

## Bill of Materials (BOM)

| Part # | Component | Qty | Description |
|--------|-----------|-----|-------------|
| 1 | Raspberry Pi Pico | 1 | Microcontroller
| 2 | Adafruit ST7735 80x160 TFT Display | 1 | TFT Display | 0.96 inch |
| 3 | Adafruit NeoPixel RGB LED (Optional) | 1 | Programmable LED |
| 4 | Push Button | 2 | User Input | Main button + Autopilot toggle |
| 5 | Rotary Encoder | 1 | Menu Navigation | KY-040 or similar |

## Wiring Connections

### TFT Display (SPI)
- **MOSI** → GP8 (SPI TX)
- **MISO** → GP4
- **SCK** → GP6 (SPI Clock)
- **CS** → GP26
- **DC** → GP22
- **RST** → GP21
- **VCC** → 3V3
- **GND** → GND

### NeoPixel RGB LED
- **Data** → GP24
- **VCC** → 3V3 
- **GND** → GND

### Push Buttons
- **Main Button** → GP28
- **Autopilot Toggle** → GP25 
- **Other Pin** → GND

### Rotary Encoder
- **CLK (A)** → GP10
- **DT (B)** → GP11 
- **SW** → GP28 
- **+** → 3V3
- **GND** → GND

## Dependencies

```ini
lib_deps = 
    adafruit/Adafruit ST7735 and ST7789 Library@^1.11.0
    adafruit/Adafruit NeoPixel
```

## Features

- Real-time flight data display (pitch, roll, heading, altitude, airspeed)
- G-load and stall/overspeed indicators
- Autopilot status display
- Rotary encoder for autpilot parameters adjustment
- RGB LED status indicator
- Engine data visualization
- Gear position animation
