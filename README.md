Real-time flight telemetry display for GeoFS flight simulator using a Raspberry Pi Pico with TFT display, rotary encoder, and LED indicator.
Demo Link: https://youtu.be/hnZdgzygMNU

Use the Tampermonkey extension to add the geofs-telemetry-sender.js script. Follow the below for guidance:
<img width="1366" height="716" alt="Screenshot (104)" src="https://github.com/user-attachments/assets/77f41332-a248-4b61-ae9a-302353ce292a" />
From Extensions click on Tampermonkey and click on "Create a new script".
<img width="1366" height="695" alt="Screenshot (106)" src="https://github.com/user-attachments/assets/838b78cf-78e7-4d25-8385-bdbfd2f6a6d7" />
Then Paste the javascript code in here and save it!

## Bill of Materials (BOM)

| Part # | Component | Qty | Description |
|--------|-----------|-----|-------------|
| 1 | Raspberry Pi Pico | 1 | Microcontroller
| 2 | ST7735 128x160 TFT Display | 1 | TFT Display | 1.8 inch |
| 3 | Push Button | 1 | User Input | Main button |
| 5 | Rotary Encoder | 1 | Parameter setting User Input | EC11 |

## Wiring Connections

<img width="2833" height="2783" alt="image" src="https://github.com/user-attachments/assets/e7ca1e31-7cd5-4bf5-baf6-1fa5e96f3ca6" />


### TFT Display (SPI)
- **MOSI** → GP27 (SPI TX)
- **SCK** → GP26 (SPI Clock)
- **CS** → GP21
- **DC** → GP19
- **RST** → GP18
- **VCC** → 3V3
- **GND** → GND

### LED
- **Onboard Green LED** → GP25

### Push Buttons
- **Main Button** → GP0
  
### Rotary Encoder
- **CLK (A)** → GP10
- **DT (B)** → GP11 
- **SW** → GP28 
- **+** → 3V3
- **GND** → GND

### Hardware Case

3D printed enclosure designed using fusion 360 (avionics_1.8inch.3mf) CAD model.

### System Architecture

```text
GeoFS Simulator → Tampermonkey Telemetry Script → Serial/Web Communication → Raspberry Pi Pico → TFT Display / LED / Inputs
```

### UI Design
Cockpit-style avionics layout
Typeface: Airbus B612 font used for all cockpit text rendering

## Dependencies

```ini
lib_deps = 
    adafruit/Adafruit ST7735 and ST7789 Library@^1.11.0
    adafruit/Adafruit NeoPixel
```

## Features

- Real-time flight data display (pitch, roll, heading, altitude, airspeed)
- G-load and stall/overspeed and other warning indicators
- Autopilot status display
- Rotary encoder for autpilot parameters adjustment
- LED status indicator
- System data visualization

## Future Plans

Currently working on adding potentiometer for thrust control. Also, will be adding a 1.28 inch round display to add radar system!
