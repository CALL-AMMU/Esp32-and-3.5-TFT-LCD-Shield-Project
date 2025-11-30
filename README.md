# Esp32-and-3.5-TFT-LCD-Shield-Project
A 3.5" TFT LCD shield is a plug-and-play display module for microcontrollers like Arduino that features a 3.5-inch color screen with 480x320 resolution, four white LED backlights, and individual pixel control.
✅ Project Overview: ESP32-S3 + ILI9488 SPI Display (Motor & Battery Temperature Monitoring)

🚀 Project Summary
I created a real-time temperature monitoring system using an ESP32-S3 microcontroller and a 3.5-inch ILI9488 SPI TFT touch Display. The system continuously displays Motor Temperature and Battery Temperature, making it useful for EV, battery packs, and industrial dashboards.

🧠 About ESP32-S3 (Brief Technical Overview)
The ESP32-S3 is a feature-rich MCU with:
Dual-core Xtensa LX7 @ 240 MHz
512 KB SRAM + external PSRAM support
Wi-Fi 2.4 GHz + BLE 5
USB-OTG
Dedicated AI acceleration instructions
Multiple SPI / I2C / UART interfaces
Low-power operation for battery-based designs


Why ESP32-S3 for this project?
High-speed SPI → smooth, flicker-free graphics
Enough RAM for large TFT buffers
Dual-core → parallel display + sensor processing
Stable and reliable for real-time temperature monitoring

🖥️ About ILI9488 (3.5-inch SPI Display)
Key features of the ILI9488 controller:

480×320 resolution
16M colors (RGB888)
High-speed 4-wire SPI mode
Bright TFT panel
Optional touch support
Smooth rendering with DMA


Why ILI9488?
Sharper and brighter than ILI9341
Ideal for dashboards & graphical UI
Efficient even over SPI at 40–60 MHz


🔌 Hardware Connections (SPI Mode)
ESP32-S3 Pin ILI9488 Pin

3.3V VCC
GND GND
GPIO 13 SCK
GPIO 11 MOSI
GPIO 12 MISO
GPIO 10 CS
GPIO 9 DC
GPIO 8 RST
GPIO 7 LED

🔥 Temperature Output Displayed
The display shows:

1️⃣ Motor Temperature
2️⃣ Battery Temperature

Example:
Temp Motor = 28.19°C
Temp Battery = 28.19°C

📜 Step-by-Step Working Explanation

1️⃣ Sensor Reading
ESP32-S3 reads temperature sensors (NTC, LM35, DS18B20, etc.).
Values are filtered & converted to °C.

2️⃣ SPI Communication Setup
SPI initialized at 40 MHz with DMA for fast, smooth drawing.

3️⃣ ILI9488 Initialization
Commands sent:
Sleep Out (0x11)
Pixel Format (0x3A)
Memory Access Control (0x36)
Display ON (0x29)

4️⃣ Screen Background
Example:
tft.fillScreen(TFT_PURPLE);

5️⃣ Text Rendering
Large fonts for readability:

tft.setCursor(20, 50);
tft.printf("Temp Motor = %.2f", motorTemp);

6️⃣ Continuous Update Loop

Read sensor
Update display
Maintain smooth real-time output


📘 ILI9488 Datasheet Highlights

480×320 pixels
Max SPI 60 MHz
RGB888 16M colors
MADCTL for rotation
GRAM-based fast refresh


🎯 Real-World Applications

EV motor temperature monitoring
Battery pack BMS dashboards
Industrial machines
Automotive sensor displays
IoT monitoring interfaces

⭐ Final Output Summary
I built a complete Motor & Battery Temperature Monitoring System using ESP32-S3 and ILI9488 SPI display, providing a bright, real-time interface for embedded monitoring 
