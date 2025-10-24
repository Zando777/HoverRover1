# HoverRover

A project for controlling a hoverboard-based rover using Arduino and ESP32 microcontrollers. Provides serial communication for basic control and a web-based interface for remote operation via WiFi.

## Components

- **ArduinoUno_Hoverboard_Controller**: Arduino Nano code for direct serial control of hoverboard motors.
- **ESP32_Hoverboard_Controller**: ESP32-based web server with a user interface for speed and steering control, including real-time feedback.

## Hardware Requirements

- Hoverboard with hacked firmware (e.g., from https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC)
- Arduino Nano (for serial control)
- ESP32 board (for web control)
- Appropriate wiring and power supply

## Software Requirements

- Arduino IDE for uploading `.ino` files
- PlatformIO for ESP32 project
- WiFi network for ESP32 web interface

## Setup

1. **Arduino Serial Control**:
   - Upload `hoverserial.ino` to Arduino Nano.
   - Connect to hoverboard serial interface.

2. **ESP32 Web Control**:
   - Open `ESP32_Hoverboard_Controller` in PlatformIO.
   - Update WiFi credentials in `main.cpp`.
   - Build and upload to ESP32.
   - Access web interface at ESP32's IP address.

## Usage

- **Serial**: Arduino sends speed commands and receives feedback.
- **Web**: Use sliders to control speed/steer, view battery/temp data.

## License

Based on hoverboard firmware hack by Emanuel FERU.
