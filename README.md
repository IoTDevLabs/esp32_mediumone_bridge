# esp32_mediumone_bridge

Bridge application for interfacing between serial UART messages and Medium One IoT Platform using an ESP32-based board. Provides an AT-style command interface on ESP32 UART with commands for connecting to Wi-Fi hotspot, connecting to Medium One MQTT broker, publishing MQTT messages to Medium One, and several utility functions.

Author: Greg Toth

### Target Boards

* Adafruit HUZZAH32
* ESP32 DevKitC and compatible variants

### Development Tools & Libraries

* Arduino IDE
* ESP32 Arduino Core Board Support Package for Arduino IDE
* PubSubClient Library for Arduino IDE
* ArduinoJson Library for Arduino IDE

### Arduino Environment Setup

* Download or clone this repo into a directory on your computer.
* Install Arduino IDE on your computer.
* Launch Arduino IDE.
* Install the ESP Arduno Core Board Support Package by following instructions at https://github.com/espressif/arduino-esp32/blob/master/docs/arduino-ide/boards_manager.md
* In Arduino IDE, from Tools > Manage Libraries... install these libraries:
    * PubSubClient by Nick O'Leary, version 2.7.0 or higher
    * ArduinoJson by Benoit Blanchon, version 6.15.0 or higher
* Open the application source code file esp32_mediumone_bridge.ino from repository directory.

### Configuring the ESP32 Serial Port

Default settings are for Adafruit HUZZAH32 board.

In esp32_mediumone_bridge.ino ```#define CLIENT_SERIAL``` controls which serial port is used for the UART AT command input/output. Only TX & RX lines are used with no hardware flow control lines. Since this app is based on the Arduino programming model, it will typically be ```Serial```, ```Serial1``` or ```Serial2```. Usually ```Serial``` refers to the primary USB serial port on the ESP32 board which may or may not be the one you want to use for AT commands. See the source code for details about UART TX/RX pin mappings for Serialx on certain target boards. For Adafruit HUZZAH32 CLIENT_SERIAL is Serial1, REMAP_UART_PINS is 0, and Serial1 TX & RX are on the board's Feather connector.

### Programming the ESP32 Board

Connect the ESP32 board to your computer using a USB cable, then compile and upload the program to the board.

### Issuing Commands

AT commands and responses are received on the CLIENT_SERIAL port, e.g. Serial1. The CONSOLE_SERIAL port (e.g. Serial on USB) is used for diagnostic messages and can be disconnected when the board is powered from an alternate source.

### AT Commands

AT commands are sent in a single string that must end with a Line Feed character (ASCII 10). Characters are not echoed. Serial UART parameters are 115200,N,8,1 which is set in the source code.

| Command | Description |
|---------|-------------|
| AT | Returns OK. Useful for verifying bridge is responding.
| AT+VER | Returns firmware version.
| AT+RESET | Reset entire bridge application. Disconnects any current connections and forgets any credentials.
| AT+CWIFI={"ssid":"yourssid","password":"yourpassword"} | Connect to Wi-Fi access point. Password can be omitted or left blank if there is no password. Returns OK message if successfully connected or ERROR message if cannot connect.
| AT+DWIFI | Disconnect from Wi-Fi access point, if currently connected.
| AT+SWIFI | Get status of Wi-Fi connection. Returns OK followed by connection status.
| AT+CMQTT={"host":"hostnameorip","port":nnnn,"username":"yourusername","password":"yourpassword"} | Connect to MQTT broker at hostname and port number using specified username and password.
| AT+DMQTT | Disconnect from MQTT broker, if currently connected.
| AT+SMQTT | Get status of MQTT connection. Returns OK followed by connection status.
| AT+PUBLISH={"topic":"publishtopic","msg":"msgtopublish"} | Publish message to connected MQTT broker.
| AT+LED={"blink":3} | Blink LED 3 times (for ESP32 boards that have onboard LED). Other values besides 3 can be used.
| AT+LED={"state":1} | Turn LED on (for ESP32 boards that have onboard LED).
| AT+LED={"state":0} | Turn LED off (for ESP32 boards that have onboard LED).

### Server Root CA Certificate for mqtt.mediumone.com

The bridge application contains the Root CA certificate for mqtt.mediumone.com which is needed for TLS MQTT connections. The current Root CA certificate expires on May 30, 2020 and will no longer work after that date. See root_ca variable in the source code.

