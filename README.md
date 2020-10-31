# esp32_mediumone_bridge

Bridge application for interfacing between serial UART messages and Medium One IoT Platform using an ESP32-based board. Provides an AT-style command interface on ESP32 UART with commands for connecting to Wi-Fi hotspot, connecting to Medium One MQTT broker, publishing MQTT messages to Medium One, and several utility functions.

Author: Greg Toth

### Target Boards

* Adafruit HUZZAH32
* ESP32 DevKitC and compatible variants
* Mikro WiFi BLE Click

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
* For the PubSubClient library, edit Arduino/libraries/PubSubClient/src/PubSubClient.h and change MQTT_MAX_PACKET_SIZE to be 1024 instead of 128.
* Open the application source code file esp32_mediumone_bridge.ino from repository directory.

### Configuring the ESP32 Serial Port

Default settings are for Adafruit HUZZAH32 board.

In esp32_mediumone_bridge.ino ```#define CLIENT_SERIAL``` controls which serial port is used for the UART AT command input/output. Only TX & RX lines are used with no hardware flow control lines. Since this app is based on the Arduino programming model, it will typically be ```Serial```, ```Serial1``` or ```Serial2```. Usually ```Serial``` refers to the primary USB serial port on the ESP32 board which may or may not be the one you want to use for AT commands. See the source code for details about UART TX/RX pin mappings for Serialx on certain target boards. For Adafruit HUZZAH32 CLIENT_SERIAL is Serial1, REMAP_UART_PINS is 0, and Serial1 TX & RX are on the board's Feather connector.

A set of pre-defined board selections are available in the source code based on #defines that begin with ```BOARD_SELECT_```. Each defined board selects an appropriate serial port configuration for that particular board, with variants for swapped TX and RX pins. To select a particular board configuration, set that #define to 1 and set all the rest to 0. The ```BOARD_SELECT_CUSTOM``` selector allows you to define custom settings.

### Programming the ESP32 Board

For ESP32 boards that have a USB serial port, connect the ESP32 board to your computer using a USB cable, then compile and upload the program to the board.

For ESP32 boards without a USB serial port, a USB-to-TTL Serial conversion cable is needed such as Adafruit 954. Instructions for programming a "bare" ESP32 module can be found in various places on the Internet and the process generally follows the steps below.

For the Mikro WiFi BLE Click, program the ESP32 using these steps:

* Power the WiFi BLE Click from a +3.3V DC source connected to mikroBUS 3V3 and GND pins. DO NOT use the USB-to-TTL Serial cable power wire since it is 5V and is too high for the +3.3V needed on the ESP32.
* Connect a USB-to-TTL Serial cable such as Adafruit 954 between the computer and the WiFi BLE Click according to this pinout:
    * Adafruit 954 Black wire (GND) to WiFi BLE Click GND.
    * Adafruit 954 White wire (RX) to TX pin on 1x5 header on WiFi BLE Click.
    * Adafruit 954 Green wire (TX) to RX pin on 1x5 header on WiFi BLE Click.
* Connect EN on 1x5 header on WiFi BLE Click to GND. This enables boot mode during the next ESP32 reset.
* Momentarily connect IO0 on 1x5 header on WiFi BLE Click to GND. This resets the ESP32 and causes it to enter bootloader mode.
* In the Arduino IDE, select board = ESP Dev Module and port = the serial port name for the USB-to-TTL Serial converter cable.
* In the Arduino IDE, upload the program to the ESP32 and monitor the programming status messages in the Arduino IDE.
* After programming, disconnect IO0 from GND and momentarily connect EN to ground to reset the ESP32. It should now restart and run the application program.

### Issuing Commands

AT commands and responses are received on the CLIENT_SERIAL port, e.g. Serial1 or Serial2. The CONSOLE_SERIAL port (e.g. Serial on USB) is used for diagnostic messages and can be disconnected when the board is powered from an alternate source.

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

Note: Explicit use of the hard-coded root CA is disabled by default (see the source code) and normally it shouldn't be needed since the ESP32 has a collection of root CAs built in.

