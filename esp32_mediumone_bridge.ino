/*
 * esp32_mediumone_bridge.ino
 * 
 * Bridge application running on ESP32 providing simplified
 * AT commands on serial UART for connecting to Wi-Fi access
 * point and interfacing with Medium One IoT platform over MQTT.
 * 
 * Copyright (c) 2020 Greg Toth. All rights reserved.
 * 
 * Dependencies:
 * 
 *    -- ESP32 Arduino Board Support Package
 *    -- PubSubClient Library
 *    -- ArduinoJson Library
 * 
 */

#define IDENT     "MediumOne ESP32 Bridge"
#define VERSION   "0.8.0"

#include <WiFiClientSecure.h>   // https://github.com/espressif/arduino-esp32/tree/master/libraries/WiFiClientSecure
#define MQTT_MAX_PACKET_SIZE 1024
#include <PubSubClient.h>
#include <ArduinoJson.h>

#define CONSOLE_SERIAL    Serial      /* console serial port for info & debug messages */
#define CLIENT_SERIAL     Serial1     /* client serial port for incoming AT commands */
#define REMAP_UART_PINS   0           /* 0=use default TX/RX pin mappings, 1=remap to alternate pins */
/* Defaults on Adafruit HUZZAH32 ESP32 Feather:
 *  Serial RX & TX are mapped to USB-to-Serial converter.
 *  Serial1 RX is mapped to GPIO16 = chip pin 27 (IO16) = JP3-3 (RX),
 *  Serial1 TX is mapped to GPIO17 = chip pin 28 (IO17) = JP3-2 (TX).
 *  For HUZZAH32 use CONSOLE_SERIAL=Serial, CLIENT_SERIAL=Serial1 and REMAP_UART_PINS=0.
 * Defaults on ESP32 DevKitC:
 *  Serial RX & TX are mapped to USB-to-Serial converter.
 *  which is also connected to TXD0 (J3-4) & RXD0 (J3-5). 
 *  Serial1 RX is mapped to GPIO9 = chip pin 17 = SD2 = J2-16,  [spi flash]
 *  Serial1 TX is mapped to GPIO10 = chip pin 18 = SD3 = J2-17. [spi flash]
 *  Serial2 RX is mapped to GPIO16 = chip pin 27 = IO16 = J3-12,
 *  Serial2 TX is mapped to GPIO17 = chip pin 28 = IO17 = J3-11.
 *  For DevKitC use CONSOLE_SERIAL=Serial and either CLIENT_SERIAL=Serial1 and REMAP_UART_PINS=1,
 *    or CLIENT_SERIAL=Serial2 and REMAP_UART_PINS=0.
 */
#if defined(REMAP_UART_PINS) && REMAP_UART_PINS == 1
/* Optional serial RX & TX port pin remapping used with CLIENT_SERIAL. */
#define CLIENT_SERIAL_RX_GPIO   4   /* GPIO4 = chip pin 26 = IO4 = J3-13 on DevKitC */
#define CLIENT_SERIAL_TX_GPIO   2   /* GPIO2 = chip pin 24 = IO2 = J3-15 on DevKitC */
#endif
/* Serial data rates */
#define CONSOLE_SERIAL_BAUD   115200
#define CLIENT_SERIAL_BAUD    115200
/* Control whether onboard LED is blinked when commands are processed */
#define CONTROL_LED     1           /* 0=no LED control, 1=control LED */
/* Number of LED blinks for OK or ERROR condition (when CONTROL_LED == 1) */
#define OK_BLINKS       1
#define ERROR_BLINKS    3

#define CHECKPOINT(msg) CONSOLE_SERIAL.println(msg);
//#define CHECKPOINT(msg)

//WiFiClient          wifiClient;     // Non-TLS
WiFiClientSecure    wifiClient;     // TLS
PubSubClient        mqttClient(wifiClient);
int                 wifiConnectActive = 0;
char                wifiSSID[40] = "";
int                 mqttConnectActive = 0;
char                mqttHostname[80] = "";
int                 mqttPort = 0;
char                mqttUsername[80] = "";
char                mqttPassword[80] = "";
char                mqttClientId[80] = "ESP32Bridge";

/* Root CA for mqtt.mediumone.com. Expires May 30, 2020. */
const char *root_ca = \
"-----BEGIN CERTIFICATE-----\n" \
"MIIFMDCCBBigAwIBAgIQV5SFWjNqKQ7ivjk88Axt3zANBgkqhkiG9w0BAQsFADBv\n" \
"MQswCQYDVQQGEwJTRTEUMBIGA1UEChMLQWRkVHJ1c3QgQUIxJjAkBgNVBAsTHUFk\n" \
"ZFRydXN0IEV4dGVybmFsIFRUUCBOZXR3b3JrMSIwIAYDVQQDExlBZGRUcnVzdCBF\n" \
"eHRlcm5hbCBDQSBSb290MB4XDTE0MTIyMjAwMDAwMFoXDTIwMDUzMDEwNDgzOFow\n" \
"gZQxCzAJBgNVBAYTAkdCMRswGQYDVQQIExJHcmVhdGVyIE1hbmNoZXN0ZXIxEDAO\n" \
"BgNVBAcTB1NhbGZvcmQxGjAYBgNVBAoTEUNPTU9ETyBDQSBMaW1pdGVkMTowOAYD\n" \
"VQQDEzFDT01PRE8gU0hBLTI1NiBEb21haW4gVmFsaWRhdGlvbiBTZWN1cmUgU2Vy\n" \
"dmVyIENBMIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEApe7aGrg+6WaE\n" \
"MyvsWocOnfpDXsxsmrOezwlTbnGbsFkuEU4wuOIVLaL2F4H5gX9fFinG7AFz7tdG\n" \
"6wq0if2JvKj4us9ewrm+NPvu/pAKAXcr4TavNHLBXeUWdpLCjP/gaAev9W3O07l4\n" \
"X8jxZ0DBh1hPUnVyGKUveZCwBAGPasSYlQfo4PcLPilqW/vHZCCOV8l8HW0CeuPO\n" \
"VPAIZYaXicngMuyRGo790YSj759Gu8wAMHxMBelVSbBwgb1gAYqANFuLJQvC57oW\n" \
"XYa8w/zKn/QJWkkfwkLyqNwhSbnQuoQByVSsGTf2ItQQzzQS+5nl1bF2pTZQfK7Q\n" \
"W36GGtK2AwIDAQABo4IBoDCCAZwwHwYDVR0jBBgwFoAUrb2YejS0Jvf6xCZU7wO9\n" \
"4CTLVBowHQYDVR0OBBYEFB6sP/wP4Sf1OpwFluiLM3n3dXnxMA4GA1UdDwEB/wQE\n" \
"AwIBhjASBgNVHRMBAf8ECDAGAQH/AgEAMB0GA1UdJQQWMBQGCCsGAQUFBwMBBggr\n" \
"BgEFBQcDAjAbBgNVHSAEFDASMAYGBFUdIAAwCAYGZ4EMAQIBMEQGA1UdHwQ9MDsw\n" \
"OaA3oDWGM2h0dHA6Ly9jcmwudXNlcnRydXN0LmNvbS9BZGRUcnVzdEV4dGVybmFs\n" \
"Q0FSb290LmNybDCBswYIKwYBBQUHAQEEgaYwgaMwPwYIKwYBBQUHMAKGM2h0dHA6\n" \
"Ly9jcnQudXNlcnRydXN0LmNvbS9BZGRUcnVzdEV4dGVybmFsQ0FSb290LnA3YzA5\n" \
"BggrBgEFBQcwAoYtaHR0cDovL2NydC51c2VydHJ1c3QuY29tL0FkZFRydXN0VVRO\n" \
"U0dDQ0EuY3J0MCUGCCsGAQUFBzABhhlodHRwOi8vb2NzcC51c2VydHJ1c3QuY29t\n" \
"MA0GCSqGSIb3DQEBCwUAA4IBAQCsPFl52og0fw7ZBzEwE0vqi8k2JA/nlQCkXv3A\n" \
"rJ8aSqmXjuAwmXUXfZlnaZMWz8nVcT+DnmxWGeqCSqtB3Fvgtmg7fO22aGAJ3H/I\n" \
"zTxPd0x+1BebdDpOB3XS/It5Wt8FGKtnJtM+jG76ESoy5DpwTHdwi+ZF9bDlJkBJ\n" \
"NwImRkjJWMMPPIJsWE03d2qs6KjxBD/B11wf+aevKFIQ0dN6pt7Gc/0AepKDRW83\n" \
"+7eYEqUoBfjcoeK86NSDwpHSXIqaVa3p0SFlSQ16VYE0YWHCPvCzI4bZJGlePq4g\n" \
"z5Z2KDJ4sJ7bbAtAU44vI1wTv2aFpY45RAIS4nAFjvwYPRVY\n" \
"-----END CERTIFICATE-----\n";

/* Client result codes */
#define RESULT_OK                           "OK"
#define RESULT_OK_WIFI_IS_CONNECTED         "OK WIFI IS CONNECTED"
#define RESULT_OK_WIFI_IS_DISCONNECTED      "OK WIFI IS DISCONNECTED"
#define RESULT_OK_WIFI_NOT_CONNECTED        "OK WIFI IS NOT CONNECTED"
#define RESULT_OK_MQTT_IS_CONNECTED         "OK MQTT IS CONNECTED"
#define RESULT_OK_MQTT_IS_DISCONNECTED      "OK MQTT IS DISCONNECTED"
#define RESULT_OK_MQTT_NOT_CONNECTED        "OK MQTT IS NOT CONNECTED"
#define RESULT_ERROR_BAD_JSON               "ERROR BAD JSON"
#define RESULT_ERROR_SSID_MISSING           "ERROR SSID MISSING"
#define RESULT_ERROR_WIFI_CONNECT_FAILED    "ERROR WIFI CONNECT FAILED"
#define RESULT_ERROR_HOST_OR_PORT_MISSING   "ERROR HOST AND/OR PORT MISSING"
#define RESULT_ERROR_MQTT_CONNECT_FAILED    "ERROR MQTT CONNECT FAILED"
#define RESULT_ERROR_TOPIC_OR_MSG_MISSING   "ERROR TOPIC AND/OR MSG MISSING"
#define RESULT_ERROR_TOPIC_IS_BLANK_OR_BAD  "ERROR TOPIC IS BLANK OR BAD FORMAT"
#define RESULT_ERROR_MSG_IS_BLANK_OR_BAD    "ERROR MSG IS BLANK OR BAD FORMAT"
#define RESULT_ERROR_MQTT_RECONNECT_FAILED  "ERROR MQTT RE-CONNECT FAILED"
#define RESULT_ERROR_MQTT_PUBLISH_FAILED_NOT_CONNECTED  "ERROR PUBLISH FAILED MQTT NOT CONNECTED"
#define RESULT_ERROR_MQTT_PUBLISH_FAILED    "ERROR PUBLISH FAILED"

#if defined(CONTROL_LED) && CONTROL_LED == 1
#define BLINK_LED_OK()    blink_led(OK_BLINKS)
#define BLINK_LED_ERROR() blink_led(ERROR_BLINKS)
void blink_led(int count)
{
  if (count <= 0) {
    return;
  }
  do {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(25);
    digitalWrite(LED_BUILTIN, LOW);
    if (count > 1) {
      delay(150);
    }
  } while (--count > 0);
}

void cmdLed(char *payload)
{
  // {"blink":3}
  // {"state":1}
  // {"state":0}
  StaticJsonDocument<256> json_payload;
  DeserializationError err = deserializeJson(json_payload, payload);
  if (err) {
    CONSOLE_SERIAL.println("ERROR: Could not parse JSON payload");
    CLIENT_SERIAL.println(RESULT_ERROR_BAD_JSON);
  } else if (json_payload.containsKey("blink")) {
    int count = json_payload["blink"].as<int>();
    CONSOLE_SERIAL.print("Blinking LED ");
    CONSOLE_SERIAL.print(count);
    CONSOLE_SERIAL.println(" times");
    blink_led(count);
    CLIENT_SERIAL.println(RESULT_OK);
  } else if (json_payload.containsKey("state")) {
    int state = json_payload["state"].as<int>();
    CONSOLE_SERIAL.print("Setting LED to ");
    CONSOLE_SERIAL.println(state);
    digitalWrite(LED_BUILTIN, state);
    CLIENT_SERIAL.println(RESULT_OK);
  } else {
    CONSOLE_SERIAL.println("ERROR: Unrecognized LED parameters");
    CLIENT_SERIAL.println(RESULT_ERROR_BAD_JSON);
  }
}
#else
#define BLINK_LED_OK()
#define BLINK_LED_ERROR()
#endif

void cmdEmpty(char *payload)
{
  BLINK_LED_OK();
  CLIENT_SERIAL.println(RESULT_OK);
}

void cmdReset(char *payload)
{
  BLINK_LED_OK();
  CLIENT_SERIAL.println(RESULT_OK);
  CONSOLE_SERIAL.println("Resetting board");
  ESP.restart();
}

void cmdVersion(char *payload)
{
  BLINK_LED_OK();

  CONSOLE_SERIAL.println("Resetting board");
  CONSOLE_SERIAL.print(IDENT); CONSOLE_SERIAL.print(" "); CONSOLE_SERIAL.print(VERSION);
  CONSOLE_SERIAL.println(" (Serial)");
  CLIENT_SERIAL.print(RESULT_OK); CLIENT_SERIAL.print(" ");
  CLIENT_SERIAL.print(IDENT); CLIENT_SERIAL.print(" "); CLIENT_SERIAL.println(VERSION); 
  ESP.restart();
}

void cmdConnectWifi(char *payload)
{
  // {"ssid":"yourssid"[,"password":"yourpassword"]}
  StaticJsonDocument<256> json_payload;
  DeserializationError err = deserializeJson(json_payload, payload);
  if (err) {
    BLINK_LED_ERROR();
    CONSOLE_SERIAL.println("ERROR: Could not parse JSON payload");
    CLIENT_SERIAL.println(RESULT_ERROR_BAD_JSON);
  } else if (!json_payload.containsKey("ssid")) {
    BLINK_LED_ERROR();
    CONSOLE_SERIAL.println("ERROR: SSID is missing");
    CLIENT_SERIAL.println(RESULT_ERROR_SSID_MISSING);
  } else {
    int retry = 3;
    int countdown;
    while (WiFi.status() != WL_CONNECTED && retry > 0) {
      CONSOLE_SERIAL.print("Connecting to Wi-Fi SSID ");
      CONSOLE_SERIAL.print((const char*)json_payload["ssid"]);
      CONSOLE_SERIAL.print("...");
      WiFi.begin((const char*)json_payload["ssid"], (const char*)json_payload["password"]);
      delay(1000);
      countdown = 5;
      while (WiFi.status() != WL_CONNECTED && countdown > 0) {
        CONSOLE_SERIAL.print(".");
        if (--countdown > 0) {
          delay(1000);
        }
      }
    retry--;
    }
    CONSOLE_SERIAL.println();
    if (WiFi.status() == WL_CONNECTED) {
      wifiConnectActive = 1;
      CONSOLE_SERIAL.print("Wi-Fi connection established, obtained IP address: "); CONSOLE_SERIAL.println(WiFi.localIP());
      BLINK_LED_OK();
      CLIENT_SERIAL.print(RESULT_OK_WIFI_IS_CONNECTED);
      CLIENT_SERIAL.print(" ");
      CLIENT_SERIAL.println(WiFi.localIP());
    } else {
      wifiConnectActive = 0;
      CONSOLE_SERIAL.println("Wi-Fi connection failed!");
      BLINK_LED_ERROR();
      CLIENT_SERIAL.println(RESULT_ERROR_WIFI_CONNECT_FAILED);
    }
  }
}

void cmdDisconnectWifi(char *payload)
{
  
  if (WiFi.status() == WL_CONNECTED) {
    CONSOLE_SERIAL.println("Disconnecting from Wi-Fi");
    WiFi.disconnect();
  } else {
    CONSOLE_SERIAL.println("Wi-Fi is not currently connected");
  }
  wifiConnectActive = 0;
  BLINK_LED_OK();
  CLIENT_SERIAL.println(RESULT_OK_WIFI_IS_DISCONNECTED);
}

void cmdStatusWifi(char *payload)
{
  BLINK_LED_OK();
  if (WiFi.status() == WL_CONNECTED) {
    CONSOLE_SERIAL.println("Wi-Fi is connected");
    CLIENT_SERIAL.println(RESULT_OK_WIFI_IS_CONNECTED);
  } else {
    CONSOLE_SERIAL.println("Wi-Fi is not currently connected");
    CLIENT_SERIAL.println(RESULT_OK_WIFI_NOT_CONNECTED);
  }
}

void cmdConnectMqtt(char *payload)
{
  // {"host":"hostname or ip","port":8883[,"username":"yourusername","password":"yourpassword"]}
  if (mqttClient.connected()) {
    CONSOLE_SERIAL.println("Disconnecting MQTT connection");
    mqttClient.disconnect();
  }
  StaticJsonDocument<256> json_payload;
  DeserializationError err = deserializeJson(json_payload, payload);
  if (err) {
    CONSOLE_SERIAL.println("ERROR: Could not parse JSON payload");
    BLINK_LED_ERROR();
    CLIENT_SERIAL.println(RESULT_ERROR_BAD_JSON);
  } else if (!json_payload.containsKey("host") || !json_payload.containsKey("port")) {
    BLINK_LED_ERROR();
    CONSOLE_SERIAL.println("ERROR: host and/or port is missing");
    CLIENT_SERIAL.println(RESULT_ERROR_HOST_OR_PORT_MISSING);
  } else {
    strcpy(mqttHostname, (const char*)json_payload["host"]);
    strcpy(mqttUsername, (const char*)json_payload["username"]);
    strcpy(mqttPassword, (const char*)json_payload["password"]);
    mqttPort = json_payload["port"].as<int>();
    mqttClient.setServer(mqttHostname, mqttPort);
    wifiClient.setCACert(root_ca);
    CONSOLE_SERIAL.print("Connecting to MQTT broker ");
    CONSOLE_SERIAL.print(mqttHostname);
    CONSOLE_SERIAL.print(" on port ");
    CONSOLE_SERIAL.println(mqttPort);
    boolean status;
    if (json_payload.containsKey("username")) {
      status = mqttClient.connect(mqttClientId, mqttUsername, mqttPassword);
    } else {
      status = mqttClient.connect(mqttClientId);
    }
    if (status) {
      mqttConnectActive = 1;
      CONSOLE_SERIAL.println("MQTT connected");
      BLINK_LED_OK();
      CLIENT_SERIAL.println("OK");
    } else {
      mqttConnectActive = 0;
      CONSOLE_SERIAL.println("ERROR: MQTT connect failed");
      BLINK_LED_ERROR();
      CLIENT_SERIAL.println(RESULT_ERROR_MQTT_CONNECT_FAILED);
    }
  }
}

void cmdDisconnectMqtt(char *payload)
{
  if (mqttClient.connected()) {
    CONSOLE_SERIAL.println("Disconnecting MQTT connection");
    mqttClient.disconnect();
  } else {
    CONSOLE_SERIAL.println("MQTT is not currently connected");
  }
  mqttConnectActive = 0;
  BLINK_LED_OK();
  CLIENT_SERIAL.println(RESULT_OK_MQTT_IS_DISCONNECTED);
}

void cmdStatusMqtt(char *payload)
{
  BLINK_LED_OK();
  if (mqttClient.connected()) {
    CONSOLE_SERIAL.println("MQTT is connected");
    CLIENT_SERIAL.println(RESULT_OK_MQTT_IS_CONNECTED);
  } else {
    CONSOLE_SERIAL.println("MQTT is not currently connected");
    CLIENT_SERIAL.println(RESULT_OK_MQTT_NOT_CONNECTED);
  }
}

void cmdPublishMqtt(char *payload)
{
  // {"topic":"thepublishtopic","msg":"msg to publish"}
  StaticJsonDocument<1024> json_payload;
  DeserializationError err = deserializeJson(json_payload, payload);
  if (err) {
    CONSOLE_SERIAL.println("ERROR: Could not parse JSON payload");
    BLINK_LED_ERROR();
    CLIENT_SERIAL.println(RESULT_ERROR_BAD_JSON);
  } else if (!json_payload.containsKey("topic") || !json_payload.containsKey("msg")) {
    CONSOLE_SERIAL.println("ERROR: topic and/or msg is missing");
    BLINK_LED_ERROR();
    CLIENT_SERIAL.println(RESULT_ERROR_TOPIC_OR_MSG_MISSING);
  } else {
    const char *topic = (const char*)json_payload["topic"];
    const char *msg = (const char*)json_payload["msg"];
    if (topic == NULL || strlen(topic) == 0) {
      CONSOLE_SERIAL.println("ERROR: topic is empty or improper string format");
      BLINK_LED_ERROR();
      CLIENT_SERIAL.println(RESULT_ERROR_TOPIC_IS_BLANK_OR_BAD);
    } else if (msg == NULL || strlen(msg) == 0) {
      CONSOLE_SERIAL.println("ERROR: msg is empty or improper string format");
      BLINK_LED_ERROR();
      CLIENT_SERIAL.println(RESULT_ERROR_MSG_IS_BLANK_OR_BAD);
    } else {      
      if (mqttConnectActive && !mqttClient.connected()) {
        /* Reconnect */
        CONSOLE_SERIAL.println("Re-connecting to MQTT broker");
        boolean status;      
        if (strlen(mqttUsername) == 0) {
          status = mqttClient.connect(mqttClientId);
        } else {
          status = mqttClient.connect(mqttClientId, mqttUsername, mqttPassword);
        }
        if (!status) {
          CONSOLE_SERIAL.println("ERROR: MQTT re-connect failed");
          BLINK_LED_ERROR();
          CLIENT_SERIAL.println(RESULT_ERROR_MQTT_RECONNECT_FAILED);            // first error msg
        }
      }
      if (!mqttClient.connected()) {
        CONSOLE_SERIAL.println("ERROR: MQTT publish failed, not connected to MQTT broker");
        BLINK_LED_ERROR();
        CLIENT_SERIAL.println(RESULT_ERROR_MQTT_PUBLISH_FAILED_NOT_CONNECTED);   // second error msg
      } else {
        CONSOLE_SERIAL.print("Topic: "); CONSOLE_SERIAL.println(topic);
        CONSOLE_SERIAL.print("Message: "); CONSOLE_SERIAL.println(msg);
        CONSOLE_SERIAL.print("Message Len: "); CONSOLE_SERIAL.println(strlen(msg));
        if (mqttClient.publish(topic, msg)) {
          CONSOLE_SERIAL.println("Publish successful");
          BLINK_LED_OK();
          CLIENT_SERIAL.println(RESULT_OK);
        } else {
          CONSOLE_SERIAL.println("Publish failed");
          BLINK_LED_ERROR();
          CLIENT_SERIAL.println(RESULT_ERROR_MQTT_PUBLISH_FAILED);
        }
      }
    }
  }
}

struct command_s {
  char *cmd_str;
  int has_payload;
  void (*cmd_func)(char *payload);
} commands[] = {
  { "AT",           0, cmdEmpty             },
  { "AT+RESET",     0, cmdReset             },
  { "AT+CWIFI=",    1, cmdConnectWifi       },
  { "AT+DWIFI",     0, cmdDisconnectWifi    },
  { "AT+SWIFI",     0, cmdStatusWifi        },
  { "AT+CMQTT=",    1, cmdConnectMqtt       },
  { "AT+DMQTT",     0, cmdDisconnectMqtt    },
  { "AT+PUBLISH=",  1, cmdPublishMqtt       },
  { "AT+VER",       0, cmdVersion           },
#if defined(CONTROL_LED) && CONTROL_LED == 1
  { "AT+LED=",      1, cmdLed               },
#endif
};

void setup() {
  /* Initialize serial console */
  CONSOLE_SERIAL.begin(CONSOLE_SERIAL_BAUD);
  while (!CONSOLE_SERIAL) {
    /* Wait for serial port to be ready */
  }
  CONSOLE_SERIAL.print(IDENT); CONSOLE_SERIAL.print(" "); CONSOLE_SERIAL.print(VERSION);
  CONSOLE_SERIAL.println(" (Serial)");
  /* Initialize command serial port */
#if defined(REMAP_UART_PINS) && REMAP_UART_PINS == 1
  CLIENT_SERIAL.begin(CLIENT_SERIAL_BAUD, SERIAL_8N1, CLIENT_SERIAL_RX_GPIO, CLIENT_SERIAL_TX_GPIO);  // remap RX/TX pins
#else
  CLIENT_SERIAL.begin(CLIENT_SERIAL_BAUD);  // use default RX/TX pins
#endif
  while (!CLIENT_SERIAL) {
    /* Wait for serial port to be ready */
  }
#if CONTROL_LED == 1
  pinMode(LED_BUILTIN, OUTPUT);
  blink_led(2);
#endif
  //delay(250);
  //CLIENT_SERIAL.print(IDENT); CLIENT_SERIAL.print(" "); CLIENT_SERIAL.println(VERSION); 
}

int input_loc = 0;
char input[512];
int have_command = 0;

int clearCommand(void)
{
  input_loc = 0;
  input[0] = 0;
  have_command = 0;
}

void recvChar(char ch)
{
  if (ch == '\n') {                       // process command when LF is detected
    have_command = 1;
  } else if (!(ch == 0 || ch == '\r')) {  // ignore any NUL or CR chars, keep rest
    input[input_loc++] = ch;
    input[input_loc] = 0;
  }
}

int haveFullCmd(void)
{
  return have_command;
}

void loop() {

  /* Read incoming bytes */

  while (!haveFullCmd() && CLIENT_SERIAL.available()) {
    recvChar(CLIENT_SERIAL.read());
  }
  
  /* Process incoming command */

  if (haveFullCmd()) {
    int i, cmd_len;
    int found = 0; 
    char *payload;
    for (i = 0; i < sizeof(commands) / sizeof(commands[0]); i++) {
      if (commands[i].has_payload) {
        cmd_len = strlen(commands[i].cmd_str);
        if (strncmp(input, commands[i].cmd_str, cmd_len) == 0) {
          found = 1;
          payload = input + cmd_len;        
          break;
        }
      } else if (strcmp(input, commands[i].cmd_str) == 0) {
          found = 1;
          payload = NULL;
          break;
      }
    }
    if (found) {
      CONSOLE_SERIAL.print("Found cmd: "); CONSOLE_SERIAL.print(commands[i].cmd_str);
      CONSOLE_SERIAL.print(", payload: '"); CONSOLE_SERIAL.print(payload); CONSOLE_SERIAL.println("'");
      (*commands[i].cmd_func)(payload);
    } else if (input[0] == 0) {
      CLIENT_SERIAL.println();
    } else {
      CONSOLE_SERIAL.print("Unknown cmd: "); CONSOLE_SERIAL.println(input);
      BLINK_LED_ERROR();
      CLIENT_SERIAL.println("ERROR UNKNOWN COMMAND");
    }
  clearCommand();      
  }
}
