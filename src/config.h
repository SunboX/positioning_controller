#ifndef APP_config
#define APP_config

// Select your modem:
#define TINY_GSM_MODEM_SIM800
// #define TINY_GSM_MODEM_SIM808
// #define TINY_GSM_MODEM_SIM868
// #define TINY_GSM_MODEM_SIM900
// #define TINY_GSM_MODEM_SIM7000
// #define TINY_GSM_MODEM_SIM5360
// #define TINY_GSM_MODEM_SIM7600
// #define TINY_GSM_MODEM_UBLOX
// #define TINY_GSM_MODEM_SARAR4
// #define TINY_GSM_MODEM_M95
// #define TINY_GSM_MODEM_BG96
// #define TINY_GSM_MODEM_A6
// #define TINY_GSM_MODEM_A7
// #define TINY_GSM_MODEM_M590
// #define TINY_GSM_MODEM_MC60
// #define TINY_GSM_MODEM_MC60E
// #define TINY_GSM_MODEM_ESP8266
// #define TINY_GSM_MODEM_XBEE
// #define TINY_GSM_MODEM_SEQUANS_MONARCH

// Adding a short yield in commands if they're ending up chopped up.
// That's sometimes needed if using a slow baud rate (like the default 9600 for the SIM800) 
// with a fast processor (like the ESP32).
#define TINY_GSM_YIELD() { delay(2); }
#define MODEM_SPEED 9600

// Increase RX buffer to capture the entire response
// Chips without internal buffering (A6/A7, ESP8266, M590)
// need enough space in the buffer for the entire response
// else data will be lost (and the http library will fail).
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb

// See all AT commands, if wanted
//#define DUMP_AT_COMMANDS

// TTGO T-Call pin definitions
#define MODEM_RST 5
#define MODEM_PWKEY 4
#define MODEM_POWER_ON 23
#define MODEM_TX 27
#define MODEM_RX 26

#define LED_GPIO             13
#define LED_ON               HIGH
#define LED_OFF              LOW

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library.
#define I2C_SSD1306_RESET 4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define I2C_SSD1306_ADDRESS 0x3C

#define I2C_ZED_FP9_SDA 18
#define I2C_ZED_FP9_SCL 19

#define I2C_SSD1306_SDA 21
#define I2C_SSD1306_SCL 22

#define ZED_F9P_ADDRESS 0x42

#define PJON_DEVICE_ID_MEDIA_CONTROLLER 10
#define PJON_DEVICE_ID_POSITIONING_CONTROLLER 20
#define PJON_DEVICE_ID_PAN_TILT_CONTROLLER 30
#define PJON_BUS_GPIO_PIN 25

#endif
