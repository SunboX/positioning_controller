#include "config.h"
#include <Arduino.h>
#include <Wire.h>
#include "utilities.h"
#include <SPI.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeMono9pt7b.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <WifiEspNowBroadcast.h>
#include <WiFi.h>
#include "base64.h" //Built-in ESP32 library
#include <ArduinoJson.h>

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial

// Set serial for AT commands (to SIM800 module)
#define SerialAT Serial1

// or Software Serial on Uno, Nano
//#include <SoftwareSerial.h>
// SoftwareSerial SerialAT(2, 3); // RX, TX

// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG SerialMon

// Uncomment this if you want to use SSL
// #define USE_SSL

// set GSM PIN, if any
#define GSM_PIN ""

// Your GPRS credentials, if any
const char apn[] = "internet.telekom";
const char gprsUser[] = "congstar";
const char gprsPass[] = "cs";

// Server details
const char casterHost[] = "www.ntrip.sachsen.de";
uint16_t casterPort = 2101;
const char mountPoint[] = "VRS_3_4G_SN"; // RTCM 3.2 MSM4
const char casterUser[] = "xxx";
const char casterUserPW[] = "xxx";

#include <TinyGsmClient.h>

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

#ifdef USE_SSL
TinyGsmClientSecure client(modem);
const int port = 443;
#else
TinyGsmClient client(modem);
// const int port = 80;
#endif

long lastReceivedRTCM_ms = 0;		// 5 RTCM messages take approximately ~300ms to arrive at 115200bps
int maxTimeBeforeHangup_ms = 10000; // If we fail to get a complete RTCM frame after 10s, then disconnect from caster

bool transmitLocation = true;		 // By default we will transmit the units location via GGA sentence.
int timeBetweenGGAUpdate_ms = 10000; // GGA is required for Rev2 NTRIP casters. Don't transmit but once every 10 seconds
long lastTransmittedGGA_ms = 0;

// Used for GGA sentence parsing from incoming NMEA
bool ggaSentenceStarted = false;
bool ggaSentenceComplete = false;
bool ggaTransmitComplete = false; // Goes true once we transmit GGA to the caster

char ggaSentence[128] = {0};
byte ggaSentenceSpot = 0;
int ggaSentenceEndSpot = 0;

TwoWire I2C_SSD1306 = TwoWire(0);
TwoWire I2C_ZED_FP9 = TwoWire(1);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2C_SSD1306, I2C_SSD1306_RESET);

TinyGsmClient ntripClient(modem);

long rtcmCount = 0;
SFE_UBLOX_GNSS myGNSS;

TaskHandle_t Task2;

// Pretty-print the fractional part with leading zeros - without using printf
// (Only works with positive numbers)
String printFractional(int32_t fractional, uint8_t places)
{
	String ret = "";
	if (places > 1)
	{
		for (uint8_t place = places - 1; place > 0; place--)
		{
			if (fractional < pow(10, place))
			{
				ret += '0';
			}
		}
	}
	ret += fractional;
	return ret;
}

bool clearAfter = false;
void showMessage_ln(String msg)
{
	if (clearAfter)
	{
		clearAfter = false;
		display.clearDisplay();
		display.setCursor(0, 0);
	}
	display.print(msg);
	display.display();

	clearAfter = true;
}

void showMessage(String msg)
{
	if (clearAfter)
	{
		clearAfter = false;
		display.clearDisplay();
		display.setCursor(0, 0);
	}
	display.print(msg);
	display.display();
}

void getPosition()
{
	// Clear the buffer
	display.clearDisplay();
	display.setCursor(0, 0);

	delay(1);

	display.print(myGNSS.getDay(), DEC);
	display.print('/');
	display.print(myGNSS.getMonth(), DEC);
	display.print('/');
	display.print(String(myGNSS.getYear()).substring(2, 4));

	display.print(F(" "));
	if (myGNSS.getHour() < 10)
	{
		display.print('0');
	}
	display.print(myGNSS.getHour(), DEC);
	display.print(':');
	if (myGNSS.getMinute() < 10)
	{
		display.print('0');
	}
	display.print(myGNSS.getMinute(), DEC);
	display.print(':');
	if (myGNSS.getSecond() < 10)
	{
		display.print('0');
	}
	display.print(myGNSS.getSecond(), DEC);
	display.print('.');
	if (myGNSS.getMillisecond() < 10)
	{
		display.print("00");
	}
	else if (myGNSS.getMillisecond() > 9 && myGNSS.getMillisecond() < 100)
	{
		display.print("0");
	}
	display.println(myGNSS.getMillisecond());

	delay(1);

	byte fixType = myGNSS.getFixType();
	display.print(F("Fix: "));
	if (fixType == 0)
	{
		display.println(F("No fix"));
	}
	else if (fixType == 1)
	{
		display.println(F("Dead reckoning"));
	}
	else if (fixType == 2)
	{
		display.println(F("2D"));
	}
	else if (fixType == 3)
	{
		display.println(F("3D"));
	}
	else if (fixType == 4)
	{
		display.println(F("GNSS + Dead reckoning"));
	}
	else if (fixType == 5)
	{
		display.println(F("Time only"));
	}

	delay(1);

	// display.setCursor(0, display.getCursorY() + 2);

	byte RTK = myGNSS.getCarrierSolutionType();
	display.print("RTK: ");
	display.print(String(RTK));
	if (RTK == 0)
	{
		display.println(F(" (None)"));
	}
	else if (RTK == 1)
	{
		display.println(F(" (floating fix)"));
	}
	else if (RTK == 2)
	{
		display.println(F(" (fix)"));
	}

	delay(1);

	// getHighResLatitude: returns the latitude from HPPOSLLH as an int32_t in degrees * 10^-7
	// getHighResLatitudeHp: returns the high resolution component of latitude from HPPOSLLH as an int8_t in degrees * 10^-9
	// getHighResLongitude: returns the longitude from HPPOSLLH as an int32_t in degrees * 10^-7
	// getHighResLongitudeHp: returns the high resolution component of longitude from HPPOSLLH as an int8_t in degrees * 10^-9
	// getElipsoid: returns the height above ellipsoid as an int32_t in mm
	// getElipsoidHp: returns the high resolution component of the height above ellipsoid as an int8_t in mm * 10^-1
	// getMeanSeaLevel: returns the height above mean sea level as an int32_t in mm
	// getMeanSeaLevelHp: returns the high resolution component of the height above mean sea level as an int8_t in mm * 10^-1
	// getHorizontalAccuracy: returns the horizontal accuracy estimate from HPPOSLLH as an uint32_t in mm * 10^-1

	// If you want to use the high precision latitude and longitude with the full 9 decimal places
	// you will need to use a 64-bit double - which is not supported on all platforms

	// To allow this example to run on standard platforms, we cheat by converting lat and lon to integer and fractional degrees

	// The high resolution altitudes can be converted into standard 32-bit float

	// First, let's collect the position data
	int32_t latitude = myGNSS.getHighResLatitude();
	int8_t latitudeHp = myGNSS.getHighResLatitudeHp();
	int32_t longitude = myGNSS.getHighResLongitude();
	int8_t longitudeHp = myGNSS.getHighResLongitudeHp();
	int32_t ellipsoid = myGNSS.getElipsoid();
	int8_t ellipsoidHp = myGNSS.getElipsoidHp();
	int32_t msl = myGNSS.getMeanSeaLevel();
	int8_t mslHp = myGNSS.getMeanSeaLevelHp();
	uint32_t accuracy = myGNSS.getHorizontalAccuracy();

	// Defines storage for the lat and lon units integer and fractional parts
	int32_t lat_int;  // Integer part of the latitude in degrees
	int32_t lat_frac; // Fractional part of the latitude
	int32_t lon_int;  // Integer part of the longitude in degrees
	int32_t lon_frac; // Fractional part of the longitude

	// Calculate the latitude and longitude integer and fractional parts
	lat_int = latitude / 10000000;				// Convert latitude from degrees * 10^-7 to Degrees
	lat_frac = latitude - (lat_int * 10000000); // Calculate the fractional part of the latitude
	lat_frac = (lat_frac * 100) + latitudeHp;	// Now add the high resolution component
	if (lat_frac < 0)							// If the fractional part is negative, remove the minus sign
	{
		lat_frac = 0 - lat_frac;
	}
	lon_int = longitude / 10000000;				 // Convert latitude from degrees * 10^-7 to Degrees
	lon_frac = longitude - (lon_int * 10000000); // Calculate the fractional part of the longitude
	lon_frac = (lon_frac * 100) + longitudeHp;	 // Now add the high resolution component
	if (lon_frac < 0)							 // If the fractional part is negative, remove the minus sign
	{
		lon_frac = 0 - lon_frac;
	}

	// display.setCursor(0, display.getCursorY() + 2);

	char val[20];

	display.print(F("Lat: "));
	display.print(lat_int, DEC); // Print the integer part of the latitude
	display.print(".");
	display.println(printFractional(lat_frac, 9)); // Print the fractional part of the latitude with leading zeros

	// display.setCursor(0, display.getCursorY() + 2);

	display.print(F("Lon: "));
	display.print(lon_int); // Print the integer part of the latitude
	display.print(".");
	display.println(printFractional(lon_frac, 9)); // Print the fractional part of the latitude with leading zeros

	// Now define float storage for the heights and accuracy
	float f_ellipsoid;
	float f_msl;
	float f_accuracy;

	// Calculate the height above ellipsoid in mm * 10^-1
	f_ellipsoid = (ellipsoid * 10) + ellipsoidHp;
	// Now convert to m
	f_ellipsoid = f_ellipsoid / 10000.0; // Convert from mm * 10^-1 to m

	// Calculate the height above mean sea level in mm * 10^-1
	f_msl = (msl * 10) + mslHp;
	// Now convert to m
	f_msl = f_msl / 10000.0; // Convert from mm * 10^-1 to m

	// Convert the horizontal accuracy (mm * 10^-1) to a float
	f_accuracy = accuracy;
	// Now convert to m
	f_accuracy = f_accuracy / 10000.0; // Convert from mm * 10^-1 to m

	// Finally, do the printing
	// Serial.print("Ellipsoid (m): ");
	// Serial.print(f_ellipsoid, 4); // Print the ellipsoid with 4 decimal places

	// display.setCursor(0, display.getCursorY() + 2);

	display.print(F("Altitude: "));
	dtostrf(f_msl, 1, 4, val);
	display.println(val);

	// display.setCursor(0, display.getCursorY() + 2);

	display.print(F("Accuracy: "));
	dtostrf(f_accuracy, 1, 4, val);
	display.println(val);

	long millisecs = millis();
	int systemUpTimeMn = int((millisecs / (1000 * 60)) % 60);
	int systemUpTimeHr = int((millisecs / (1000 * 60 * 60)) % 24);
	int systemUpTimeDy = int((millisecs / (1000 * 60 * 60 * 24)) % 365);

	display.print(F("Uptime (d:h:m): "));
	display.print(systemUpTimeDy);
	display.print(F(":"));

	display.print(systemUpTimeHr);
	display.print(F(":"));
	display.println(systemUpTimeMn);

	display.display();

	StaticJsonDocument<512> doc;
	doc["type"] = "position";
	doc["latitude"] = (String(lat_int) + "." + String(lat_frac)).toDouble();
	doc["longitude"] = (String(lon_int) + "." + String(lon_frac)).toDouble();
	doc["altitude"] = f_msl;
	String payload = "";
	serializeJson(doc, payload);
	// bus.send(PJON_DEVICE_ID_PAN_TILT_CONTROLLER, payload.c_str(), sizeof(payload.c_str()));

	char msg[200];
	int len = snprintf(msg, sizeof(msg), payload.c_str());
	WifiEspNowBroadcast.send(reinterpret_cast<const uint8_t *>(msg), len);

	Serial.println("ESP-NOW message send");
	Serial.println(msg);
}

void processRx(const uint8_t mac[WIFIESPNOW_ALEN], const uint8_t *buf, size_t count, void *arg)
{
	Serial.printf("Message from %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

	for (size_t i = 0; i < count; ++i)
	{
		Serial.print(static_cast<char>(buf[i]));
	}
}

void task2Loop(void *pvParameters)
{
	Serial.print("Task2 running on core ");
	Serial.println(xPortGetCoreID());

	for (;;)
	{
		getPosition();

		WifiEspNowBroadcast.loop();
		delay(10000);
	}
}

void setup()
{
	/*
	pinMode(PJON_BUS_GPIO_PIN, OUTPUT);
	return;
	*/

	digitalWrite(12, LOW);
	digitalWrite(14, LOW);

	// Set console baud rate
	SerialMon.begin(115200);
	delay(10);

	btStop();

	I2C_SSD1306.begin(I2C_SSD1306_SDA, I2C_SSD1306_SCL, 400000); // 100kHz
	I2C_ZED_FP9.begin(I2C_ZED_FP9_SDA, I2C_ZED_FP9_SCL, 400000); // 400kHz

	// Keep power when running from battery
	bool isOk = setPowerBoostKeepOn(1);

	// Set-up modem reset, enable, power pins
	pinMode(MODEM_PWKEY, OUTPUT);
	pinMode(MODEM_POWER_ON, OUTPUT);

	// Turn on the Modem power first
	digitalWrite(MODEM_POWER_ON, HIGH);

	// Pull down PWRKEY for more than 1 second according to manual requirements
	digitalWrite(MODEM_PWKEY, HIGH);
	delay(100);
	digitalWrite(MODEM_PWKEY, LOW);
	delay(1000);
	digitalWrite(MODEM_PWKEY, HIGH);

	// Initialize the indicator as an output
	pinMode(LED_GPIO, OUTPUT);
	digitalWrite(LED_GPIO, LED_OFF);

	// SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
	if (!display.begin(SSD1306_SWITCHCAPVCC, I2C_SSD1306_ADDRESS))
	{
		Serial.println(F("SSD1306 allocation failed"));
		for (;;)
		{
			; // Don't proceed, loop foreverh
		}
	}

	// Clear the buffer
	display.clearDisplay();

	// display.setTextSize(1); // Normal 1:1 pixel scale
	// display.setFont(&FreeMono9pt7b);
	display.setTextColor(SSD1306_WHITE); // Draw white text
	display.setCursor(0, 0);			 // Start at top-left corner

	showMessage_ln("booting...");
	delay(10);

	WiFi.persistent(false);

	bool ok = WifiEspNowBroadcast.begin("ESPNOW", 3);
	if (!ok)
	{
		Serial.println("WifiEspNowBroadcast.begin() failed");
		ESP.restart();
	}
	// WifiEspNowBroadcast.begin() function sets WiFi to AP+STA mode.
	// The AP interface is also controlled by WifiEspNowBroadcast.
	// You may use the STA interface after calling WifiEspNowBroadcast.begin().
	// For best results, ensure all devices are using the same WiFi channel.

	WifiEspNowBroadcast.onReceive(processRx, nullptr);

	display.clearDisplay();
	display.setCursor(0, 0);
	display.println(F("mac address"));
	display.println(WiFi.softAPmacAddress());
	display.display();

	delay(10000);

	// Set GSM module baud rate and UART pins
	SerialAT.begin(MODEM_SPEED, SERIAL_8N1, MODEM_RX, MODEM_TX);
	delay(6000);

	// Restart takes quite some time
	// To skip it, call init() instead of restart()
	modem.setBaud(MODEM_SPEED);
	modem.restart();
	// modem.init();

	String modemInfo = modem.getModemInfo();

	// Unlock your SIM card with a PIN if needed
	if (GSM_PIN && modem.getSimStatus() != 3)
	{
		modem.simUnlock(GSM_PIN);
	}

	Serial.println("NTRIP testing");
	showMessage_ln("NTRIP testing");

	Wire.begin(); // Start I2C

	if (myGNSS.begin(I2C_ZED_FP9, ZED_F9P_ADDRESS) == false) // Connect to the Ublox module using Wire port
	{
		Serial.println(F("u-blox GPS not detected at default I2C address. Please check wiring. Freezing."));
		showMessage_ln("u-blox GPS not detected");
		while (1)
			;
	}
	Serial.println(F("u-blox module connected"));
	showMessage_ln("u-blox module connected");

	myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA);								  // Set the I2C port to output both NMEA and UBX messages
	myGNSS.setPortInput(COM_PORT_I2C, COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_RTCM3); // Be sure RTCM3 input is enabled. UBX + RTCM3 is not a valid state.

	myGNSS.enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_I2C); // Verify the GGA sentence is enabled

	myGNSS.setNavigationFrequency(1); // Set output in Hz.

	Serial.println("Waiting for network...");
	showMessage_ln("Waiting for network...");
	if (!modem.waitForNetwork())
	{
		Serial.println(" fail");
		showMessage(" fail");
		delay(10000);
		return;
	}
	Serial.println(" success");
	showMessage(" success");

	if (modem.isNetworkConnected())
	{
		Serial.println("Network connected");
		showMessage_ln("Network connected");
	}

	// GPRS connection parameters are usually set after network registration
	Serial.print(F("Connecting to "));
	Serial.print(apn);
	showMessage("Connecting to ");
	showMessage_ln(apn);
	if (!modem.gprsConnect(apn, gprsUser, gprsPass))
	{
		Serial.println(" fail");
		showMessage(" fail");
		delay(10000);
		return;
	}
	Serial.println(" success");
	showMessage(" success");

	if (modem.isGprsConnected())
	{
		Serial.println("GPRS connected");
	}

	Serial.println(F("Subscribing to Caster"));

	// create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
	xTaskCreatePinnedToCore(
		task2Loop, // Task function.
		"Task2",   // name of task.
		10000,	   // Stack size of task
		NULL,	   // parameter of the task
		1UL,	   // priority of the task
		&Task2,	   // Task handle to keep track of created task
		1);		   // pin task to core 1
}

// Connect to NTRIP Caster, receive RTCM, and push to ZED module over I2C
void loop()
{
	/*
	delay(1);
	digitalWrite(PJON_BUS_GPIO_PIN, LOW);
	delay(1);
	digitalWrite(PJON_BUS_GPIO_PIN, HIGH);
	return;
	*/

	myGNSS.checkUblox();

	// Connect if we are not already. Limit to 5s between attempts.
	if (ntripClient.connected() == false)
	{
		Serial.print(F("Opening socket to "));
		Serial.println(casterHost);

		if (ntripClient.connect(casterHost, casterPort) == false) // Attempt connection
		{
			Serial.println(F("Connection to caster failed"));
			delay(10);
			return;
		}
		else
		{
			Serial.print(F("Connected to "));
			Serial.print(casterHost);
			Serial.print(F(": "));
			Serial.println(casterPort);

			Serial.print(F("Requesting NTRIP Data from mount point "));
			Serial.println(mountPoint);

			const int SERVER_BUFFER_SIZE = 512;
			char serverRequest[SERVER_BUFFER_SIZE];

			snprintf(serverRequest,
					 SERVER_BUFFER_SIZE,
					 "GET /%s HTTP/1.0\r\nUser-Agent: NTRIP SparkFun/1.0\r\nNtrip-Version: Ntrip/2.0\r\n",
					 mountPoint);

			char credentials[512];
			if (strlen(casterUser) == 0)
			{
				strncpy(credentials, "Accept: */*\r\nConnection: close\r\n", sizeof(credentials));
			}
			else
			{
				// Pass base64 encoded user:pw
				char userCredentials[sizeof(casterUser) + sizeof(casterUserPW) + 1]; // The ':' takes up a spot
				snprintf(userCredentials, sizeof(userCredentials), "%s:%s", casterUser, casterUserPW);

				Serial.print(F("Sending credentials: "));
				Serial.println(userCredentials);

				// Encode with ESP32 built-in library
				base64 b;
				String strEncodedCredentials = b.encode(userCredentials);
				char encodedCredentials[strEncodedCredentials.length() + 1];
				strEncodedCredentials.toCharArray(encodedCredentials, sizeof(encodedCredentials)); // Convert String to char array

				snprintf(credentials, sizeof(credentials), "Authorization: Basic %s\r\n", encodedCredentials);
			}
			strncat(serverRequest, credentials, SERVER_BUFFER_SIZE);
			strncat(serverRequest, "\r\n", SERVER_BUFFER_SIZE);

			Serial.print(F("serverRequest size: "));
			Serial.print(strlen(serverRequest));
			Serial.print(F(" of "));
			Serial.print(sizeof(serverRequest));
			Serial.println(F(" bytes available"));

			Serial.println(F("Sending server request:"));
			Serial.println(serverRequest);
			ntripClient.write(serverRequest);

			// Wait for response
			unsigned long timeout = millis();
			while (ntripClient.available() == 0)
			{
				if (millis() - timeout > 5000)
				{
					Serial.println(F("Caster timed out!"));
					ntripClient.stop();
					delay(10);
					return;
				}
				delay(10);
			}

			// Check reply
			bool connectionSuccess = false;
			char response[512];
			int responseSpot = 0;
			while (ntripClient.available())
			{
				if (responseSpot == sizeof(response) - 1)
				{
					break;
				}

				response[responseSpot++] = ntripClient.read();
				if (strstr(response, "200") > 0) // Look for '200 OK'
				{
					connectionSuccess = true;
				}
				if (strstr(response, "401") > 0) // Look for '401 Unauthorized'
				{
					Serial.println(F("Hey - your credentials look bad! Check you caster username and password."));
					connectionSuccess = false;
				}
			}
			response[responseSpot] = '\0';

			Serial.print(F("Caster responded with: "));
			Serial.println(response);

			if (connectionSuccess == false)
			{
				Serial.print(F("Failed to connect to "));
				Serial.println(casterHost);
				delay(10);
				return;
			}
			else
			{
				Serial.print(F("Connected to "));
				Serial.println(casterHost);
				lastReceivedRTCM_ms = millis(); // Reset timeout
				ggaTransmitComplete = true;		// Reset to start polling for new GGA data
			}
		} // End attempt to connect
	}	  // End connected == false

	if (ntripClient.connected() == true)
	{
		uint8_t rtcmData[512 * 4]; // Most incoming data is around 500 bytes but may be larger
		rtcmCount = 0;

		// Print any available RTCM data
		while (ntripClient.available())
		{
			// Serial.write(ntripClient.read()); //Pipe to serial port is fine but beware, it's a lot of binary data
			rtcmData[rtcmCount++] = ntripClient.read();
			if (rtcmCount == sizeof(rtcmData))
			{
				break;
			}
		}

		if (rtcmCount > 0)
		{
			lastReceivedRTCM_ms = millis();

			// Push RTCM to GNSS module over I2C
			myGNSS.pushRawData(rtcmData, rtcmCount, false);
			Serial.print(F("RTCM pushed to ZED: "));
			Serial.println(rtcmCount);
		}
	}

	// Provide the caster with our current position as needed
	if (ntripClient.connected() == true && transmitLocation == true && (millis() - lastTransmittedGGA_ms) > timeBetweenGGAUpdate_ms && ggaSentenceComplete == true && ggaTransmitComplete == false)
	{
		Serial.print(F("Pushing GGA to server: "));
		Serial.println(ggaSentence);

		lastTransmittedGGA_ms = millis();

		// Push our current GGA sentence to caster
		ntripClient.print(ggaSentence);
		ntripClient.print("\r\n");

		ggaTransmitComplete = true;

		// Wait for response
		unsigned long timeout = millis();
		while (ntripClient.available() == 0)
		{
			if (millis() - timeout > 5000)
			{
				Serial.println(F("Caster timed out!"));
				ntripClient.stop();
				delay(10);
				return;
			}
			delay(10);
		}

		// Check reply
		bool connectionSuccess = false;
		char response[512];
		int responseSpot = 0;
		while (ntripClient.available())
		{
			if (responseSpot == sizeof(response) - 1)
			{
				break;
			}

			response[responseSpot++] = ntripClient.read();
			if (strstr(response, "200") > 0) // Look for '200 OK'
			{
				connectionSuccess = true;
			}

			if (strstr(response, "401") > 0) // Look for '401 Unauthorized'
			{
				Serial.println(F("Hey - your credentials look bad! Check you caster username and password."));
				connectionSuccess = false;
			}
		}
		response[responseSpot] = '\0';

		Serial.print(F("Caster responded with: "));
		Serial.println(response);
	}

	// Close socket if we don't have new data for 10s
	if (millis() - lastReceivedRTCM_ms > maxTimeBeforeHangup_ms)
	{
		Serial.println(F("RTCM timeout. Disconnecting..."));
		if (ntripClient.connected() == true)
		{
			ntripClient.stop();
		}
		delay(10);
		return;
	}

	delay(10);
}

// This function gets called from the SparkFun u-blox Arduino Library
// As each NMEA character comes in you can specify what to do with it
// We will look for and copy the GGA sentence
void SFE_UBLOX_GNSS::processNMEA(char incoming)
{
	// Take the incoming char from the u-blox I2C port and check to see if we should record it or not
	if (incoming == '$' && ggaTransmitComplete == true)
	{
		ggaSentenceStarted = true;
		ggaSentenceSpot = 0;
		ggaSentenceEndSpot = sizeof(ggaSentence);
		ggaSentenceComplete = false;
	}

	if (ggaSentenceStarted == true)
	{
		ggaSentence[ggaSentenceSpot++] = incoming;

		// Make sure we don't go out of bounds
		if (ggaSentenceSpot == sizeof(ggaSentence))
		{
			// Start over
			ggaSentenceStarted = false;
		}
		// Verify this is the GGA setence
		else if (ggaSentenceSpot == 5 && incoming != 'G')
		{
			// Ignore this sentence, start over
			ggaSentenceStarted = false;
		}
		else if (incoming == '*')
		{
			// We're near the end. Keep listening for two more bytes to complete the CRC
			ggaSentenceEndSpot = ggaSentenceSpot + 2;
		}
		else if (ggaSentenceSpot == ggaSentenceEndSpot)
		{
			ggaSentence[ggaSentenceSpot] = '\0'; // Terminate this string
			ggaSentenceComplete = true;
			ggaTransmitComplete = false; // We are ready for transmission

			// Serial.print("GGA Parsed - ");
			// Serial.println(ggaSentence);

			// Start over
			ggaSentenceStarted = false;
		}
	}
}