//* Libraries
#include <Arduino.h>
#include "BME280/BME280I2C.h"
#include "BME280/EnvironmentCalculations.h"
#include <HardwareSerial.h>
#include <TaskScheduler.h>
#include <ArduinoJson.h>
#include <TinyGPS++.h>
#include <TimeLib.h>
#include <WiFiUdp.h>
#include <Time.h>
#include <Wire.h>
#include <SPI.h>
#include <MedianFilterLib.h>
#include "EEPROM.h"
#include <Update.h>
#include <Wire.h>

//*  -- RAFT Settings
#define MICRO_BAUD_RATE 115200 // Microcontroller Serial Port Baud Rate
#define DEBUG_MODE             // Enables Debug Mode
#define uS_TO_S_FACTOR 1000000 // Conversion factor for micro seconds to seconds [DO NOT CHANGE]

//* -- INDICATED LED SETTINGS
#define LEDPin 18              // LED Indicator Pin
#define LEDPin1 19             // LED Indicator Pin
#define INDICATOR_ENABLED true // LED Indicators Enabled

//* -- BATTERY SETTINGS
#define BATTMAXVOLT 4.2   // Maximum Battery Voltage
#define BATTMINVOLT 3.6   // Minimum Battery Voltage
#define BATTERYPIN 34     // Battery PIN
#define BATTERYRATIO 0.78 // Battery voltage divider ratio

//* -- GSM DEFINITIONS
#define TINY_GSM_MODEM_SIM800 // GSM/GPRS Module Model
#define GSM_BAUD 9600         // GSM/GPRS Module Baud Rate
#define GSM_RX 17             // GSM/GPRS Module RX Pin
#define GSM_TX 16             // GSM/GPRS Module TX Pin
//#define GSM_ENABLED   // Use GSM/GPRS for Data Telemetry
#define SMS_ENABLED false // SMS Enable

//* -- ULTRASONIC SENSOR SETTINGS [FOR FLOOD DEPTH]
#define US_RX 14          // Ultrasonic Module RX Pin
#define US_TX 12          // Ultrasonic Module TX
#define US_MAXHEIGHT 600  // Ultrasonic Max Height (cm)
#define US_resetButton 13 // Reset Height Button

//* -- GPS MODULE SETTINGS
#define GPS_BAUD 9600 // GSM/GPRS Module TX Pin
#define GPS_RX 26     // GSM/GPRS Module RX Pin
#define GPS_TX 27     // GSM/GPRS Module TX Pin

TinyGPSPlus gps;
HardwareSerial SerialGPS(1);

//* -- RAIN GAUGE SETTINGS
#define GPIO_PIN_BITMASK 0x800004000 // GPIO 35 (2^35 in Hex)
#define rainGaugePin 35              // Rain Guage Pin
#define tipAmount 0.3636             //  This has not been calibrated yet as it will depend on your printing dimensions.
/* 
To calculate the tipAmount (amount of rainfall per tip), calibrate the rain gauge by slowly putting 100mL of water into the rain gauge. Then
tipAmount = (100mL/#ofTips)/(pi*r*r)
Example:
Rain gauge tips 38 times with 100mL of water.
Rain gauge radius is 5.72 cm.
tipAmount  = (100mL/38tips)/(pi*5.72cm*5.72cm)
tipAmount = 0.0256cm/tip or 0.256mm 

NOTE: tipAmount should be in millimeters (mm)
*/

//* -- ULTRASONIC SENSOR SETTINGS
const int datasizeUS = 15;

//* -- BAROMETER SETTINGS
#define SEALEVELPRESSURE_HPA (1013.25) // Standard sea level pressure
BME280I2C::Settings settings(
    BME280::OSR_X1,
    BME280::OSR_X1,
    BME280::OSR_X1,
    BME280::Mode_Forced,
    BME280::StandbyTime_1000ms,
    BME280::Filter_16,
    BME280::SpiEnable_False,
    BME280I2C::I2CAddr_0x76);
BME280I2C bme(settings);

//* WiFi Access Point Settings
const char *ssid = "SKYbroadband9CE4";
const char *wifi_pass = "286170965";

//* GSM Internet Settings
const char *apn = "smartlte"; // For Smart Telecom
//const char *apn = "internet.globe.com.ph";  // For Globe Telecom
const char *gprsUser = "";
const char *gprsPass = "";
#define GATEWAY_NUMBER "+639569109001"

//* RAFT Credentials
	const char *clientID = "29411b5610ebe12914592a9f8aae2f242e2b";
	const char *username = "29411b5610ebe12914592a9f8aae2f242e2b";
	const char *password = "e9b2bb429ee2d6192d4ee48b69e2443126c2";
	const char *streamIDData = "RGAPI";
const char *streamIDInfo = "RAFT_Info";

#include <WiFi.h>
#include <AsyncMqttClient.h>

WiFiClient client;
AsyncMqttClient rainflowMQTT; // Instance Creation of MQTT Client
bool wifiConnected = false;

//Adafruit MQTT for GSM
#ifdef GSM_ENABLED
#include <TinyGsmClient.h>
#include "PubSubClient/PubSubClient.h"

#ifdef DEBUG_MODE
#define DUMP_AT_COMMANDS
#endif

HardwareSerial SerialGSM(2);
TinyGsm modem(SerialGSM);
TinyGsmClient client(modem);

Adafruit_MQTT_Client mqtt(&client, "rainflow.live", 1883, clientID, username, password);
bool gprsConnected = false;
#endif