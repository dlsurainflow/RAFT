//  -- RAFT DEFINTIONS
#define MICRO_BAUD_RATE 115200 // Microcontroller Serial Port Baud Rate
#define BATTMAXVOLT 4.2        // Maximum Battery Voltage
#define BATTMINVOLT 3.6        // Minimum Battery Voltage
#define BATTERYPIN 34          // Battery PIN
#define BATTERYRATIO 0.78      // Battery voltage divider ratio
#define LEDPin 2               // LED Indicator Pin
#define uS_TO_S_FACTOR 1000000 // Conversion factor for micro seconds to seconds */
#define MODEM_WIFI             // Use Wifi for Data Telemetry
//#define MODEM_GSM                     // Use GSM/GPRS for Data Telemetry
#define DEBUG_MODE // DEBUG MODE ON
//* -- GSM DEFINITIONS
#define TINY_GSM_MODEM_SIM800 // GSM/GPRS Module Model
#define GSM_RX 17             // GSM/GPRS Module RX Pin
#define GSM_TX 16             // GSM/GPRS Module TX Pin
#define GSM_BAUD 9600         // GSM/GPRS Module Baud Rate
//* -- ULTRASONIC SENSOR DEFINITIONS [FOR FLOOD DEPTH]
#define US_RX 14          // Ultrasonic Module RX Pin
#define US_TX 12          // Ultrasonic Module TX
#define US_MAXHEIGHT 600  // Ultrasonic Max Height (cm)
#define US_resetButton 13 // Reset Height Button
//* -- GPS MODULE DEFINITIONS
#define GPS_RX 25     // GSM/GPRS Module RX Pin
#define GPS_TX 26     // GSM/GPRS Module TX Pin
#define GPS_BAUD 9600 // GSM/GPRS Module TX Pin
//* -- RAIN GAUGE DEFINITIONS
#define rainGaugePin 35              // Rain Guage 1 Pin
#define rainGaugePin2 32             // Rain Gauge 2
#define GPIO_PIN_BITMASK 0x900004000 // (2^35 + 2^32)Hex
#define tipAmount 0.3636             // 0.3636 mm of rainfall per tip
#define tipAmount2 0.2555            // S
//* -- BAROMETER DEFINITIONS
#define SEALEVELPRESSURE_HPA (1013.25) // Change to mean sea level pressure in your area

#include <Arduino.h>
#include <Adafruit_BME280.h>
#include <AsyncMqttClient.h>
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

#ifdef MODEM_GSM
#include <TinyGsmClient.h>
#endif
#ifdef MODEM_WIFI
#include <WiFi.h>
#endif

//* WiFi Access Point Details
const char *ssid = "Hidden Network";
const char *wifi_pass = "mmbmh15464";

//* GSM Internet Details
const char *apn = "smartlte";
const char *gprsUser = "";
const char *gprsPass = "";

//* Ultrasonic Sensor Variables
RTC_DATA_ATTR float raftHeight = 0; // Set original Height
float floodDepth = 0;               // Water level
float timeMean = 0;
long duration = 0;
float distanceH = 0;
float distanceD = 0;
float medianGetHeight = 0;
float medianHeight = 0;
float medianDepth = 0;
long lastDetectedTipMillisUS = 0;
bool heightWrite = false;
// int buttonState = 0;
const int datasizeUS = 30;
MedianFilter<int> medianFilter(3);

//* RAFT Details
const char *clientID = "bbb1691accc836be0958909cf8426e22b246";
const char *username = "bbb1691accc836be0958909cf8426e22b246";
const char *password = "aeff9fb2b53b2eba1b2ca8b218514615f995";
const char *streamID = "RGAPI";

RTC_DATA_ATTR int bootCount = 0;
//int incomingByte = 0;
long currentmillis = 0;

RTC_DATA_ATTR int currentMode = 0;

#ifdef DEBUG_MODE
#define DEBUG_PRINT(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#endif

AsyncMqttClient rainflowMQTT; // Instance Creation of MQTT Client
Adafruit_BME280 bme;          // I2C

void publishData();
void modeCheck();
void rainfallAmountReset();

TinyGPSPlus gps;
HardwareSerial SerialGPS(1);
//NewPing sonar(US_RX, US_TX, US_MAXHEIGHT);

Task publishDataScheduler(19 * 60e3, TASK_FOREVER, &publishData);  // Every 10 minutes -10min*60seconds*1000
Task checkMode(5 * 60e3, TASK_FOREVER, &modeCheck);                // Every 5 minutes
Task rainfallReset(10 * 60e3, TASK_FOREVER, &rainfallAmountReset); // Every 5 minutes
// Task rainfallReset(10 * 60e3, TASK_FOREVER, &rainfallAmountReset); // Every 5 minutes
Task setHeightTask(10, TASK_ONCE, &setHeight);
WiFiUDP ntpUDP;
Scheduler Runner;

/////////////////////////  RAIN GAUGE  ///////////////////////
//  This has not been calibrated yet as it will depend on your printing dimensions.
//  As a rough guide, this rain gauge tips 15 times per 100 mL, over a radious of 5.72 cm
//  One tip is ~ 6.67mL or 6.67cm^3.
//  Catachment area = pi*r*r=102.79cm^2
//  Rainfall per tip = 6.67mL/102.79 = 0.6489mL/tip

RTC_DATA_ATTR int tipCount = 0, tipCount2 = 0;                                                         // Total Amount of Tips
RTC_DATA_ATTR int rainGaugeDate = 0;                                                                   // Rain Gauge Date
static unsigned long lastDetectedTipMillis = 0, tipTime = 0, lastDetectedTipMillis2 = 0, tipTime2 = 0; // Time of Last Rain Guage Tip

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR tippingBucket()
{

  // Debounce for a quarter secon = max 4 counts/second
  if (millis() - lastDetectedTipMillis > 250)
  {
    portENTER_CRITICAL_ISR(&mux);

    tipTime = millis() - lastDetectedTipMillis;
    tipCount++;

    DEBUG_PRINT("Tip Count:" + String(tipCount));

    lastDetectedTipMillis = millis();
    // Serial.println("Rainfall rate:" + String(rainfallRate(),2));
    // Serial.println("Rainfall Amount:" + String(rainfallAmount(),2));

    portEXIT_CRITICAL_ISR(&mux);
  }
}

void IRAM_ATTR tippingBucket2()
{
  //  static unsigned long lastDetectedTipMillis2;  // Time of Last Rain Guage Tip

  // Debounce for a quarter secon = max 4 counts/second
  if (millis() - lastDetectedTipMillis2 > 250)
  {
    tipTime2 = millis() - lastDetectedTipMillis2;
    portENTER_CRITICAL_ISR(&mux);
    tipCount2++;

    DEBUG_PRINT("Tip Count:" + String(tipCount2));

    lastDetectedTipMillis2 = millis();
    portEXIT_CRITICAL_ISR(&mux);
  }
}

void attachRainGauge()
{
  DEBUG_PRINT("Rain Gauge 1 attached to pin: " + String(rainGaugePin));
  pinMode(rainGaugePin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rainGaugePin), tippingBucket, FALLING);
  //esp_sleep_enable_ext0_wakeup(GPIO_NUM_X1, 0);

  DEBUG_PRINT("Rain Gauge 2 attached to pin: " + String(rainGaugePin2));
  pinMode(rainGaugePin2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rainGaugePin2), tippingBucket2, FALLING);
}

void rainfallAmountReset()
{
  smartDelay(5000);
  if (gps.date.isValid())
  {
    if (rainGaugeDate != gps.date.day())
    {
      DEBUG_PRINT("Resetting Rain Gauge paramaters.");
      tipCount = 0;
      tipTime = 0;
      lastDetectedTipMillis = 0;
      tipCount2 = 0;
      tipTime2 = 0;
      lastDetectedTipMillis2 = 0;
      rainGaugeDate = gps.date.day();
    }
  }
  if ((millis() - lastDetectedTipMillis) >= 3.6e6)
  {
    lastDetectedTipMillis = 0;
    tipTime = 0;
  }
}

double rainfallRate()
{
  if (tipCount == 0 || tipTime == 0)
    return 0;
  else
  {
    return (double)((double)tipAmount * (double)3.6e6 / (double)(tipTime / 1.00));
  }
}

double rainfallAmount()
{
  return (double)((double)tipCount * tipAmount);
}

float rainfallRate2()
{
  if (tipCount2 == 0 || tipTime2 == 0)
    return 0;
  else
  {
    return (double)((double)tipAmount2 * (double)3.6e6 / (double)(tipTime2 / 1.00));
  }
}

float rainfallAmount2()
{
  return (double)((double)tipCount2 * tipAmount2);
}
/////////////////////////  END RAIN GAUGE  ///////////////////////

/////////////////////////  GSM/GPRS  ///////////////////////
#ifdef MODEM_GSM
void connectGSM(int gsm_baud, int gsm_tx, int gsm_rx, const char *apn, const char *gprsUser, const char *gprsPass)
{
  SerialGSM.begin(gsm_baud, SERIAL_8N1, gsm_tx, gsm_rx); // Initialise serial connection to GSM module
  DEBUG_PRINT("Initialising GSM.");
  DEBUG_PRINT("BAUD: " + String(gsm_baud) + "\tTX" + String(gsm_tx) + "\tRX" + String(gsm_rx));
  DEBUG_PRINT("Initialising GSM.");
  while (!modem.restart())
  { // Restarts GSM Modem
    DEBUG_PRINT("Failed to restart modem. Trying in 5s.");
    wait(1000);
    SerialGSM.begin(gsm_baud, SERIAL_8N1, gsm_tx, gsm_rx);
    wait(4000);
  }

  DEBUG_PRINT("Modem Name: " + modem.getModemName());
  DEBUG_PRINT("Modem Info: " + modem.getModemInfo());
  DEBUG_PRINT("Signal Quality: " + String(modem.getSignalQuality()));

  //Connect to GPRS
  gprsConnect();
}

void gprsConnect()
{
  DEBUG_PRINT("Connecting to GPRS.");
  while (!modem.gprsConnect(apn, gprsUser, gprsPass))
  {
    DEBUG_PRINT("Connecting to " + String(apn));
    //    delay(10000);
    wait(5000);
  }

  if (modem.isGprsConnected())
  {
    DEBUG_PRINT("Connected to " + String(apn));
  }
  else
  {
    DEBUG_PRINT("Disconnected!");
  }

  //DEBUG_PRINT("IMEI:           " + String(modem.getIMEI()));
  DEBUG_PRINT("Operator:       " + String(modem.getOperator()));
  DEBUG_PRINT("Signal Quality: " + String(modem.getSignalQuality()));
  //DEBUG_PRINT("GSM Date:       " + String(modem.getGSMDateTime(DATE_DATE)));
  //DEBUG_PRINT("GSM Time:       " + String(modem.getGSMDateTime(DATE_TIME)));
}

void sleepGSM()
{
  DEBUG_PRINT("Sleeping GSM.");
  SerialGSM.println("AT+CSCLK=2");
}

void wakeupGSM()
{
  DEBUG_PRINT("Waking Up GSM.");
  SerialGSM.println("AT");
  wait(500);
  SerialGSM.println("AT+CSCLK=0");
}

#endif
///////////////////////// END GSM/GPRS  ///////////////////////

///////////////////////// WIFI  ///////////////////////
#ifdef MODEM_WIFI
void connectWifi(const char *ssid, const char *password)
{
  uint8_t i = 0;

  DEBUG_PRINT("WIFI Status = " + String(WiFi.getMode()));
  WiFi.disconnect(true);
  wait(1000);

  WiFi.mode(WIFI_STA);
  wait(2000);

  DEBUG_PRINT("WIFI Status = " + String(WiFi.getMode()));
  WiFi.begin(ssid, password);
  DEBUG_PRINT("Connecting to " + String(ssid));

  while (WiFi.status() != WL_CONNECTED)
  {
    DEBUG_PRINT("Attempting to connect...");
    wait(500);

    if ((++i % 16) == 0)
    {
      DEBUG_PRINT("Still attempting to connect...");
      return;
    }
  }

  wait(10000);
}

void disconnectWifi()
{
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
}

int32_t getRSSI(const char *target_ssid)
{
  byte available_networks = WiFi.scanNetworks();

  for (int network = 0; network < available_networks; network++)
  {
    if (strcmp(WiFi.SSID(network).c_str(), target_ssid) == 0)
    {
      return WiFi.RSSI(network);
    }
  }
  return 0;
}
#endif
///////////////////////// END WIFI  ///////////////////////

///////////////////////// GPS  ///////////////////////
void attachGPS()
{
  SerialGPS.begin(GPS_BAUD, SERIAL_8N1, GPS_TX, GPS_RX);
  DEBUG_PRINT("Attached GPS @ RX: " + String(GPS_RX) + "  TX: " + String(GPS_TX));
}

String getUnixTime()
{
  tmElements_t te;
  time_t unixTime = 1590060818;
  smartDelay(5000);
  gps.encode(SerialGPS.read());
  while (!gps.time.isValid())
  {
    gps.encode(SerialGPS.read());
    smartDelay(5000);
  }
  te.Second = gps.time.second();
  te.Hour = gps.time.hour();
  te.Minute = gps.time.minute();
  te.Day = gps.date.day();
  te.Month = gps.date.month();
  te.Year = gps.date.year() - (uint32_t)1970;
  unixTime = makeTime(te);
  return (String)unixTime;
}

void sendPacket(byte *packet, byte len)
{
  for (byte i = 0; i < len; i++)
  {
    SerialGPS.write(packet[i]); // GPS is HardwareSerial
  }
}

void GPS_powerSaveMode()
{
  DEBUG_PRINT("Setting GPS to Power Save Mode (PSM).");
  byte packet[] = {0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92};
  sendPacket(packet, sizeof(packet));
}

void GPS_maxPerformanceMode()
{
  DEBUG_PRINT("Setting GPS to Power Save Mode (PSM).");
  byte packet[] = {0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x00, 0x21, 0x91};
  sendPacket(packet, sizeof(packet));
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (SerialGPS.available())
      gps.encode(SerialGPS.read());
  } while (millis() - start < ms);
}

///////////////////////// END GPS  ///////////////////////

///////////////////////// ULTRASONIC  ///////////////////////
void attachUS()
{
  DEBUG_PRINT("Attaching Ultrasonic Sensor. TX:" + String(US_TX) + " RX:" + String(US_RX) + " USReset:" + String(US_resetButton));
  pinMode(US_RX, OUTPUT);
  pinMode(US_TX, INPUT);
  attachInterrupt(digitalPinToInterrupt(US_resetButton), usISR, FALLING);
}

void getHeight()
{
  indicatorLED(true);
  wait(100);
  indicatorLED(false);
  wait(100);
  indicatorLED(true);
  wait(100);
  indicatorLED(false);
  wait(100);
  indicatorLED(true);
  wait(100);
  indicatorLED(false);
  wait(100);
  indicatorLED(true);

  for (int i = 0; i < datasizeUS; i++)
  {
    // DEBUG_PRINT(i);
    float temp = getTemperature();
    float hum = getHumidity();
    float speedOfSound = 331.4 + (0.606 * temp) + (0.0124 * hum);
    float soundCM = speedOfSound / 10000;
    // DEBUG_PRINT("Temperature:" + String(temp));
    // DEBUG_PRINT("Humidity:" + String(hum));
    // DEBUG_PRINT("Speed of Sound in CM: " + String(soundCM));
    digitalWrite(US_RX, LOW);
    delayMicroseconds(2);

    digitalWrite(US_RX, HIGH);
    delayMicroseconds(20);
    digitalWrite(US_RX, LOW);

    duration = pulseIn(US_TX, HIGH);

    distanceH = (duration / 2) * ((331.4 + (0.606 * temp) + (0.0124 * hum)) / 10000);
    // distanceH = duration * 0.034 / 2;
    raftHeight = distanceH;

    unsigned long timeCount = micros();
    medianGetHeight = medianFilter.AddValue(raftHeight);
    // Serial.println(medianHeight);
    timeCount = micros() - timeCount;
    timeMean += timeCount;

    // DEBUG_PRINT(raftHeight);
    wait(1000);
  }
  // DEBUG_PRINT("Current Height: " + String(medianGetHeight));
  indicatorLED(false);
}

void usISR()
{
  if (millis() - lastDetectedTipMillisUS > 10000)
  {
    DEBUG_PRINT("Height write enabled.");
    lastDetectedTipMillisUS = millis();
    heightWrite = true;
  }
}

void setHeight()
{
  DEBUG_PRINT("Saving height to memory.");
  EEPROM.writeFloat(0, medianGetHeight);
  EEPROM.commit();
  delay(50);
  setHeightTask.disable();
  DEBUG_PRINT("Succesfully saved height of " + String(medianGetHeight) + " in memory.");
}

float getDepth()
{
  // medianHeight = EEPROM.read(0);
  medianHeight = EEPROM.readFloat(0);
  DEBUG_PRINT("Median Height: " + String(medianHeight));
  for (int i = 0; i < datasizeUS; i++)
  {
    float temp = getTemperature();
    float hum = getHumidity();
    float speedOfSound = 331.4 + (0.606 * temp) + (0.0124 * hum);
    float soundCM = speedOfSound / 10000;

    digitalWrite(US_RX, LOW);
    delayMicroseconds(2);

    digitalWrite(US_RX, HIGH);
    delayMicroseconds(20);
    digitalWrite(US_RX, LOW);

    duration = pulseIn(US_TX, HIGH, 26000);
    distanceD = (duration / 2) * soundCM;

    // distanceD = duration * 0.034 / 2;
    //Serial.println(distanceD);
    floodDepth = distanceD;
    //Serial.println(floodDepth);
    unsigned long timeCount = micros();
    medianDepth = medianFilter.AddValue(floodDepth);
    timeCount = micros() - timeCount;
    timeMean += timeCount;

    //Serial.println(medianDepth);

    medianDepth = medianHeight - medianDepth;
    wait(1000);
  }
  floodDepth = medianDepth;
  DEBUG_PRINT("Flood Depth: " + String(medianDepth));
  if (floodDepth < 0)
    return 0;
  else
    return floodDepth;
}

///////////////////////// END ULTRASONIC  ///////////////////////

///////////////////////// BATTERY ///////////////////////
float getBatteryLevel()
{ // Returns State Of Charge (SOC) by voltage.
  double voltage = getBatteryVoltage();
  if (voltage <= BATTMINVOLT)
  {
    return 0.00;
  }
  else if (voltage > BATTMAXVOLT)
  {
    return 100.00;
  }
  else
  {
    float batteryLevel = (voltage - BATTMINVOLT) / (BATTMAXVOLT - BATTMINVOLT) * 100.00;
    return batteryLevel;
  }
}

double getBatteryVoltage()
{
  int sampleRate = 50;
  double sum = 0;
  for (int j = 0; j < sampleRate; j++)
  {
    sum += ReadVoltage(BATTERYPIN) / BATTERYRATIO;

    wait(5);
  }
  double voltage = sum / sampleRate;
  return voltage;
}

double ReadVoltage(byte pin)
{
  double reading = analogRead(pin); // Reference voltage is 3v3 so maximum reading is 3v3 = 4095 in range 0 to 4095
  if (reading < 1 || reading > 4095)
    return 0;
  return -0.000000000000016 * pow(reading, 4) + 0.000000000118171 * pow(reading, 3) - 0.000000301211691 * pow(reading, 2) + 0.001109019271794 * reading + 0.034143524634089;
  wait(5);
}
///////////////////////// END BATTERY ///////////////////////

///////////////////////// BAROMETRIC SENSOR ///////////////////////

void attachBarometer()
{
  DEBUG_PRINT("Attaching Barometer.");
  if (!bme.begin(0x76))
  {
    DEBUG_PRINT("Could not find a valid BME280 sensor, check wiring!");
    return;
  }
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1, // Temperautre
                  Adafruit_BME280::SAMPLING_X1, // Pressure
                  Adafruit_BME280::SAMPLING_X1, // Humidity
                  Adafruit_BME280::FILTER_OFF);
}

float getAltitude()
{
  return bme.readAltitude(SEALEVELPRESSURE_HPA);
}

float getPressure()
{
  return (bme.readPressure() / 100.0F);
}

float getTemperature()
{
  return bme.readTemperature();
}

float getHumidity()
{
  return bme.readHumidity();
}

///////////////////////// END BAROMETRIC SENSOR ///////////////////////
///////////////////////// MQTT ///////////////////////
void dataPublish()
{
  DynamicJsonDocument payloadData(1024);
  String payloadBuffer;
  String topic;
  int len;

  payloadData["data_type"] = "event";
  payloadData["stream_id"] = streamID;

  DEBUG_PRINT("Retrieving data.");
  String unixTime = getUnixTime();
  DEBUG_PRINT("Current time: " + String(unixTime));
  JsonObject payload_Data = payloadData.createNestedObject("data");
  JsonObject objectLatitude = payload_Data.createNestedObject("LAT1");
  objectLatitude["time"] = unixTime;
  objectLatitude["value"] = gps.location.lat();

  JsonObject objectLongitude = payload_Data.createNestedObject("LNG1");
  objectLongitude["time"] = unixTime;
  objectLongitude["value"] = gps.location.lng();

  JsonObject objectAltitude = payload_Data.createNestedObject("ALT1");
  objectAltitude["time"] = unixTime;
  objectAltitude["value"] = getAltitude();

  JsonObject objectRainRate = payload_Data.createNestedObject("RR1");
  objectRainRate["time"] = unixTime;
  objectRainRate["value"] = rainfallRate();

  JsonObject objectRainAmount = payload_Data.createNestedObject("RA1");
  objectRainAmount["time"] = unixTime;
  objectRainAmount["value"] = rainfallAmount();

  JsonObject objectRainRate2 = payload_Data.createNestedObject("RR2");
  objectRainRate2["time"] = unixTime;
  objectRainRate2["value"] = rainfallRate2();

  JsonObject objectRainAmount2 = payload_Data.createNestedObject("RA2");
  objectRainAmount2["time"] = unixTime;
  objectRainAmount2["value"] = rainfallAmount2();

  JsonObject objectTemp = payload_Data.createNestedObject("TMP1");
  objectTemp["time"] = unixTime;
  objectTemp["value"] = getTemperature();

  JsonObject objectPress = payload_Data.createNestedObject("PR1");
  objectPress["time"] = unixTime;
  objectPress["value"] = getPressure();

  JsonObject objectHumid = payload_Data.createNestedObject("HU1");
  objectHumid["time"] = unixTime;
  objectHumid["value"] = getHumidity();

  JsonObject objectBatt = payload_Data.createNestedObject("BV1");
  objectBatt["time"] = unixTime;
  objectBatt["value"] = getBatteryVoltage();

  JsonObject objectFloodDepth = payload_Data.createNestedObject("FD1");
  objectFloodDepth["time"] = unixTime;
  objectFloodDepth["value"] = getDepth();

  serializeJson(payloadData, payloadBuffer);
  topic = "RAFT_Data";
  len = strlen(payloadBuffer.c_str());                                                 // Calculates Payload Size
  rainflowMQTT.publish(topic.c_str(), 2, false, payloadBuffer.c_str(), len, false, 0); // Publishes payload to server
  DEBUG_PRINT("Published @ " + String(topic) + "\n" + String(payloadBuffer));
  DEBUG_PRINT("payloadData Mem:" + String(payloadData.memoryUsage()));
  wait(2000);
}

void onMqttConnect(bool sessionPresent)
{
  DEBUG_PRINT("MQTT Connected. Session present:" + String(sessionPresent));
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
  DEBUG_PRINT("MQTT Disconnected.");
  // Serial.println("Disconnected from MQTT.");
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos)
{
  DEBUG_PRINT("Subscribe acknowledged. PacketID:" + String(packetId));
}

void onMqttUnsubscribe(uint16_t packetId)
{
  DEBUG_PRINT("Unsubscribe acknowledged. PacketID:" + String(packetId));
}

void onMqttMessage(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total)
{
  DEBUG_PRINT("Published Received. ");
  DEBUG_PRINT("Topic" + String(topic));
  DEBUG_PRINT("Payload" + String(payload));
}

void onMqttPublish(uint16_t packetId)
{
  DEBUG_PRINT("Publish acknowledged. PacketID:" + String(packetId));
}
///////////////////////// END MQTT ///////////////////////

void modeCheck()
{
  DEBUG_PRINT("Checking Mode.");
  // 0: Standby, 1: Continuous Monitoring, 2: Batery Saver
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  float minFloodDepth = 10; // Minimum flood depth is 10 cm;
  float curFloodDepth = getDepth();
  unsigned long lastTipTime = 1.8e6; // Last rain gauge tip time should be greater/equal to 30 minutes
  unsigned long currentTime = millis();
  float curBattLevel = getBatteryLevel();
  DEBUG_PRINT("Last Detected Millis: " + String(lastDetectedTipMillis) + " " + String(lastDetectedTipMillis2));
  DEBUG_PRINT("Current Time: " + String(currentTime));
  DEBUG_PRINT("Current Flood Depth: " + String(curFloodDepth));
  DEBUG_PRINT("Current Battery Level: " + String(curBattLevel));
  // DEBUG_PRINT((currentTime - lastDetectedTipMillis) >= lastTipTime);
  // mode_Standby();
  // mode_ContinuousMonitoring();
  // mode_BatterySaver();
  if (((curFloodDepth < minFloodDepth) && ((currentTime - lastDetectedTipMillis) >= lastTipTime) && ((currentTime - lastDetectedTipMillis2) >= lastTipTime)) || ((currentMode == 0) && (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER)))
  {
    mode_Standby();
  }
  else if (((curFloodDepth >= minFloodDepth) || ((currentTime - lastDetectedTipMillis) < lastTipTime) || ((currentTime - lastDetectedTipMillis2) < lastTipTime)) && (curBattLevel > 20.00))
  {
    mode_ContinuousMonitoring();
  }
  else if (((curFloodDepth >= minFloodDepth) || ((currentTime - lastDetectedTipMillis) < lastTipTime) || ((currentTime - lastDetectedTipMillis2) < lastTipTime)) && (curBattLevel <= 20.00))
  {
    mode_BatterySaver();
  }
}

void mode_Standby()
{
  DEBUG_PRINT("Standby Mode.");
  currentMode = 0;
  int minutesToSleep = 30;
  publishDataScheduler.disable();
  checkMode.disable();
  publishData();
  sleep(minutesToSleep * 60); // minutesToSleep (min) * (60 seconds/min)
}

void mode_ContinuousMonitoring()
{
  DEBUG_PRINT("Mode: Continuous Monitoring");
  if (!publishDataScheduler.isEnabled() || currentMode != 1)
  {
    publishDataScheduler.disable();
    publishDataScheduler.setInterval(5 * 60e3); // Every 5 minutes
    publishDataScheduler.enable();
  }
  if (!checkMode.isEnabled() || currentMode != 1)
  {
    checkMode.disable();
    checkMode.setInterval(30 * 60e3); // Every 30 minutes
    checkMode.enable();
  }
  currentMode = 1;
}

void mode_BatterySaver()
{
  DEBUG_PRINT("Mode: Battery Saver");
  if (!publishDataScheduler.isEnabled() || currentMode != 2)
  {
    publishDataScheduler.disable();
    publishDataScheduler.setInterval(30 * 60e3);
    publishDataScheduler.enable();
  }
  if (!checkMode.isEnabled() || currentMode != 2)
  {
    checkMode.disable();
    checkMode.setInterval(15 * 60e3);
    checkMode.enable();
  }
  currentMode = 2;
}

void sleep(int time_to_sleep)
{
  //esp_sleep_enable_ext0_wakeup(GPIO_NUM_21, ESP_EXT1_WAKEUP_ALL_LOW:);
  esp_sleep_enable_ext1_wakeup(GPIO_PIN_BITMASK, ESP_EXT1_WAKEUP_ALL_LOW);
  esp_sleep_enable_timer_wakeup(time_to_sleep * uS_TO_S_FACTOR);
  DEBUG_PRINT("Sleeping for " + String(time_to_sleep) + " Seconds");
  rainflowMQTT.disconnect();
  DEBUG_PRINT("Disconnected from server.");
  wait(5000);
  Serial.flush();
  esp_deep_sleep_start();
}

void print_wakeup_reason()
{
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason)
  {
  case ESP_SLEEP_WAKEUP_EXT0:
    DEBUG_PRINT("Wakeup caused by external signal using RTC_IO");
    break;
  case ESP_SLEEP_WAKEUP_EXT1:
    DEBUG_PRINT("Wakeup caused by external signal using RTC_CNTL");
    break;
  case ESP_SLEEP_WAKEUP_TIMER:
    DEBUG_PRINT("Wakeup caused by timer");
    break;
  default:
    DEBUG_PRINT("Wakeup was not caused by deep sleep:");
    DEBUG_PRINT(String(wakeup_reason));
    break;
  }
}

void wait(unsigned long interval)
{
  // In milliseconds (1s == 1000 ms)
  unsigned long time_now = millis();
  while (!(millis() - time_now >= interval))
  {
  }
}

void publishData()
{
  indicatorLED(true);
  bool connected = false;

#ifdef MODEM_WIFI

  if (WiFi.status() != WL_CONNECTED)
  {
    connectWifi(ssid, wifi_pass);
    wait(5000);
  }
  if (WiFi.status() == WL_CONNECTED)
  {
    connected = true;
  }
#endif
#ifdef MODEM_GSM
  connectGSM(GSM_BAUD, GSM_TX, GSM_RX, apn, gprsUser, gprsPass);
  gprsConnect();
#endif

  if (connected == true)
  {
    rainflowMQTT.connect();
    wait(5000);
    if (rainflowMQTT.connected())
      dataPublish();
    rainflowMQTT.disconnect();
  }

#ifdef MODEM_WIFI
  disconnectWifi();
#endif
#ifdef MODEM_GSM
#endif

  indicatorLED(false);
}

void indicatorLED(bool status)
{
  if (status == true)
  {
    pinMode(LEDPin, OUTPUT);
    digitalWrite(LEDPin, HIGH);
  }
  if (status == false)
  {
    digitalWrite(LEDPin, LOW);
    pinMode(LEDPin, INPUT);
  }
}

void setup()
{
#ifdef DEBUG_MODE
  Serial.begin(115200);
  wait(1000); // Delay is necessary as Serial takes some time
#endif
  DEBUG_PRINT("System initialising");
  setCpuFrequencyMhz(80); // CPU Frequency set to 80MHz to reduce power consumption
  bootCount++;
  print_wakeup_reason();
  esp_sleep_wakeup_cause_t esp_sleep_get_wakeup_cause();
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT1)
  {
    uint64_t GPIO_reason = esp_sleep_get_ext1_wakeup_status();
    // int GPIO = log(GPIO_reason) / log(2);
    // DEBUG_PRINT("GPIO Reason:" + String(GPIO_reason));
    // DEBUG_PRINT("GPIO:" + String(GPIO));

    if (GPIO_reason != 0)
    {
      int pin = __builtin_ffsll(GPIO_reason) - 1;
      Serial.printf("Wake up from GPIO %d\n", pin);
    }
    else
    {
      Serial.printf("Wake up from GPIO\n");
    }

    // if (GPIO == rainGaugePin)
    // {
    //   tippingBucket();
    //   tipTime = 0;
    // }

    // if (GPIO == rainGaugePin2)
    // {
    //   tippingBucket2();
    //   tipTime2 = 0;
    // }
    tipTime = 0;
  }

  if (!EEPROM.begin(64))
  {
    DEBUG_PRINT("Failed to initialise EEPROM");
    DEBUG_PRINT("Restarting...");
    wait(1000);
    ESP.restart();
  }
  // -- Attach Sensors
  pinMode(BATTERYPIN, INPUT); // Have to check if still necessary
  attachGPS();
  attachRainGauge();
  attachBarometer();
  attachUS();
  if (bootCount == 1)
  {
    getHeight();
    if (heightWrite == true)
      setHeight();
  }
  if ((wakeup_reason != ESP_SLEEP_WAKEUP_EXT1) && (wakeup_reason != ESP_SLEEP_WAKEUP_TIMER))
  {
    //    int GPIO_reason = esp_sleep_get_ext1_wakeup_status();
    rainfallAmountReset();
    GPS_powerSaveMode();
  }

  //#ifdef MODEM_WIFI
  //  connectWifi(ssid, wifi_pass);
  //#endif
  //#ifdef MODEM_GSM
  //  connectGSM(GSM_BAUD, GSM_TX, GSM_RX, apn, gprsUser, gprsPass);
  //#endif
  //
  // rainflow.rainflow(clientID, username, password); // Set RainFLOW API to use this client modem
  //  rainflow.connectServer(clientID, username, password); // Connects to RainFLOW Server

  rainflowMQTT.setClientId(clientID);
  rainflowMQTT.setCredentials(username, password);
  rainflowMQTT.setServer("rainflow.live", 1883);
  rainflowMQTT.onConnect(onMqttConnect);
  rainflowMQTT.onDisconnect(onMqttDisconnect);
  rainflowMQTT.onSubscribe(onMqttSubscribe);
  rainflowMQTT.onUnsubscribe(onMqttUnsubscribe);
  rainflowMQTT.onMessage(onMqttMessage);
  rainflowMQTT.onPublish(onMqttPublish);

  Runner.init();
  Runner.addTask(publishDataScheduler);
  Runner.addTask(checkMode);
  Runner.addTask(rainfallReset);
  Runner.addTask(setHeightTask);
  rainfallReset.enable();
  modeCheck();
}

void loop()
{
  Runner.execute();
}
