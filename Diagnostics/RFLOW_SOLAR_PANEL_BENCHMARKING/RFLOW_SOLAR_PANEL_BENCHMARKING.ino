// Definitions
#define MICRO_BAUD_RATE 115200  // Microcontroller Baud Rate
#define BATTMAXVOLT 4.2         // Maximum Battery Voltage
#define BATTMINVOLT 3.3         // Minimum Battery Voltage
#define BATTERYPIN 33           // BATTERY PIN
#define BATTERYRATIO 0.71825
#define LED    5
#define DEBUG_MODE

#define BATTMAXVOLT 4.2         // Maximum Battery Voltage
#define BATTMINVOLT 3.2         // Minimum Battery Voltage
#define BATTERYPIN 33           // BATTERY PIN
#define BATTERYRATIO 0.76029

#ifdef DEBUG_MODE
#define DEBUG_PRINT(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#endif

#include "rainflow.h"
#include <ArduinoJson.h>
#include <HardwareSerial.h>
#include <TaskScheduler.h>
//#include <ESP8266WiFi.h>
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  300        /* Time ESP32 will go to sleep (in seconds) */
//#define TIME_TO_SLEEP  60        /* Time ESP32 will go to sleep (in seconds) */

RTC_DATA_ATTR int bootCount = 0;


//WiFi Details
const char* ssid                  = "Hidden Network";
const char* password              = "mmbmh15464";


// RAFT Details
//const char* APIKey  = "1429fedd-4956-4187-95aa";   // Change to API-Key
const char* APIKey  = "861db3ff0-9c48-43ab-91b8";



WiFiClient client;
RainFLOW rainflow;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

void publishData();

Task publishDataScheduler(10000, TASK_FOREVER, &publishData);
Scheduler runner;

String message;
DynamicJsonDocument doc(1024);

void printLocalTime() {
  String formattedDate = timeClient.getFormattedDate();
  Serial.println(formattedDate);
}

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}

void connectWifi(const char* ssid, const char* password) {
  WiFi.mode(WIFI_STA);
  // int a = esp_wifi_set_protocol(WIFI_STA, WIFI_PROTOCOL_LR);
  // Serial.println(a);
  WiFi.begin(ssid, password);
  DEBUG_PRINT("Connecting to " + String(ssid));

  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED) {
    //DEBUG_PRINT("Attempting to connect..."        );
    delay(500);
    if ((++i % 100) == 0)
    {
      DEBUG_PRINT("Failed to connect.");
      break;
    }
    if ((++i % 10) == 0)
    {
      DEBUG_PRINT("Still attempting to connect...");
    }

  }
  DEBUG_PRINT("Connected.");
  DEBUG_PRINT("My IP Address: ");
  DEBUG_PRINT(WiFi.localIP());
  digitalWrite(LED, HIGH);
  delay(2500);
  digitalWrite(LED, LOW);
}

int32_t getRSSI(const char* target_ssid) {
  byte available_networks = WiFi.scanNetworks();

  for (int network = 0; network < available_networks; network++) {
    if (strcmp(WiFi.SSID(network).c_str(), target_ssid) == 0) {
      return WiFi.RSSI(network);
    }
  }
  return 0;
}



float getBatteryLevel() {
  double voltage = getBatteryVoltage();
  if (voltage <= BATTMINVOLT) {
    return 0.00;
  }
  else if (voltage > BATTMAXVOLT) {
    return 100.00;
  }
  else {
    float batteryLevel = ((getBatteryVoltage() - BATTMINVOLT) / (BATTMAXVOLT - BATTMINVOLT)) * 100.00;
    return batteryLevel;
  }
}

double getBatteryVoltage() {
  int sampleRate = 50;
  double sum = 0;
  for (int j = 0; j < sampleRate; j++) {
    sum += ReadVoltage(BATTERYPIN) / BATTERYRATIO;

    delay(5);
  }
  double voltage = sum / sampleRate;
  return voltage;
}


String uptime() {
  long days = 0, hours = 0, mins = 0, secs = 0;
  //secs = currentmillis / 1000;    // Convert milliseconds to seconds
  secs = millis() / 1000;           // Convert milliseconds to seconds
  mins = secs / 60;                 // Convert seconds to minutes
  hours = mins / 60;                // Convert minutes to hours
  days = hours / 24;                // Convert hours to days
  secs = secs - (mins * 60);        // Subtract the coverted seconds to minutes in order to display 59 secs max
  mins = mins - (hours * 60);       // Subtract the coverted minutes to hours in order to display 59 minutes max
  hours = hours - (days * 24);      // Subtract the coverted hours to days in order to display 23 hours max

  String uptimeBuffer = String(days) + ":" + String(hours) + ":" + String(mins) + ":" + String(secs);
  return uptimeBuffer;
}

double ReadVoltage(byte pin) {
  double reading = analogRead(pin); // Reference voltage is 3v3 so maximum reading is 3v3 = 4095 in range 0 to 4095
  if (reading < 1 || reading > 4095) return 0;
  return -0.000000000000016 * pow(reading, 4) + 0.000000000118171 * pow(reading, 3) - 0.000000301211691 * pow(reading, 2) + 0.001109019271794 * reading + 0.034143524634089;
  delay(5);
}

void getData() {
  timeClient.forceUpdate();
  String formattedDate = timeClient.getFormattedDate();
  int splitT = formattedDate.indexOf("T");
  String dayStamp = formattedDate.substring(0, splitT);
  String timeStamp = formattedDate.substring(splitT + 1, formattedDate.length() - 1);
  rainflow.addData("Date",              dayStamp);
  rainflow.addData("Time",              timeStamp);
  rainflow.addData("Uptime",            uptime());
  rainflow.addData("SP1 Current",       doc["SP1 Current"]);
  rainflow.addData("SP1 Voltage",       doc["SP1 Voltage"]);
  rainflow.addData("SP1 Power",         doc["SP1 Power"]);
  rainflow.addData("SPR Current",       doc["SPR Current"]);
  rainflow.addData("BattLevelNano",     doc["BattLevelNano"]);
  rainflow.addData("BattVoltageNano",   doc["BattVoltageNano"]);
  rainflow.addData("BattLevelESP",        String(getBatteryLevel()));
  rainflow.addData("BattVoltageESP",      String(getBatteryVoltage()));
  rainflow.addData("WiFiRSSI",            String(getRSSI(ssid)));
  rainflow.addData("BootCount",           String(bootCount));
  doc.clear();
}

void publishData() {
  getData();
  if (WiFi.status() != WL_CONNECTED) {
    connectWifi(ssid, password);
  }
  if (WiFi.status() == WL_CONNECTED) {
    rainflow.publishData(APIKey);
    digitalWrite(LED, HIGH);
    delay(25);
    digitalWrite(LED, LOW);
    delay(50);
    digitalWrite(LED, HIGH);
    delay(25);
    digitalWrite(LED, LOW);
    delay(10000);
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
                   " Seconds");
    Serial.println("Going to sleep now");
    delay(1000);
    WiFi.disconnect();
    Serial.flush();
    esp_deep_sleep_start();
    //esp_light_sleep_start();
  }
}

void setup() {
  setCpuFrequencyMhz(80);
  Serial.begin(115200);
  delay(1000);
  ++bootCount;
  void print_wakeup_reason();
  Serial.println("Boot number: " + String(bootCount));
  print_wakeup_reason();
  Serial.println("System initialising");
  rainflow.rainflow(client);
  pinMode(LED, OUTPUT);
  connectWifi(ssid, password);
  runner.init();
  runner.addTask(publishDataScheduler);
  publishDataScheduler.enable();
  timeClient.begin();
  timeClient.setTimeOffset(28800);
  timeClient.forceUpdate();
  printLocalTime();
  rainflow.connectServer(APIKey);
  int32_t rssi = getRSSI(ssid);
  Serial.println(rssi);
}

void loop() {
  while (Serial.available()) {
    message = Serial.readString();
    Serial.println("Received message: ");
    Serial.println(message);
    DeserializationError error = deserializeJson(doc, message);
    if (error)
      return;
    timeClient.update();
    printLocalTime();
    publishData();
  }
    //runner.execute();
}
