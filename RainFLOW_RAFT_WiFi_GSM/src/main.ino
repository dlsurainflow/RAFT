#include "settings.h"

#define FIRMWARE_VER 1.0

#ifdef DEBUG_MODE
#define DEBUG_PRINT(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#endif

void modeCheck();
void setHeight();
void publishData();
void rainfallAmountReset();

Task publishDataScheduler(10 * 60e3, TASK_FOREVER, &publishData);  // Every 10 minutes -10min*60seconds*1000
Task checkMode(5 * 60e3, TASK_FOREVER, &modeCheck);                // Every 5 minutes
Task rainfallReset(10 * 60e3, TASK_FOREVER, &rainfallAmountReset); // Every 10 minutes
Task setHeightTask(10, TASK_ONCE, &setHeight);
Scheduler Runner;
RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR int currentMode = 0;
long currentmillis = 0;

/////////////////////////  RAIN GAUGE  ///////////////////////

RTC_DATA_ATTR int tipCount = 0;                              // Total Amount of Tips
RTC_DATA_ATTR int rainGaugeDate = 0;                         // Rain Gauge Date
static unsigned long lastDetectedTipMillis = 0, tipTime = 0; // Time of Last Rain Guage Tip

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR tippingBucket()
{

  // Debounce for a quarter second = max 4 counts/second
  if (millis() - lastDetectedTipMillis > 250)
  {
    portENTER_CRITICAL_ISR(&mux);

    tipTime = millis() - lastDetectedTipMillis;
    tipCount++;

    DEBUG_PRINT("Tip Count:" + String(tipCount));

    lastDetectedTipMillis = millis();

    portEXIT_CRITICAL_ISR(&mux);
  }
}
void attachRainGauge()
{
  DEBUG_PRINT("Rain Gauge 1 attached to pin: " + String(rainGaugePin));
  pinMode(rainGaugePin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(rainGaugePin), tippingBucket, FALLING);
}
void rainfallAmountReset()
{
  smartDelay(5000);
  if (gps.date.isValid())
  {
    if (rainGaugeDate != gps.date.day())
    {
      DEBUG_PRINT("Resetting rain gauge paramaters.");
      tipCount = 0;
      tipTime = 0;
      lastDetectedTipMillis = 0;
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
//////////////////////  END RAIN GAUGE  ///////////////////////

/////////////////////////  GSM/GPRS  ///////////////////////
#ifdef GSM_ENABLED
void connectGSM(int gsm_baud, int gsm_tx, int gsm_rx, const char *apn, const char *gprsUser, const char *gprsPass)
{
  resetModem();
  SerialGSM.begin(gsm_baud, SERIAL_8N1, gsm_tx, gsm_rx); // Initialise serial connection to GSM module
  DEBUG_PRINT("Initialising GSM.");
  DEBUG_PRINT("BAUD: " + String(gsm_baud) + "\tTX" + String(gsm_tx) + "\tRX" + String(gsm_rx));
  unsigned long currentMillis = millis();
  while (!modem.init() && (millis() - currentMillis >= 30000)) // 30 Second timeout.
  {
    DEBUG_PRINT("Failed to initialise modem. Trying in 5s.");
    wait(1000);
    SerialGSM.begin(gsm_baud, SERIAL_8N1, gsm_tx, gsm_rx);
    wait(4000);
  }

  DEBUG_PRINT("Modem Name: " + modem.getModemName());
  DEBUG_PRINT("Modem Info: " + modem.getModemInfo());
  DEBUG_PRINT("Signal Quality: " + String(modem.getSignalQuality()));
}
bool gprsConnect()
{
  resetModem();
  wait(30000);
  modem.init();

  unsigned long currentMillis = millis();
  DEBUG_PRINT("Connecting to GPRS.");
  if (!modem.gprsConnect(apn, gprsUser, gprsPass))
  {
    DEBUG_PRINT("GPRS Connect failed.");
    return false;
  }

  DEBUG_PRINT("GPRS Connect success");
  return true;
  // while (!modem.gprsConnect(apn, gprsUser, gprsPass) && (millis() - currentMillis >= 30000)) // Timeout of 30 seconds
  // {
  //   DEBUG_PRINT("Connecting to " + String(apn));
  //   wait(5000);
  // }
  // return modem.isGprsConnected();
}
int signalQuality()
{
  return modem.getSignalQuality();
}
void publishSMS(String topic, String _payload)
{
  DynamicJsonDocument payload(1024);
  payload["topic"] = topic;
  payload["payload"] = _payload;

  String payloadBuffer;
  serializeJson(payload, payloadBuffer);
  DEBUG_PRINT("Publishing via SMS: " + String(payloadBuffer) + " to gateway " + String(GATEWAY_NUMBER));
  int res = modem.sendSMS(GATEWAY_NUMBER, payloadBuffer);
  DEBUG_PRINT("SMS Sent: " + String(res));
}
void gprsDisconnect()
{
  modem.gprsDisconnect();
  DEBUG_PRINT("Disconnect GPRS.");
  gprsConnected = false;
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
void resetModem()
{
  DEBUG_PRINT("Resetting GSM Modem.");
  pinMode(GSM_RE, OUTPUT);
  digitalWrite(GSM_RE, LOW);
  wait(2500);
  digitalWrite(GSM_RE, HIGH);
  wait(1000);
  pinMode(GSM_RE, INPUT);
}
#endif
///////////////////////// END GSM/GPRS  ///////////////////////
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

  unsigned long start = millis();
  while ((WiFi.status() != WL_CONNECTED) && ((millis() - start) >= 30000))
  {
    DEBUG_PRINT("Attempting to connect...");
    wait(500);

    if ((++i % 16) == 0)
    {
      DEBUG_PRINT("Still attempting to connect...");
    }

    // if ((millis() - start) >= 30000) // 30s timeout
    //   break;
  }

  wait(5000);
}
void disconnectWifi()
{
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
}
int32_t getRSSI()
{
  byte available_networks = WiFi.scanNetworks();

  for (int network = 0; network < available_networks; network++)
  {
    if (strcmp(WiFi.SSID(network).c_str(), ssid) == 0)
    {
      return WiFi.RSSI(network);
    }
  }
  return 0;
}
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
  while (millis() - start < ms)
  {
    while (SerialGPS.available())
      gps.encode(SerialGPS.read());
  }
}

///////////////////////// END GPS  ///////////////////////

///////////////////////// ULTRASONIC  ///////////////////////
//* Ultrasonic Sensor Variables
RTC_DATA_ATTR float raftHeight = 0;            // Set original Height
RTC_DATA_ATTR unsigned long lastDepthTime = 0; // Last Depth Measurement Time
unsigned long lastDetectedTipMillisUS = 0;     // Last time
float medianGetHeight = 0;
float medianHeight = 0;
float medianDepth = 0;
bool heightWrite = false;
MedianFilter<int> medianFilter(datasizeUS);

void attachUS()
{
  DEBUG_PRINT("Attaching Ultrasonic Sensor. TX:" + String(US_TX) + " RX:" + String(US_RX) + " USReset:" + String(US_resetButton));
  pinMode(US_RX, INPUT);
  pinMode(US_TX, INPUT);
  attachInterrupt(digitalPinToInterrupt(US_resetButton), usISR, FALLING);
}
void usISR()
{
  if (millis() - lastDetectedTipMillisUS > 2500) // 2.5 seconds timeout
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
  indicatorLED(true);
  wait(100);
  indicatorLED(false);
  wait(100);
  indicatorLED(true);
  wait(100);
  indicatorLED(false);
}
void getHeight()
{
  float duration = 0;
  indicatorLED(true);

  int i = 0, counter = 0;
  for (; i < datasizeUS;)
  {
    float temp = getTemperature();
    float hum = getHumidity();
    pinMode(US_RX, OUTPUT);
    // float speedOfSound = 331.4 + (0.606 * temp) + (0.0124 * hum);
    // float soundCM = speedOfSound / 10000;

    digitalWrite(US_RX, LOW);
    delayMicroseconds(2);

    digitalWrite(US_RX, HIGH);
    delayMicroseconds(20);
    digitalWrite(US_RX, LOW);

    duration = pulseIn(US_TX, HIGH, 26000);

    raftHeight = (duration / 2) * ((331.4 + (0.606 * temp) + (0.0124 * hum)) / 10000);

    if (abs(raftHeight) <= US_MAXHEIGHT)
    {
      medianGetHeight = medianFilter.AddValue(raftHeight);
      i++;
    }

    medianGetHeight = medianFilter.GetFiltered();
    DEBUG_PRINT("RAFT Height " + String(i) + ": - " + String(medianGetHeight));
    counter++;

    if (counter >= (datasizeUS + 10)) // Max attempts of datasizeUS + 10 counts
      break;

    wait(1000); // 1 Second Delay
  }
  pinMode(US_RX, INPUT);
  indicatorLED(false);
}
float getDepth()
{
  float duration = 0, floodDepth = 0;
  medianHeight = EEPROM.readFloat(0);
  DEBUG_PRINT("Median Height: " + String(medianHeight));

  indicatorLED(true);
  pinMode(US_RX, OUTPUT);

  int i = 0, counter = 0;
  for (; i < datasizeUS;)
  {
    float temp = getTemperature();
    float hum = getHumidity();

    digitalWrite(US_RX, LOW);
    delayMicroseconds(2);

    digitalWrite(US_RX, HIGH);
    delayMicroseconds(20);
    digitalWrite(US_RX, LOW);

    duration = pulseIn(US_TX, HIGH, 26000);

    floodDepth = (duration / 2) * ((331.4 + (0.606 * temp) + (0.0124 * hum)) / 10000);

    if (abs(floodDepth) <= US_MAXHEIGHT)
    {
      medianDepth = medianFilter.AddValue(floodDepth);
      i++;
    }

    medianDepth = medianHeight - medianDepth;
    DEBUG_PRINT("Flood Depth " + String(i) + ": " + String(medianDepth));
    counter++;

    if (counter >= (datasizeUS + 10)) // Max attempts of datasizeUS + 10 counts
      break;

    wait(1000); // 1 Second Delay
  }
  // floodDepth = medianDepth;
  floodDepth = medianFilter.GetFiltered();
  pinMode(US_RX, INPUT);
  indicatorLED(false);

  DEBUG_PRINT("Flood Depth: " + String(medianDepth));
  if (floodDepth < 0)
    return 0;
  else
    return floodDepth;
}

///////////////////////// END ULTRASONIC  ///////////////////////

///////////////////////// BATTERY ///////////////////////
double getBatteryLevel()
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
  adcAttachPin(BATTERYPIN);
  DEBUG_PRINT("Battery Pin: " + String(BATTERYPIN));
  int sampleRate = 50;
  double sum = 0;
  for (int j = 0; j < sampleRate; j++)
  {
    double currentVoltage = ((double)ReadVoltage(BATTERYPIN) + BATT_OFFSET) / (double)BATTERYRATIO;
    sum += currentVoltage;
    // DEBUG_PRINT("Current Voltage: " + String(currentVoltage));
    wait(10);
  }
  double voltage = sum / (double)sampleRate;
  DEBUG_PRINT("Battery Sum: " + String(sum));
  DEBUG_PRINT("Battery Voltage: " + String(voltage));
  return voltage;
}
double ReadVoltage(int pin)
{
  double reading = analogRead(pin); // Reference voltage is 3v3 so maximum reading is 3v3 = 4095 in range 0 to 4095
  if (reading < 1 || reading > 4095)
    return 0;
  // return -0.000000000009824 * pow(reading,3) + 0.000000016557283 * pow(reading,2) + 0.000854596860691 * reading + 0.065440348345433;
  return -0.000000000000016 * pow(reading, 4) + 0.000000000118171 * pow(reading, 3) - 0.000000301211691 * pow(reading, 2) + 0.001109019271794 * reading + 0.034143524634089;
} // Added an improved polynomial, use either, comment out as required
///////////////////////// END BATTERY ///////////////////////

///////////////////////// BAROMETRIC SENSOR ///////////////////////

void attachBarometer()
{
  DEBUG_PRINT("Attaching Barometer.");
  Wire.begin();
  unsigned long status = millis();
  while ((millis() - status < 5000) && !bme.begin())
  {
    DEBUG_PRINT("Connecting Barometer.");
  }

  switch (bme.chipModel())
  {
  case BME280::ChipModel_BME280:
    DEBUG_PRINT("Found BME280 sensor!");
    break;
  case BME280::ChipModel_BMP280:
    DEBUG_PRINT("Found BMP280 sensor! No Humidity available.");
    break;
  default:
    DEBUG_PRINT("Found UNKNOWN sensor! Error!");
  }
}
float getAltitude()
{
  return EnvironmentCalculations::Altitude(getPressure(), EnvironmentCalculations::AltitudeUnit_Meters, SEALEVELPRESSURE_HPA, getTemperature(), EnvironmentCalculations::TempUnit_Celsius);
}
float getPressure()
{
  return bme.pres();
}
float getTemperature()
{
  return bme.temp();
}
float getHumidity()
{
  if (bme.chipModel() == BME280::ChipModel_BME280)
    return bme.hum();
  else
    return 0;
}
float getDewPoint()
{
  if (bme.chipModel() == BME280::ChipModel_BME280)
    return EnvironmentCalculations::DewPoint(getTemperature(), getHumidity(), EnvironmentCalculations::TempUnit_Celsius);
  else
    return 0;
}
float getHeadIndex()
{
  if (bme.chipModel() == BME280::ChipModel_BME280)
    return EnvironmentCalculations::HeatIndex(getTemperature(), getHumidity(), EnvironmentCalculations::TempUnit_Celsius);
  else
    return 0;
}
///////////////////////// END BAROMETRIC SENSOR ///////////////////////
///////////////////////// MQTT ///////////////////////
//* FOTA Functions
String getHeaderValue(String header, String headerName)
{
  return header.substring(strlen(headerName.c_str()));
}
String getBinName(String url)
{
  int index = 0;

  // Search for last /
  for (int i = 0; i < url.length(); i++)
  {
    if (url[i] == '/')
    {
      index = i;
    }
  }

  String binName = "";

  // Create binName
  for (int i = index; i < url.length(); i++)
  {
    binName += url[i];
  }

  return binName;
}
String getHostName(String url)
{
  int index = 0;

  // Search for last /
  for (int i = 0; i < url.length(); i++)
  {
    if (url[i] == '/')
    {
      index = i;
      break;
    }
  }

  String hostName = "";

  // Create binName
  for (int i = 0; i < index; i++)
  {
    hostName += url[i];
  }

  return hostName;
}
void update(char *payload)
{
  int contentLength = 0;
  bool isValidContentType = false;

  DynamicJsonDocument doc(200);
  deserializeJson(doc, payload);
  double FirmwareVer = doc["FirmwareVer"];
  String url = doc["url"];
  DEBUG_PRINT("URL: " + url);
  int port = 80;

  if (FirmwareVer > FIRMWARE_VER)
  {

    rainflowMQTT.publish("update", 2, true, "", 0, false, 0); // Remove retained message
    String bin = getBinName(url);
    String host = getHostName(url);

    Serial.println("Connecting to: " + host);
    if (client.connect(host.c_str(), port))
    {
      // Connection Succeed.
      // Fecthing the bin
      Serial.println("Fetching Bin: " + bin);

      // Get the contents of the bin file
      client.print(String("GET ") + bin + " HTTP/1.1\r\n" +
                   "Host: " + host + "\r\n" +
                   "Cache-Control: no-cache\r\n" +
                   "Connection: close\r\n\r\n");

      unsigned long timeout = millis();

      while (client.available() == 0)
      {
        if (millis() - timeout > 5000)
        {
          Serial.println("Client Timeout !");
          client.stop();
          return;
        }
      }
      while (client.available())
      {
        // read line till /n
        String line = client.readStringUntil('\n');
        // remove space, to check if the line is end of headers
        line.trim();

        // if the the line is empty,
        // this is end of headers
        // break the while and feed the
        // remaining `client` to the
        // Update.writeStream();
        if (!line.length())
        {
          //headers ended
          break; // and get the OTA started
        }

        // Check if the HTTP Response is 200
        // else break and Exit Update
        if (line.startsWith("HTTP/1.1"))
        {
          if (line.indexOf("200") < 0)
          {
            Serial.println("Got a non 200 status code from server. Exiting OTA Update.");
            break;
          }
        }

        // extract headers here
        // Start with content length
        if (line.startsWith("Content-Length: "))
        {
          contentLength = atoi((getHeaderValue(line, "Content-Length: ")).c_str());
          Serial.println("Got " + String(contentLength) + " bytes from server");
        }

        // Next, the content type
        if (line.startsWith("Content-Type: "))
        {
          String contentType = getHeaderValue(line, "Content-Type: ");
          Serial.println("Got " + contentType + " payload.");
          if (contentType == "application/octet-stream")
          {
            isValidContentType = true;
          }
        }
      }
    }
    else
    {
      // Connect to S3 failed
      // May be try?
      // Probably a choppy network?
      Serial.println("Connection to " + host + " failed. Please check your setup");
      // retry??
    }

    // Check what is the contentLength and if content type is `application/octet-stream`
    Serial.println("contentLength : " + String(contentLength) + ", isValidContentType : " + String(isValidContentType));

    // check contentLength and content type
    if (contentLength && isValidContentType)
    {
      // Check if there is enough to OTA Update
      bool canBegin = Update.begin(contentLength);
      if (canBegin)
      {
        Serial.println("Begin OTA. This may take 2 - 5 mins to complete. Things might be quite for a while.. Patience!");
        size_t written = Update.writeStream(client);

        if (written == contentLength)
        {
          Serial.println("Written : " + String(written) + " successfully");
        }
        else
        {
          Serial.println("Written only : " + String(written) + "/" + String(contentLength) + ". Retry?");
          // retry??
        }

        if (Update.end())
        {
          Serial.println("OTA done!");
          if (Update.isFinished())
          {
            Serial.println("Update successfully completed. Rebooting.");
            ESP.restart();
          }
          else
          {
            Serial.println("Update not finished? Something went wrong!");
          }
        }
        else
        {
          Serial.println("Error Occurred. Error #: " + String(Update.getError()));
        }
      }
      else
      {
        // not enough space to begin OTA
        // Understand the partitions and
        // space availability
        Serial.println("Not enough space to begin OTA.");
        client.flush();
      }
    }
    else
    {
      Serial.println("There was no content in the response");
      client.flush();
    }
  }
}
void onMqttConnect(bool sessionPresent)
{
  DEBUG_PRINT("MQTT Connected. Session present: " + String(sessionPresent));
}
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
  DEBUG_PRINT("MQTT Disconnected.");
}
void onMqttSubscribe(uint16_t packetId, uint8_t qos)
{
  DEBUG_PRINT("Subscribe acknowledged. PacketID:" + String(packetId));
}
void onMqttUnsubscribe(uint16_t packetId)
{
  DEBUG_PRINT("Unsubscribe acknowledged. PacketID: " + String(packetId));
}
void onMqttMessage(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total)
{
  DEBUG_PRINT("Message Received. ");
  DEBUG_PRINT("Topic: " + String(topic));
  DEBUG_PRINT("Payload: " + String(payload));

  if (String(topic) == "update")
  {
    update(payload);
  }
}
void onMqttPublish(uint16_t packetId)
{
  DEBUG_PRINT("Publish acknowledged. PacketID: " + String(packetId));
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
  DEBUG_PRINT("Last Detected Millis: " + String(lastDetectedTipMillis));
  DEBUG_PRINT("Current Time: " + String(currentTime));
  DEBUG_PRINT("Current Flood Depth: " + String(curFloodDepth));
  DEBUG_PRINT("Current Battery Level: " + String(curBattLevel));
  // mode_Standby();
  // mode_ContinuousMonitoring();
  // mode_BatterySaver();
  if (getBatteryVoltage() <= BATTMINVOLT) // Sleep immediately to conserve power and avoid data corruption
    sleepNoInterrupt(1800);               // Deep Sleep without Interrupt for 30 minutes)

  if (((curFloodDepth < minFloodDepth) && ((currentTime - lastDetectedTipMillis) >= lastTipTime)) || ((currentMode == 0) && (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER)))
  {
    mode_Standby();
  }
  else if (((curFloodDepth >= minFloodDepth) || ((currentTime - lastDetectedTipMillis) < lastTipTime)) && (curBattLevel > 20.00))
  {
    mode_ContinuousMonitoring();
  }
  else if (((curFloodDepth >= minFloodDepth) || ((currentTime - lastDetectedTipMillis) < lastTipTime)) && (curBattLevel <= 20.00))
  {
    mode_BatterySaver();
  }
}
void mode_Standby()
{
  DEBUG_PRINT("---Standby Mode.---");
  currentMode = 0;
  int minutesToSleep = 30;
  publishDataScheduler.disable();
  checkMode.disable();
  publishData();
  sleep(minutesToSleep * 60); // minutesToSleep (min) * (60 seconds/min)
}
void mode_ContinuousMonitoring()
{
  DEBUG_PRINT("---Continuous Monitoring---");
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
  DEBUG_PRINT("---Battery Saver---");
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
  esp_sleep_enable_ext1_wakeup(GPIO_PIN_BITMASK, ESP_EXT1_WAKEUP_ALL_LOW);
  esp_sleep_enable_timer_wakeup(time_to_sleep * uS_TO_S_FACTOR);
  DEBUG_PRINT("Sleeping for " + String(time_to_sleep) + " Seconds");
  wait(5000);
#ifdef GSM_ENABLED
  sleepGSM();
#endif
  Serial.flush();
  indicatorLED(true);
  indicatorLED1(true);
  wait(100);
  indicatorLED(false);
  indicatorLED1(false);
  esp_deep_sleep_start();
}
void sleepNoInterrupt(int time_to_sleep)
{
  esp_sleep_enable_timer_wakeup(time_to_sleep * uS_TO_S_FACTOR);
  DEBUG_PRINT("Sleeping with no interrupt for " + String(time_to_sleep) + " Seconds");
  Serial.flush();
  wait(1000);
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
void dataPublish(bool _wifiConnected, bool _gprsConnected)
{
  DynamicJsonDocument payloadData(1024);
  String payloadBuffer;
  String topic;
  int len;

  payloadData["data_type"] = "event";
  payloadData["stream_id"] = streamIDData;

  DEBUG_PRINT("Retrieving data.");
  String unixTime = getUnixTime();
  DEBUG_PRINT("Current time: " + String(unixTime));
  JsonObject payload_Data = payloadData.createNestedObject("data");

  DEBUG_PRINT("Retrieving Latitude.");
  JsonObject objectLatitude = payload_Data.createNestedObject("LAT1");
  objectLatitude["time"] = unixTime;
  objectLatitude["value"] = gps.location.lat();

  DEBUG_PRINT("Retrieving Longitude.");
  JsonObject objectLongitude = payload_Data.createNestedObject("LNG1");
  objectLongitude["time"] = unixTime;
  objectLongitude["value"] = gps.location.lng();

  DEBUG_PRINT("Retrieving Altitude.");
  JsonObject objectAltitude = payload_Data.createNestedObject("ALT1");
  objectAltitude["time"] = unixTime;
  objectAltitude["value"] = getAltitude();

  DEBUG_PRINT("Retrieving Rain Rate.");
  JsonObject objectRainRate = payload_Data.createNestedObject("RR1");
  objectRainRate["time"] = unixTime;
  objectRainRate["value"] = rainfallRate();

  DEBUG_PRINT("Retrieving Rain Amount.");
  JsonObject objectRainAmount = payload_Data.createNestedObject("RA1");
  objectRainAmount["time"] = unixTime;
  objectRainAmount["value"] = rainfallAmount();

  DEBUG_PRINT("Retrieving Temperature.");
  JsonObject objectTemp = payload_Data.createNestedObject("TMP1");
  objectTemp["time"] = unixTime;
  objectTemp["value"] = getTemperature();

  DEBUG_PRINT("Retrieving Pressure.");
  JsonObject objectPress = payload_Data.createNestedObject("PR1");
  objectPress["time"] = unixTime;
  objectPress["value"] = getPressure();

  DEBUG_PRINT("Retrieving Humidity.");
  JsonObject objectHumid = payload_Data.createNestedObject("HU1");
  objectHumid["time"] = unixTime;
  objectHumid["value"] = getHumidity();

  DEBUG_PRINT("Retrieving DewPoint.");
  JsonObject objectDew = payload_Data.createNestedObject("DP1");
  objectDew["time"] = unixTime;
  objectDew["value"] = getDewPoint();

  DEBUG_PRINT("Retrieving Heat Index.");
  JsonObject objectHeat = payload_Data.createNestedObject("HI1");
  objectHeat["time"] = unixTime;
  objectHeat["value"] = getHeadIndex();

  DEBUG_PRINT("Retrieving Battery Votlage.");
  JsonObject objectBatt = payload_Data.createNestedObject("BV1");
  objectBatt["time"] = unixTime;
  objectBatt["value"] = getBatteryVoltage();

  DEBUG_PRINT("Retrieving Flood Depth.");
  JsonObject objectFloodDepth = payload_Data.createNestedObject("FD1");
  objectFloodDepth["time"] = unixTime;
  objectFloodDepth["value"] = getDepth();

  serializeJson(payloadData, payloadBuffer);
  topic = "RAFT_Data";
  len = strlen(payloadBuffer.c_str()); // Calculates Payload Size

  if (_wifiConnected)
  {
    rainflowMQTT.publish(topic.c_str(), 2, false, payloadBuffer.c_str(), len, false, 0); // Publishes payload to server
    DEBUG_PRINT("Published @ " + String(topic) + "\n" + String(payloadBuffer));
    DEBUG_PRINT("payloadData Mem:" + String(payloadData.memoryUsage()));
  }
#ifdef GSM_ENABLED
  if (_gprsConnected)
  {
    bool published = mqtt.publish(topic.c_str(), payloadBuffer.c_str());
    DEBUG_PRINT("Published @ " + String(topic) + "\n" + String(payloadBuffer));
    DEBUG_PRINT("Published: " + String(published));
  }
  else if (!_gprsConnected && SMS_ENABLED)
  {
    publishSMS(topic.c_str(), payloadBuffer.c_str());
  }

#endif

  wait(1000);
}
void infoPublish(bool _wifiConnected, bool _gprsConnected)
{
  DynamicJsonDocument payloadData(1024);
  String payloadBuffer;
  String topic;
  int len;

  payloadData["data_type"] = "event";
  payloadData["stream_id"] = streamIDInfo;

  DEBUG_PRINT("Retrieving info.");
  String unixTime = getUnixTime();
  DEBUG_PRINT("Current time: " + String(unixTime));
  JsonObject payload_Data = payloadData.createNestedObject("data");
  JsonObject objectFirmware = payload_Data.createNestedObject("FirmwareVer");
  objectFirmware["time"] = unixTime;
  objectFirmware["value"] = FIRMWARE_VER;

  JsonObject objectBoot = payload_Data.createNestedObject("bootCount");
  objectBoot["time"] = unixTime;
  objectBoot["value"] = bootCount;

  JsonObject objectDate = payload_Data.createNestedObject("RGDate");
  objectDate["time"] = unixTime;
  objectDate["value"] = rainGaugeDate;

  JsonObject objectTip = payload_Data.createNestedObject("tipCount");
  objectTip["time"] = unixTime;
  objectTip["value"] = tipCount;

  JsonObject objectHeight = payload_Data.createNestedObject("Height");
  objectHeight["time"] = unixTime;
  objectHeight["value"] = medianGetHeight;

  JsonObject objectMode = payload_Data.createNestedObject("Mode");
  objectMode["time"] = unixTime;
  objectMode["value"] = currentMode;

  if (_wifiConnected)
  {
    JsonObject objectWifi = payload_Data.createNestedObject("wifiRSSI");
    objectWifi["time"] = unixTime;
    objectWifi["value"] = getRSSI();
  }

#ifdef GSM_ENABLED
  if (!__wifiConnected)
  {
    JsonObject objectWifi = payload_Data.createNestedObject("wifiRSSI");
    objectWifi["time"] = unixTime;
    objectWifi["value"] = signalQuality();
  }
#endif

  serializeJson(payloadData, payloadBuffer);
  topic = "RAFT_Info";
  len = strlen(payloadBuffer.c_str()); // Calculates Payload Size

  if (_wifiConnected)
  {
    rainflowMQTT.publish(topic.c_str(), 2, false, payloadBuffer.c_str(), len, false, 0); // Publishes payload to server
    DEBUG_PRINT("Published @ " + String(topic) + "\n" + String(payloadBuffer));
    DEBUG_PRINT("payloadData Mem:" + String(payloadData.memoryUsage()));
  }

#ifdef GSM_ENABLED
  if (_gprsConnected)
  {
    bool published = mqtt.publish(topic.c_str(), payloadBuffer.c_str());
    DEBUG_PRINT("Published @ " + String(topic) + "\n" + String(payloadBuffer));
    DEBUG_PRINT("Published: " + String(published));
  }
  else if (!_gprsConnected && SMS_ENABLED)
  {
    publishSMS(topic.c_str(), payloadBuffer.c_str());
  }

#endif

  wait(1000);
}
void publishData()
{
  indicatorLED1(true);

  if (WiFi.status() != WL_CONNECTED)
  {
    connectWifi(ssid, wifi_pass);
  }
  if (WiFi.status() == WL_CONNECTED)
  {
    wifiConnected = true;
  }

  if (wifiConnected)
  {
    rainflowMQTT.connect();
    wait(5000);
    if (rainflowMQTT.connected())
    {
      rainflowMQTT.subscribe("inbox", 2);
      dataPublish(rainflowMQTT.connected(), false);
      infoPublish(rainflowMQTT.connected(), false);
    }

    rainflowMQTT.disconnect();
  }

#ifdef GSM_ENABLED
  if (wifiConnected == false)
  {
    connectGSM(GSM_BAUD, GSM_TX, GSM_RX, apn, gprsUser, gprsPass);
    gprsConnected = gprsConnect();

    if (gprsConnected)
    {
      mqtt.setServer("rainflow.live", 1883);
      unsigned long startTime = millis();
      DEBUG_PRINT("Conneecting to MQTT server.");
      while (((!mqtt.connect(clientID, username, password)) && (millis() - startTime >= 30000)))
      { // connect will return 0 for connected
        DEBUG_PRINT("Retrying MQTT connection in 5 seconds...");
        mqtt.disconnect();
        wait(5000); // wait 5 seconds
      }
      if (mqtt.connected())
      {
        DEBUG_PRINT("Connected to MQTT server.");
        dataPublish(mqtt.connected());
        infoPublish(mqtt.connected());
        mqtt.disconnect();
        DEBUG_PRINT("Disconnected from MQTT server.");
      }
      else if (!mqtt.connected() && SMS_ENABLED)
      {
        dataPublish(mqtt.connected());
        infoPublish(mqtt.connected());
      }
    }
    else if (!wifiConnected && !gprsConnected && SMS_ENABLED)
    {
      dataPublish(wifiConnected, mqtt.connected());
      infoPublish(wifiConnected, mqtt.connected());
    }

    gprsDisconnect();
    DEBUG_PRINT("GPRS disconnected.");
  }
#endif

  disconnectWifi();
  indicatorLED1(false);
}
void indicatorLED(bool status)
{
  if (status == true)
  {
    if (INDICATOR_ENABLED)
    {
      pinMode(LEDPin, OUTPUT);
      digitalWrite(LEDPin, HIGH);
    }
  }
  if (status == false)
  {
    if (INDICATOR_ENABLED)
    {
      digitalWrite(LEDPin, LOW);
      pinMode(LEDPin, INPUT);
    }
  }
}
void indicatorLED1(bool status)
{
  if (status == true)
  {
    if (INDICATOR_ENABLED)
    {
      pinMode(LEDPin1, OUTPUT);
      digitalWrite(LEDPin1, HIGH);
    }
  }
  if (status == false)
  {
    if (INDICATOR_ENABLED)
    {
      digitalWrite(LEDPin1, LOW);
      pinMode(LEDPin1, INPUT);
    }
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

    if (GPIO_reason != 0)
    {
      int pin = __builtin_ffsll(GPIO_reason) - 1;
      Serial.printf("Wake up from GPIO %d\n", pin);
    }
    else
    {
      Serial.printf("Wake up from GPIO\n");
    }
    tipTime = 0;
  }

  if (getBatteryVoltage() <= BATTMINVOLT) // Sleep immediately to conserve power and avoid data corruption
    sleepNoInterrupt(1800);               // Deep Sleep without Interrupt for 30 minutes

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
#ifdef GSM_ENABLED
  connectGSM(GSM_BAUD, GSM_TX, GSM_RX, apn, gprsUser, gprsPass);
  modem.init();
#endif
  if (bootCount == 1)
  {
    wait(5000);
    getHeight();
    if (heightWrite == true)
      setHeight();
  }
  if ((wakeup_reason != ESP_SLEEP_WAKEUP_EXT1) && (wakeup_reason != ESP_SLEEP_WAKEUP_TIMER))
  {
    rainfallAmountReset();
    GPS_powerSaveMode();
  }

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