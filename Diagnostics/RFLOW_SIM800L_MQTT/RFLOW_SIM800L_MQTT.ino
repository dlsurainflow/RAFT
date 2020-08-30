
#define TINY_GSM_MODEM_SIM800

#include <HardwareSerial.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>

HardwareSerial SerialAT(1); // RX, TX

//Network details
const char apn[]  = "smartlte";
const char user[] = "";
const char pass[] = "";

// MQTT details
const char* broker = "test.mosquitto.org";
const char* topicOut = "rainflow_test";
const char* topicIn = "rainflow_test";

TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient mqtt(client);

void setup()
{
  Serial.begin(115200);
  SerialAT.begin(9600, SERIAL_8N1, 16, 17);

  Serial.println("System start.");
  modem.restart();
  Serial.println("Modem: " + modem.getModemInfo());
  Serial.println("Searching for telco provider.");
  if (!modem.waitForNetwork())
  {
    Serial.println("fail");
    while (true);
  }
  Serial.println("Connected to telco.");
  Serial.println("Signal Quality: " + String(modem.getSignalQuality()));

  Serial.println("Connecting to GPRS network.");
  if (!modem.gprsConnect(apn, user, pass))
  {
    Serial.println("fail");
    while (true);
  }
  Serial.println("Connected to GPRS: " + String(apn));

//  if (modem.isGprsConnected()){
//    Serial.println("GPRS Status: Connected");
//  }
//  else {
//    Serial.println("GPRS Status: Disconnected");
//  }
  Serial.println("IMEI:           " + String(modem.getIMEI()));
  Serial.println("Signal Quality: " + String(modem.getSignalQuality()));
  Serial.println("Operator:       " + String(modem.getOperator()));
  

  
  mqtt.setServer(broker, 1883);
  mqtt.setCallback(mqttCallback);
  Serial.println("Connecting to MQTT Broker: " + String(broker));
  while (mqttConnect() == false) continue;
  Serial.println();


}

void loop()
{
  if (Serial.available())
  {
    delay(10);
    String message = "lol what";
    while (Serial.available()) message += (char)Serial.read();
    mqtt.publish(topicOut, message.c_str());
  }

  if (mqtt.connected())
  {
    mqtt.loop();
  }
}

boolean mqttConnect()
{
  if (!mqtt.connect("GsmClientTest"))
  {
    Serial.print(".");
    return false;
  }
  Serial.println("Connected to broker.");
  mqtt.subscribe(topicIn);
  return mqtt.connected();
}

void mqttCallback(char* topic, byte* payload, unsigned int len)
{
  Serial.print("Message receive: ");
  Serial.write(payload, len);
  Serial.println();
}
