#include "rainflow.h"
#include "PubSubClient.h"
#include <ArduinoJson.h>
#include <HardwareSerial.h>


#ifdef DEBUG_MODE
#define DEBUG_PRINT(x) Serial.println(x)
#else
#define DEBUG_PRINT(x) Serial.println(x)
#endif

PubSubClient rainflowMQTT;



void RainFLOW::connectServer(const char* APIKey) {
  rainflowMQTT.setServer("192.168.1.9", 1883);
//  rainflowMQTT.setCallback(rainflowCallback);
  DEBUG_PRINT("Connecting to RainFLOW Server.");
  int i = 0;
  while (connectMqtt(APIKey) == false){
    if(i == 10){
      DEBUG_PRINT("Failed to connect to RainFLOW Server.");
      break;
    }
    
    i++;
    //continue;
  }
}

bool RainFLOW::connectMqtt(const char* APIKey) {
  if (!rainflowMQTT.connect(APIKey)) {
    DEBUG_PRINT(".");
    return false;
  }
  DEBUG_PRINT("Connected to server!");
  char topic[45];
  strcpy(topic, "rainflow/device/");
  strcat(topic, APIKey);
  rainflowMQTT.subscribe(topic);                  // Subscribe to device management channel
  DEBUG_PRINT("Subscribed to: " + String(topic));
  return rainflowMQTT.connected();
}


void rainflowCallback(char* topic, byte* payload, unsigned int len) {
  DEBUG_PRINT("Received message: ");
  Serial.write(payload, len);
}

void RainFLOW::addData(String topic, String payload) {
  payloadData[topic] = payload;                                             // Add payload to JSON variable
}


void RainFLOW::publishData(const char* APIKey) {
  int i = 0;
  while (connectMqtt(APIKey) == false) {
    if(i == 10){
      DEBUG_PRINT("Failed to connect to RainFLOW Server.");
      break;
    }
  }
  char topicBuffer[1024];
  String payloadBuffer;
  String topic;
  int len;

  serializeJson(payloadData, payloadBuffer);
  topic =   "rainflow/data/";
  topic  +=  APIKey;                                                          // Append API Key to data
  len       = strlen(payloadBuffer.c_str());                                  // Calculates Payload Size

  rainflowMQTT.publish(topic.c_str(), payloadBuffer.c_str(), len);            // Publishes payload to server
  DEBUG_PRINT("Published @ " + String(topic) + "\n" + String(payloadBuffer));
  payloadData.clear();
  rainflowMQTT.disconnect();
}


void RainFLOW::rainflow(Client& client) {
  rainflowMQTT.setClient(client);
}
