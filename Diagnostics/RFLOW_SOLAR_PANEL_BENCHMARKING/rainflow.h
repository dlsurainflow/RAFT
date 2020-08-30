#ifndef rainflow_h
#define rainflow_h

#include <HardwareSerial.h>
#include <ArduinoJson.h>
#include "PubSubClient.h"

#ifdef DEBUG_MODE
#define DEBUG_PRINT(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#endif


class RainFLOW {
  private:
    //Client* client;

    PubSubClient rainflowMQTT;    // Instance Creation of MQTT Client
    DynamicJsonDocument payloadData;

  public:

    RainFLOW() : payloadData(1024) {}
    void connectServer(const char* APIKey);
    bool connectMqtt(const char* APIKey);
    void addData(String topic, String payload);
    void publishData(const char* APIKey);
    void rainflowCallback(char* topic, byte* payload, unsigned int len);
    void rainflow(Client& client);
};
void rainflowCallback(char* topic, byte * payload, unsigned int len);


#endif
