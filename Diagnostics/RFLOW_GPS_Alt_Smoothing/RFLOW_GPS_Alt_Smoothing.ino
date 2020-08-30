#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <Smoothed.h>   // Include the library
/*
   This sample sketch demonstrates the normal use of a TinyGPS++ (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/
static const int RXPin = 26, TXPin = 27;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
HardwareSerial ss(1);
Smoothed <float> alt1;
Smoothed <float> alt2;  

void setup()
{
  Serial.begin(115200);
  ss.begin(9600, SERIAL_8N1, TXPin, RXPin);
  alt1.begin(SMOOTHED_AVERAGE, 1000);
  alt2.begin(SMOOTHED_EXPONENTIAL, 1);
  Serial.println("Current, Avg, Exp");
  Serial.println("---------------");
  
}

void loop()
{
  // This sketch displays information every time a new sentence is correctly encoded.
  while (ss.available() > 0)
    if (gps.encode(ss.read()))
      displayInfo();

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }
}

void displayInfo()
{
  if(gps.altitude.isValid()){
    float currentAlt = gps.altitude.meters();
    alt1.add(currentAlt);
    alt2.add(currentAlt);
    float smoothedAltAvg = alt1.get();
    float smoothedAltExp = alt2.get();
    //Serial.println(String(currentAlt) + "\t" + String(smoothedAltAvg) + "\t" + String(smoothedAltExp));
    Serial.println(String(currentAlt) + ", " + String(smoothedAltAvg) + ", " + String(smoothedAltExp));
    //Serial.println(currentAlt);
  }
}
