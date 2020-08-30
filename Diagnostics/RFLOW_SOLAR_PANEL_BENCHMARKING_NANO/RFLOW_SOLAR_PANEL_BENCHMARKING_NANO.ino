#define SP1C_PIN A1
#define SP1V_PIN A0
#define SPRC_PIN A2
#define BV_PIN   A5
#define sampleRate 25
#define offsetVoltage 2400
#define sensitivity 185

#include <ArduinoJson.h>
#include <Battery.h>

#include <TaskScheduler.h>
#include <Smoothed.h>

void publishData();

Task publishDataScheduler(60000, TASK_FOREVER, &publishData);
Scheduler runner;

Smoothed<double> SP1C;  // Solar Panel 1 Current
Smoothed<double> SP1V;  // Solar panel 1 Voltage
Smoothed<double> SPRC;  // Solar Panel Reference Current
Smoothed<double> BV;    // Battery Voltage

DynamicJsonDocument data(256);
Battery battery(3400, 4200, BV_PIN);

void setup() {
  Serial.begin(115200);
//  Serial.println("System initialising");
  runner.init();
  runner.addTask(publishDataScheduler);
  publishDataScheduler.enable();

  SP1C.begin(SMOOTHED_EXPONENTIAL, 10);  // Solar Panel 1 Current
  SP1V.begin(SMOOTHED_EXPONENTIAL, 10);  // Solar panel 1 Voltage
  SPRC.begin(SMOOTHED_EXPONENTIAL, 10);  // Solar Panel Reference Current
  BV.begin(SMOOTHED_EXPONENTIAL, 10);    // Battery Voltage

  battery.begin(5000, 1.0, &sigmoidal);
}

void getData() {
  //  double offsetVoltage = 2400, sensitivity = 185;

  double SP1Current = 0.0, SP1Voltage = 0.0, SPRCurrent = 0.0, BVoltage = 0.0;
  double SP1CVoltage = 0.0, SP1CValue = 0.0, SP1CSamples = 0.0, SP1CRaw = 0.0;
  double SP1VoltValue1 = 0.0, SP1VValue = 0.0, SP1VSamples = 0.0, SP1VRaw = 0.0;
  double SPRCVoltage = 0.0, SPRCValue = 0.0, SPRCSamples = 0.0, SPRCRaw = 0.0;
  double BValue1 = 0.0, BVValue = 0.0, BVSamples = 0.0, BVRaw = 0.0;

  // Solar Panel 1
  for (int x = 0; x < sampleRate; x++) {
    float currentSensorValue1 = analogRead(SP1C_PIN);
    SP1C.add(currentSensorValue1);
    SP1CValue = SP1C.get();
    SP1CSamples = SP1CSamples + SP1CValue;

    float SP1VoltValue1 = analogRead(SP1V_PIN);
    SP1V.add(SP1VoltValue1);
    SP1VValue = SP1V.get();
    SP1VSamples = SP1VSamples + SP1VValue;

    // Read normal Arduino Value
    int in1 = analogRead(SPRC_PIN);
    float val1 = in1 * 5.0 / 1024.0;
    // read correct supply voltage
    float supply = readVcc() / 1000.0;
    float val1Corrected = supply / 5 * val1;

    float currentSensorValue2 = analogRead(SPRC_PIN);
    SPRC.add(val1Corrected);
    SPRCValue = SPRC.get();
    SPRCSamples = SPRCSamples + SPRCValue;

    //float SPRCVoltage = analogRead(BV_PIN);
    //    float SPRCVoltage = battery.voltage();
    //    BV.add(SPRCVoltage);
    //    BVValue = BV.get();
    //    BVSamples = BVSamples + BVValue;

    delay (5);
  }

  SP1CRaw = (SP1CSamples / sampleRate);
  SP1CVoltage = ((SP1CRaw / 1024.0)) * 5.0;
  SP1Current = SP1CVoltage;

  SP1VRaw = (SP1VSamples / sampleRate);
  SP1Voltage = ((SP1VRaw * 5.0 / 1024.0)) * 5.0;
  SP1Voltage = (SP1Voltage);
  double SP1Power = (SP1Voltage) * (SP1Current);

  SPRCRaw = (SPRCSamples / sampleRate);
  //SPRCVoltage = ((SPRCRaw / 1024.0)) * 5.0;
  SPRCVoltage = SPRCRaw;
  float supplyVoltage = readVcc() / 1000.000;
  float supplyVoltageHalfed = supplyVoltage / 2.000;
  SPRCurrent = ((SPRCVoltage - supplyVoltageHalfed) / 0.185);

  //
  //  BVRaw = (BVSamples / sampleRate);
  //  //  BVoltage = ((BVRaw / 1024.0)) * 5.0;
  //  BVoltage = BVRaw / 1000;

  // read normal Arduino value
  int in0 = analogRead(BV_PIN);
  float val0 = in0 * 5.0 / 1024.0;
  // read correct supply voltage
  float supply = readVcc() / 1000.0;
  float val0Corrected = supply / 5 * val0;


  float battLevel = (val0Corrected - 3.3) / (4.2 - 3.3) * 100;

  //  data["SP1 Raw"] =          SP1CRaw;
  //  data["SP1 ACS Voltage"] =  SP1CVoltage;
  data["SP1 Current"] =      SP1Current;
  //  data["SP1 VRaw"] =         SP1VRaw;
  data["SP1 Voltage"] =      SP1Voltage;
  data["SP1 Power"] =        SP1Power;
  //  data["SPR Raw"] =          SPRCRaw;
  //  data["SPR ACS Voltage"] =  SPRCVoltage;
  data["SPR Current"] =      SPRCurrent;
  data["BattLevelNano"] =    battLevel;
  data["BattVoltageNano"] =  val0Corrected;

  SP1C.clear();
  SP1V.clear();
  SPRC.clear();
  BV.clear();
}

void publishData() {
  getData();
  serializeJson(data, Serial);
  Serial.println();
  //serializeJsonPretty(data, Serial);

  analogWrite(10, 200);
  delay(25);
  analogWrite(10, 0);
}

long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA, ADSC));
  result = ADCL;
  result |= ADCH << 8;
  result = 1126400L / result; // Calculate Vcc (in mV); 1126400 = 1.1*1024*1000
  return result;
}

void loop() {
  runner.execute();
}
