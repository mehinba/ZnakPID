#include <Arduino.h>
#include "ClosedLoopStepper/ClosedLoopStepper.h"

int accelSpeedParam = 100;
float kPparam = 10;
float kIparam = 0.01;


int determineCurrentAngularPosition(int);
int determineNumOfStepps(int, int);
void encoderOperator();
void calibrateSensor();
void parseData();

ClosedLoopStepper cls(accelSpeedParam, kPparam, kIparam);
int data = 0;

void setup()
{
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  bool ledState = false;
  for (int i = 0; i < 20; i++)
  {
    digitalWrite(13, ledState);
    delay(100);
    ledState = !ledState;
  }

  cls.setup();
}

uint32_t lastSerialReceived = 0;

void loop()
{

  parseData();
  cls.loop();
}
void parseData()
{
  if (Serial.available() > 0)
  {

    int dataTemp = Serial.parseInt();
    Serial.read();
    Serial.flush();
    //Serial.println(data);
    uint32_t elapsedFromLastValidData = millis() - lastSerialReceived;

    if (dataTemp >= 0 && dataTemp <= 360 && elapsedFromLastValidData > 10000)
    {
      //validSetpointReceived
      lastSerialReceived = millis();
      cls.resetData();
      cls.targetRotation = map(dataTemp, 0, 360, 0, 42);
    }
  }
}
