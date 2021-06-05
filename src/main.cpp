#include <Arduino.h>
#include "ClosedLoopStepper/ClosedLoopStepper.h"
//#include "string.h"
//int Setpoint;
//int PrevSetpoint = 0;

int determineCurrentAngularPosition(int);
int determineNumOfStepps(int, int);
void encoderOperator();
void calibrateSensor();
void parseData();

ClosedLoopStepper cls;
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
uint32_t lastSet = 0;
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
      cls.stepper.setPointReachedTime = millis();
      cls.startAccelerating = true;
      cls.accelSpeed = 10;
      lastSet = millis();
      lastSerialReceived = millis();
      cls.stepper.setReached = false;
      cls.targetRotation = map(dataTemp, 0, 360, 0, 40);
    }
  }
}
