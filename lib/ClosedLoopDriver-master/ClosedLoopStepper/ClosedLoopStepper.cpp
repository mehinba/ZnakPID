//
//  ClosedLoopStepper.cpp
//  ClosedLoopStepper
//
//  Created by Kris Temmerman on 30/01/15.
//
//

#include "ClosedLoopStepper.h"

void ClosedLoopStepper::setup()
{
   // Serial.begin(115200);
    prevTime = 0;
    encoderPositionOld = 0;
    encoderTime = 0;

    stepper.setup(4, 3, 11, 3200);
    myEnc.setup(8, 9);
    stepper.setTragetSpeed(0);
    targetRotation = 0;

    Kp = 20;
    accel = 50;
    maxSpeed = 300;
    pinMode(absPosEncoder, INPUT_PULLUP);

    calibrateEncoder();
}
void ClosedLoopStepper::loop()
{
    //

    unsigned long currentTime = micros();
    unsigned long timeElapsed = currentTime - prevTime;
    prevTime = currentTime;

    //
    //updateSerial();
    //
    if (targetRotation != prevTargetRotation)
    {
        startAccelerating = true;
        accelSpeed = 200;
    }
    prevTargetRotation = targetRotation;

    updateEncoder(timeElapsed);
    encoderOperator();

    stepper.loop(timeElapsed);
}
void ClosedLoopStepper::updateEncoder(unsigned long timeElapsed)
{
    encoderTime += timeElapsed;
    if (encoderTime > 10000)
    {
        rotationPosition = encoderPosition;

        float posChange = encoderPosition - encoderPositionOld;
        float rotChange = posChange * (360 / 42);               //change in deg
        rotationSpeed = rotChange / ((float)encoderTime / 100); // rotspeed deg/sec

        float targetSpeed = (targetRotation - rotationPosition) * Kp;

        if(abs(targetRotation - rotationPosition)<2){
            stepper.setReached = true;
        }

        if (targetSpeed > maxSpeed)
        {

            targetSpeed = maxSpeed;
        }
        if (targetSpeed < -maxSpeed)
        {
            targetSpeed = -maxSpeed;
        }
        int tempTargetSpeed = targetSpeed;
        if (startAccelerating)
        {
            if (targetSpeed > accelSpeed)
            {
                tempTargetSpeed = accelSpeed;
                accelSpeed += 1;
                //Serial.println("AccelSpeedIncreased");
            }
            else if (targetSpeed < -accelSpeed)
            {
                if(accelSpeed>0){
                    accelSpeed = accelSpeed*(-1);
                }
                tempTargetSpeed = accelSpeed;
                accelSpeed -= 1;
                //Serial.println("AccelSpeedDecreased");
            }
            else
            {
                startAccelerating = false;
                accelSpeed = 0;
            }
        }
        if (abs(tempTargetSpeed) < 80)
        {
            startAccelerating = true;
        }

        if(tempTargetSpeed<0){
            if(tempTargetSpeed>-80){
                tempTargetSpeed = -80;
            }
        }else{
            if(tempTargetSpeed<80){
                tempTargetSpeed = 80;
            }
        }
        stepper.setTragetSpeed(tempTargetSpeed);

        encoderPositionOld = encoderPosition;
        encoderTime = -timeElapsed;
    }
}

void ClosedLoopStepper ::encoderOperator()
{
    bool absTemp = digitalRead(absPosEncoder);
    if (lastAbs != absTemp)
    {

        if (absTemp == 0 && stepper.rotatingLeft)
        {
            calibrated = true;
            myEnc.write(0);
        }
    }
    lastAbs = absTemp;

    long newPosition = myEnc.read();
    if (newPosition != oldPosition)
    {
        oldPosition = newPosition;
    }

    encoderPosition = newPosition % encoderIncrements;
}

bool ClosedLoopStepper ::calibrateEncoder()
{
    unsigned long calStart = millis();

    while (!calibrated && calStart<60000)
    {
        unsigned long currentTime = micros();
        unsigned long timeElapsed = currentTime - prevTime;
        prevTime = currentTime;
        stepper.setTragetSpeed(-150); /* code */
        encoderOperator();
        stepper.loop(timeElapsed);
    }
    return calibrated;
}