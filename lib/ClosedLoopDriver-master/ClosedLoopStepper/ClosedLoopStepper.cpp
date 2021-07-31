//
//  ClosedLoopStepper.cpp
//  ClosedLoopStepper
//
//  Created by Kris Temmerman on 30/01/15.
//
//

#include "ClosedLoopStepper.h"
ClosedLoopStepper::ClosedLoopStepper(int accelSpeed_, float kp_, float ki_){
        Kp = kp_;
    Ki =ki_;
    accel = accelSpeed_;
    maxSpeed = 150;
}
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


    pinMode(absPosEncoder, INPUT_PULLUP);

    calibrateEncoder();
}
void ClosedLoopStepper::loop()
{
    unsigned long currentTime = micros();
    unsigned long timeElapsed = currentTime - prevTime;
    prevTime = currentTime;

    encoderOperator();
    updateEncoder(timeElapsed);

    stepper.loop(timeElapsed);
}
void ClosedLoopStepper::updateEncoder(unsigned long timeElapsed)
{
    encoderTime += timeElapsed;
    if (encoderTime > 10000)
    {
        float rotationPosition = encoderPosition;

        float posChange = encoderPosition - encoderPositionOld;
        float rotChange = posChange * (360 / 42);               //change in deg
        rotationSpeed = rotChange / ((float)encoderTime / 100); // rotspeed deg/sec

        unsigned long ellpasedFromStart = millis() - startTime;
        if(ellpasedFromStart>60000){
            stepper.disable();
            encoderPositionOld = encoderPosition;
            encoderTime = -timeElapsed;
            return;
        }
        float integralSpeed;
        int direction = abs( targetRotation - rotationPosition ) > (encoderIncrements/2) ? 1 : -1;
        float targetSpeed = abs(targetRotation - rotationPosition) * Kp * direction;

        if(stepper.setReached == false){
            integralSpeed = (float)ellpasedFromStart * Ki;
            if(integralSpeed > integralLimit){
                integralSpeed = integralLimit;
            }
            if(targetSpeed < 0){
                integralSpeed *= -1;
            }
        }
        targetSpeed += integralSpeed;

        if(abs(targetSpeed) < abs(prevTargetSpeed)){
            startAccelerating = false;
        }
        prevTargetSpeed = targetSpeed;


        float tempTargetSpeed = 0;
        if(startAccelerating){
           accelSpeed++;
           if(accelSpeed>maxSpeed){
               accelSpeed = maxSpeed;
               startAccelerating = false;
           } 
           if(targetSpeed>0){
               tempTargetSpeed = accelSpeed;
           }else{
               tempTargetSpeed = -accelSpeed;
           }
        }else{
            tempTargetSpeed = targetSpeed;
        }

        

        if(abs(targetRotation - rotationPosition)<2 || abs(targetRotation - rotationPosition)>(encoderIncrements-2)){
            if(stepper.setReached == false){
                stepper.setPointReachedTime = millis();
            }
            stepper.setReached = true;
            tempTargetSpeed = 0;

        }

        if(tempTargetSpeed<0){
            if(tempTargetSpeed > -maxSpeed){
                tempTargetSpeed = -maxSpeed;
            }
        }else if(tempTargetSpeed<maxSpeed && tempTargetSpeed != 0){
            tempTargetSpeed = maxSpeed;
        }

        stepper.setTragetSpeed(tempTargetSpeed);
        encoderPositionOld = encoderPosition;
        encoderTime = -timeElapsed;
    }
}

void ClosedLoopStepper ::encoderOperator()
{
    bool absTemp = digitalRead(absPosEncoder);
    if ((lastAbs != absTemp) && (!absTemp))
    {
        calibrated = true;
        myEnc.write(0);
        encoderPositionOld = 0;
        
        Serial.println("Nulirano");
    }
    lastAbs = absTemp;

    encoderPosition = myEnc.read() % encoderIncrements;
    if(encoderPosition<0){
        encoderPosition += encoderIncrements;
    }
}

bool ClosedLoopStepper ::calibrateEncoder()
{
    unsigned long calStart = millis();

    while (!calibrated && (millis()-calStart)<60000)
    {
        unsigned long currentTime = micros();
        unsigned long timeElapsed = currentTime - prevTime;
        prevTime = currentTime;
        stepper.setTragetSpeed(-150); 
        encoderOperator();
        stepper.loop(timeElapsed);
    }
    return calibrated;
}

void ClosedLoopStepper:: resetData(){
      startAccelerating = true;
      accelSpeed = accel;
      stepper.setReached = false;
      prevTargetSpeed = 0;
      startTime = millis();
}