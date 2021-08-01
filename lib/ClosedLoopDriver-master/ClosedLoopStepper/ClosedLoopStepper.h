//
//  ClosedLoopStepper.h
//  ClosedLoopStepper
//
//  Created by Kris Temmerman on 30/01/15.
//
//

#ifndef ClosedLoopStepper_ClosedLoopStepper_h
#define ClosedLoopStepper_ClosedLoopStepper_h

#include <Arduino.h>
#include "StepperSpeedControler.h"
#include "Encoder.h"
//Encoder myEnc(8,9);

#define SERIAL_PORT_SPEED 115200

//#define  DEBUG_INPUT 1


class ClosedLoopStepper
{

public:
    ClosedLoopStepper(int accel_, float kp_, float ki_);
    void setup();
    void loop();

    //void updateSerial();
    void updateEncoder(unsigned long timeElapsed);
    void encoderOperator();
    bool calibrateEncoder();
    void resetData();
    

    float Kp;
    float Ki;
    float accel;
    int maxSpeed;
    int accelSpeed = 50;
    unsigned long startTime = 0;
    float integralLimit = 200;
    float prevTargetSpeed = 0;


    bool startAccelerating = true;
    bool allowPosChange = true;
    int direction = 1;
    int targetRotation;
    int prevTargetRotation;
    int encoderPosition;
    int encoderPositionOld;
    float targetSpeed = 0;
    unsigned long prevTime;
    unsigned long encoderTime;

    float rotationSpeed;

    StepperSpeedControler stepper;
    int pinA = 8;
    int pinB = 9;
    Encoder myEnc;
    bool calibrated = false;
    
    
    uint8_t absPosEncoder = 7;
    uint8_t encoderIncrements = 42;
    bool lastAbs = false;

    long newPosition = 0;
};

#endif
