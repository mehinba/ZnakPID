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
    ClosedLoopStepper(){};
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
    float integralLimit = 100;
    float prevTargetSpeed = 0;


    bool startAccelerating = true;
    int targetRotation;
    int prevTargetRotation;
    int encoderPosition;
    int encoderPositionOld;
    float targetSpeed = 0;
    unsigned long prevTime;
    unsigned long encoderTime;

    float rotationSpeed;
    float rotationPosition;

    StepperSpeedControler stepper;
    int pinA = 8;
    int pinB = 9;
    Encoder myEnc;
    bool calibrated = false;
    
    
    uint8_t absPosEncoder = 7;
    uint8_t encoderIncrements = 42;
    bool lastAbs = false;

    long oldPosition = -999;
    long newPosition = 0;
};

#endif
