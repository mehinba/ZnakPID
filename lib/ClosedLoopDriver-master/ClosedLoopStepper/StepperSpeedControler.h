//
//  StepperSpeedControler.h
//  ClosedLoopStepper
//
//  Created by Kris Temmerman on 31/01/15.
//
//

#ifndef ClosedLoopStepper_StepperSpeedControler_h
#define ClosedLoopStepper_StepperSpeedControler_h
#include <Arduino.h>
class StepperSpeedControler
{
    
    
public:
    StepperSpeedControler(){};
    void setup(int _pullPin, int _dirPin,int _enablePin, int _stepsRot);
    void enable();
    void disable();
    void setTragetSpeed(float speed);
    void loop( unsigned long timeEllapsed);
    
    unsigned long currentTime;
    unsigned long setPointReachedTime = 0;
    bool setReached = false;
    int stepPauze;
    int pullPin;
    int dirPin;
    int enablePin;
    int stepsRot;
    float stepDeg;
    bool stepState ;
    bool rotatingLeft = false;
};

#endif
