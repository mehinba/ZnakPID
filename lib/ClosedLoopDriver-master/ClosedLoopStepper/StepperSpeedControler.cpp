//
//  StepperSpeedControler.cpp
//  ClosedLoopStepper
//
//  Created by Kris Temmerman on 31/01/15.
//
//

#include "StepperSpeedControler.h"

/*int stepPauze;
int pullPin;
int dirPin;
int stepsRot;*/
 
 
void StepperSpeedControler::setup(int _pullPin, int _dirPin, int _enablePin, int _stepsRot)
{

    pullPin =_pullPin;
    dirPin = _dirPin;
    enablePin = _enablePin;
    
    pinMode(pullPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enablePin, OUTPUT);

    stepDeg = (float)_stepsRot/360.f;
    currentTime =0;
    stepState =false;

}
void StepperSpeedControler::setTragetSpeed(float speed)
{
    //speed =deg/sec
    if(speed==0)
    {
        /*
        if(setReached  == false){
            setReached = true;
        }*/
        unsigned long elapsedFromReach = millis() - setPointReachedTime;
        if(elapsedFromReach> 60000 && setReached == true){
            disable();
        }
        stepPauze=-1;
        return;
    }
    if(speed<0)
    {
        //setReached = false;
        if(setReached == false){
        enable();
        }else/*{
            disable();
        }*/
        speed *=-1;
        digitalWrite(dirPin, HIGH);
        rotatingLeft = true;
        
    }else
    {
        //setReached = false;
        if(setReached == false){
        enable();
        }else/*{
            disable();
        }*/
        rotatingLeft = false;
        digitalWrite(dirPin, LOW);
    
    }
      
    float stepsSec =stepDeg *speed;
    stepPauze =500000/ stepsSec  ; //2 "pauzes" for 1 step
    
  
 
  
    

}
void StepperSpeedControler::loop( unsigned long timeElapsed)
{
    if(stepPauze==-1)return;
    
    currentTime +=timeElapsed;
    if(currentTime>=stepPauze)
    {
        digitalWrite(pullPin, stepState);
        stepState =!stepState;
        currentTime-=stepPauze;
        if(currentTime>stepPauze)currentTime=0;
    }


}

void StepperSpeedControler::enable(){
    digitalWrite(enablePin, LOW);
}


void StepperSpeedControler::disable(){
    digitalWrite(enablePin, HIGH);
}
 