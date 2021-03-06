/*******************************************************************
* Gripper library that allows to enagage and disengage the grippers
* of the robot. This code is designed for the Vex 29 motor controllers.
*
* April 2020 - Josue Contreras and Trevor Rizzo, Swarm Construction MQP
******************************************************************/

#include "Gripper.h"

////////////////////////////////////////////////////////////////
// CONSTRUCTOR
////////////////////////////////////////////////////////////////

/**
 * Empty contructor to build object
 **/
Gripper::Gripper(){
}

/**
 * Constructor to added object properties. Two arguments are optional, zeroposition:
 * sets the middle value for the range of values that are mapped. Threshold is not used
 * but could serve to implement the write to motor function.
 **/
Gripper::Gripper(int gripperPin, bool rotationDirection, bool isEngaged, int buttonPin, bool gripper){

    gripperPin = gripperPin;
    buttonPin = buttonPin;
    pinMode(buttonPin, INPUT_PULLUP);
    turnsItterator = 0;
    rotationDirection = rotationDirection;
    threshold = threshold;
    medianPulse = 1500; //motor stops at this pulse width
    isEngaged = false;
    if(gripper){
      turns=4;
    }
    else{
      turns=144;
    }

    

    if(rotationDirection){
      maxSpeedCCW = -255;
      maxSpeedCW = 255;
      maxPulse = 2000; //unique value to VEX 29 motorcontrollers
      minPulse = 1000; //unique value to VEX 29 motorcontrollers
    }else if(!rotationDirection){
      maxSpeedCCW = 255;
      maxSpeedCW = -255;
      maxPulse = 1000; //unique value to VEX 29 motorcontrollers
      minPulse = 2000; //unique value to VEX 29 motorcontrollers
    }
    grip.attach(gripperPin);
    
}

// void Gripper::begin(void){
//    pinMode(buttonPin, INPUT_PULLUP);
//   attachInterrupt(digitalPinToInterrupt(buttonPin), Gripper::intService, RISING); 
// }

// static void Gripper::intService(void){
//   incrementIterator();
// }

////////////////////////////////////////////////////////////////
// FUNCTIONS
////////////////////////////////////////////////////////////////

/**
 * Gripper screw turns functions
 **/
void Gripper::incrementIterator(void){
  turnsItterator++;
  last_update = millis();
}

long Gripper::getTurns(void){
  return turnsItterator;
}

void Gripper::resetItterator(void){
  turnsItterator = 0;
}

void Gripper::screwTurns(void){
  if(analogRead(buttonPin)){
    incrementIterator();
  }
}

/**
 * Motor spins CW for pulse widths less than 1500 uS and CCW
 * for pulse widths greater than 1500. map() scales the power to a
 * pulse width.
 **/
void Gripper::write(int power){

    pulseWidth = map(power, maxSpeedCCW, maxSpeedCW, maxPulse, minPulse);
    grip.writeMicroseconds(pulseWidth);
    
}

/**
 * For gripper engagment and disengament
 * TODO: when button added to gripper, add ticks instead of the time delay.
 **/

bool Gripper::setGripper(int gState){

  if(resetTime){
    resetItterator();
    resetTime = false;
    last_update = millis();
  }

  switch(gState){
    case 0: //nothing happens
        write(0);
        gripperFinished = true;
        break;
    case 1: //engage gripper
      if(abs(millis() - last_update) < gripper_wait_timeout){
        write(maxSpeedCCW);
        gripperFinished = false;
        isE = false;
        Serial.print("Turns:   ");
        Serial.println(getTurns());
      }else{
        write(0);
        resetTime = true;
        gripperFinished = true;
        setEngaged(true);
        gState = 0;
      }
        break;
    case 2: //disengage gripper
      if(getTurns() < turns+2){
        write(maxSpeedCW);
        gripperFinished = false;
        isE = true;
        Serial.print("Turns:   ");
        Serial.println(getTurns());
      }else{
        write(0);
        resetTime = true;
        gripperFinished = true;
        setEngaged(false);
      }
        break;
    default: //if none are selected
        write(0);
        gripperFinished = true;
        break;
  }

  return gripperFinished;

}


//// Funciton that uses time to screw
bool Gripper::setGripperwTime(int gState){

  if(resetTime){
    startTime = millis();
    resetTime = false;
  }

  switch(gState){
    case 0: //nothing happens
        write(0);
        gripperFinished = true;
        break;
    case 1: //engage gripper
      if((int)(millis() - startTime) < time){
        write(maxSpeedCCW);
        gripperFinished = false;
        isE = false;
      }else{
        write(0);
        resetTime = true;
        gripperFinished = true;
        setEngaged(true);
      }
        break;
    case 2: //disengage gripper
      if((int)(millis() - startTime) < time){
        write(maxSpeedCW);
        gripperFinished = false;
        isE = true;
      }else{
        write(0);
        resetTime = true;
        gripperFinished = true;
        setEngaged(false);
      }
        break;
    default: //if none are selected
        write(0);
        gripperFinished = true;
        break;
  }

  return gripperFinished;
}