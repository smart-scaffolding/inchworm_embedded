/*******************************************************************
* Storage library that allows to move the storage four bar from rest
* to load position of the robot. This code is designed for the JX servos.
*
* April 2020 - Josue Contreras and Trevor Rizzo, Swarm Construction MQP
******************************************************************/

#include "Storage.h"

////////////////////////////////////////////////////////////////
// CONSTRUCTOR
////////////////////////////////////////////////////////////////

/**
 * Default contructor to build object
 **/
Storage::Storage(){
}

Storage::Storage(int pin){

    pin = pin;
    storageMotor.attach(pin);
}

////////////////////////////////////////////////////////////////
// FUNCTIONS
////////////////////////////////////////////////////////////////
    
void Storage::restPosition(){
    storageMotor.write(160);
}

void Storage::loadPosition(){
    storageMotor.write(35);
}