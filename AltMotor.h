/*
 * AltMotor.cpp
 *
 *  Created on: Feb 3, 2017
 *      Author: Chester Marshall, mentor for 6055 and 5721
 *
 *  This library can operate a Bosch seat motor
 *
 */

#ifndef AltMotor_h
#define AltMotor_h

#include "WPILib.h"
#include "stdlib.h"


#define COUNTS_PER_ROTATION 179

class AltMotor
{
private:
	VictorSP* bMotor;
	AnalogTrigger* mAnalogTrigger; // create an encoder pulse trigger
	Counter* mCounter; // count the encoder pulse triggers in current direction
	int tickCount;
	bool countFlag;
	
public:
    AltMotor(int mChannel, int aChannel); //passes PWM channel for motor and AIO channel for encoder feedback
    ~AltMotor();  //destructor
    double Speed;
    int TickTarget;
    void Execute();
    void Reset();
    void SetInverted(bool invert);
    void SetSafetyEnabled(bool enabled);
};

#endif
