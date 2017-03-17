/*
 * AltMotor.cpp
 *
 *  Created on: Feb 3, 2017  - base on code from FRC Team 4237 Lakeshore High School
 *      Author: Chester Marshall, mentor for 6055 and 5721
 *
 *  This is a quick and dirty library to operate a Bosch seat motor
 *
 *  BOSCH AHC-2 12V  6004.RA3.194-06 174.9:1 gear w/ encoder 1 tick per motor revolution on roboRIO analog 5 volt bus
 *
 *  Encoder wiring:
 *
 *  PWM cable from Analog port on roboRIO
 *  Red wire    -> 200 ohm resistor -> Motor Pin #2
 *  White wire  -> Motor Pin #2
 *  Black wire  -> Motor Pin #4
 *
 *  Motor wiring:
 *  PDP -> motor controller -> Motor Pin #1
 *  PDP -> motor controller -> Motor Pin #3
 *
 */
#include "AltMotor.h"
#include "WPILib.h"

AltMotor::AltMotor(int mChannel, int aChannel)
{
	bMotor = new VictorSP(mChannel);
	mAnalogTrigger = new AnalogTrigger(aChannel);
	mAnalogTrigger->SetLimitsVoltage(3.5, 3.8); // values higher than the highest minimum (pulse floor), lower than the lowest maximum (pulse ceiling)
	mCounter = new Counter(mAnalogTrigger);
	countFlag = false;
	tickCount = 0;		//internal counter of ticks
	TickTarget = 0;		//set by user to determine how many ticks to move     1 tick = about 2 degrees
	Speed = 0.0;		//set by user to determine speed and direction - zeroed by program when destination reached
}

AltMotor::~AltMotor()
{
	delete bMotor;
	bMotor = nullptr;
	delete mAnalogTrigger;
	mAnalogTrigger = nullptr;
	delete mCounter;
	mCounter = nullptr;
}

void AltMotor::Execute()
{
	if (TickTarget > 0)
	{
		if(!countFlag)
		{
			mCounter->Reset(); //zero the counter
			tickCount = 0;     //zero tick accumulator
			countFlag = true;  //set flag to start counting
		}
		tickCount = tickCount + mCounter->Get();
		mCounter->Reset();
		printf("ticks=%i\n",tickCount);
		if(tickCount >= TickTarget)
		{
			Speed = 0.0;
			TickTarget = 0;
			countFlag = false;
		}
	}
	bMotor->Set(Speed);
}

void AltMotor::Reset()
{
	mCounter->Reset(); //zero the counter
	tickCount = 0;     //zero tick accumulator
	Speed = 0.0;
	TickTarget = 0;
	countFlag = false;
	bMotor->Set(Speed);
}

void AltMotor::SetInverted(bool invert)
{
	bMotor->SetInverted(invert);
}

void AltMotor::SetSafetyEnabled(bool enabled)
{
	bMotor->SetSafetyEnabled(enabled);
}

