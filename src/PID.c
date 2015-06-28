/*
 * PID.c
 *
 *  Created on: Mar 15, 2015
 *      Author: Katarzyna Stachyra
 */

#include "PID.h"
#define WINDUP_GUARD_GAIN 500.0 //should be 0-100% for the throttle?

float PIDupdate(PID * PIDval, float target, float cur, float deltaTime){

	float error;
	float windupGuard;
	// determine how badly we are doing
	error = target - cur;
	// the pTerm is the view from now, the pgain judges
	// how much we care about error at this instant.
	PIDval->pTerm =PIDval->pgain * error;
	// iState keeps changing over time; it's
	// overall "performance" over time, or accumulated error
	PIDval->iState += error * deltaTime;
	// to prevent the iTerm getting huge despite lots of
	// error, we use a "windup guard"
	// (this happens when the machine is first turned on and
	// it cant help be cold despite its best efforts)
	// not necessary, but this makes windup guard values
	// relative to the current iGain

//	windupGuard = WINDUP_GUARD_GAIN / PIDval->igain;
//	if (PIDval->iState > windupGuard)
//		PIDval->iState = windupGuard;
//	else if (PIDval->iState < -windupGuard)
//		PIDval->iState = -windupGuard;

	PIDval->iTerm = PIDval->igain * PIDval->iState;
	// the dTerm, the difference between the temperature now
	// and our last reading, indicated the "speed,"
	// how quickly the temp is changing. (aka. Differential)
	PIDval->dTerm = (PIDval->dgain * (cur - PIDval->last)) / deltaTime;
	// now that we've use lastTemp, put the current temp in
	// our pocket until for the next round
	PIDval->last = cur;
	// the magic feedback bit
	return PIDval->pTerm + PIDval->iTerm - PIDval->dTerm;
}

void PIDresetError(PID * PIDval){
	PIDval->iState = 0;
}

