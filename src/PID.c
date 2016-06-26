/*
 * PID.c
 *
 *  Created on: Mar 15, 2015
 *      Author: Katarzyna Stachyra
 */

#include "PID.h"
#include <stdio.h> // TODO remove (debugging)
#include "serial.h" // TODO remove (debugging)
#include "leds.h"
#define WINDUP_GUARD_GAIN 150.0 //should be 0-100% for the throttle?

float windupGuard;
extern int tx_cnt;
extern const int TX_CNT_MAX;

// TODO http://hackaday.com/2016/05/18/flying-with-proportional-integral-derivative-control/

float PIDupdate(PID * PIDval, float target, float cur, int deltaTime){

    char buf[32];
	float error;

	// determine how badly we are doing
	error = target - cur;

        if(tx_cnt == TX_CNT_MAX) {
            sprintf(buf, "%06d %06d ", (int)(target * 100.0), (int)(cur * 100.0));
            serial_puts(buf);
        }

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

        windupGuard = WINDUP_GUARD_GAIN / PIDval->igain;
        if (PIDval->iState > windupGuard)
                PIDval->iState = windupGuard;
        else if (PIDval->iState < -windupGuard)
                PIDval->iState = -windupGuard;

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

