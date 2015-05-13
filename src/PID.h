/*
 * PID.h
 *
 *  Created on: Mar 15, 2015
 *      Author: Katarzyna Stachyra
 */

#ifndef PID_H_
#define PID_H_

typedef struct PID{
	float pgain, igain, dgain;
	float pTerm, iTerm, dTerm;
	float iState;
	float last;
}PID;

float PIDupdate(PID * PIDval, float target, float cur, float deltaTime); //deltatime - time between each call
#endif /* PID_H_ */
