/*
 * FlightControl.c
 *
 *  Created on: Mar 15, 2015
 *      Author: Katarzyna Stachyra <kas.stachyra@gmail.com>
 */
#include "stm32f4xx_gpio.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include "FlightControl.h"
#include "PID.h"
#include "motor.h"
#include "drv_mpu6050.h"
#include <stdio.h>

#include "serial.h"

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
//check values of imu - max, orientation (where is +/-, left/right), center it

#define IMU_TICKS_WAIT   100
extern xQueueHandle imu_queue;

#define RF_TICKS_WAIT   100
extern xQueueHandle rf_queue;

extern xQueueHandle motors_queue;

#define FLIGHT_CONTROL_QUEUE_SIZE   64
static void flight_control_task(void *parameters);

PID levelRollPID;// = PID(6.1, 0.0, 0.9);
PID levelPitchPID; // = PID(6.1, 0.0, 0.9);
PID levelYawPID; // = PID(6.0, 0, 0.0);

// check the delta time
//currentTime = micros();
//deltaTime = currentTime - previousTime;
//previousTime = currentTime;
float deltat;


int flight_control_init(void){

	levelRollPID.pgain=6.1;
	levelRollPID.igain=0;
	levelRollPID.dgain=0.9;

	levelPitchPID.pgain=6.1;
	levelPitchPID.igain=0;
	levelPitchPID.dgain=0.9;

	levelYawPID.pgain=6;
	levelYawPID.igain=0;
	levelYawPID.dgain=0;


	    portBASE_TYPE ret = xTaskCreate(flight_control_task, NULL,
	                                    256, NULL, 2, NULL);
	    if(ret != pdPASS)
	        return pdFALSE;


	    return pdTRUE;
}

void ProcessFlightControl(void){

//	static angles CurrentPosition;
	static angles TargetPosition;
	angles CorrectPosition;
	speed MotorsSpeed;
	char buf[32] = {0,};
	int int_yaw;
	int int_pitch;
	int int_roll;
	//if(xQueueReceive(imu_queue, &CurrentPosition, 0))

	if(xSemaphoreTake(imu_data_rdy, 0))
    {
		if(xSemaphoreTake(imu_data_update, 0))
		{
//			int_yaw=(imu_position.yaw);
//			int_pitch=(imu_position.pitch);
//			int_roll=(imu_position.roll);
//
//			sprintf(buf, "%d,", int_yaw);
//			serial_puts(buf);
//			sprintf(buf, "%d,", int_pitch);
//			serial_puts(buf);
//			sprintf(buf, "%d\r\n", int_roll);
//			serial_puts(buf);


//			const float factor = 180000.0 / M_PI;
//
//			int_yaw=(int)(imu_position.yaw*1000);
//			int_pitch=(int)(imu_position.pitch*1000);
//			int_roll=(int)(imu_position.roll*1000);
//
//			xSemaphoreGive(imu_data_update);
//
//			sprintf(buf, "%d,", int_yaw);
//			serial_puts(buf);
//			sprintf(buf, "%d,", int_pitch);
//			serial_puts(buf);
//			sprintf(buf, "%d\r\n", int_roll);
//			serial_puts(buf);


//
//			sprintf(buf, "%d.%03d,", int_yaw / 1000, abs(int_yaw) % 1000);
//			serial_puts(buf);
//			sprintf(buf, "%d.%03d,", int_pitch / 1000, abs(int_pitch) % 1000);
//			serial_puts(buf);
//			sprintf(buf, "%d.%03d\r\n", int_roll/ 1000, abs(int_roll) % 1000);
//			serial_puts(buf);





			int_yaw=(imu_position.yaw*1000*180.0f/M_PI);
			int_pitch=(imu_position.pitch*1000*180.0f/M_PI);
			int_roll=(imu_position.roll*1000*180.0f/M_PI);

			xSemaphoreGive(imu_data_update);

			sprintf(buf, "%d.%03d,", int_yaw / 1000, abs(int_yaw) % 1000);
			serial_puts(buf);
			sprintf(buf, "%d.%03d,", int_pitch / 1000, abs(int_pitch) % 1000);
			serial_puts(buf);
			sprintf(buf, "%d.%03d\r\n", int_roll/ 1000, abs(int_roll) % 1000);
			serial_puts(buf);

		}
    }

//	if(xQueueReceive(rf_queue, &TargetPosition, RF_TICKS_WAIT))
//    {
//    	serial_puts("target");
//    }


	/*
	CorrectPosition.roll = PIDupdate(&levelRollPID, TargetPosition.roll, constrain(CurrentPosition.roll, -50, 50), deltat);
	// Positive values mean the frontend is up
	// Constrain to 45 degrees, because beyond that, we're fucked anyway
	CorrectPosition.pitch = PIDupdate(&levelPitchPID,TargetPosition.pitch, constrain(CurrentPosition.pitch, -50, 50), deltat);
	// Positive values are to the right
	CorrectPosition.yaw = PIDupdate(&levelRollPID, TargetPosition.yaw, CurrentPosition.yaw, deltat);
	// Apply offsets to all motors evenly to ensure we pivot on the center

	//check first if engines are in move..
		motor_set_speed(MOTOR_FL, motor_get_speed(MOTOR_FL) - rollAdjust - pitchAdjust + yawAdjust);
		motor_set_speed(MOTOR_FR, motor_get_speed(MOTOR_FR) + rollAdjust - pitchAdjust - yawAdjust);
		motor_set_speed(MOTOR_BL, motor_get_speed(MOTOR_BL) - rollAdjust + pitchAdjust - yawAdjust);
		motor_set_speed(MOTOR_BR, motor_get_speed(MOTOR_BR) + rollAdjust + pitchAdjust + yawAdjust);
*/

//	MotorsSpeed.fl = motor_get_speed(MOTOR_FL) - CorrectPosition.roll - CorrectPosition.pitch + CorrectPosition.yaw;
//	MotorsSpeed.fr = motor_get_speed(MOTOR_FR) + CorrectPosition.roll - CorrectPosition.pitch - CorrectPosition.yaw;
//	MotorsSpeed.bl = motor_get_speed(MOTOR_BL) - CorrectPosition.roll + CorrectPosition.pitch - CorrectPosition.yaw;
//	MotorsSpeed.br = motor_get_speed(MOTOR_BR) + CorrectPosition.roll + CorrectPosition.pitch + CorrectPosition.yaw;

//	if((TargetPosition.pitch!=CurrentPosition.pitch)||(TargetPosition.roll!=CurrentPosition.roll)||(TargetPosition.yaw!=CurrentPosition.yaw))
//	{
//		xQueueSend( motors_queue, ( void * ) &MotorsSpeed, 0);
//	}

}


static void flight_control_task(void *parameters)
{
	while(1){
		ProcessFlightControl();
		GPIO_ToggleBits(GPIOA, GPIO_Pin_6);

	}
}
