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
#include <timers.h>

#include "FlightControl.h"
#include "PID.h"
#include "motor.h"
#include "drv_mpu6050.h"
#include "nrf24l.h"
#include "link.h"
#include "utils.h"
#include <stdio.h>
#include <string.h>

#include "serial.h"
#include "leds.h"

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
//check values of imu - max, orientation (where is +/-, left/right), center it

#define IMU_TICKS_WAIT   100
extern xQueueHandle imu_queue;

#define RF_TICKS_WAIT   100
extern xQueueHandle rf_queue;

extern xQueueHandle motors_queue;

#define FLIGHT_CONTROL_QUEUE_SIZE   64
static void flight_control_task(void *parameters);

xSemaphoreHandle command_rdy, command_update;
volatile angles target_position;
volatile int throttle = 0;
static void command_rx_task(void *parameters);
static void pid_coefs_task(void *parameters);
extern float windupGuard;
const int TX_CNT_MAX = 500;

PID levelRollPID;// = PID(6.1, 0.0, 0.9);
PID levelPitchPID; // = PID(6.1, 0.0, 0.9);
PID levelYawPID; // = PID(6.0, 0, 0.0);

// RF link watchdog, tries to land if the link is down
static xTimerHandle safe_timer;

static inline void reset_safe_timer()
{
    xTimerReset(safe_timer, 500 / portTICK_PERIOD_MS);
}

static void safe_timer_callback(xTimerHandle timer)
{
    (void) timer;

    serial_puts("LNKXXX\r\n");

    // TODO uncomment in final version
    /*target_position.yaw = 0;*/
    /*target_position.pitch = 0;*/
    /*target_position.roll = 0;*/

    if(throttle >= 50) {
        throttle -= 50;
    }
    else {
        // It has already either crashed or landed, turn off safety timer
        // until a new radio packet is received
        throttle = 0;
        xTimerStop(safe_timer, 0);
    }

    xSemaphoreGive(command_update);
    xSemaphoreGive(command_rdy);
}

int flight_control_init(void){

	levelRollPID.pgain=0.0;
	levelRollPID.igain=0.0; //0.000005;
	levelRollPID.dgain=0.0;

	levelPitchPID.pgain=0.0;
	levelPitchPID.igain=0.0; //0.000005;
	levelPitchPID.dgain=0.0;

	levelYawPID.pgain=0.0;
	levelYawPID.igain=0.0;
	levelYawPID.dgain=0.0; //.001;

    target_position.yaw = 0;
    target_position.pitch = 0;
    target_position.roll = 0;
    throttle = 0;

	 command_rdy = xSemaphoreCreateBinary();
	 if(command_rdy == NULL)
		 return pdFALSE;

	 command_update = xSemaphoreCreateMutex();
	 if(command_update == NULL)
		 return pdFALSE;

         /*
    safe_timer = xTimerCreate("safe_timer", 500 / portTICK_PERIOD_MS,
                              pdTRUE, (void*) 0, safe_timer_callback);
    if(safe_timer == NULL)
        return pdFALSE;
        */

    // safe_timer is started when the first radio packet arrives
/*
    if(xTaskCreate(flight_control_task, NULL, 256, NULL, 2, NULL) != pdPASS)
        return pdFALSE;
        */

    if(xTaskCreate(command_rx_task, NULL, 256, NULL, 2, NULL) != pdPASS)
        return pdFALSE;

    if(xTaskCreate(pid_coefs_task, NULL, 256, NULL, 2, NULL) != pdPASS)
        return pdFALSE;

    return pdTRUE;
}

angles CurrentPosition;

int tx_cnt = 0;
volatile angles TargetPosition = {0.0f, 0.0f, 0.0f};

void ProcessFlightControl(int deltat){
	angles CorrectPosition;
	speed MotorsSpeed;
	char buf[32] = {0,};

	if(xSemaphoreTake(imu_data_rdy, 0))
    {
		if(xSemaphoreTake(imu_data_update, 0))
		{
			// ccw negative, cw positive
			CurrentPosition.yaw=(imu_position.yaw*180.0f/M_PI);
			// down negative, up positive
			CurrentPosition.pitch=(imu_position.pitch*180.0f/M_PI);
			// left side up negative, right side up positive
			CurrentPosition.roll=(imu_position.roll*180.0f/M_PI);

			xSemaphoreGive(imu_data_update);

            /*sprintf(buf, "%d,%d,%d\r\n", (int) CurrentPosition.yaw, (int) CurrentPosition.pitch, (int) CurrentPosition.roll);*/
            /*serial_puts(buf);*/
		}
    }

	if(xSemaphoreTake(command_rdy, 0))
    {
		if(xSemaphoreTake(command_update, 0))
		{
			TargetPosition.yaw=target_position.yaw;
			TargetPosition.pitch=target_position.pitch;
			TargetPosition.roll=target_position.roll;

            /*sprintf(buf, "%d,", (int) throttle);*/
            /*serial_puts(buf);*/
            /*sprintf(buf, "%d,", (int) TargetPosition.yaw);*/
            /*serial_puts(buf);*/
            /*sprintf(buf, "%d,", (int) TargetPosition.pitch);*/
            /*serial_puts(buf);*/
            /*sprintf(buf, "%d\r\n", (int) TargetPosition.roll);*/
            /*serial_puts(buf);*/

			xSemaphoreGive(command_update);
		}
    }

	// Positive values are to the left
	CorrectPosition.yaw = PIDupdate(&levelYawPID, TargetPosition.yaw, CurrentPosition.yaw, deltat);
	// Positive values mean the frontend is up
	// Constrain to 45 degrees, because beyond that, we're fucked anyway
	CorrectPosition.pitch = PIDupdate(&levelPitchPID,TargetPosition.pitch, constrain(CurrentPosition.pitch, -50, 50), deltat);
        // negaitve value means left side is up
	CorrectPosition.roll = PIDupdate(&levelRollPID, TargetPosition.roll, constrain(CurrentPosition.roll, -50, 50), deltat);

	// Apply offsets to all motors evenly to ensure we pivot on the center

	//char buf[16]={0,};
//	sprintf(buf, "%d\r\n", deltat);
//	serial_puts(buf);
//
        float roll_err = CorrectPosition.roll - TargetPosition.roll;
        float pitch_err = CorrectPosition.pitch - TargetPosition.pitch;
//
        if(pitch_err < 2.0 && pitch_err > -2.0 && roll_err < 2.0 && roll_err > -2.0) {
            leds_on(GREEN);
        } else {
            leds_off(GREEN);
        }

	//check first if engines are in move..
//		motor_set_speed(MOTOR_FL, motor_get_speed(MOTOR_FL) - rollAdjust - pitchAdjust + yawAdjust);
//		motor_set_speed(MOTOR_FR, motor_get_speed(MOTOR_FR) + rollAdjust - pitchAdjust - yawAdjust);
//		motor_set_speed(MOTOR_BL, motor_get_speed(MOTOR_BL) - rollAdjust + pitchAdjust - yawAdjust);
//		motor_set_speed(MOTOR_BR, motor_get_speed(MOTOR_BR) + rollAdjust + pitchAdjust + yawAdjust);

        if(throttle >= MIN_SPEED) {
            MotorsSpeed.fl = throttle - CorrectPosition.roll + CorrectPosition.pitch; // + CorrectPosition.yaw;
            MotorsSpeed.fr = 0; // throttle + CorrectPosition.roll + CorrectPosition.pitch; // - CorrectPosition.yaw;
            MotorsSpeed.bl = 0; // throttle - CorrectPosition.roll - CorrectPosition.pitch; // - CorrectPosition.yaw;
            MotorsSpeed.br = throttle + CorrectPosition.roll - CorrectPosition.pitch; // + CorrectPosition.yaw;
        } else {
            MotorsSpeed.fl = 0;
            MotorsSpeed.fr = 0;
            MotorsSpeed.bl = 0;
            MotorsSpeed.br = 0;
            levelRollPID.iState = 0;
            levelPitchPID.iState = 0;
            levelYawPID.iState = 0;
        }

        if(tx_cnt == TX_CNT_MAX) {
            sprintf(buf, "%04d %04d %04d %04d\r\n", MotorsSpeed.fl, MotorsSpeed.fr, MotorsSpeed.bl, MotorsSpeed.br);
            serial_puts(buf);
            tx_cnt = 0;
        } else {
            ++tx_cnt;
        }

        // TODO orson: uncomment?
	/*if((TargetPosition.pitch!=CurrentPosition.pitch)||(TargetPosition.roll!=CurrentPosition.roll)||(TargetPosition.yaw!=CurrentPosition.yaw))*/
	{
		/*if(cnt >= 100) {
			sprintf(buf, "%d\t%d\t%d\t%d\r\n", MotorsSpeed.fl, MotorsSpeed.fr, MotorsSpeed.bl, MotorsSpeed.br);
			serial_puts(buf);
			cnt = 0;
		}
		*/

		//xQueueSend( motors_queue, ( void * ) &MotorsSpeed, 0);
        motor_set_speed(MOTOR_FL, MotorsSpeed.fl);
        motor_set_speed(MOTOR_FR, MotorsSpeed.fr);
        motor_set_speed(MOTOR_BL, MotorsSpeed.bl);
        motor_set_speed(MOTOR_BR, MotorsSpeed.br);
	}
}


static void flight_control_task(void *parameters)
{
	portTickType previousTime=0;
	portTickType currentTime=0;
	portTickType deltaTime=0;
	int deltat;

	previousTime = xTaskGetTickCount();

	while(1){
		currentTime = xTaskGetTickCount();
		deltaTime = currentTime - previousTime;
		previousTime = currentTime;
		deltat = deltaTime * portTICK_RATE_MS; //time in milliseconds

                if(deltat > 0)
                    ProcessFlightControl(deltat);

		vTaskDelay(2);
	}
}


static void command_rx_task(void *parameters)
{
    char c;
    uint8_t buf[PACKET_TOTAL_SIZE];
    uint8_t buf_count = 0;
    const struct packet const* pkt = (struct packet*) &buf;
    struct packet response;

    while(1){
        if(nrf24l_getc(&c)) {
            buf[buf_count++] = c;

            if(buf_count == PACKET_TOTAL_SIZE) {
                buf_count = 0;

                /*for(int i = 0; i < PACKET_TOTAL_SIZE; ++i) {*/
                    /*sprintf(tmp, "%.2x ", buf[i]);*/
                    /*serial_puts(tmp);*/
                /*}*/
                /*serial_puts("\r\n");*/

                /*sprintf(tmp, "%d %d %d %d %d\r\n",*/
                        /*pkt->data.joy.throttle, pkt->data.joy.yaw,*/
                        /*pkt->data.joy.pitch, pkt->data.joy.roll,*/
                        /*pkt->data.joy.buttons);*/
                /*serial_puts(tmp);*/
                if(pkt->type == PT_JOYSTICK &&
                        xSemaphoreTake(command_update, 50 / portTICK_PERIOD_MS))
                {
                    throttle = pkt->data.joy.throttle;
                    target_position.yaw   = pkt->data.joy.yaw;
                    target_position.pitch = pkt->data.joy.pitch;
                    target_position.roll  = pkt->data.joy.roll;

                    reset_safe_timer();

                    xSemaphoreGive(command_update);
                    xSemaphoreGive(command_rdy);
                } else if(pkt->type & PT_REPORT) {
                    int report_type = pkt->type & ~PT_REPORT;

                    memset(&response, 0x00, PACKET_TOTAL_SIZE);
                    response.type = pkt->type;

                    switch(report_type) {
                        case RPT_MOTOR:
                            response.data.rpt_motor.fl = motor_get_speed(MOTOR_FL);
                            response.data.rpt_motor.fr = motor_get_speed(MOTOR_FR);
                            response.data.rpt_motor.bl = motor_get_speed(MOTOR_BL);
                            response.data.rpt_motor.br = motor_get_speed(MOTOR_BR);
                            break;

                        case RPT_IMU:
                            response.data.rpt_imu.yaw   = (int16_t) CurrentPosition.yaw;
                            response.data.rpt_imu.pitch = (int16_t) CurrentPosition.pitch;
                            response.data.rpt_imu.roll  = (int16_t) CurrentPosition.roll;
                            break;

                        default:
                            serial_write("BADRPT", 6);
                            break;
                    }

                    nrf24l_write((const char*)&response, PACKET_TOTAL_SIZE);
                }
            }
        }

        GPIO_ToggleBits(GPIOA, GPIO_Pin_5);
    }
}


// Receives packets with new PID settings over the serial port.
static void pid_coefs_task(void *parameters)
{
    #define BUF_SIZE 32
    char buf[BUF_SIZE] = {0,};
    int cnt = 0;

    float coef = 0.0;
    char type = 0, axis = 0;
    PID*pid_controller = 0;
    volatile float*target = 0;

    while(1) {
        if(serial_getc(&buf[cnt])) {
            if(buf[cnt] == '\n') {       // received end of packet
                if(cnt > 0) {
                    type = buf[1];
                    axis = buf[0];

                    coef = ct_atof(&buf[2]);
                    // select axis
                    switch(axis) {
                        case 'y':
                            pid_controller = &levelYawPID;
                            target = &(TargetPosition.yaw);
                            break;

                        case 'p':
                            pid_controller = &levelPitchPID;
                            target = &(TargetPosition.pitch);
                            break;

                        case 'r':
                            pid_controller = &levelRollPID;
                            target = &(TargetPosition.roll);
                            break;

                        default:
                            pid_controller = 0;
                            target = 0;
                            break;
                    }

                    // apply the new coefficient

                    // TODO semaphore
                    switch(type) {
                        case 'p':
                            if(pid_controller)
                                pid_controller->pgain = coef;
                            break;
                        case 'i':
                            if(pid_controller)
                                pid_controller->igain = coef;
                            break;
                        case 'd':
                            if(pid_controller)
                                pid_controller->dgain = coef;
                            break;
                        case 't':
                            throttle = coef;
                            break;
                        case 'w':
                            windupGuard = coef;
                            break;
                        case 'a':
                            if(target)
                                *target = coef;
                            break;
                        default:
                            serial_puts("INV\r\n");
                            break;
                    }
                }

                // prepare for the next command
                memset(buf, 0, BUF_SIZE);
                cnt = 0;
                type = 0;
                coef = 0;
            } else {
                ++cnt;
            }

            if(cnt == BUF_SIZE)
                cnt = 0;
        }
    }
}
