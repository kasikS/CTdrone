/*
 * Copyright (C) 2015 Maciej Suminski <orson@orson.net.pl>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "motor.h"
#include "stm32f4xx_conf.h"

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include <inttypes.h>

#define MOTORS_QUEUE_SIZE   64

const int MIN_SPEED = 0;
const int MAX_SPEED = 100;

// Timer frequency [Hz]
#define TIMER_CLK   3200000
// Control signal period [s]
#define PERIOD      20e-3
// Duty cycle responding to the period (100%) [counter units]
#define MAX_DUTY    ((uint32_t)(PERIOD * TIMER_CLK))
// One millisecond [counter units]
#define MILLISECOND ((uint32_t)(MAX_DUTY / (PERIOD * 1000)))

#define LED_TEST

xQueueHandle motors_queue;
static void motors_task(void *parameters);
static int motor_speed[MOTOR_NUMBER] = { 0, };

// PWM1 outputs
// CH1: PA8
// CH2: PA9
// CH3: PA10
// CH4: PA11

int motor_init(void)
{
    GPIO_InitTypeDef gpio_conf;
    TIM_TimeBaseInitTypeDef timer_conf;
    TIM_OCInitTypeDef channel_conf;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    gpio_conf.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
    gpio_conf.GPIO_Mode = GPIO_Mode_AF;
    gpio_conf.GPIO_Speed = GPIO_Speed_100MHz;
    gpio_conf.GPIO_OType = GPIO_OType_PP;
    gpio_conf.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &gpio_conf);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_TIM1);

    TIM_TimeBaseStructInit(&timer_conf);
    timer_conf.TIM_Period = MAX_DUTY;
    timer_conf.TIM_Prescaler = (uint16_t) ((SystemCoreClock) / TIMER_CLK) - 1;
    timer_conf.TIM_ClockDivision = TIM_CKD_DIV1;
    timer_conf.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &timer_conf);

    TIM_OCStructInit(&channel_conf);
    channel_conf.TIM_OCMode = TIM_OCMode_PWM1;
    channel_conf.TIM_OutputState = TIM_OutputState_Enable;
    channel_conf.TIM_Pulse = 0;
    channel_conf.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM1, &channel_conf);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2Init(TIM1, &channel_conf);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC3Init(TIM1, &channel_conf);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC4Init(TIM1, &channel_conf);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    // FreeRTOS
    motors_queue = xQueueCreate(MOTORS_QUEUE_SIZE, sizeof(speed));  // or send as pointers..? but it will be overwritten...
    if(motors_queue == 0)
        return pdFALSE;

    portBASE_TYPE ret = xTaskCreate(motors_task, NULL,
                                    configMINIMAL_STACK_SIZE, NULL, 2, NULL);
    if(ret != pdPASS)
        return pdFALSE;

    return pdTRUE;
}

void motor_set_speed(int m, int speed)
{
    if(speed < MIN_SPEED)
        speed = MIN_SPEED;
    else if(speed > MAX_SPEED)
        speed = MAX_SPEED;

    // PPM output:
    // |------|____________|----..
    // <-tHI-> <---tLOW--->
    // tHI + tLOW = 20 ms (
    // min speed for tHI = 1 ms
    // max speed for tHI = 2 ms
#ifdef LED_TEST
    int val = speed * 20 * MILLISECOND / MAX_SPEED;
#else
    int val = speed * MILLISECOND / MAX_SPEED + MILLISECOND;
#endif

    switch(m)
    {
        case MOTOR_FL: TIM_SetCompare1(TIM1, val); break;
        case MOTOR_FR: TIM_SetCompare2(TIM1, val); break;
        case MOTOR_BL: TIM_SetCompare3(TIM1, val); break;
        case MOTOR_BR: TIM_SetCompare4(TIM1, val); break;
    }

    motor_speed[m] = speed;
}

int motor_get_speed(int m)
{
    if(m >= MOTOR_NUMBER)
        return -1;

    return motor_speed[m];
}

void motors_task(void *parameters)
{
   speed SettingsFromPID;

   while(1)
    {
        if(xQueueReceive(motors_queue, &SettingsFromPID, portMAX_DELAY))
        {
            motor_set_speed(MOTOR_FL, SettingsFromPID.fl);
            motor_set_speed(MOTOR_FR, SettingsFromPID.fr);
            motor_set_speed(MOTOR_BL, SettingsFromPID.bl);
            motor_set_speed(MOTOR_BR, SettingsFromPID.br);
        }

        vTaskDelay(100);
    }
}
