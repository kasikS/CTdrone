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

const int MIN_SPEED = 1000;
const int MAX_SPEED = 2000;

// Timer frequency [kHz]
#define TIMER_CLK   3200
// Control signal period [ms]
#define PERIOD      3
// Duty cycle responding to the period (100%) [counter units]
#define MAX_DUTY    ((uint32_t)(PERIOD * TIMER_CLK))
// One millisecond [counter units]
#define MILLISECOND ((uint32_t)(MAX_DUTY / PERIOD))

/*#define LED_TEST*/

xQueueHandle motors_queue;
static void motors_task(void *parameters);
static int motor_speed[MOTOR_NUMBER] = { 0, };

// PWM1 outputs
// CH1: PA8
// CH2: PA9
// CH3: PA10
// CH4: PA11

static __IO uint32_t* timers[4];

int motor_init(void)
{
    timers[0] = &TIM1->CCR1;
    timers[1] = &TIM1->CCR2;
    timers[2] = &TIM1->CCR3;
    timers[3] = &TIM1->CCR4;

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
    timer_conf.TIM_Prescaler = (uint16_t) ((SystemCoreClock) / (TIMER_CLK * 1000)) - 1;
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
    int val = (speed - 1000) / 50.0 * MILLISECOND;
#else
    int val = (speed * MILLISECOND) / 1000;
    /*int val = (speed / 1000.0) * MILLISECOND;*/
#endif

    /**timers[m] = val;*/

    switch(m)
    {
        case MOTOR_FL: TIM1->CCR1 = val; break; //TIM_SetCompare1(TIM1, val); break;
        case MOTOR_BL: TIM1->CCR2 = val; break; //TIM_SetCompare2(TIM1, val); break;
        case MOTOR_FR: TIM1->CCR3 = val; break; //TIM_SetCompare3(TIM1, val); break;
        case MOTOR_BR: TIM1->CCR4 = val; break; //TIM_SetCompare4(TIM1, val); break;
    }

    motor_speed[m] = speed;
}

int motor_get_speed(int m)
{
    if(m >= MOTOR_NUMBER || m < 0)
        return -1;

    return motor_speed[m];
}
