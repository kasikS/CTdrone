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

#include "stm32f4xx_conf.h"

#include "delay_timer.h"

// Multiplier adjusted to reduce rounding error for prescaler
#define TIMER_FREQ_MPLIER   16
#define TIMER_FREQ          (1000000 * TIMER_FREQ_MPLIER)
#define TIMER_USECS_LIMIT   (65535 / TIMER_FREQ_MPLIER - 1)

void delay_init(void)
{
    TIM_TimeBaseInitTypeDef timer_conf;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    TIM_TimeBaseStructInit(&timer_conf);
    timer_conf.TIM_Period = 65535;
    timer_conf.TIM_Prescaler = (uint16_t) ((SystemCoreClock) / (TIMER_FREQ)) - 1;
    timer_conf.TIM_ClockDivision = TIM_CKD_DIV1;
    timer_conf.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &timer_conf);

    TIM_Cmd(TIM3, ENABLE);
}

void delay_us(int usecs)
{
    // Split the delay value to smaller ones if necessary
    while(usecs > TIMER_USECS_LIMIT)
    {
        delay_us(TIMER_USECS_LIMIT);
        usecs -= TIMER_USECS_LIMIT;
    }

    assert_param(usecs > 0);
    uint32_t finish = TIMER_FREQ_MPLIER * usecs;

    TIM_SetCounter(TIM3, 0);
    while(TIM_GetCounter(TIM3) < finish);
}
