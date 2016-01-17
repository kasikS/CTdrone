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
#include "leds.h"

#define LED_PORT        GPIOA
#define RED_GPIO        GPIO_Pin_6
#define GREEN_GPIO      GPIO_Pin_7

static const int LED_GPIO[] = { RED_GPIO, GREEN_GPIO };

void leds_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = RED_GPIO | GREEN_GPIO;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(LED_PORT, &GPIO_InitStructure);
}

void leds_on(led_color_t color)
{
    /*assert(color < LEDS_NUMBER);*/
    GPIO_SetBits(LED_PORT, LED_GPIO[color]);
}

void leds_off(led_color_t color)
{
    /*assert(color < LEDS_NUMBER);*/
    GPIO_ResetBits(LED_PORT, LED_GPIO[color]);
}

void leds_toggle(led_color_t color)
{
    /*assert(color < LEDS_NUMBER);*/
    GPIO_ToggleBits(LED_PORT, LED_GPIO[color]);
}

int leds_state(led_color_t color)
{
    // TODO
    return 0;
}

