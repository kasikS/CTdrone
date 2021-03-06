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

#ifndef LEDS_H
#define LEDS_H

typedef enum LED_COLOR {
    GREEN = 0,
    RED = 1,
    LEDS_NUMBER         // sentinel
} led_color_t;

// TODO comments

void leds_init(void);

void leds_on(led_color_t color);

void leds_off(led_color_t color);

void leds_toggle(led_color_t color);

int leds_state(led_color_t color);

#endif /* LEDS_H */

