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

#ifndef DELAY_TIMER_H
#define DELAY_TIMER_H

///> @brief Hardware timer to implement accurate delays. The implementation
///> is FreeRTOS independent.

/**
 * Initializes the timer to measure delay times.
 * @return false in case of error.
 */
void delay_init(void);

/**
 * Delays code execution by *at least* a specified number of microseconds.
 * @param usecs is the number of microseconds.
 */
void delay_us(int usecs);

/**
 * Delays code execution by *at least* a specified number of milliseconds.
 * @param msecs is the number of milliseconds.
 */
static inline void delay_ms(int msecs)
{
    delay_us(1000 * msecs);
}

#endif
