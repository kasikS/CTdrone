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

#ifndef MOTOR_H
#define MOTOR_H

enum {
    MOTOR_FL = 0,			//front left
    MOTOR_FR = 1,			//front right
    MOTOR_BL = 2,			//back left
    MOTOR_BR = 3,			//back right
    MOTOR_NUMBER = 4
} motor_id;

typedef struct speed {
	int fl;
	int fr;
	int bl;
	int br;
} speed;

extern const int MIN_SPEED;
extern const int MAX_SPEED;

/**
 * Initalizes the timer and outputs to work in PWM mode.
 * @return false in case of error.
 */
int motor_init(void);

/**
 * Changes speed for a motor.
 * @param motor is the number of the motor (preferably motor_id enum should be
 * used, otherwise numbers 0-3 are allowed).
 * @param speed is the new speed value (0-100).
 */
void motor_set_speed(int motor, int speed);

/**
 * Returns the current motor speed.
 * @param motor is the number of the requested motor.
 * @return Current set speed. Note that there is no measurement of the speed,
 * so the value is the one that was previously requested.
 */
int motor_get_speed(int motor);

#endif /* MOTOR_H */

