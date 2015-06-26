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

#ifndef JOYSTICK_H
#define JOYSTICK_H

#define JOYSTICK_NAME_LEN 128

enum CONTROL {
    THROTTLE_AXIS   = 0,
    YAW_AXIS        = 1,
    PITCH_AXIS      = 2,
    ROLL_AXIS       = 3,
    CONTROLS_NUMBER
};

struct axis_calibration
{
    // Axis offset
    int device_offset;

    // Device range
    int device_max, device_min;

    // Desired range
    int target_max, target_min;

    // Factors computed basing on the above data
    float scale_neg, scale_pos;
};

// TODO desc
int joystick_init(const char* device);
void joystick_close(void);
int joystick_update(void);

void joystick_calibrate(struct axis_calibration *calibration);
void joystick_set_axis_mapping(int *mapping);
void joystick_set_axis_calibration(struct axis_calibration *calibration);

int joystick_get_buttons(void);
int joystick_get_control_raw(enum CONTROL control);
int joystick_get_control_val(enum CONTROL control);

void joystick_print_report(void);

#endif /* JOYSTICK_H */
