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

#ifndef CONFIG_H
#define CONFIG_H

#include "joystick.h"

extern const char*CONFIG_DEFAULT_FILE;

#define DEVICE_NAME_LEN 32

struct config {
    char serial_device[DEVICE_NAME_LEN];
    int serial_speed;

    char joystick_device[DEVICE_NAME_LEN];
    struct axis_calibration joystick_calibration[CONTROLS_NUMBER];
    int joystick_mapping[CONTROLS_NUMBER];
};

// TODO
int config_load(const char* filename, struct config*conf);
int config_save(const char* filename, struct config*conf);
int config_default(struct config*conf);
void config_compute_joy_scales(struct config*conf);

#endif /* CONFIG_H */
