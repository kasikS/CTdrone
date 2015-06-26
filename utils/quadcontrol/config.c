/*
 * Copyright (C) 2015 Maciej Suminski <orson@orson.net.pl>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 s* (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "config.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <termios.h>

const char*CONFIG_DEFAULT_FILE = "quadcontrol.cfg";

static void strip_newline(char* string, int len)
{
    for(int i = 0; i < len; ++i) {
        if(string[i] == '\n') {
            string[i] = 0;
            return;
        }
    }
}

int config_load(const char* filename, struct config* conf)
{
    FILE*file = fopen(filename, "r");
    char buf[128];

    if(!file)
        return 1;

    fgets(conf->serial_device, DEVICE_NAME_LEN, file);
    strip_newline(conf->serial_device, DEVICE_NAME_LEN);

    fgets(buf, sizeof(buf), file);
    int baud_rate = atoi(buf);
    switch(baud_rate) {
        case 9600:   conf->serial_speed = B9600; break;
        case 19200:  conf->serial_speed = B19200; break;
        case 38400:  conf->serial_speed = B38400; break;
        case 57600:  conf->serial_speed = B57600; break;
        case 115200: conf->serial_speed = B115200; break;
        case 230400: conf->serial_speed = B230400; break;
        case 460800: conf->serial_speed = B460800; break;
        default:        // not handled baud rate
            return 2;
    }

    fgets(conf->joystick_device, DEVICE_NAME_LEN, file);
    strip_newline(conf->joystick_device, DEVICE_NAME_LEN);

    fgets(buf, sizeof(buf), file);
    int controls_number = atoi(buf);

    if(controls_number != CONTROLS_NUMBER)
        return 3;

    for(int i = 0; i < CONTROLS_NUMBER; ++i) {
        fgets(buf, sizeof(buf), file);
        conf->joystick_calibration[i].device_offset = atoi(buf);
        fgets(buf, sizeof(buf), file);
        conf->joystick_calibration[i].device_max = atoi(buf);
        fgets(buf, sizeof(buf), file);
        conf->joystick_calibration[i].device_min = atoi(buf);
        fgets(buf, sizeof(buf), file);
        conf->joystick_calibration[i].target_max = atoi(buf);
        fgets(buf, sizeof(buf), file);
        conf->joystick_calibration[i].target_min = atoi(buf);
    }

    fgets(buf, sizeof(buf), file);
    conf->joystick_mapping[THROTTLE_AXIS] = atoi(buf);
    fgets(buf, sizeof(buf), file);
    conf->joystick_mapping[ROLL_AXIS]     = atoi(buf);
    fgets(buf, sizeof(buf), file);
    conf->joystick_mapping[PITCH_AXIS]    = atoi(buf);
    fgets(buf, sizeof(buf), file);
    conf->joystick_mapping[YAW_AXIS]      = atoi(buf);

    config_compute_joy_scales(conf);

    return 0;
}

int config_save(const char* filename, struct config* conf)
{
    FILE*file = fopen(filename, "w");
    char buf[128];

    if(!file)
        return 1;

    fputs(conf->serial_device, file); fputs("\n", file);

    switch(conf->serial_speed) {
        case B9600:   fputs("9600\n", file); break;
        case B19200:  fputs("19200\n", file); break;
        case B38400:  fputs("38400\n", file); break;
        case B57600:  fputs("57600\n", file); break;
        case B115200: fputs("115200\n", file); break;
        case B230400: fputs("230400\n", file); break;
        case B460800: fputs("460800\n", file); break;
        default:        // not handled baud rate
            return 2;
    }

    fputs(conf->joystick_device, file); fputs("\n", file);

    sprintf(buf, "%d\n", CONTROLS_NUMBER);
    fputs(buf, file);

    for(int i = 0; i < CONTROLS_NUMBER; ++i) {
        sprintf(buf, "%d\n", conf->joystick_calibration[i].device_offset);
        fputs(buf, file);
        sprintf(buf, "%d\n", conf->joystick_calibration[i].device_max);
        fputs(buf, file);
        sprintf(buf, "%d\n", conf->joystick_calibration[i].device_min);
        fputs(buf, file);
        sprintf(buf, "%d\n", conf->joystick_calibration[i].target_max);
        fputs(buf, file);
        sprintf(buf, "%d\n", conf->joystick_calibration[i].target_min);
        fputs(buf, file);
    }

    sprintf(buf, "%d\n", conf->joystick_mapping[THROTTLE_AXIS]);
    fputs(buf, file);
    sprintf(buf, "%d\n", conf->joystick_mapping[ROLL_AXIS]);
    fputs(buf, file);
    sprintf(buf, "%d\n", conf->joystick_mapping[PITCH_AXIS]);
    fputs(buf, file);
    sprintf(buf, "%d\n", conf->joystick_mapping[YAW_AXIS]);
    fputs(buf, file);

    return 0;
}

int config_default(struct config* conf)
{
    strcpy(conf->serial_device, "/dev/ttyACM0");
    conf->serial_speed = B115200;

    strcpy(conf->joystick_device, "/dev/input/js0");

    conf->joystick_calibration[THROTTLE_AXIS].device_offset = 0;
    conf->joystick_calibration[THROTTLE_AXIS].device_min = -32768;
    conf->joystick_calibration[THROTTLE_AXIS].device_max = 32767;
    conf->joystick_calibration[THROTTLE_AXIS].target_min = 0;
    conf->joystick_calibration[THROTTLE_AXIS].target_max = 1000;

    conf->joystick_calibration[ROLL_AXIS].device_offset = 0;
    conf->joystick_calibration[ROLL_AXIS].device_min = -32768;
    conf->joystick_calibration[ROLL_AXIS].device_max = 32767;
    conf->joystick_calibration[ROLL_AXIS].target_min = -45;
    conf->joystick_calibration[ROLL_AXIS].target_max = 45;

    conf->joystick_calibration[PITCH_AXIS].device_offset = 0;
    conf->joystick_calibration[PITCH_AXIS].device_min = -32768;
    conf->joystick_calibration[PITCH_AXIS].device_max = 32767;
    conf->joystick_calibration[PITCH_AXIS].target_min = -45;
    conf->joystick_calibration[PITCH_AXIS].target_max = 45;

    conf->joystick_calibration[YAW_AXIS].device_offset = 0;
    conf->joystick_calibration[YAW_AXIS].device_min = -32768;
    conf->joystick_calibration[YAW_AXIS].device_max = 32767;
    conf->joystick_calibration[YAW_AXIS].target_min = -180;
    conf->joystick_calibration[YAW_AXIS].target_max = 180;

    conf->joystick_mapping[THROTTLE_AXIS] = 1;
    conf->joystick_mapping[ROLL_AXIS]     = 3;
    conf->joystick_mapping[PITCH_AXIS]    = 4;
    conf->joystick_mapping[YAW_AXIS]      = 0;

    config_compute_joy_scales(conf);
}

void config_compute_joy_scales(struct config* conf)
{
    for(int i = 0; i < CONTROLS_NUMBER; ++i) {
        struct axis_calibration* axis = &conf->joystick_calibration[i];

        if(axis->device_offset < 0) {
            axis->scale_neg = axis->device_offset;
            axis->scale_pos = 0;
        } else {
            axis->scale_neg = 0;
            axis->scale_pos = axis->device_offset;
        }

        axis->scale_neg = (float) axis->target_min / (axis->device_min - axis->scale_neg);
        axis->scale_pos = (float) axis->target_max / (axis->device_max - axis->scale_pos);

        printf("axis %d scale = %f, %f\n", i, axis->scale_neg, axis->scale_pos);
    }
}
