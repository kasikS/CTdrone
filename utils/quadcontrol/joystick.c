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

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <assert.h>
#include <linux/joystick.h>

#include "joystick.h"
#include "utils.h"
#include "debug.h"

static struct joystick
{
    int fd;
    unsigned char axes;
    unsigned char buttons;
    int version;
    char name[JOYSTICK_NAME_LEN];
    int *axis;
    int *button;
    int *mapping;
    struct axis_calibration *calibration;
} joy;

int joystick_init(const char* device)
{
    if((joy.fd = open(device, O_RDONLY)) < 0) {
        return 1;
    }

    joy.axis = 0;
    joy.button = 0;
    joy.mapping = 0;
    joy.calibration = 0;

    ioctl(joy.fd, JSIOCGNAME(JOYSTICK_NAME_LEN), joy.name);
    ioctl(joy.fd, JSIOCGAXES, &joy.axes);
    ioctl(joy.fd, JSIOCGBUTTONS, &joy.buttons);
    ioctl(joy.fd, JSIOCGVERSION, &joy.version);

    /*MSG("Joystick (%s) has %d axes and %d buttons. Driver version is %d.%d.%d.\n",*/
        /*joy.name, joy.axes, joy.buttons,*/
        /*joy.version >> 16, (joy.version >> 8) & 0xff, joy.version & 0xff);*/

    joy.axis = calloc(joy.axes, sizeof(int));
    joy.button = calloc(joy.buttons, sizeof(char));

    return 0;
}

void joystick_close(void)
{
    free(joy.axis);
    free(joy.button);
}

int joystick_update(void)
{
    struct js_event js;

    if(read(joy.fd, &js, sizeof(struct js_event)) != sizeof(struct js_event)) {
        return 1;
    }

    switch(js.type & ~JS_EVENT_INIT) {
    case JS_EVENT_BUTTON:
            joy.button[js.number] = js.value;
            break;
    case JS_EVENT_AXIS:
            joy.axis[js.number] = js.value;
            break;
    }

    return 0;
}

void joystick_calibrate(struct axis_calibration *calibration)
{
    printf("Joystick calibration not implemented\n");
    return;

    int key;
    int min[CONTROLS_NUMBER], max[CONTROLS_NUMBER];

    // TODO allow to change mapping
    printf("Joystick calibration\n");
    printf("--------------------\n");

    printf("Leave the joystick controls in the neutral position and press Enter.\n");
    do {
        joystick_update();
        printf("update\n");
    } while(!key_pressed(&key));

    for(int i = 0; i < CONTROLS_NUMBER; ++i)
        calibration[i].device_offset = joystick_get_control_raw(i);

    printf("Move joystick controls in circles a few times and press Enter.\n");
    do {
        joystick_update();

        for(int i = 0; i < CONTROLS_NUMBER; ++i) {
            int val = joystick_get_control_raw(i);

            if(val < min[i])
                min[i] = val;

            if(val > max[i])
                max[i] = val;
        }
    } while(!key_pressed(&key));

    for(int i = 0; i < CONTROLS_NUMBER; ++i) {
        // TODO
        calibration[i].scale_neg = 0;
        calibration[i].scale_pos = 0;
    }

    printf("Calibration finished.\n");
}

void joystick_set_axis_mapping(int *mapping)
{
    joy.mapping = mapping;
}

void joystick_set_axis_calibration(struct axis_calibration *calibration)
{
    joy.calibration = calibration;
}

int joystick_get_buttons(void)
{
    if(!joy.buttons)
        return 0;

    int ret = 0;

    for(int i = 0; i < joy.buttons; ++i) {
        if(joy.button[i]) {
            ret |= (1 << i);
        }
    }

    return ret;
}

int joystick_get_control_raw(enum CONTROL control)
{
    assert(control < CONTROLS_NUMBER);
    assert(joy.mapping);
    return joy.axis[joy.mapping[control]];
}

int joystick_get_control_val(enum CONTROL control)
{
    assert(joy.calibration);
    int val = joystick_get_control_raw(control);
    struct axis_calibration* axis = &joy.calibration[control];

    val -= axis->device_offset;
    val *= val > 0 ? axis->scale_pos : axis->scale_neg;

    if(val > axis->target_max)
        val = axis->target_max;
    else if(val < axis->target_min)
        val = axis->target_min;

    return val;
}

void joystick_print_report(void)
{
    // Full joystick report
    if(joy.axes) {
        printf("Axes: ");
        for(int i = 0; i < joy.axes; i++)
            printf("%2d:%6d ", i, joy.axis[i]);
    }

    if(joy.buttons) {
        printf("Buttons: ");
        for(int i = 0; i < joy.buttons; i++)
            printf("%2d:%s ", i, joy.button[i] ? "on " : "off");
    }
}
