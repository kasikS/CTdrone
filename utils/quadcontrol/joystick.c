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
#include "debug.h"

void joystick_init(struct joystick* joy, const char* device)
{
    if((joy->fd = open(device, O_RDONLY)) < 0) {
        perror("could not open joystick device");
        exit(1);
    }

    ioctl(joy->fd, JSIOCGNAME(JOYSTICK_NAME_LEN), joy->name);
    ioctl(joy->fd, JSIOCGAXES, &joy->axes);
    ioctl(joy->fd, JSIOCGBUTTONS, &joy->buttons);
    ioctl(joy->fd, JSIOCGVERSION, &joy->version);

    /*MSG("Joystick (%s) has %d axes and %d buttons. Driver version is %d.%d.%d.\n",*/
        /*joy->name, joy->axes, joy->buttons,*/
        /*joy->version >> 16, (joy->version >> 8) & 0xff, joy->version & 0xff);*/

    joy->axis = calloc(joy->axes, sizeof(int));
    joy->button = calloc(joy->buttons, sizeof(char));
    // TODO calibrate
}

void joystick_update(struct joystick* joy)
{
    struct js_event js;

    if(read(joy->fd, &js, sizeof(struct js_event)) != sizeof(struct js_event)) {
        perror("error reading joystick");
        exit(1);
    }

    switch(js.type & ~JS_EVENT_INIT) {
    case JS_EVENT_BUTTON:
            joy->button[js.number] = js.value;
            break;
    case JS_EVENT_AXIS:
            joy->axis[js.number] = js.value;
            break;
    }
}

void joystick_close(struct joystick* joy)
{
    free(joy->axis);
    free(joy->button);
}
