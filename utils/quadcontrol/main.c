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
#include <string.h>
#include <signal.h>

#include "serial.h"
#include "link.h"
#include "joystick.h"
#include "debug.h"

static const char *default_joystick = "/dev/input/js0";
static const char *default_serial = "/dev/ttyACM0";

// Axis mapping
// http://www.modelflight.com.au/blog/how-to-fly-remote-control-quadcopter/
// TODO configuration file?
static const int THROTTLE_AXIS = 1;
static const int ROLL_AXIS = 3;        // aileron
static const int PITCH_AXIS = 4;       // elevator
static const int YAW_AXIS = 0;         // rudder

void sig_handler(int signo);
static int active = 1;

int main(int argc, char **argv)
{
    struct joystick joy;
    struct packet pkt = {0,};
    char buf[32] = {0,};
    int16_t crc;

    if(argc >= 2 && !strcmp("--help", argv[1])) {
        puts("Usage: quadcontrol [joystick device] [serial device]");
        puts("e.g. quadcontrol /dev/input/js0 /dev/ttyUSB0");
        exit(1);
    }

   if(signal(SIGINT, sig_handler) == SIG_ERR) {
        fputs("An error occurred while setting a signal handler.\n", stderr);
        exit(1);
    }

    /*joystick_init(&joy, argc >= 2 ? argv[1] : default_joystick);*/
    /*serial_init(argc >= 3 ? argv[2] : default_serial, B115200);*/
    joystick_init(&joy, default_joystick);
    serial_init(default_serial, B115200);
    link_init();

    while(active) {
        joystick_update(&joy);

#if 0
        // Full joystick report
        if (axes) {
            printf("Axes: ");
            for (i = 0; i < axes; i++)
                printf("%2d:%6d ", i, axis[i]);
        }

        if (buttons) {
            printf("Buttons: ");
            for (i = 0; i < buttons; i++)
                printf("%2d:%s ", i, button[i] ? "on " : "off");
        }
#else
        printf("\rthrottle: %6d \t yaw:%6d \t pitch:%6d \t roll:%6d",
                joy.axis[THROTTLE_AXIS], joy.axis[YAW_AXIS], joy.axis[PITCH_AXIS], joy.axis[ROLL_AXIS]);

        // TODO sensitivity parameter
        // Prepare & send a packet
        pkt.type = PT_JOYSTICK;
        pkt.data.joy.throttle = -joy.axis[THROTTLE_AXIS];
        pkt.data.joy.yaw      =  joy.axis[YAW_AXIS];
        pkt.data.joy.pitch    =  joy.axis[PITCH_AXIS];
        pkt.data.joy.roll     =  joy.axis[ROLL_AXIS];
        crc = link_crc(&pkt);
        serial_write((const char*) &pkt, PACKET_TOTAL_SIZE);
        serial_write((const char*) &crc, CRC_SIZE);

        // Show response from the radio
        if(serial_read(buf, sizeof(buf)) > 0) {
            printf("\treceived: %s", buf);
            memset(buf, 0, sizeof(buf));
        }
#endif
        fflush(stdout);
    }


    serial_close();
    joystick_close(&joy);

    return 1;
}

void sig_handler(int signo)
{
    if(signo == SIGINT)
        active = 0;
}
