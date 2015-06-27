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

#include "config.h"
#include "serial.h"
#include "link.h"
#include "joystick.h"
#include "debug.h"

// http://www.modelflight.com.au/blog/how-to-fly-remote-control-quadcopter/

// I do not know why I keep getting warnings about implicit usleep declaration..
extern int usleep(int usec);

void sig_handler(int signo);
static volatile int active = 0;

int main(int argc, char **argv)
{
    struct config conf;
    const char*config_filename;
    struct packet pkt = {0,};
    char buf[32] = {0,};
    int16_t crc;
    int default_cfg = 0;

    if(argc == 2 && !strcmp("--help", argv[1])) {
        puts("Usage: quadcontrol [config_file.cfg]");
        exit(1);
    }

   if(argc == 2)
       config_filename = argv[1];
   else
       config_filename = CONFIG_DEFAULT_FILE;

   if(config_load(config_filename, &conf)) {
        fputs("WARNING: Could not load configuration file. Reverting to defaults.\n", stderr);
        config_default(&conf);
        config_save(CONFIG_DEFAULT_FILE, &conf);
        default_cfg = 1;
   }

   if(signal(SIGINT, sig_handler) == SIG_ERR) {
        fputs("ERROR: Could not set a signal handler.\n", stderr);
        exit(1);
    }

    if(joystick_init(conf.joystick_device)) {
        perror("could not open joystick device");
        exit(1);
    }

    joystick_set_axis_mapping(conf.joystick_mapping);
    joystick_set_axis_calibration(conf.joystick_calibration);

    if(default_cfg) {
        joystick_calibrate(conf.joystick_calibration);
        config_save(CONFIG_DEFAULT_FILE, &conf);
    }

    if(serial_init(conf.serial_device, conf.serial_speed)) {
        perror("could not open serial port");
        exit(1);
    }

    link_init();

    active = 1;
    while(active) {
        // Prepare & send a packet
        pkt.type = PT_JOYSTICK;
        pkt.data.joy.throttle =  joystick_get_control_val(THROTTLE_AXIS);
        pkt.data.joy.yaw      =  joystick_get_control_val(YAW_AXIS);
        pkt.data.joy.pitch    =  joystick_get_control_val(PITCH_AXIS);
        pkt.data.joy.roll     =  joystick_get_control_val(ROLL_AXIS);
        pkt.data.joy.buttons  =  joystick_get_buttons();
        crc = link_crc(&pkt);

        printf("\nthrottle: %6d\tyaw:%6d\tpitch:%6d\troll:%6d\tbuttons:%6d",
                pkt.data.joy.throttle, pkt.data.joy.yaw,
                pkt.data.joy.pitch, pkt.data.joy.roll,
                pkt.data.joy.buttons);

        serial_write((const char*) &pkt, PACKET_TOTAL_SIZE);
        serial_write((const char*) &crc, CRC_SIZE);

        // Show response from the radio
        if(serial_read(buf, sizeof(buf)) > 0) {
            printf("\treceived: %s", buf);
            memset(buf, 0, sizeof(buf));
        }

        fflush(stdout);
        usleep(20000);
    }

    serial_close();
    joystick_close();
    config_save(CONFIG_DEFAULT_FILE, &conf);

    return 1;
}

void sig_handler(int signo)
{
    if(signo == SIGINT)
        active = 0;
}
