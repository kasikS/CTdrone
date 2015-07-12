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

// Options:
/*#define REQUEST_REPORTS*/
#define SHOW_JOY
#define SHOW_RECV_RAW
#define SHOW_SEND_RAW

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
    char buf[64] = {0,};
    int16_t crc;
    int default_cfg = 0;
    int bad_pkts_cnt = 0;
    int pkt_cnt = 0;

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
        memset(&pkt, 0, PACKET_TOTAL_SIZE);

#ifdef REQUEST_REPORTS
        // Report request
        if(pkt_cnt % 2 == 0) {
            pkt.type = PT_REPORT | RPT_IMU;
            /*printf("\nreport request sent\n");*/
        } else
#endif
        {
            pkt.type = PT_JOYSTICK;
            pkt.data.joy.throttle =  joystick_get_control_val(THROTTLE_AXIS);
            pkt.data.joy.yaw      =  joystick_get_control_val(YAW_AXIS);
            pkt.data.joy.pitch    =  joystick_get_control_val(PITCH_AXIS);
            pkt.data.joy.roll     =  joystick_get_control_val(ROLL_AXIS);
            pkt.data.joy.buttons  =  joystick_get_buttons();

#ifdef SHOW_JOY
            printf("\nthrottle: %6d\tyaw:%6d\tpitch:%6d\troll:%6d\tbuttons:%6d",
                    pkt.data.joy.throttle, pkt.data.joy.yaw,
                    pkt.data.joy.pitch, pkt.data.joy.roll,
                    pkt.data.joy.buttons);
#endif
        }

        crc = link_crc(&pkt);
        serial_write((const uint8_t*) &pkt, PACKET_TOTAL_SIZE);
        serial_write((const uint8_t*) &crc, CRC_SIZE);

#ifdef SHOW_SEND_RAW
        printf("\nsent: ");
        for(int i = 0; i < PACKET_TOTAL_SIZE; ++i)
            printf("%.2x ", ((uint8_t*)(&pkt))[i]);

        printf("\tcrc = %.2x", crc);
#endif

        // Show response from the radio
        int cnt = serial_read(buf, sizeof(buf));
        if(cnt > 0) {

#ifdef SHOW_RECV_RAW
            // HEX version
            printf("\treceived: ");
            for(int i = 0; i < cnt; ++i)
                printf("%.2x ", (uint8_t) buf[i]);

            // ASCII version
            printf("  (");
            for(int i = 0; i < cnt; ++i)
                printf("%c", buf[i]);
            printf(")");
#endif

            if(cnt == 1 && buf[0] == 'E')
                printf("\n!!! DRONE IS NOT RESPONDING !!!\n");

            // Parse reports
            if(buf[0] == 'T' && buf[1] == 'R')
            {
                struct packet* pkt = (struct packet*) &buf[2];
                if(pkt->type & PT_REPORT) {
                    int report_type = pkt->type & ~PT_REPORT;

                    switch(report_type) {
                        case RPT_MOTOR:
                            printf("motors = FL: %d FR:%d BL:%d BR:%d\n",
                                    pkt->data.rpt_motor.fl,
                                    pkt->data.rpt_motor.fr,
                                    pkt->data.rpt_motor.bl,
                                    pkt->data.rpt_motor.br);
                            break;

                        case RPT_IMU:
                            printf("imu = yaw: %d pitch:%d roll:%d\n",
                                    pkt->data.rpt_imu.yaw,
                                    pkt->data.rpt_imu.pitch,
                                    pkt->data.rpt_imu.roll);
                            break;

                        default:
                            printf("invalid report type\n");
                            break;
                    }
                }
            }

            memset(buf, 0, sizeof(buf));
            bad_pkts_cnt = 0;
        } else {
            if(++bad_pkts_cnt == 10) {
                printf("\n!!! CONTROLLER IS NOT RESPONDING !!!\n");
                //link_init();
            }
        }

        ++pkt_cnt;

        fflush(stdout);
        usleep(35000);
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
