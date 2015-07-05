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

#ifndef LINK_H
#define LINK_H

#include <stdint.h>

enum PACKET_TYPE {
    PT_STATUS       = 0x00,
    PT_JOYSTICK     = 0x01,
    PT_REPORT       = 0x80,
    PT_BOOTLOADER   = 0x11
};

// or-combined with the packet type
enum REPORT_TYPE {
    RPT_MOTOR       = 0x01,
    RPT_IMU         = 0x02
};

#define PACKET_TOTAL_SIZE   ((int)sizeof(struct packet))
#define PACKET_DATA_SIZE    ((int)sizeof(((struct packet*)(0))->data))

struct packet
{
    uint8_t type;       // see PACKET_TYPE

    union {             // be sure to have all fields with PACKET_DATA_SIZE length
        struct {
            int16_t throttle;
            int16_t yaw;
            int16_t pitch;
            int16_t roll;
            uint8_t buttons;
        } __attribute__((packed)) joy;

        struct {
            int16_t fl;
            int16_t fr;
            int16_t bl;
            int16_t br;
            uint8_t __padding;
        } __attribute__((packed)) rpt_motor;

        struct {
            int16_t yaw;
            int16_t pitch;
            int16_t roll;
            uint8_t __padding[3];
        } __attribute__((packed)) rpt_imu;

        uint8_t text[9];
    } data;
} __attribute__((packed));

typedef uint8_t crc_t;
#define CRC_SIZE ((int)sizeof(crc_t))

// TODO docs
void link_init(void);
crc_t link_crc(const struct packet* pkt);

#endif /* LINK_H */
