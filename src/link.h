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

enum PACKET_TYPE { PT_STATUS = 0x00, PT_JOYSTICK = 0x01, PT_BOOTLOADER = 0x11 };

#define PACKET_TOTAL_SIZE   ((int)sizeof(struct packet))
#define PACKET_DATA_SIZE    ((int)sizeof(((struct packet*)(0))->data)

struct packet
{
    uint8_t type;       // see PACKET_TYPE

    union {             // be sure to have all fields with PACKET_DATA_SIZE length
        struct {
            int16_t throttle;
            int16_t roll;
            int16_t pitch;
            int16_t yaw;
            int8_t buttons;
        } joy;

        char text[9];
    } data;
};

typedef int16_t crc_t;
#define CRC_SIZE 2

// TODO docs
void link_init(void);
crc_t link_crc(const struct packet* pkt);

#endif /* LINK_H */
