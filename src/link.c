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
#include <unistd.h>
#include <assert.h>

#include "link.h"
#include "serial.h"
#include "debug.h"

void link_init(void)
{
    struct packet pkt;
    crc_t crc;

    const char zero = 0x00;
    char* rx_ptr = (char*) &pkt;
    int rx_len = 0;
    int read_len;
    int errors = 0;

    memset(&pkt, 0, sizeof(struct packet));

    // Try to communicate
    for(int i = 0; i < 64; ++i) {
        serial_write(&zero, 1);
        usleep(10000);

        if((read_len = serial_read(rx_ptr, PACKET_TOTAL_SIZE - rx_len)) > 0) {
            rx_len += read_len;
            rx_ptr += read_len;

            if(rx_len >= PACKET_TOTAL_SIZE)
                break;
        }
    }

    if(serial_read((char*) &crc, CRC_SIZE) != 2)       // TODO lame
        ++errors;

    // Verify the response
    if(pkt.type != PT_STATUS) {
        fprintf(stderr, "invalid response type\n");
        ++errors;
    }

    if(strncmp(pkt.data.text, "CTDrone", 7)) {
        fprintf(stderr, "invalid software version: %.8s\n", pkt.data.text);
        ++errors;
    }

    if(crc != link_crc(&pkt)) {
        fprintf(stderr, "invalid crc\n");
        ++errors;
    }

    if(errors) {
        serial_close();
        exit(1);
    }

    MSG("Link initialized, got response: %.8s\n", pkt.data.text);
}
