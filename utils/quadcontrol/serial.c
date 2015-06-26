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
#include <fcntl.h>
#include <unistd.h>
#include <assert.h>

#include "serial.h"
#include "debug.h"

static int serial_fd = -1;

int serial_init(const char* path, speed_t baud)
{
    struct termios tio;

    memset(&tio, 0, sizeof(tio));
    tio.c_iflag = 0;
    tio.c_oflag = 0;
    tio.c_cflag = CS8|CREAD|CLOCAL;  // 8n1, see termios.h for more information
    tio.c_lflag = 0;
    /*tio.c_cc[VMIN] = 0;*/         // does not matter in nonblocking
    /*tio.c_cc[VTIME] = 2;*/

    serial_fd = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);

    if(serial_fd < 0)
        return 1;

    cfsetospeed(&tio, baud);
    cfsetispeed(&tio, baud);

    tcflush(serial_fd, TCIOFLUSH);
    tcsetattr(serial_fd, TCSANOW, &tio);

    MSG("Serial port %s opened\n", path);

    return 0;
}

void serial_close(void)
{
    assert(serial_fd >= 0);

    close(serial_fd);
}

int serial_write(const char* src, int length)
{
    assert(serial_fd >= 0);
    int ret = write(serial_fd, src, length);
    tcdrain(serial_fd);

    return ret;
}

int serial_read(char* dest, int length)
{
    assert(serial_fd >= 0);

    return read(serial_fd, dest, length);
}
