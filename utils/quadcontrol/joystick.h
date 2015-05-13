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

#ifndef JOYSTICK_H
#define JOYSTICK_H

#define JOYSTICK_NAME_LEN 128

struct joystick
{
    int fd;
    unsigned char axes;
    unsigned char buttons;
    int version;
    char name[JOYSTICK_NAME_LEN];
    int *axis;
    int *button;
};

// TODO desc
void joystick_init(struct joystick* joy, const char* device);
void joystick_update(struct joystick* joy);
void joystick_close(struct joystick* joy);

#endif /* JOYSTICK_H */
