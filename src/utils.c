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

#include <ctype.h>

float ct_atof(char *s)
{
    float a = 0.0;
    int e = 0;
    int c;

    while ((c = *s++) != '\0' && isdigit(c)) {
        a = a*10.0 + (c - '0');
    }

    if (c == '.') {
        while ((c = *s++) != '\0' && isdigit(c)) {
            a = a*10.0 + (c - '0');
            e = e-1;
        }
    }

    if (c == 'e' || c == 'E') {
        int sign = 1;
        int i = 0;

        c = *s++;

        if (c == '+') {
            c = *s++;
        } else if (c == '-') {
            c = *s++;
            sign = -1;
        }

        while (isdigit(c)) {
            i = i*10 + (c - '0');
            c = *s++;
        }

        e += i*sign;
    }

    while (e > 0) {
        a *= 10.0;
        e--;
    }

    while (e < 0) {
        a *= 0.1;
        e++;
    }
    return a;
}



