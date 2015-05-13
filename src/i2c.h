/*
 * Copyright (C) 2015 Maciej Suminski <orson@orson.net.pl>, Katarzyna Stachyra
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

#ifndef I2C_H
#define I2C_H

#include <inttypes.h>
#include <stdbool.h>

int i2c_init(void);
bool i2cWriteBuffer(uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data);
bool i2cWrite(uint8_t addr_, uint8_t reg, uint8_t data);
bool i2cWriteBit(uint8_t addr_, uint8_t reg_, uint8_t bitNum, uint8_t data);
bool i2cWriteBits(uint8_t addr_, uint8_t reg_, uint8_t bitStart, uint8_t length, uint8_t data);
bool i2cRead(uint8_t addr_, uint8_t reg, uint8_t len, uint8_t* buf);
int8_t i2cReadBit(uint8_t addr_,uint8_t reg_, uint8_t bitNum, uint8_t* buf);
int8_t i2cReadBits(uint8_t addr_, uint8_t reg_, uint8_t bitStart, uint8_t length, uint8_t *data);
uint16_t i2cGetErrorCounter(void);
#endif
