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

#ifndef _NRF24L_H
#define _NRF24L_H

#include <stdint.h>
#include "stm32f4xx.h"

// Connections;
// PB3 SCK
// PB4 MISO
// PB5 MOSI
// PC10 CS
// PC12 IRQ
// PC9 CE (enable RX/TX)
#define NRF24L_CS_PORT      GPIOC
#define NRF24L_CS_PIN       GPIO_Pin_10

#define NRF24L_IRQ_PORT     GPIOC
#define NRF24L_IRQ_PIN      GPIO_Pin_12

#define NRF24L_CE_PORT      GPIOC
#define NRF24L_CE_PIN       GPIO_Pin_9

/**
 * @brief Initialises the NRF24L module. After initialisation, the module
 * stays in RX mode.
 * @return false in case of error.
 */
int nrf24l_init(void);

/**
 * @brief Sends a byte through the radio link.
 * @param data is the byte to be sent.
 */
int nrf24l_putc(char data);

/**
 * @brief Sends an array of bytes through the radio link.
 * @param buf is the array to be sent.
 */
int nrf24l_puts(const char *buf);

/**
 * Sends a specified number of characters through the radio link (non-blocking).
 * @param string is the data to be sent.
 * @param len is number of characters to be sent.
 * @return false in case of error.
 */
int nrf24l_write(const char *string, int len);

/**
 * @brief Reads a character from the RX buffer (non-blocking).
 * @param c is a pointer to the variable where the read character will be saved.
 * If there is nothing in the buffer, it will not be modified.
 * @return true if a character was received.
 */
int nrf24l_getc(char *c);

/**
 * @brief Checks the module status. See nrf24l_STATUS_xxx defines for details.
 * @return Status word.
 */
uint8_t nrf24l_get_status(void);

/**
 * @brief Sets channel (i.e. radio frequency) used for communication.
 * @param channel is the new channel number. Valid values are in range 0-125.
 */
void nrf24l_set_channel(uint8_t channel);

//void nrf24l_set_power(void);

/**
 * @brief Requests interrupt handler execution. It should be called in
 * nIRQ falling edge interrupt handler.
 */
int nrf24l_irq(void);

#endif /* _NRF24L_H */
