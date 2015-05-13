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
// PC14 CE (RX/TX)
#define NRF24L_CS_PORT      GPIOC
#define NRF24L_CS_PIN       GPIO_Pin_10

#define NRF24L_IRQ_PORT     GPIOC
#define NRF24L_IRQ_PIN      GPIO_Pin_12

#define NRF24L_CE_PORT      GPIOC
#define NRF24L_CE_PIN       GPIO_Pin_14

/**
 * @brief Initialises the NRF24L module. After initialisation, the module
 * stays in IDLE mode. Use nrf24l_set_mode() to change to either TX or RX mode.
 * @return false in case of error.
 */
int nrf24l_init(void);

/**
 * @brief Changes the current mode to TX, RX or IDLE.
 * @param mode is the requested mode.
 */
//void nrf24l_set_mode(enum MODE mode);

/**
 * @brief Sends a byte through the radio link. Requires TX mode.
 * @param data is the byte to be sent.
 */
void nrf24l_write(char data);

/**
 * @brief Sends an array of bytes through the radio link. Requires TX mode.
 * @param buf is the array to be sent.
 */
void nrf24l_puts(const char *buf);

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

//void nrf24l_set_freq(void);
//void nrf24l_set_power(void);

/**
 * @brief Requests interrupt handler execution. It should be called in
 * nIRQ falling edge interrupt handler.
 */
int nrf24l_irq(void);

#endif /* _NRF24L_H */
