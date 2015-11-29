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

/**
 * @brief NRF24L register definitions.
 */

#ifndef NRF24L_REG_H
#define NRF24L_REG_H

///> Possible operation modes
enum MODE {UNKNOWN, PWR_DOWN, STANDBY, TX, RX};

// Commands
// (000A AAAA) Read command and status registers. AAAAA = 5 bit Register Map Address
#define NRF24L_R_REGISTER           0x00

// (001A AAAA) Write command and status registers. AAAAA = 5 bit Register Map Address
// Executable in power down or standby modes only
#define NRF24L_W_REGISTER           0x20

// Read RX-payload: 1 – 32 bytes. A read operation always starts at byte 0.
// Payload is deleted from FIFO after it is read. Used in RX mode.
#define NRF24L_R_RX_PAYLOAD         0x61

// Write TX-payload: 1 – 32 bytes. A write operation always starts at byte 0 used in TX payload.
#define NRF24L_W_TX_PAYLOAD         0xA0

// Flush TX FIFO, used in TX mode.
#define NRF24L_FLUSH_TX             0xE1

// Flush RX FIFO, used in RX mode. Should not be executed during transmission of
// acknowledge, that is, acknowledge package will not be completed.
#define NRF24L_FLUSH_RX             0xE2

// Used for a PTX device. Reuse last transmitted payload. TX payload reuse is active until
// W_TX_PAYLOAD or FLUSH TX is executed. TX payload reuse must not be activated or deactivated
// during package transmission.
#define NRF24L_REUSE_TX_PL          0xE3

// Read RX payload width for the top R_RX_PAYLOAD in the RX FIFO.
// Note: Flush RX FIFO if the read value is larger than 32 bytes.
// (requires bits in FEATURE register)
#define NRF24L_R_RX_PL_WID          0x60

// (1010 1PPP) Used in RX mode. Write Payload to be transmitted together with ACK packet on
// PIPE PPP. (PPP valid in the range from 000 to 101). Maximum three ACK packet payloads can be pending. Payloads with
// same PPP are handled using first in - first out principle. Write payload: 1– 32 bytes. A write
// operation always starts at byte 0.
// (requires bits in FEATURE register)
#define NRF24L_W_ACK_PAYLOAD        0xA8

// Used in TX mode. Disables AUTOACK on this specific packet.
// (requires bits in FEATURE register)
#define NRF24L_W_TX_PAYLOAD_NO_ACK  0xB0

// No Operation. Might be used to read the STATUS register.
#define NRF24L_NOP                  0xFF


// Registers
#define NRF24L_CONFIG           0x00  // Configuration Register
#define NRF24L_EN_AA            0x01  // Enable 'Auto Acknowledgement' Function
#define NRF24L_EN_RXADDR        0x02  // Enabled RX Addresses
#define NRF24L_SETUP_AW         0x03  // Setup of Address Widths
#define NRF24L_SETUP_RETR       0x04  // Setup of Automatic Retransmission
#define NRF24L_RF_CH            0x05  // RF Channel
#define NRF24L_RF_SETUP         0x06  // RF Setup Register
#define NRF24L_STATUS           0x07  // Status Register
#define NRF24L_OBSERVE_TX       0x08  // Transmit observe register
#define NRF24L_RPD              0x09  // Received Power Detector (Carrier Detect)
#define NRF24L_RX_ADDR_P0       0x0A  // Receive address data pipe 0
#define NRF24L_RX_ADDR_P1       0x0B  // Receive address data pipe 1
#define NRF24L_RX_ADDR_P2       0x0C  // Receive address data pipe 2
#define NRF24L_RX_ADDR_P3       0x0D  // Receive address data pipe 3
#define NRF24L_RX_ADDR_P4       0x0E  // Receive address data pipe 4
#define NRF24L_RX_ADDR_P5       0x0F  // Receive address data pipe 5
#define NRF24L_TX_ADDR          0x10  // Transmit address
#define NRF24L_RX_PW_P0         0x11  // Number of bytes in RX payload in data pipe 0
#define NRF24L_RX_PW_P1         0x12  // Number of bytes in RX payload in data pipe 1
#define NRF24L_RX_PW_P2         0x13  // Number of bytes in RX payload in data pipe 2
#define NRF24L_RX_PW_P3         0x14  // Number of bytes in RX payload in data pipe 3
#define NRF24L_RX_PW_P4         0x15  // Number of bytes in RX payload in data pipe 4
#define NRF24L_RX_PW_P5         0x16  // Number of bytes in RX payload in data pipe 5
#define NRF24L_FIFO_STATUS      0x17  // FIFO Status Register
#define NRF24L_DYNPD            0x1C  // Enable dynamic payload length
#define NRF24L_FEATURE          0x1D  // Feature Register
#define NRF24L_REG_MAX          0x1D


// Register bits
// CONFIG (0x00)
#define NRF24L_CONF_MASK_RX_DR      0x40
#define NRF24L_CONF_MASK_TX_DS      0x20
#define NRF24L_CONF_MASK_MAX_RT     0x10
#define NRF24L_CONF_EN_CRC          0x08
#define NRF24L_CONF_CRCO            0x04
#define NRF24L_CONF_PWR_UP          0x02
#define NRF24L_CONF_PRIM_RX         0x01

// EN_AA (0x01)
#define NRF24L_EN_AA_P5             0x20
#define NRF24L_EN_AA_P4             0x10
#define NRF24L_EN_AA_P3             0x08
#define NRF24L_EN_AA_P2             0x04
#define NRF24L_EN_AA_P1             0x02
#define NRF24L_EN_AA_P0             0x01

// EN_RXADDR (0x02)
#define NRF24L_RXADDR_P5            0x20
#define NRF24L_RXADDR_P4            0x10
#define NRF24L_RXADDR_P3            0x08
#define NRF24L_RXADDR_P2            0x04
#define NRF24L_RXADDR_P1            0x02
#define NRF24L_RXADDR_P0            0x01

// SETUP_AW (0x03)
#define NRF24L_AW_3B                0x01
#define NRF24L_AW_4B                0x02
#define NRF24L_AW_5B                0x03

// SETUP_RETR (0x04)
#define NRF24L_SET_RETR_ARD(x)      ((x & 0x0F) << 4)
#define NRF24L_ARD_250_US           0x00
#define NRF24L_ARD_500_US           0x01
#define NRF24L_ARD_750_US           0x02
#define NRF24L_ARD_1000_US          0x03
#define NRF24L_ARD_1250_US          0x04
#define NRF24L_ARD_1500_US          0x05
#define NRF24L_ARD_1750_US          0x06
#define NRF24L_ARD_2000_US          0x07
#define NRF24L_ARD_2250_US          0x08
#define NRF24L_ARD_2500_US          0x09
#define NRF24L_ARD_2750_US          0x0A
#define NRF24L_ARD_3000_US          0x0B
#define NRF24L_ARD_3250_US          0x0C
#define NRF24L_ARD_3500_US          0x0D
#define NRF24L_ARD_3750_US          0x0E
#define NRF24L_ARD_4000_US          0x0F
#define NRF24L_SET_RETR_ARC(x)      ((x & 0x0F))

// RF_CH (0x05)
//#define NRF24L_RF_CH_FREQ(x)        ((x - 2400) / ??)

// RF_SETUP (0x06)
#define NRF24L_RF_SET_CONT_WAVE     0x80
#define NRF24L_RF_SET_RF_DR_LOW     0x20
#define NRF24L_RF_SET_PLL_LOCK      0x10
#define NRF24L_RF_SET_RF_DR_HIGH    0x08
#define NRF24L_RF_SET_PWR_MIN_18DBM 0x00
#define NRF24L_RF_SET_PWR_MIN_12DBM 0x02
#define NRF24L_RF_SET_PWR_MIN_6DBM  0x04
#define NRF24L_RF_SET_PWR_0DBM      0x06
#define NRF24L_RF_SET_RF_2MBPS      (NRF24L_RF_SET_RF_DR_HIGH)
#define NRF24L_RF_SET_RF_1MBPS      0x00
#define NRF24L_RF_SET_RF_250KBPS    (NRF24L_RF_SET_RF_DR_LOW)

// STATUS (0x07)
#define NRF24L_STATUS_RX_DR         0x40
#define NRF24L_STATUS_TX_DS         0x20
#define NRF24L_STATUS_MAX_RT        0x10
#define NRF24L_STATUS_RX_P_NO(x)    ((x & 0x07) << 1)
#define NRF24L_STATUS_TX_FULL       0x01

// OBSERVE_TX (0x08)
//#define NRF24L_OBS_TX_PLOS_CNT(x)
//#define NRF24L_OBS_TX_ARC_CNT(x)

// FIFO_STATUS (0x17)
#define NRF24L_FIFO_STAT_TX_REUSE   0x40
#define NRF24L_FIFO_STAT_TX_FULL    0x20
#define NRF24L_FIFO_STAT_TX_EMPTY   0x10
#define NRF24L_FIFO_STAT_RX_FULL    0x02
#define NRF24L_FIFO_STAT_RX_EMPTY   0x01

// DPL (0x1C)
#define NRF24L_DPL_P5               0x20
#define NRF24L_DPL_P4               0x10
#define NRF24L_DPL_P3               0x08
#define NRF24L_DPL_P2               0x04
#define NRF24L_DPL_P1               0x02
#define NRF24L_DPL_P0               0x01

// FEATURE (0x1D)
#define NRF24L_FEAT_EN_DPL          0x04
#define NRF24L_FEAT_EN_ACK_PAY      0x02
#define NRF24L_FEAT_EN_DYN_ACK      0x01

#endif /* NRF24L_REG_H */
