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

#include "stm32f4xx_conf.h"

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include "nrf24l.h"
#include "nrf24l_reg.h"
#include "link.h"

///> Size of tx/rx queues (in bytes)
#define NRF24L_QUEUE_SIZE   64

///> Maximum number of ticks to wait when handling queues
#define NRF24L_TICKS_WAIT   32

// Chip select control
static inline void nrf24l_cs_enable() { GPIO_WriteBit(NRF24L_CS_PORT, NRF24L_CS_PIN, 0); }
static inline void nrf24l_cs_disable() { GPIO_WriteBit(NRF24L_CS_PORT, NRF24L_CS_PIN, 1); }

// RF enable
static inline void nrf24l_ce_enable() { GPIO_WriteBit(NRF24L_CE_PORT, NRF24L_CE_PIN, 1); }
static inline void nrf24l_ce_disable() { GPIO_WriteBit(NRF24L_CE_PORT, NRF24L_CE_PIN, 0); }

static void nrf24l_irq_handler_task(void *parameter);
static void nrf24l_transmitter_task(void *parameter);

static xQueueHandle nrf24l_tx_queue,          // data to be sent
                    nrf24l_rx_queue;          // received data

static SemaphoreHandle_t nrf24l_irq_sem,      // starts irq handler
                         nrf24l_tx_sem,       // enables transmitter
                         nrf24l_tx_queue_mtx; // prevents mixing multibyte packets

static volatile enum MODE nrf24l_mode = PWR_DOWN;

// Debug info TODO to be removed
#include "serial.h"
/*void int16_to_hex(uint16_t, const char*);*/
int serial_puts(const char*);
static uint16_t display_stat(void) {
    char buf[7] = {0x00, 0x00, 0x00, 0x00, '\r', '\n', 0x00};
    int16_t status;

    status = nrf24l_get_status();
    /*int16_to_hex(status, buf);*/
    serial_puts(buf);

    return status;
}
// TODO remove finish or move to utils.c?

// Raw communication functions
inline static void spi_tx(uint8_t data)
{
    // NOTE: you have to handle chipselect!
    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(SPI1, data);
}

inline static uint8_t spi_tx_rx(uint8_t data)
{
    // NOTE: you have to handle chipselect!
    spi_tx(data);
    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
    return SPI_I2S_ReceiveData(SPI1);
}

static void nrf24l_raw_multi(const uint8_t *tx, uint8_t *rx, uint8_t len)
{
    int i;
    nrf24l_cs_enable();

    if(rx) {
        for(i = 0; i < len; ++i) *rx++ = spi_tx_rx(*tx++);
    } else {
        for(i = 0; i < len; ++i) spi_tx(*tx++);
    }

    nrf24l_cs_disable();
}

static uint16_t nrf24l_raw_16b(uint16_t data)
{
    uint16_t read;

    nrf24l_cs_enable();
    read = spi_tx_rx((uint8_t)(data >> 8));
    read <<= 8;
    read |= spi_tx_rx((uint8_t)(data & 0xff));
    nrf24l_cs_disable();

    return read;
}

static uint8_t nrf24l_raw_8b(uint8_t data)
{
    uint8_t read;

    nrf24l_cs_enable();
    read = spi_tx_rx(data);
    nrf24l_cs_disable();

    return read;
}

static uint8_t nrf24l_read_reg(uint8_t reg)
{
    assert_param(reg <= NRF24L_REG_MAX);
    return (uint8_t) nrf24l_raw_16b((NRF24L_R_REGISTER | reg) << 8);
}

static void nrf24l_write_reg(uint8_t reg, uint8_t val)
{
    assert_param(reg <= NRF24L_REG_MAX);
    nrf24l_raw_16b(((NRF24L_W_REGISTER | reg) << 8) | val);
}

// Higher-level functions
static void nrf24l_flush_rx_fifo(void) { nrf24l_raw_8b(NRF24L_FLUSH_RX); }
static void nrf24l_flush_tx_fifo(void) { nrf24l_raw_8b(NRF24L_FLUSH_TX); }

static void nrf24l_clear_irq(void)
{
    nrf24l_write_reg(NRF24L_STATUS, NRF24L_STATUS_RX_DR |
                                    NRF24L_STATUS_TX_DS |
                                    NRF24L_STATUS_MAX_RT);
}

// TODO add in the header?
void nrf24l_set_channel(uint8_t channel)
{
    assert_param(channel <= 125);

    // TODO assert, verify the parameter
    nrf24l_write_reg(NRF24L_RF_CH, channel);
}

static void nrf24l_set_mode(enum MODE mode)
{
    const int default_conf = NRF24L_CONF_EN_CRC | NRF24L_CONF_CRCO;

    // TODO power down -> standby 1.5ms (internal osc), 150 us( external clk)
    // standby->tx/rx (130us) p.22

    // TODO enable/disable related tasks
    if(nrf24l_mode == mode)
        return;

    switch(mode)
    {
        case PWR_DOWN:
            nrf24l_ce_disable();
            nrf24l_write_reg(NRF24L_CONFIG, default_conf);
            break;

        case STANDBY:
            nrf24l_ce_disable();
            nrf24l_write_reg(NRF24L_CONFIG, default_conf | NRF24L_CONF_PWR_UP);
            if(nrf24l_mode == PWR_DOWN) {     // TODO compare with tpd2stby (4.5ms in the worst case)
                for(int i = 0; i < 60000; ++i) __asm("nop");
            }
            break;

        case TX:
            nrf24l_clear_irq();
            nrf24l_write_reg(NRF24L_CONFIG, default_conf | NRF24L_CONF_PWR_UP);
            /*nrf24l_ce_enable();*/ // TODO?
            break;

        case RX:
            nrf24l_flush_rx_fifo();
            nrf24l_clear_irq();
            nrf24l_ce_disable();
            nrf24l_write_reg(NRF24L_CONFIG, default_conf | NRF24L_CONF_PWR_UP |
                             NRF24L_CONF_PRIM_RX);
            nrf24l_ce_enable();
            break;
    }

    nrf24l_mode = mode;
}

int nrf24l_init(void)
{
    GPIO_InitTypeDef gpio_conf;
    SPI_InitTypeDef spi_conf;
    EXTI_InitTypeDef exti_conf;
    NVIC_InitTypeDef nvic_conf;

    // Clocks
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    // GPIO configuration
    // MISO, MOSI, SCK
    gpio_conf.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    gpio_conf.GPIO_Mode = GPIO_Mode_AF;
    gpio_conf.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_conf.GPIO_OType = GPIO_OType_PP;
    gpio_conf.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &gpio_conf);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_SPI1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_SPI1);

    // CS
    gpio_conf.GPIO_Mode = GPIO_Mode_OUT;
    gpio_conf.GPIO_Pin = NRF24L_CS_PIN;
    GPIO_Init(NRF24L_CS_PORT, &gpio_conf);

    // CE
    gpio_conf.GPIO_Mode = GPIO_Mode_OUT;
    gpio_conf.GPIO_Pin = NRF24L_CE_PIN;
    GPIO_Init(NRF24L_CE_PORT, &gpio_conf);

    // INT
    gpio_conf.GPIO_Mode = GPIO_Mode_IN;
    gpio_conf.GPIO_PuPd = GPIO_PuPd_UP;
    gpio_conf.GPIO_Pin = NRF24L_IRQ_PIN;
    GPIO_Init(NRF24L_IRQ_PORT, &gpio_conf);

    // Initially disable CS & RF part
    nrf24l_cs_disable();
    nrf24l_ce_disable();

    // Configure IRQ pin interrupt
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource12);

    exti_conf.EXTI_Line = EXTI_Line12;
    exti_conf.EXTI_Mode = EXTI_Mode_Interrupt;
    exti_conf.EXTI_Trigger = EXTI_Trigger_Falling;
    exti_conf.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti_conf);

    nvic_conf.NVIC_IRQChannel = EXTI15_10_IRQn;
    nvic_conf.NVIC_IRQChannelPreemptionPriority = 5;
    nvic_conf.NVIC_IRQChannelSubPriority = 0;
    nvic_conf.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_conf);

    // SPI configuration
    spi_conf.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spi_conf.SPI_Mode = SPI_Mode_Master;
    spi_conf.SPI_DataSize = SPI_DataSize_8b;
    spi_conf.SPI_CPOL = SPI_CPOL_Low;
    spi_conf.SPI_CPHA = SPI_CPHA_1Edge;
    spi_conf.SPI_NSS = SPI_NSS_Soft;
    spi_conf.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16; // TODO max 8 MHz => prescaler 2
    spi_conf.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_Init(SPI1, &spi_conf);

    SPI_Cmd(SPI1, ENABLE);

    // FreeRTOS stuff
    nrf24l_irq_sem = xSemaphoreCreateBinary();
    if(nrf24l_irq_sem == 0)
        return pdFALSE;

    nrf24l_tx_sem = xSemaphoreCreateCounting(NRF24L_QUEUE_SIZE, 0);
    if(nrf24l_tx_sem == 0)
        return pdFALSE;

    nrf24l_tx_queue_mtx = xSemaphoreCreateMutex();
    if(nrf24l_tx_queue_mtx == 0)
        return pdFALSE;

    nrf24l_tx_queue = xQueueCreate(NRF24L_QUEUE_SIZE, sizeof(uint8_t));
    if(nrf24l_tx_queue == 0)
        return pdFALSE;

    nrf24l_rx_queue = xQueueCreate(NRF24L_QUEUE_SIZE, sizeof(uint8_t));
    if(nrf24l_rx_queue == 0)
        return pdFALSE;

    if(!xTaskCreate(nrf24l_irq_handler_task, NULL, configMINIMAL_STACK_SIZE, NULL,
                    tskIDLE_PRIORITY, NULL))
        return pdFALSE;

    if(!xTaskCreate(nrf24l_transmitter_task, NULL, configMINIMAL_STACK_SIZE, NULL,
                    tskIDLE_PRIORITY, NULL))
        return pdFALSE;



    // TODO temporarily disable autoack
    /*nrf24l_write_reg(NRF24L_EN_AA, 0);*/

    // Enable dynamic payload length
    /*nrf24l_write_reg(NRF24L_FEATURE, NRF24L_FEAT_EN_DPL);*/ // TODO

    // Retransmit delay 1500us (permits all ACK payload sizes) // TODO decrease if ACK payload is not used
    /*nrf24l_write_reg(NRF24L_SETUP_RETR, NRF24L_SET_RETR_ARD(ARD_1500_US));*/





    // TODO software reset
    nrf24l_set_channel(10);

    // Set pipelines size
    nrf24l_write_reg(NRF24L_RX_PW_P0, PACKET_TOTAL_SIZE);
    nrf24l_write_reg(NRF24L_RX_PW_P1, PACKET_TOTAL_SIZE);
    nrf24l_write_reg(NRF24L_RX_PW_P2, PACKET_TOTAL_SIZE);
    nrf24l_write_reg(NRF24L_RX_PW_P3, PACKET_TOTAL_SIZE);
    nrf24l_write_reg(NRF24L_RX_PW_P4, PACKET_TOTAL_SIZE);
    nrf24l_write_reg(NRF24L_RX_PW_P5, PACKET_TOTAL_SIZE);

    // Set data rate and output power // TODO increase power & data rate?
    nrf24l_write_reg(NRF24L_RF_SETUP, NRF24L_RF_SET_PWR_MIN_6DBM | NRF24L_RF_SET_RF_1MBPS);

    nrf24l_write_reg(NRF24L_CONFIG, NRF24L_CONF_EN_CRC | NRF24L_CONF_CRCO);

    /* Enable auto-acknowledgment for all pipes */
    nrf24l_write_reg(NRF24L_EN_AA, 0x3F);
    /* Enable RX addresses */
    nrf24l_write_reg(NRF24L_EN_RXADDR, 0x3F);
    /* Auto retransmit delay: 1000 (4x250) us and Up to 15 retransmit trials */
    nrf24l_write_reg(NRF24L_SETUP_RETR, 0x4F);
    /* Dynamic length configurations: No dynamic length */
    nrf24l_write_reg(NRF24L_DYNPD, NRF24L_DPL_P0 | NRF24L_DPL_P1 | NRF24L_DPL_P2 | NRF24L_DPL_P3 | NRF24L_DPL_P4 | NRF24L_DPL_P5);

    nrf24l_flush_tx_fifo();
    nrf24l_flush_rx_fifo();

    nrf24l_clear_irq();

    /*nrf24l_set_mode(STANDBY);*/   // TODO
    //set mode rx?
    nrf24l_set_mode(RX);   // TODO

    return pdTRUE;
}




void nrf24l_tx_direct(uint8_t *data) {
    uint8_t cmd[PACKET_TOTAL_SIZE + 1] = { NRF24L_W_TX_PAYLOAD, 0, };
    // TODO lame?
    for(uint8_t i = 0; i < PACKET_TOTAL_SIZE; ++i)
        cmd[i+1] = data[i];

    nrf24l_ce_disable(); // TODO ??
    nrf24l_set_mode(TX); // TODO ??
    nrf24l_flush_tx_fifo();
    nrf24l_raw_multi(cmd, NULL, PACKET_TOTAL_SIZE + 1);
    nrf24l_ce_enable();
}

uint8_t nrf24l_rx_data_ready(void)
{
    // slightly modified get_status(), so we could get both fifo & general
    // status in the same request
    const uint8_t cmd[2] = { NRF24L_R_REGISTER | NRF24L_FIFO_STATUS, 0 };
    uint8_t read[2];

    nrf24l_raw_multi(cmd, read, 2);

    if(read[0] & NRF24L_STATUS_RX_DR)               // general status
        return 1;

    if(!(read[1] & NRF24L_FIFO_STAT_RX_EMPTY))      // fifo status
        return 1;

    return 0;
}

#if 0
// TODO remove
void nrf24l_rx_direct(uint8_t* data)
{
    nrf24l_read_fifo(data);
    // clear RX_DR flag
    nrf24l_write_reg(NRF24L_STATUS, NRF24L_STATUS_RX_DR);
}
#endif




void nrf24l_write(char data)
{
    // TODO give semaphore?
    xQueueSend(nrf24l_tx_queue, (void*) &data, NRF24L_TICKS_WAIT);
}

void nrf24l_puts(const char *buf)
{
    while(*buf) nrf24l_write(*buf++);
}

int nrf24l_getc(char *c)
{
    nrf24l_set_mode(RX);
    return xQueueReceive(nrf24l_rx_queue, (void*) c, NRF24L_TICKS_WAIT);
}

static void nrf24l_read_fifo(uint8_t *data)
{
    const uint8_t cmd[PACKET_TOTAL_SIZE + 1] = { NRF24L_R_RX_PAYLOAD, 0, };
    nrf24l_raw_multi(cmd, data, PACKET_TOTAL_SIZE + 1);
}

static uint8_t nrf24l_get_fifo_status(void)
{
    return nrf24l_read_reg(NRF24L_FIFO_STATUS);
}

uint8_t nrf24l_get_status(void)
{
    // TODO use NOP - it is only one byte
    return nrf24l_read_reg(NRF24L_STATUS);
}

uint8_t nrf24l_get_lost_packets(void)
{
    return (nrf24l_read_reg(NRF24L_OBSERVE_TX) >> 4);
}

uint8_t nrf24l_get_retransmitted_packets(void)
{
    return (nrf24l_read_reg(NRF24L_OBSERVE_TX) & 0x0f);
}

uint8_t nrf24l_get_rx_power(void)
{
    return nrf24l_read_reg(NRF24L_RPD);
}

// TODO
/*
void nrf24l_set_power(void)
{
}
nrf24l_write_ack_payload
nrf24l_read_ack_payload
set_rx_addr
set_tx_addr
*/

int nrf24l_irq(void)
{
    portBASE_TYPE xHigherPriorityTaskWoken;
    xSemaphoreGiveFromISR(nrf24l_irq_sem, &xHigherPriorityTaskWoken);
    return xHigherPriorityTaskWoken;
}

static void nrf24l_irq_handler_task(void *parameter)
{
    (void) parameter;   // suppress warning
    uint8_t status, i, irq_src;
    uint8_t rx_buffer[PACKET_TOTAL_SIZE + 1];

    while(1) {
        if(xSemaphoreTake(nrf24l_irq_sem, portMAX_DELAY) == pdTRUE) {
            GPIO_ToggleBits(GPIOA, GPIO_Pin_5);
            status = nrf24l_get_status();
            irq_src = status & (NRF24L_STATUS_RX_DR | NRF24L_STATUS_TX_DS | NRF24L_STATUS_MAX_RT);

            if(status & NRF24L_STATUS_RX_DR) {
                serial_putc('R');

                // TODO should whole fifo be read at once?
                do {
                    nrf24l_read_fifo(rx_buffer);

                    // rx_buffer[0] contains status, start with rx_buffer[1]
                    for(i = 1; i < PACKET_TOTAL_SIZE + 1; ++i)
                        xQueueSend(nrf24l_rx_queue, &rx_buffer[i], NRF24L_TICKS_WAIT);

                    status = nrf24l_get_fifo_status();
                }
                while(!(status & NRF24L_FIFO_STAT_RX_EMPTY));
            }

            if(status & NRF24L_STATUS_TX_DS) {
                serial_putc('T');
                /*nrf24l_set_mode(RX);*/

                /*if(!(status & NRF24L_STATUS_TX_FULL))*/
                    /*xSemaphoreGive(nrf24l_tx_sem);*/

                /*status = nrf24l_read_reg(NRF24L_FIFO_STATUS);*/
                /*if(status & NRF24L_FIFO_STAT_TX_EMPTY)*/
                    /*nrf24l_set_mode(RX);*/
                /*else if(status & NRF24L_FIFO_STAT_TX_FULL)*/
                    /*xSemaphoreGive(nrf24l_tx_sem);*/
            }

#if 0
            if(status & NRF24L_STATUS_MAX_RT) {
                // TODO
            }
#endif

            // Clear IRQ
            nrf24l_write_reg(NRF24L_STATUS, irq_src);
        }
    }
}

static void nrf24l_transmitter_task(void *parameter)
{
    (void) parameter;   // suppress warning
    uint8_t data = 0;
    /*uint8_t tx_buf[PACKET_TOTAL_SIZE + 1] = {0,};*/

    while(1) {
        // Wait for some space in the register
        /*if(xSemaphoreTake(nrf24l_tx_sem, NRF24L_TICKS_WAIT) == pdTRUE) {*/

            // Wait for data to be sent
            /*while(uxQueueMessagesWaiting(nrf24l_tx_queue) == 0);*/
            /*while(nrf24l_mode != TX);*/

            // TODO check if fifo is not full
            /*nrf24l_get_status();*/

            uint8_t count = 0;
            while(xQueueReceive(nrf24l_tx_queue, (void*) &data, 0) && count < PACKET_TOTAL_SIZE) {
                GPIO_ToggleBits(GPIOA, GPIO_Pin_5);      // blink, blink
                /*nrf24l_write_reg(NRF24L_W_TX_PAYLOAD, data);*/
                ++count;
            }

            if(count > 0) {
                // Transmit packet
                nrf24l_set_mode(TX);
                /*serial_puts("cnt");*/     //TODO remove
                /*serial_putc(count);*/
            }
        /*}*/
    }
}
