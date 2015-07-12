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

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include "stm32f4xx_conf.h"
#include "i2c.h"
#include "stm32f4xx_i2c.h"

#define I2C_DEFAULT_TIMEOUT 30000
//#define I2C_DEFAULT_TIMEOUT 35

#define I2C1_SCL_PIN GPIO_Pin_8
#define I2C1_SCL_PIN_SOURCE GPIO_PinSource8
#define I2C1_SDA_PIN GPIO_Pin_9
#define I2C1_SDA_PIN_SOURCE GPIO_PinSource9

static void i2c_er_handler_task(void *parameter);
static void i2c_ev_handler_task(void *parameter);
static void i2cUnstick(I2C_TypeDef *I2C);

/*static xSemaphoreHandle i2c_ev_irq, i2c_er_irq;*/

void I2C1_ER_IRQHandler(void)
{
#if 0
	portBASE_TYPE xHigherPriorityTaskWoken;
	 // Reset all the error bits to clear the interrupt
	I2C1->SR1 &= ~(I2C_SR1_OVR  |
				   I2C_SR1_AF   |
				   I2C_SR1_ARLO |
				   I2C_SR1_BERR );
	xSemaphoreGiveFromISR(i2c_ev_irq, &xHigherPriorityTaskWoken);
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
#endif
    i2c_er_handler_task(0);
    portEND_SWITCHING_ISR(pdTRUE);
}


void I2C1_EV_IRQHandler(void)
{
#if 0
	portBASE_TYPE xHigherPriorityTaskWoken;
	xSemaphoreGiveFromISR(i2c_er_irq, &xHigherPriorityTaskWoken);
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
#endif
    i2c_ev_handler_task(0);
    portEND_SWITCHING_ISR(pdTRUE);
}


static volatile uint16_t i2cErrorCount = 0;
static volatile uint16_t i2c1ErrorCount = 0;
static volatile uint16_t i2c2ErrorCount = 0;

static volatile bool error = false;
static volatile bool busy;


static volatile uint8_t addr;
static volatile uint8_t reg;
static volatile uint8_t bytes;
static volatile uint8_t writing;
static volatile uint8_t reading;
static volatile uint8_t* write_p;
static volatile uint8_t* read_p;


static void i2c_er_handler_task(void *parameters)
{
	// while(1) {
 //   		if(xSemaphoreTake(i2c_er_irq, 0)) {
			volatile uint32_t SR1Register;

			SR1Register = I2C1->SR1;                                              // Read the I2Cx status register

			if (SR1Register & (I2C_SR1_AF   |
							   I2C_SR1_ARLO |
							   I2C_SR1_BERR ))                                    // If AF, BERR or ARLO, abandon the current job and commence new if there are jobs
			{
				I2C_ITConfig(I2C1, I2C_IT_BUF, DISABLE);                          // Disable the RXNE/TXE interrupt - prevent the ISR tailchaining onto the ER (hopefully)
				if (!(SR1Register & I2C_SR1_ARLO) && !(I2C1->CR1 & I2C_CR1_STOP)) // If we dont have an ARLO error, ensure sending of a stop
				{
					if (I2C1->CR1 & I2C_CR1_START)                                // We are currently trying to send a start, this is very bad as start,stop will hang the peripheral
					{
						while (I2C1->CR1 & I2C_CR1_START);                        // Wait for any start to finish sending
						I2C_GenerateSTOP(I2C1, ENABLE);                           // Send stop to finalise bus transaction
						while (I2C1->CR1 & I2C_CR1_STOP);                         // Wait for stop to finish sending
						i2c_init();                                            // Reset and configure the hardware
					} else
					{
						I2C_GenerateSTOP(I2C1, ENABLE);                           // Stop to free up the bus
						I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_ERR, DISABLE);     // Disable EVT and ERR interrupts while bus inactive
					}
				}
			}

			busy = 0;
   	// 	}
   	// }
}


int i2c_init()
{
    // TODO error handling
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef  I2C_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

    ///////////////////////////////////

    // I2Cx = I2C;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,  ENABLE);

	// i2cUnstick(I2C1);                                         // Clock out stuff to make sure slaves arent stuck

	GPIO_StructInit(&GPIO_InitStructure);
	I2C_StructInit(&I2C_InitStructure);

	// Init pins
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9;;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	//GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);

	// Init I2C
	I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_ERR, DISABLE);

	I2C_DeInit(I2C1);

	I2C_StructInit(&I2C_InitStructure);

	I2C_InitStructure.I2C_ClockSpeed          = 400000;
	I2C_InitStructure.I2C_Mode                = I2C_Mode_I2C; //
	I2C_InitStructure.I2C_DutyCycle           = I2C_DutyCycle_2; //
	I2C_InitStructure.I2C_OwnAddress1         = 0; //
	I2C_InitStructure.I2C_Ack                 = I2C_Ack_Disable; //
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; //

	I2C_Init(I2C1, &I2C_InitStructure);

	I2C_Cmd(I2C1, ENABLE);

	// I2C ER Interrupt
	NVIC_InitStructure.NVIC_IRQChannel                   = I2C1_ER_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;

	NVIC_Init(&NVIC_InitStructure);

	// I2C EV Interrupt
	NVIC_InitStructure.NVIC_IRQChannel                   = I2C1_EV_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;

	NVIC_Init(&NVIC_InitStructure);

 	/*i2c_ev_irq = xSemaphoreCreateBinary();
 	if(i2c_ev_irq == NULL)
 		return pdFALSE;

 	i2c_er_irq = xSemaphoreCreateBinary();
	if(i2c_er_irq == NULL)
		return pdFALSE;

	portBASE_TYPE ret = xTaskCreate(i2c_ev_handler_task, NULL,
									256, NULL, 2, NULL);
	if(ret != pdPASS)
		return pdFALSE;

	ret = xTaskCreate(i2c_er_handler_task, NULL,
									256, NULL, 2, NULL);
	if(ret != pdPASS)
		return pdFALSE;*/

return pdTRUE;
}

bool i2cWriteBuffer(uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data)
{
	uint8_t i;
	uint8_t my_data[16];
	uint32_t timeout = I2C_DEFAULT_TIMEOUT;

	addr = addr_ << 1;
	reg = reg_;
	writing = 1;
	reading = 0;
	write_p = my_data;
	read_p = my_data;
	bytes = len_;
	busy = 1;

	if (len_ > 16)
	    return false;

	for (i = 0; i < len_; i++)
	    my_data[i] = data[i];

	if (!(I2C1->CR2 & I2C_IT_EVT))                                      // If we are restarting the driver
	{
	    if (!(I2C1->CR1 & I2C_CR1_START))                               // Ensure sending a start
	    {
	        while (I2C1->CR1 & I2C_CR1_STOP) { ; }                      // Wait for any stop to finish sending
	        I2C_GenerateSTART(I2C1, ENABLE);                            // Send the start for the new job
	    }
	    I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_ERR, ENABLE);            // Allow the interrupts to fire off again
	}

	while (busy && --timeout > 0);
	if (timeout == 0) {
	    i2c1ErrorCount++;
	    i2c_init();                                                  // Reinit peripheral + clock out garbage
	    return false;
	}

	return true;
}

bool i2cWrite(uint8_t addr_, uint8_t reg_, uint8_t data)
{
    return i2cWriteBuffer(addr_, reg_, 1, &data);
}

bool i2cWriteBit(uint8_t addr_, uint8_t reg_, uint8_t bitNum, uint8_t data)
{
    uint8_t b;

    i2cRead(addr_ ,reg_, 1, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));

    return i2cWrite(addr_, reg_, b);
}

bool i2cWriteBits(uint8_t addr_, uint8_t reg_, uint8_t bitStart, uint8_t length, uint8_t data) {
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value

    //if(length > 0) {
        uint8_t b = 0;

        if(i2cRead(addr_,reg_,1, &b) != 0)
        { //get current data
            uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
            data <<= (bitStart - length + 1); // shift data into correct position
            data &= mask; // zero all non-important bits in data
            b &= ~(mask); // zero all important bits in existing byte
            b |= data; // combine data with existing byte
            return i2cWrite(addr_, reg_, b);
        }
    //}

    return false;
}

bool i2cRead(uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{
	uint32_t timeout = I2C_DEFAULT_TIMEOUT;

	addr = addr_ << 1;
	reg = reg_;
	writing = 0;
	reading = 1;
	read_p = buf;
	write_p = buf;
	bytes = len;
	busy = 1;

	if (!(I2C1->CR2 & I2C_IT_EVT))                                      // If we are restarting the driver
	{
	    if (!(I2C1->CR1 & I2C_CR1_START))                               // Ensure sending a start
	    {
	        while (I2C1->CR1 & I2C_CR1_STOP) { ; }                      // Wait for any stop to finish sending
	        I2C_GenerateSTART(I2C1, ENABLE);                            // Send the start for the new job
	    }
	    I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_ERR, ENABLE);            // Allow the interrupts to fire off again
	}

	while (busy && --timeout > 0);
	if (timeout == 0) {
	    i2c1ErrorCount++;
	    i2c_init();                                                  // Reinit peripheral + clock out garbage
	    return false;
	}

	return true;
}

int8_t i2cReadBit(uint8_t addr_,uint8_t reg_, uint8_t bitNum, uint8_t* buf)
{
    uint8_t b;
    uint8_t count = i2cRead(addr_, reg_, 1, &b);

    *buf = b & (1 << bitNum);

    return count;
}

int8_t i2cReadBits(uint8_t addr_, uint8_t reg_, uint8_t bitStart, uint8_t length, uint8_t *data)
{
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    int8_t count = 0;

    if(length > 0)
    {
        uint8_t b;
        if((count = i2cRead(addr_, reg_, 1, &b)) != 0)
        {
            uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
            b &= mask;
            b >>= (bitStart - length + 1);
            *data = b;
        }
    }

    return count;
}

void i2c_ev_handler_task(void *parameters)
{
    static uint8_t subaddress_sent, final_stop;                         // Flag to indicate if subaddess sent, flag to indicate final bus condition
    static int8_t index;                                                // Index is signed -1==send the subaddress

//    while(1) {
//    	if(xSemaphoreTake(i2c_ev_irq, 0)) {
			uint8_t SReg_1 = I2C1->SR1;                                         // Read the status register here

			if (SReg_1 & I2C_SR1_SB)                                            // We just sent a start - EV5 in ref manual
			{
				I2C1->CR1 &= ~I2C_CR1_POS;                                      // Reset the POS bit so ACK/NACK applied to the current byte
				I2C_AcknowledgeConfig(I2C1, ENABLE);                            // Make sure ACK is on
				index = 0;                                                      // Reset the index

				if (reading && (subaddress_sent || 0xFF == reg))                // We have sent the subaddr or no subaddress to send
				{
					subaddress_sent = 1;                                        // Make sure this is set in case of no subaddress, so following code runs correctly
					if (bytes == 2)
						I2C1->CR1 |= I2C_CR1_POS;                               // Set the POS bit so NACK applied to the final byte in the two byte read
					I2C_Send7bitAddress(I2C1, addr, I2C_Direction_Receiver);    // Send the address and set hardware mode
				}
				else                                                            // Direction is Tx, or we havent sent the sub and rep start
				{
					I2C_Send7bitAddress(I2C1, addr, I2C_Direction_Transmitter); // Send the address and set hardware mode
					if (reg != 0xFF)                                            // 0xFF as subaddress means it will be ignored, in Tx or Rx mode
						index = -1;                                             // Send a subaddress
				}
			}
			else if (SReg_1 & I2C_SR1_ADDR)                                     // We just sent the address - EV6 in ref manual
			{
				#pragma GCC diagnostic push
				#pragma GCC diagnostic ignored "-Wunused-but-set-variable"

				volatile uint8_t a;                                             // Read SR1, SR2 to clear ADDR

				#pragma GCC diagnostic pop

				__DMB(); // memory fence to control hardware
				if (bytes == 1 && reading && subaddress_sent)                   // We are receiving 1 byte - EV6_3
				{
					I2C_AcknowledgeConfig(I2C1, DISABLE);                       // Turn off ACK
					__DMB();
					a = I2C1->SR2;                                              // Clear ADDR after ACK is turned off
					I2C_GenerateSTOP(I2C1, ENABLE);                             // Program the stop
					final_stop = 1;
					I2C_ITConfig(I2C1, I2C_IT_BUF, ENABLE);                     // Allow us to have an EV7
				}
				else                                                            // EV6 and EV6_1
				{
					a = I2C1->SR2;                                              // Clear the ADDR here
					__DMB();
					if (bytes == 2 && reading && subaddress_sent)               // Rx 2 bytes - EV6_1
					{
						I2C_AcknowledgeConfig(I2C1, DISABLE);                   // Turn off ACK
						I2C_ITConfig(I2C1, I2C_IT_BUF, DISABLE);                // Disable TXE to allow the buffer to fill
					}
					else if (bytes == 3 && reading && subaddress_sent)          // Rx 3 bytes
						I2C_ITConfig(I2C1, I2C_IT_BUF, DISABLE);                // Make sure RXNE disabled so we get a BTF in two bytes time
					else                                                        // Receiving greater than three bytes, sending subaddress, or transmitting
						I2C_ITConfig(I2C1, I2C_IT_BUF, ENABLE);
				}
			}
			else if (SReg_1 & I2C_SR1_BTF)                                      // Byte transfer finished - EV7_2, EV7_3 or EV8_2
			{
				final_stop = 1;
				if (reading && subaddress_sent)                                 // EV7_2, EV7_3
				{
					if (bytes > 2)                                              // EV7_2
					{
						I2C_AcknowledgeConfig(I2C1, DISABLE);                   // Turn off ACK
						read_p[index++] = I2C_ReceiveData(I2C1);                // Read data N-2
						I2C_GenerateSTOP(I2C1, ENABLE);                         // Program the Stop
						final_stop = 1;                                         // Required to fix hardware
						read_p[index++] = I2C_ReceiveData(I2C1);                // Read data N-1
						I2C_ITConfig(I2C1, I2C_IT_BUF, ENABLE);                 // Enable RxNE to allow the final EV7 to read data N
					}
					else                                                        // EV7_3
					{
						if (final_stop)
							I2C_GenerateSTOP(I2C1, ENABLE);                     // Program the Stop
						else
							I2C_GenerateSTART(I2C1, ENABLE);                    // Program a rep start

						read_p[index++] = I2C_ReceiveData(I2C1);                // Read data N-1
						read_p[index++] = I2C_ReceiveData(I2C1);                // Read data N
						index++;                                                // To show job completed
					}
				}
				else                                                            // EV8_2, which may be due to a subaddress sent or a write completion
				{
					if (subaddress_sent || (writing)) {
						if (final_stop)
							I2C_GenerateSTOP(I2C1, ENABLE);                     // Program the Stop
						else
							I2C_GenerateSTART(I2C1, ENABLE);                    // Program a rep start

						index++;                                                // To show that the job is complete
					}
					else                                                        // We need to send a subaddress
					{
						I2C_GenerateSTART(I2C1, ENABLE);                        // Program the repeated Start
						subaddress_sent = 1;                                    // This is set back to zero upon completion of the current task
					}
				}
				while (I2C1->CR1 & I2C_CR1_START) { ; }                         // We must wait for the start to clear, otherwise we get constant BTF
			}
			else if (SReg_1 & I2C_SR1_RXNE)                                     // Byte received - EV7
			{
				read_p[index++] = I2C_ReceiveData(I2C1);
				if (bytes == (index + 3))
					I2C_ITConfig(I2C1, I2C_IT_BUF, DISABLE);                    // Disable RxNE to allow the buffer to flush so we can get an EV7_2

				if (bytes == index)                                             // We have completed a final EV7
					index++;                                                    // To show job is complete
			}
			else if (SReg_1 & I2C_SR1_TXE)                                      // Byte transmitted - EV8/EV8_1
			{
				if (index != -1)                                                // We dont have a subaddress to send
				{
					I2C_SendData(I2C1, write_p[index++]);
					if (bytes == index)                                         // We have sent all the data
						I2C_ITConfig(I2C1, I2C_IT_BUF, DISABLE);                // Disable TXE to allow the buffer to flush
				}
				else
				{
					index++;
					I2C_SendData(I2C1, reg);                                    // Send the subaddress
					if (reading || !bytes)                                      // If receiving or sending 0 bytes, flush now
						I2C_ITConfig(I2C1, I2C_IT_BUF, DISABLE);                // Disable TXE to allow the buffer to flush
				}
			}
			if (index == bytes + 1)                                             // We have completed the current job
			{
				subaddress_sent = 0;                                            // Reset this here

				if (final_stop)                                                 // If there is a final stop and no more jobs, bus is inactive, disable interrupts to prevent BTF
					I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_ERR, DISABLE);       // Disable EVT and ERR interrupts while bus inactive
				busy = 0;
			}
   	// 	}
   	// }
}
