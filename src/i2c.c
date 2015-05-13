#include <FreeRTOS.h>
#include "stm32f4xx_hal.h"

#include "i2c.h"

static I2C_HandleTypeDef i2c;

/*#define I2C_DEFAULT_TIMEOUT 30000*/
#define I2C_DEFAULT_TIMEOUT 35

void HAL_I2C_MspInit(I2C_HandleTypeDef *i2c)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    __I2C1_CLK_ENABLE();
    __GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*HAL_NVIC_SetPriority(I2C1_EV_IRQn, 5, 0);*/
    /*HAL_NVIC_SetPriority(I2C1_ER_IRQn, 5, 0);*/
}

int i2c_init()
{
    // TODO error handling
    i2c.Instance = I2C1;
    i2c.Init.ClockSpeed = 400000;
    i2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
    i2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    i2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
    i2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;    // TODO?
    i2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;        // TODO?
    i2c.State = HAL_I2C_STATE_RESET;    // TODO necessary?
    HAL_I2C_Init(&i2c);         // if( != HAL_OK)

    /*HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);*/
    /*HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);*/

    /*__HAL_I2C_ENABLE_IT(&i2c, I2C_IT_EVT);*/
    /*__HAL_I2C_ENABLE_IT(&i2c, I2C_IT_ERR);*/

    return pdTRUE;
}

// TEST
bool i2cWriteBuffer(uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data)
{
    return HAL_I2C_Mem_Write(&i2c, addr_ << 1, reg_, 1, data, len_, I2C_DEFAULT_TIMEOUT*100) == HAL_OK;
    /*return HAL_I2C_Mem_Write_IT(&i2c, addr_ << 1, reg_, 1, data, len_);*/
}

// OK
bool i2cWrite(uint8_t addr_, uint8_t reg_, uint8_t data)
{
    return i2cWriteBuffer(addr_, reg_, 1, &data);
}

// OK
bool i2cWriteBit(uint8_t addr_, uint8_t reg_, uint8_t bitNum, uint8_t data)
{
    uint8_t b;

    i2cRead(addr_ ,reg_, 1, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));

    return i2cWrite(addr_, reg_, b);
}

// OK
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

// TEST
bool i2cRead(uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{
    return HAL_I2C_Mem_Read(&i2c, addr_ << 1, reg_, 1, buf, len, I2C_DEFAULT_TIMEOUT*100) == HAL_OK;
    /*return HAL_I2C_Mem_Read_IT(&i2c, addr_ << 1, reg_, 1, buf, len);*/
}

// OK
int8_t i2cReadBit(uint8_t addr_,uint8_t reg_, uint8_t bitNum, uint8_t* buf)
{
    uint8_t b;
    uint8_t count = i2cRead(addr_, reg_, 1, &b);

    *buf = b & (1 << bitNum);

    return count;
}

// OK
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
