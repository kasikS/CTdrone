#include "stm32f4xx_conf.h"

#include <FreeRTOS.h>
#include <task.h>
#include <timers.h>

#include <stdio.h>
#include <string.h>

#include "serial.h"
#include "motor.h"
#include "nrf24l.h"
#include "leds.h"
#include "i2c.h"
#include "delay_timer.h"
#include "drv_mpu6050.h"
//#include "drv_hmc5883l.h"
#include "FlightControl.h"
#include "link.h"

#ifndef CONTROLLER
void Task_LED_Blink(void *params);
#else
void controller_task(void *parameters);
#endif

int main(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    SystemCoreClockUpdate();

    // Turn on clocks for stuff we use
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4 | RCC_APB1Periph_TIM5 | RCC_APB1Periph_I2C1 |  RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM8, ENABLE);

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    RCC_ClearFlag();
    NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

    // TODO remove not necessary pins
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    delay_init();
    serial_init(460800);
    nrf24l_init();

#ifdef CONTROLLER
    xTaskCreate(controller_task, NULL, 256, NULL, 2, NULL);
    leds_init();
#else
    motor_init();
    i2c_init();

    serial_puts("siema!");
    xTaskCreate(Task_LED_Blink, NULL, configMINIMAL_STACK_SIZE + 20, NULL, 2, NULL);
#endif

    vTaskStartScheduler();

    while(1);
}

#ifdef CONTROLLER
// Task supposed to run on the module attached to PC as a serial to RF link.
void controller_task(void *parameters) {
    // Buffers to store rx/tx data
    uint8_t buf_mosi[PACKET_TOTAL_SIZE + CRC_SIZE] = {0,};
    uint8_t buf_miso[PACKET_TOTAL_SIZE + CRC_SIZE] = {0,};

    // Pointers to ease buffer management
    uint8_t* ptr_mosi = buf_mosi;
    uint8_t* ptr_miso = buf_miso;
    struct packet* pkt_miso = (struct packet*) buf_miso;
    const struct packet const* pkt_mosi = (struct packet*) buf_mosi;
    crc_t* crc_miso = (crc_t*) &buf_miso[PACKET_TOTAL_SIZE];
    const crc_t const* crc_mosi = (crc_t*) &buf_mosi[PACKET_TOTAL_SIZE];
    int cnt_mosi = 0, cnt_miso = 0;

    uint8_t buf[16];

    // mosi = serial to rf
    // miso = rf to serial

    while(1) {
        while(serial_getc((char*)ptr_mosi)) {
            ++cnt_mosi; ++ptr_mosi;

            if(cnt_mosi == PACKET_TOTAL_SIZE + CRC_SIZE) {  // serial port sends crc too
                // Got a full packet (and crc) from the serial port
                GPIO_ToggleBits(GPIOA, GPIO_Pin_5);

                // Reset pointer to receive next packet
                cnt_mosi = 0;
                ptr_mosi = buf_mosi;

                if(*crc_mosi != link_crc(pkt_mosi)) {      // invalid packet, skip
                    sprintf((char*)buf, "CRCXX%.2x", *crc_mosi);
                    serial_write((char*)buf, 7);
                    continue;
                }

                // Transmit through rf (without crc), send reply through the serial port
                switch(pkt_mosi->type) {
                    case PT_STATUS:
                        for(int i = 0; i < PACKET_DATA_SIZE; ++i) {
                            if(pkt_mosi->data.text[i] != 0x00)
                                serial_puts("XXX");
                                continue;       // incorrect status query
                        }

                        pkt_miso->type = PT_STATUS;
                        strncpy((char*)pkt_miso->data.text, "CTDrone10\x0", PACKET_DATA_SIZE);
                        *crc_miso = link_crc(pkt_miso);
                        serial_write((const char*) pkt_miso, PACKET_TOTAL_SIZE + CRC_SIZE);
                        break;

                    default:
                        /*sprintf(buf, "CRCOK%.2x", *crc_mosi);*/
                        /*serial_write((char*)buf, 7);*/
                        nrf24l_write((const char*)pkt_mosi, PACKET_TOTAL_SIZE);
                        break;
                }

                break; // give a chance to retrieve radio packet
            }
        }

        // Forward data received from the radio
        while(nrf24l_getc((char*)ptr_miso)) {
            ++cnt_miso; ++ptr_miso;

            if(cnt_miso == PACKET_TOTAL_SIZE) {
                cnt_miso = 0;
                ptr_miso = buf_miso;

                *crc_miso = link_crc(pkt_miso);
                serial_write((const char*) pkt_miso, PACKET_TOTAL_SIZE + CRC_SIZE);

                break; // give a chance to retrieve serial port packet
            }
        }
    }
}
#endif /* CONTROLLER */

void Task_LED_Blink(void *parameters) {
    vTaskDelay(500);
    imu_init();
    vTaskDelay(100);
    flight_control_init();

    while(1){
        GPIO_ToggleBits(GPIOA, GPIO_Pin_5);
        vTaskDelay(1000);
    }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
