#include "stm32f4xx_conf.h"

#include <FreeRTOS.h>
#include <task.h>
#include <timers.h>
#include <queue.h>

#include "serial.h"
#include "motor.h"
#include "nrf24l.h"
//#include "i2c.h"
//#include "drv_mpu6050.h"
//#include "drv_hmc5883l.h"
//#include "FlightControl.h"
#include "link.h"

#include <string.h>

// XXX XXX XXX XXX
#define CONTROLLER





#ifdef CONTROLLER
void controller_task(void *parameters);
#else /* CONTROLLER */
void Task_LED_Blink(void *parameters);
#endif

void delay(uint32_t ms)
{
    while (ms--)
    {}//delayMicroseconds(1000);
}

int main(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    SystemCoreClockUpdate();

    // Turn on clocks for stuff we use
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4 | RCC_APB1Periph_TIM5 | RCC_APB1Periph_I2C1 |  RCC_APB1Periph_USART2 , ENABLE);
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM8, ENABLE);
    // RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_DMA2, ENABLE);
    // RCC_ClearFlag();

    /* GPIOD Periph clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    RCC_ClearFlag();
    NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    serial_init(115200);
    motor_init();
    nrf24l_init();

#ifdef CONTROLLER
    xTaskCreate(controller_task, NULL, configMINIMAL_STACK_SIZE + 20, NULL, 2, NULL);
#else /* CONTROLLER */
    xTaskCreate(Task_LED_Blink, NULL, configMINIMAL_STACK_SIZE + 20, NULL, 2, NULL);
#endif /* CONTROLLER */
    vTaskStartScheduler();

    while(1);
}

#ifdef CONTROLLER
// Task supposed to run on the module attached to PC as a serial to RF link.
void controller_task(void *parameters) {
    struct packet pkt;
    char* ptr = (char*) &pkt;
    crc_t crc;
    int cnt = 0;

    while(1) {
        if(serial_getc(ptr)) {
            ++cnt;
            ++ptr;

            if(cnt == PACKET_TOTAL_SIZE) { // now we should receive crc
                ptr = (char*) &crc;

                        // Correct packet
                        GPIO_ToggleBits(GPIOA, GPIO_Pin_5);
            }

            else if(cnt == PACKET_TOTAL_SIZE + 2) {  // serial port sends crc too
                cnt = 0;
                ptr = (char*) &pkt;

                if(crc != link_crc(&pkt))       // invalid packet, skip
                    continue;

                // TODO tx through rf (without crc), send reply through the serial port
                // TODO error message through serial port
                switch(pkt.type) {
                    case PT_STATUS:
                        strncpy(pkt.data.text, "CTDrone1\x0", 9);
                        crc = link_crc(&pkt);
                        serial_write((const char*) &pkt, PACKET_TOTAL_SIZE);
                        serial_write((const char*) &crc, CRC_SIZE);

                }
            }
        }
    }
}
#endif /* CONTROLLER */


void Task_LED_Blink(void *parameters) {
    char c;
    //int speed = 0;
    uint8_t buf[9] = {0,};
    uint8_t tx_buf[] = "nrf24_00";

    serial_puts("reset\r");

    while(1){
        // Serial port echo
        /*if(serial_getc(&c))
            serial_putc(c);*/

        /*c = nrf24l_get_status();*/
        /*serial_putc(nrf24l_get_status());*/
        /*nrf24l_puts("test");*/

        /*serial_puts("orson");*/

        while(nrf24l_getc(&c))
            serial_putc(c);

#ifdef NRF24L_RX
        if(nrf24l_data_ready())
        {
            nrf24l_rx_direct(buf);
            serial_puts(buf);
        }
#endif

#ifdef NRF24L_TX
        nrf24l_tx_direct(tx_buf);
        if(++tx_buf[7] == ':') {
            tx_buf[7] = '0';

            if(++tx_buf[6] == ':') {
                tx_buf[6] = '0';
            }
        }
#endif

        // Motor test
        /*speed += 10;
        if(speed > 100)
            speed = 0;
        motor_set_speed(0, speed);*/

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
