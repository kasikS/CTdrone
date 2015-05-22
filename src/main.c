#include "stm32f4xx_conf.h"

#include <FreeRTOS.h>
#include <task.h>
#include <timers.h>
#include <queue.h>

#include "serial.h"
#include "motor.h"
#include "nrf24l.h"
#include "i2c.h"
#include "drv_mpu6050.h"
//#include "drv_hmc5883l.h"
#include "FlightControl.h"
#include "link.h"

void Task_LED_Blink(void *params);

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

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    RCC_ClearFlag();
    NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    serial_init(460800);
    motor_init();
    i2c_init();
    //nrf24l_init();

    serial_puts("siema!");

    xTaskCreate(Task_LED_Blink, NULL, configMINIMAL_STACK_SIZE + 20, NULL, 2, NULL);
    vTaskStartScheduler();

    while(1);
}


void Task_LED_Blink(void *parameters) {
    vTaskDelay(500);
    imu_init();
    vTaskDelay(100);
    flight_control_init();

    int power = 0;

    while(1){
        // Serial port echo
        /*if(serial_getc(&c))
            serial_putc(c);*/

        power += 10;
        if(power == 110)
            power = 0;
        motor_set_speed(0, power);
        motor_set_speed(1, power);
        motor_set_speed(2, power);
        motor_set_speed(3, power);

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
