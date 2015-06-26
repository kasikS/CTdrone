CC      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy

PROJECT_NAME = ctdrone
PROJECT_SRC = src
STM_SRC = Drivers/STM32F4xx_StdPeriph_Driver/src/
OBJ_DIR = build

vpath %.c $(PROJECT_SRC)
vpath %.c $(STM_SRC)

SRCS = main.c
SRCS += serial.c
SRCS += nrf24l.c
SRCS += motor.c
SRCS += link_common.c
SRCS += i2c.c
SRCS += delay_timer.c
SRCS += drv_mpu6050.c
SRCS += drv_hmc5883l.c
SRCS += PID.c
SRCS += FlightControl.c

SRCS += Device/startup_stm32f401xe.s

SRCS += stm32f4xx_it.c
SRCS += system_stm32f4xx.c

SRCS += Drivers/FreeRTOS/Source/croutine.c
SRCS += Drivers/FreeRTOS/Source/event_groups.c
SRCS += Drivers/FreeRTOS/Source/list.c
SRCS += Drivers/FreeRTOS/Source/queue.c
SRCS += Drivers/FreeRTOS/Source/tasks.c
SRCS += Drivers/FreeRTOS/Source/timers.c
SRCS += Drivers/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
SRCS += Drivers/FreeRTOS/Source/portable/MemMang/heap_1.c

EXT_SRCS = stm32f4xx_gpio.c
EXT_SRCS += stm32f4xx_exti.c
EXT_SRCS += stm32f4xx_i2c.c
EXT_SRCS += stm32f4xx_rcc.c
EXT_SRCS += stm32f4xx_syscfg.c
EXT_SRCS += stm32f4xx_spi.c
EXT_SRCS += stm32f4xx_tim.c
EXT_SRCS += stm32f4xx_usart.c
EXT_SRCS += misc.c

EXT_OBJ = $(addprefix $(OBJ_DIR)/, $(EXT_SRCS:.c=.o))

INC_DIRS  = src/
INC_DIRS += Drivers/STM32F4xx_StdPeriph_Driver/inc/
INC_DIRS += Drivers/CMSIS/Device/ST/STM32F4xx/Include/
INC_DIRS += Drivers/CMSIS/Include/

INC_DIRS += Drivers/FreeRTOS/Source/include/
INC_DIRS += Drivers/FreeRTOS/Source/portable/GCC/ARM_CM4F/

INCLUDE = $(addprefix -I,$(INC_DIRS))

DEFS = -DSTM32F401xx -DUSE_STDPERIPH_DRIVER

CFLAGS += -ggdb -O0 -std=c99
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -fsingle-precision-constant -mthumb-interwork -Wl,--gc-sections

WFLAGS += -Wall -Wextra -Warray-bounds -Wno-unused-parameter -Wno-unused-function
LFLAGS = -TDevice/gcc.ld -lm -lc -lnosys

# Create a directory for object files
$(shell mkdir $(OBJ_DIR) > /dev/null 2>&1)

.PHONY: all
all: $(PROJECT_NAME)

.PHONY: $(PROJECT_NAME)
$(PROJECT_NAME): $(PROJECT_NAME).elf

$(PROJECT_NAME).elf: $(SRCS) $(EXT_OBJ)
	$(CC) $(INCLUDE) $(DEFS) $(CFLAGS) $^ $(WFLAGS) $(LFLAGS) -o $@
	$(OBJCOPY) -O ihex $(PROJECT_NAME).elf   $(PROJECT_NAME).hex
	$(OBJCOPY) -O binary $(PROJECT_NAME).elf $(PROJECT_NAME).bin

$(OBJ_DIR)/%.o: %.c
	$(CC) -c -o $@ $(INCLUDE) $(DEFS) $(CFLAGS) $^

clean:
	rm -rf $(OBJ_DIR) $(PROJECT_NAME).elf $(PROJECT_NAME).hex $(PROJECT_NAME).bin

flash: $(PROJECT_NAME).elf
	openocd -f interface/stlink-v2-1.cfg -f target/stm32f4x_stlink.cfg -f scripts/flash.cfg
