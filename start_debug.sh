#!/bin/sh
GDB=/opt/gcc-arm-none-eabi-4_8-2014q1/bin/arm-none-eabi-gdb
ELF=ctdrone.elf

openocd -f interface/stlink-v2-1.cfg  -f target/stm32f4x_stlink.cfg -f scripts/init.cfg &
${GDB} -ex "target remote localhost:3333" \
    -ex "monitor reset halt" \
    -ex "layout sp" ${ELF}
killall openocd
