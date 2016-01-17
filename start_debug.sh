#!/bin/sh
GDB=arm-none-eabi-gdb
ELF=ctdrone.elf

openocd -f interface/stlink-v2-1.cfg  -f target/stm32f4x_stlink.cfg -f scripts/init.cfg &
${GDB} -ex "target remote localhost:3333" \
    -ex "monitor reset halt" \
    -ex "layout sp" ${ELF}
killall openocd
