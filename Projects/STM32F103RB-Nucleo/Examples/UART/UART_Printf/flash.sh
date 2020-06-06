#!/bin/bash
FLASH_ADDR=0x8000000
USB_DEVICE=/dev/ttyUSB0
stm32flash -w std32f1-printf.bin  -v -g ${FLASH_ADDR} ${USB_DEVICE} 
#stm32flash -g ${FLASH_ADDR} ${USB_DEVICE}
