##!/bin/bash

echo "Cleaning up"
rm a.out
rm test.hex

echo "Compiling code"
avr-gcc -Os -mmcu=atmega8 -I=/usr/lib/avr/include keypad.c 14seg.c message.c utils.c menu.c range.c

echo "Converting code to intel HEX"
avr-objcopy -O ihex a.out test.hex

#cat test.hex

echo "Write to target"
avrdude -P /dev/spidev0.0:/dev/gpiochip0 -c linuxspi -p m8 -U flash:w:test.hex:i

avr-size -C --mcu=atmega8
avr-nm --numeric-sort > symbols
