##!/bin/bash

echo "Cleaning up"
rm a.out
rm test.hex

echo "Compiling code"

if [ $1 = m328pb ]
then
	avr-gcc -Os -mmcu=atmega328pb -I=/usr/lib/avr/include keypad.c 14seg.c message.c utils.c menu.c range.c io.c telephone.c chess.c
else
	avr-gcc -Os -mmcu=atmega8 -I=/usr/lib/avr/include keypad.c 14seg.c message.c utils.c menu.c range.c io.c telephone.c chess.c
fi

echo "Converting code to intel HEX"
avr-objcopy -O ihex a.out test.hex

#cat test.hex

echo "Write to target"

if [ $1 = m328pb ]
then
	avrdude -P /dev/spidev0.0:/dev/gpiochip0 -c linuxspi -p m328pb -U flash:w:test.hex:i
	avr-size -C --mcu=atmega328pb
else
	avrdude -P /dev/spidev0.0:/dev/gpiochip0 -c linuxspi -p m8 -U flash:w:test.hex:i
	avr-size -C --mcu=atmega8
fi


avr-nm --numeric-sort > symbols
