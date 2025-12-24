/*
 * Chess.c
 *
 * Created: 24/12/2025 14:21:40
 *  Author: peter
 */

#include "Utils.h"
#include "Compile.h"
#include <avr/wdt.h>
#include <util/twi.h>


#ifdef	CODE_SECTION_CHESS

#define IIC_addr (0x29)								// I²C IIC_address of NFC readers

int Chess_Init() {
    return 0;
}

// At the moment just send some rubbish out of the IIC port
int Chess_Poll() {
    TWI_send_data[0] = 0xAA;
    for(int tries = 3; tries > 0 ; tries--) {
        i2cTransfer(IIC_addr, 1, TW_WRITE);
        while((TWCR1 & _BV(TWIE)) != 0) wdt_reset(); // as long as interrupts are ion the II2 is busy
        if ((TWI_flags & TWI_flag_error) == 0) return 0;
    }
    return -1;
}

#endif /* CODE_SECTION_CHESS */