/*
 * Chess.c
 *
 * Created: 24/12/2025 14:21:40
 *  Author: peter
 */

#include "Utils.h"
#include "Compile.h"
#include "Keypad.h"
#include <avr/wdt.h>
#include <util/twi.h>


#ifdef	CODE_SECTION_CHESS

#define IIC_addr_Chess (0x24)								// I²C IIC_address of NFC readers

int Chess_Init() {
    i2cInit();									// prepare the IIC interface
    return 0;
}

int NFCReadReg(unsigned char II2Addr, unsigned char reg) {
    TWI_send_data[0] = reg;

    for(int tries = 3; tries > 0 ; tries--) {
        i2cTransfer(II2Addr, 1, TW_WRITE);
        timeOut = timeOut_500ms;
        while(((TWCR1 & _BV(TWIE)) != 0) && timeOut) wdt_reset(); // as long as interrupts are on the II2 is busy
        if ((TWI_flags & _BV(TWI_flag_error)) == 0) {
            i2cTransfer(II2Addr, 1, TW_READ);
            timeOut = timeOut_500ms;
            while(((TWCR1 & _BV(TWIE)) != 0) && timeOut) wdt_reset(); // as long as interrupts are on the II2 is busy
            if (((TWI_flags & _BV(TWI_flag_error)) == 0) && !timeOut) return -1;
        }
    }
    return 0;										// return 0 to indicate fail after three tries
}


// At the moment just send some rubbish out of the IIC port
int Chess_Poll() {
    return NFCReadReg(IIC_addr_Chess, 0x37);
}

#endif /* CODE_SECTION_CHESS */

