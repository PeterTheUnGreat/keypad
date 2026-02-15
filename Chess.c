/*
 * Chess.c
 *
 * Created: 24/12/2025 14:21:40
 *  Author: peter
 */

#include "Compile.h"
#include "Keypad.h"
#include "Utils.h"
#include <avr/wdt.h>
#include "NFC.h"


#ifdef	CODE_SECTION_CHESS

int Chess_Init() {
    PN532_init();									// prepare the PN532 interface
    PN532_SAMConfig();								// and set up the module
    PN532_setPassiveActivationRetries(0);			// set for non-blocking manuall polling
    return 0;
}

// Just ask for the firmware version
uint16_t Chess_Poll() {
    return PN532_readPassiveTargetID(0);			// 0 is the baudrate of most NFC stickers etc.
}

#endif /* CODE_SECTION_CHESS */

