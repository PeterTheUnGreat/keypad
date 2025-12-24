/*
 * Telephone.c
 *
 * Created: 23/07/2025 18:04:31
 *  Author: peter
 */

#include "Compile.h"
#include "Keypad.h"
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "Telephone.h"
#include "Utils.h"

#ifdef CODE_SECTION_TELEPHONE

const short ringingCadence[4] = { 16, 8, 16, 80 };

#define CADENCE_MAX	4

//_______________________________________________________________________________________
// Set up everything needed for telephone
//_______________________________________________________________________________________
//
void initTelephone() {
    cadenceTablePtr = 0;
    cadenceCount = 0;
    counter_20Hz = 0;
    phase_20Hz = false;

    OCR3A = T1_25ms;

    TIMSK3 |= _BV(OCIE3A);							// Set timer 3 to trigger an interrupt every 25ms using output compare A

#ifdef CLOCK_X8
    TCCR3B = _BV(CS31) + _BV(CS30) + _BV(WGM32);	// Start timer 3 with prescaler of 64 and in CTC mode (we have sped up the clock by a factor of 8)
#else
    TCCR3B = _BV(CS31) + _BV(WGM32);		// Start timer 3 with prescaler of 8 and in CTC mode
#endif // CLOCK_X8

}

ISR(TIMER3_COMPA_vect) {								// handle timer 3 output compare A interrupt
    // turn the ringer on and off in the right pattern
    if(!(cadenceTablePtr & 0x01) && phase_20Hz) { // && (statusFlags & _BV(STAT_RINGING))
#ifdef CODE_SECTION_DEBUG
        debugOn();
#endif // CODE_SECTION_DEBUG
        statusFlags |= _BV(STAT_RING);
    } else {
#ifdef CODE_SECTION_DEBUG
        debugOff();
#endif // CODE_SECTION_DEBUG
        statusFlags &= ~_BV(STAT_RING);
    }

    cadenceCount++;
    if(cadenceCount >= ringingCadence[cadenceTablePtr]) {
        cadenceCount = 0;
        cadenceTablePtr++;
        if(cadenceTablePtr >= CADENCE_MAX) cadenceTablePtr = 0;
    }

    phase_20Hz = !phase_20Hz;
}

#endif //CODE_SECTION_TELEPHONE
