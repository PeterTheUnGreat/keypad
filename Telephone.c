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

#ifdef CODE_SECTION_TELEPHONE

const uint8_t sine_table[48] = {
    0x80, 0x96, 0xAC, 0xC1, 0xD4, 0xE4, 0xEF, 0xF5,
    0xF5, 0xF0, 0xE6, 0xD8, 0xC6, 0xB2, 0x9C, 0x85,
    0x6F, 0x59, 0x46, 0x34, 0x26, 0x1A, 0x10, 0x0A,
    0x0A, 0x0F, 0x18, 0x24, 0x32, 0x43, 0x56, 0x6B,
    0x81, 0x97, 0xAC, 0xC0, 0xD2, 0xE1, 0xEC, 0xF3,
    0xF5, 0xF3, 0xEC, 0xE0, 0xD0, 0xBC, 0xA7, 0x91
};

#define SINE_TABLE_MAX 48

const uint16_t ringingCadence[4] = { 400, 200, 400, 2000 };

#define CADENCE_MAX	4


//_______________________________________________________________________________________
// Set up everything needed for telephone
//_______________________________________________________________________________________
//
void initTelephone() {
    sineTablePtr = 0;
    cadenceTablePtr = 0;
    cadenceCount = 0;

    OCR1A = T1_1ms;
    TIMSK |= _BV(OCIE1A);							// Set the timer to trigger an interrupt every 2ms using output compare A
    TCCR1B = _BV(CS11) + _BV(WGM12);				// Start timer 1 with prescaler of 8 and in CTC mode

    TCCR2 = _BV(WGM21) + _BV(WGM20) + _BV(COM21) + _BV(CS20);	// Set timer 2 to fast PWM mode with OC2 as non-inverted output and presacaler of 1
    DDRB |= _BV(PB3);								// Make sure PB3 is set as output so we can see PWM
    PORTB &= ~_BV(PB3);								// portB off if OC2 disconnected

    OCR2 = 0;

    SPCR = 0;										// disable SPI
}

ISR(TIMER1_COMPA_vect) {								// handle timer 1 output compare A interrupt
    // turn the ringer on and off in the right pattern
    if (cadenceTablePtr & 0x01) OCR2 = 0;
    else OCR2 = pgm_read_byte(&sine_table[sineTablePtr++]);

    if(sineTablePtr >= SINE_TABLE_MAX) sineTablePtr = 0; // wrap around pointer

    cadenceCount++;
    if(cadenceCount >= ringingCadence[cadenceTablePtr]) {
        cadenceCount = 0;
        cadenceTablePtr++;
        if(cadenceTablePtr >= CADENCE_MAX) cadenceTablePtr = 0;
    }
}

#endif /*CODE_SECTION_TELEPHONE*/
