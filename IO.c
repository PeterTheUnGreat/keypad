/*
 * IO.c
 *
 * Created: 05/06/2025 16:21:20
 *  Author: peter
 */

#include <avr/io.h>
#include <avr/pgmspace.h>

#include "Compile.h"
#include "14seg.h"
#include "Utils.h"
#include "Keypad.h"
#include "message.h"
#include "IO.h"

#ifdef CODE_SECTION_IO

triggerStruct triggers[TRIG_MAX];					// space to hold triggers

//_______________________________________________________________________________________
// This section defines some default triggers to use if none have been programmed into EEPROM
//_______________________________________________________________________________________
//

#define DEFAULT_DDRB  _BV(PB6)
#define DEFAULT_DDRD (_BV(PD6) + _BV(PD7))
#define PORTB_DEFAULT (_BV(PB1) + _BV(PB2) + _BV(PB4) + _BV(PB6) + _BV(PB7))	// Pull ups enabled on input pins
#define PORTD_DEFAULT (_BV(PD3) + _BV(PD4) + _BV(PD5))

const triggerStruct defaultTriggers[] PROGMEM = {
    // This trigger turns the unlock output on for 2s when the correct code is entered
    {
        0, _BV(PD7), 0,								// Output (PortB, PortD, Status)
        0, 0, _BV(STAT_UNLOCKED),					// Input (PortB, PortD, Status)
        _BV(TRIG_EDGE) + _BV(TRIG_RISE) + _BV(TRIG_CLEAR) + _BV(TRIG_SET), 0x80, 0		// Flags, Timer (in 16ms), 0
    }
#ifdef CODE_SECTION_SAFE
    // This trigger makes a click (pulse of approx. 32ms) using the relay
    , {
        0, _BV(PD6), 0,								// Output (PortB, PortD, Status)
        0, 0, _BV(STAT_PULSE),						// Input (PortB, PortD, Status)
        _BV(TRIG_EDGE) + _BV(TRIG_RISE) + _BV(TRIG_CLEAR) + _BV(TRIG_SET), 2, 0		// Flags, Timer (in 16ms), 0
    },
    // This trigger sets a flag if PB7 is held low (by the presence of the stethoscope
    {
        0, 0, _BV(STAT_STETH),					// Output (PortB, PortD, Status)
        _BV(PB7), 0, 0,								// Input (PortB, PortD, Status)
        0, 0, 0									// Following trigger - no timer (inverted)
    },

    // This trigger turns on the heartbeat light if PB7 is held low (by the presence of the stethoscope)
    {
        _BV(PB6), 0, 0,					// Output (PortB, PortD, Status)
        _BV(PB7), 0, 0,								// Input (PortB, PortD, Status)
        0, 0, 0									// Following trigger - no timer (inverted)
    }
#endif /*CODE_SECTION_SAFE*/
};

#define NUM_DEFAULT_TRIGGERS (sizeof(defaultTriggers) / sizeof(triggerStruct))

void setDefaultTriggers() {
    trigCount = NUM_DEFAULT_TRIGGERS;
    memcpy_P(triggers, defaultTriggers, sizeof(defaultTriggers));
    // and set default DDRS and values
    DDRB = (DDRB & ~DDR_MSK_B) | DEFAULT_DDRB;		// set default DDR and port values
    DDRD = (DDRD & ~DDR_MSK_D) | DEFAULT_DDRD;
    PORTB = (PORTB & ~IO_MSK_B) | (PORTB_DEFAULT & IO_MSK_B);
    PORTD = (PORTD & ~IO_MSK_D) | (PORTD_DEFAULT & IO_MSK_D);
}

//_______________________________________________________________________________________
// Read all of the IO stuff from EEPROM and set up DDRs etc.
//_______________________________________________________________________________________
//
void initIO() {
    int EEPROMpointer;
    unsigned char flagsIn;

    EEPROMtrigCount = EEPROM_read(EEPROM_TRIG_COUNT);
    if(EEPROMtrigCount == 0xFF) EEPROMtrigCount = 0;			// just in case EEPROM has bee wiped

    // if there are any triggers in EEPROM then use these in preference to the defaults
    if(EEPROMtrigCount) {
        trigCount = EEPROMtrigCount;
        for(int n = 0; n < trigCount; n++) {				// read in all of the triggers into the trigger array
            EEPROMpointer = (n << 3) + EEPROM_TRIG;	// give 8 bytes for each trigger
            triggers[n].outB = EEPROM_read(EEPROMpointer++);
            triggers[n].outD = EEPROM_read(EEPROMpointer++);
            triggers[n].outStatus = EEPROM_read(EEPROMpointer++);
            triggers[n].inB = EEPROM_read(EEPROMpointer++);
            triggers[n].inD = EEPROM_read(EEPROMpointer++);
            triggers[n].statusIn = EEPROM_read(EEPROMpointer++);
            flagsIn = EEPROM_read(EEPROMpointer++);

            triggers[n].flags = flagsIn & 0x0F;			// incoming flags only in lower 4 bits
            triggers[n].timerValue = flagsIn & 0xF0;	// incoming timer in upper nibble. which should make the incoming timer approx 1/4s of a second
            triggers[n].timerActive = 0;
        }

        DDRB = (DDRB & ~DDR_MSK_B) | EEPROM_read(EEPROM_DDRB);		// read the EEPROM copies
        DDRD = (DDRD & ~DDR_MSK_D) | EEPROM_read(EEPROM_DDRD);
        PORTB = (PORTB & ~IO_MSK_B) | EEPROM_read(EEPROM_B_DEFAULT);
        PORTD = (PORTD & ~IO_MSK_D) | EEPROM_read(EEPROM_D_DEFAULT);
    } else {
        setDefaultTriggers();                        // otherwise copy in the defaults
    }
    // save current state as previous
    portBprevious = PINB;
    portDprevious = PIND;
    statusPrevious = statusFlags;
}

//_______________________________________________________________________________________
// Clear all of the IO stuff from EEPROM and reset DDRs etc.
//_______________________________________________________________________________________
//
void resetIO() {
    EEPROMtrigCount = 0;

    EEPROM_write(EEPROM_DDRB, 0);					// reset the EEPROM copies to all inputs
    EEPROM_write(EEPROM_DDRD, 0);
    EEPROM_write(EEPROM_TRIG_COUNT, EEPROMtrigCount); // delete trigger count table

    setDefaultTriggers();							// and copy in the defaults
}

//_______________________________________________________________________________________
// set DDRs
//_______________________________________________________________________________________
//
void setDDRD(unsigned char portBDDR, unsigned char portDDDR, unsigned char portBdefault, unsigned char portDdefault) {
    // Set this to stop irritating display blanking if SS is set to input
    portBDDR &= DDR_MSK_B;				// remove any unwanted bits
    portBdefault &= IO_MSK_B;
    portDDDR &= DDR_MSK_D;
    portDdefault &= IO_MSK_D;

    DDRB = (DDRB & ~DDR_MSK_B) | portBDDR;
    DDRD = (DDRD & ~DDR_MSK_D) | portDDDR;

    PORTB = (PORTB & ~IO_MSK_B) | portBdefault;
    PORTD = (PORTD & ~IO_MSK_D) | portDdefault;

    EEPROM_write(EEPROM_DDRB, portBDDR);				// update the EEPROM copies
    EEPROM_write(EEPROM_DDRD, portDDDR);
    EEPROM_write(EEPROM_B_DEFAULT, portBdefault);
    EEPROM_write(EEPROM_D_DEFAULT, portDdefault);
}

//_______________________________________________________________________________________
// Set one or more outputs to the given state
//_______________________________________________________________________________________
//
void setOutput(unsigned char portBmask, unsigned char portDmask, unsigned char portBin, unsigned char portDin) {
    portBmask &= IO_MSK_B;
    portBin &= IO_MSK_B;		// remove any unwanted bits
    portDmask &= IO_MSK_D;
    portDin &= IO_MSK_D;

    PORTD = (PORTD & ~portDmask) | portDin;
    PORTB = (PORTB & ~portBmask) | portBin;
}
//_______________________________________________________________________________________
// Write a new trigger into the table
//_______________________________________________________________________________________
//
int IOsetTrig(unsigned char portBout, unsigned char portDout, unsigned char statusOut, unsigned char portBin, unsigned char portDin, unsigned char statusIn, unsigned char flagsIn) {
    // as soon as we receive a trigger to go into an empty EEPROM the get rid of the defaults
    if(EEPROMtrigCount == 0) trigCount = 0;

    if(trigCount >= TRIG_MAX) return false;		// we are full
    triggers[trigCount].outB = portBout & IO_MSK_B;
    triggers[trigCount].outD = portDout & IO_MSK_D;
    triggers[trigCount].outStatus = statusOut & IO_MSK_D;
    triggers[trigCount].inB = portBin & IO_MSK_B;
    triggers[trigCount].inD = portDin & IO_MSK_D;
    triggers[trigCount].statusIn = statusIn;
    triggers[trigCount].flags = flagsIn & 0x0F;			// incoming flags only in lower 4 bits
    triggers[trigCount].timerValue = flagsIn & 0xF0;	// incoming timer in upper nibble. which should make the incoming timer approx 1/4s of a second
    triggers[trigCount].timerActive = 0;

    int EEPROMpointer = (trigCount << 3) + EEPROM_TRIG;	// give 8 bytes for each trigger
    EEPROM_write(EEPROMpointer++, portBout & IO_MSK_B);
    EEPROM_write(EEPROMpointer++, portDout & IO_MSK_D);
    EEPROM_write(EEPROMpointer++, statusOut & IO_MSK_D);
    EEPROM_write(EEPROMpointer++, portBin & IO_MSK_B);
    EEPROM_write(EEPROMpointer++, portDin & IO_MSK_D);
    EEPROM_write(EEPROMpointer++, statusIn);
    EEPROM_write(EEPROMpointer++, flagsIn);

    trigCount++;
    EEPROM_write(EEPROM_TRIG_COUNT, trigCount);
    return true;
}

//_______________________________________________________________________________________
// Return a message with the state of the inputs
//_______________________________________________________________________________________
//
void IOreadInputs() {
    txData[0] = IO_MSG_IN;				// send an input pin message
    txData[1] = PINB & IO_MSK_B;		// only get the inputs of interest
    txData[2] = PIND & IO_MSK_D;		// only get the inputs of interest

    sendMsg(MSG_IO, 3);					// Write message to the serial port
}

void actionBits(triggerStruct *t, int f) {
    if(f) {
        statusFlags |= t->outStatus;
        PORTB |= t->outB;
        PORTD |= t->outD;
    } else {
        statusFlags &= ~t->outStatus;
        PORTB &= ~t->outB;
        PORTD &= ~t->outD;
    }
}

//_______________________________________________________________________________________
// Cycle through all of the IO triggers and act on them
//_______________________________________________________________________________________
//
void doIO() {
    // find out what has changed in order to do edge triggers
    unsigned char portBCopy = PINB & IO_MSK_B;					// take a snapshot of the inputs
    unsigned char portDCopy = PIND & IO_MSK_D;
    unsigned char statusCopy = statusFlags;
    unsigned char portBchanged = portBCopy ^ portBprevious;
    unsigned char portDchanged = portDCopy ^ portDprevious;
    unsigned char statusChanged = statusCopy ^ statusPrevious;
    unsigned char tempA, tempB;

    for(int n = 0; n < trigCount; n++) {				// step through all of the triggers into the trigger array
        // clear triggered output if timer expired
        if(triggers[n].flags & _BV(TRIG_TIMER)) {
            actionBits(&triggers[n], !(triggers[n].flags & _BV(TRIG_SET)));
            triggers[n].flags &= ~_BV(TRIG_TIMER);		// clear the timer finished flag
        }

        tempA = (portBchanged & triggers[n].inB) | (portDchanged & triggers[n].inD) | (statusChanged & triggers[n].statusIn); // isolate a relevant trigger change
        tempB = (portBCopy & triggers[n].inB) | (portDCopy & triggers[n].inD) | (statusCopy & triggers[n].statusIn); // and the current state of the relevant bit

        if((triggers[n].flags & _BV(TRIG_EDGE)) && tempA) {
            if(!(!!(triggers[n].flags & _BV(TRIG_RISE)) ^ !!tempB)) {
                actionBits(&triggers[n], (triggers[n].flags & _BV(TRIG_SET)));
                triggers[n].timerActive = triggers[n].timerValue;
                if (triggers[n].flags & _BV(TRIG_CLEAR)) statusFlags &= ~triggers[n].statusIn;
            }
        }

        // !( !!a ^ !!b) implements a robust XNOR for all integer values
        if(!(triggers[n].flags & _BV(TRIG_EDGE))) actionBits(&triggers[n], (!(!!(triggers[n].flags & _BV(TRIG_SET)) ^ !!tempB)));

    }

// save current state as previous
    portBprevious = portBCopy;
    portDprevious = portDCopy;
    statusPrevious = statusCopy;
}

//_______________________________________________________________________________________
// Process an incoming IO message and do the necessary
// return false if the message was formatted incorrectly
//_______________________________________________________________________________________
//
int recieveIOaction(int numData, unsigned char *data) {
    switch(data[0]) {
    case IO_MSG_RESET:				// 00 - Reset all IO. (1 byte)
        if(numData != 1) return false; // wrong number of data bytes
        resetIO();
        break;
    case IO_MSG_DDRD:				// 01 - Set DDRD (5 bytes BBDD DDRs  BBDD defaults)
        if(numData != 5) return false; // wrong number of data bytes
        setDDRD(data[1], data[2], data[3], data[4]);
        break;
    case IO_MSG_OUT:				// 02 - Set output (5 bytes BBDDBBDD mask:output)
        if(numData != 5) return false; // wrong number of data bytes
        setOutput(data[1], data[2], data[3], data[4]);
        break;
    case IO_MSG_IN:					// 03 - Read Input (1 byte) replies with 03 message with 3 bytes BBDD
        if(numData != 1) return false; // wrong number of data bytes
        IOreadInputs();
        return false;				// make sure we don't send an ACK message
        break;
    case IO_MSG_TRIG:				// 04 - Set trigger (8 bytes) BBDDSS (output) BBDDSS (source) FF (flags)
        if(numData != 8) return false; // wrong number of data bytes
        return IOsetTrig(data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
        break;
    default:
        return false;					// unknown - return error
        break;
    }
    return true;						// message processed successfully
}
//_______________________________________________________________________________________
// Decrements any active time triggers (This is called from interrupt routine)
//_______________________________________________________________________________________
//
void maintainTimeTriggers() {
    for(int n = 0; n < trigCount; n++) {				// step through all of the triggers into the trigger array
        if(triggers[n].timerActive != 0) {
            triggers[n].timerActive--;					// decrement any active timer
            if(triggers[n].timerActive == 0) triggers[n].flags |= _BV(TRIG_TIMER); // and set a timeout flag if it has reached 0
        }
    }
}

#endif /*CODE_SECTION_IO*/