/*
 * IO.c
 *
 * Created: 05/06/2025 16:21:20
 *  Author: peter
 */

#include <avr/io.h>

#include "Compile.h"
#include "Utils.h"
#include "Keypad.h"
#include "message.h"
#include "IO.h"

#ifdef CODE_SECTION_IO

struct Trigger triggers[TRIG_MAX];									// space to hold triggers

//_______________________________________________________________________________________
// Read all of the IO stuff from EEPROM and set up DDRs etc.
//_______________________________________________________________________________________
//
void initIO() {
    int EEPROMpointer;
    unsigned char flagsIn;

    trigCount = EEPROM_read(EEPROM_TRIG_COUNT);
    if(trigCount == 0xFF) trigCount = 0;			// just in case EEPROM has bee wiped

    for(int n = 0; n < trigCount; n++) {				// read in all of the triggers into the trigger array
        EEPROMpointer = (n << 3) + EEPROM_TRIG;	// give 8 bytes for each trigger
        triggers[trigCount].outB = EEPROM_read(EEPROMpointer++);
        triggers[trigCount].outD = EEPROM_read(EEPROMpointer++);
        triggers[trigCount].inB = EEPROM_read(EEPROMpointer++);
        triggers[trigCount].inD = EEPROM_read(EEPROMpointer++);
        triggers[trigCount].statusIn = EEPROM_read(EEPROMpointer++);
        flagsIn = EEPROM_read(EEPROMpointer++);

        triggers[trigCount].flags = flagsIn & 0x0F;			// incoming flags only in lower 4 bits
        triggers[trigCount].timerValue = (flagsIn & 0xF0) >> 1;	// incoming timer in upper nibble. which should make the incoming timer approx 1/10s of a second
        triggers[trigCount].timerActive = 0;
    }

    DDRB = (DDRB & ~IO_MSK_B) | EEPROM_read(EEPROM_DDRB);		// read the EEPROM copies
    DDRD = (DDRD & ~IO_MSK_D) | EEPROM_read(EEPROM_DDRD);
    PORTB = (PORTB & ~IO_MSK_B) | EEPROM_read(EEPROM_B_DEFAULT);
    PORTD = (PORTD & ~IO_MSK_D) | EEPROM_read(EEPROM_D_DEFAULT);

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
    // set unused pins to inputs
    DDRD &=	~IO_MSK_D;
    DDRB &=	~IO_MSK_B;

    EEPROM_write(EEPROM_DDRB, 0);					// reset the EEPROM copies to all inputs
    EEPROM_write(EEPROM_DDRD, 0);
    EEPROM_write(EEPROM_TRIG_COUNT, 0);				// delete trigger count table
}

//_______________________________________________________________________________________
// set DDRs
//_______________________________________________________________________________________
//
void setDDRD(unsigned char portBDDR, unsigned char portDDDR, unsigned char portBdefault, unsigned char portDdefault) {
    portBDDR &= IO_MSK_B;		// remove any unwanted bits
    portBdefault &= IO_MSK_B;
    portDDDR &= IO_MSK_D;
    portDdefault &= IO_MSK_D;

    DDRB = (DDRB & ~IO_MSK_B) | portBDDR;
    DDRD = (DDRD & ~IO_MSK_D) | portDDDR;

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
int IOsetTrig(unsigned char portBout, unsigned char portDout, unsigned char portBin, unsigned char portDin, unsigned char statusIn, unsigned char flagsIn) {
    if(trigCount == TRIG_MAX) return false;		// we are full
    triggers[trigCount].outB = portBout & IO_MSK_B;
    triggers[trigCount].outD = portDout & IO_MSK_D;
    triggers[trigCount].inB = portBin & IO_MSK_B;
    triggers[trigCount].inD = portDin & IO_MSK_D;
    triggers[trigCount].statusIn = statusIn;
    triggers[trigCount].flags = flagsIn & 0x0F;			// incoming flags only in lower 4 bits
    triggers[trigCount].timerValue = (flagsIn & 0xF0) >> 1;	// incoming timer in upper nibble. which should make the incoming timer approx 1/10s of a second
    triggers[trigCount].timerActive = 0;

    int EEPROMpointer = (trigCount << 3) + EEPROM_TRIG;	// give 8 bytes fro each trigger
    EEPROM_write(EEPROMpointer++, portBout & IO_MSK_B);
    EEPROM_write(EEPROMpointer++, portDout & IO_MSK_D);
    EEPROM_write(EEPROMpointer++, portBin & IO_MSK_B);
    EEPROM_write(EEPROMpointer++, portDin & IO_MSK_D);
    EEPROM_write(EEPROMpointer++, statusIn);
    EEPROM_write(EEPROMpointer++, flagsIn);

    EEPROM_write(EEPROM_TRIG_COUNT, trigCount);

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

    int affectBit;
    int bitState;

    for(int n = 0; n < trigCount; n++) {				// step through all of the triggers into the trigger array
        // clear triggered output if timer expired
        if(triggers[n].flags & _BV(TRIG_TIMER)) {
            if(triggers[n].flags & _BV(TRIG_SET)) {
                PORTB &= ~triggers[n].outB;
                PORTD &= ~triggers[n].outD;
            } else {
                PORTB |= triggers[n].outB;
                PORTD |= triggers[n].outD;
            }
            triggers[n].flags &= ~_BV(TRIG_TIMER);		// clear the timer finished flag
        }

        affectBit = false;
        bitState = true;

        tempA = (portBchanged & triggers[n].inB) | (portDchanged & triggers[n].inD) | (statusChanged & triggers[n].statusIn); // isolate a relevant trigger change
        tempB = (portBCopy & triggers[n].inB) | (portDCopy & triggers[n].inB) | (statusCopy & triggers[n].statusIn); // and the current state of the relevant bit

        if(triggers[n].flags & _BV(TRIG_EDGE)) {
            if(tempA && (((triggers[n].flags & _BV(TRIG_RISE)) && tempB) || (!(triggers[n].flags & _BV(TRIG_RISE)) && !tempB))) {
                affectBit = true;
                triggers[n].timerActive = triggers[n].timerValue;
                if (triggers[n].flags & _BV(TRIG_CLEAR)) statusFlags &= ~triggers[n].statusIn;
            }
        } else {
            affectBit = true;
            bitState = tempB;
        }

        // Invert the action if the TRIG_SET bit is clear
        if(triggers[n].flags & _BV(TRIG_SET)) bitState = !bitState;

        if(affectBit) {
            if(bitState) {
                PORTB |= triggers[n].outB;
                PORTD |= triggers[n].outD;
            } else {
                PORTB &= ~triggers[n].outB;
                PORTD &= ~triggers[n].outD;
            }
        }
    }

// save current state as previous
    portBprevious = PINB;
    portDprevious = PIND;
    statusPrevious = statusFlags;
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
    case IO_MSG_TRIG:				// 04 - Set trigger (7 bytes) BBDD (output) BBDDSS (source) FF (flags)
        if(numData != 7) return false; // wrong number of data bytes
        return IOsetTrig(data[1], data[2], data[3], data[4], data[5], data[6]);
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
            if(triggers[n].timerActive == 0) triggers[n].flags |= TRIG_TIMER; // and set a timeout flag if it has reached 0
        }
    }
}

#endif /*CODE_SECTION_IO*/