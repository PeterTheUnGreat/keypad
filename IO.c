/*
 * IO.c
 *
 * Created: 05/06/2025 16:21:20
 *  Author: peter
 */ 


#include "IO.h"

#ifdef CODE_SECTION_IO

//_______________________________________________________________________________________
// Read all of the IO stuff from EEPROM and set up DDRs etc.
//_______________________________________________________________________________________
//
void initIO() {
	
}

//_______________________________________________________________________________________
// Cycle through all of the IO triggers and act on them
//_______________________________________________________________________________________
//
void doIO() {
	
	
// clear all active non-timed triggers
}

//_______________________________________________________________________________________
// Process an incoming message and add to the IO action table
//_______________________________________________________________________________________
//
void recieveIOaction() {
	
}
//_______________________________________________________________________________________
// Decrements any active time triggers (This is called from interrupt routine)
//_______________________________________________________________________________________
//
void maintainTimeTriggers() {
	
}

#endif /*CODE_SECTION_IO*/