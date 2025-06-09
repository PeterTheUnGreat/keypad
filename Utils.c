// Peter's utilities to drive the lock board (c) 2024

									
#include <avr/io.h>
#include <avr/wdt.h>
#include <util/twi.h>
#include <avr/interrupt.h>
#include <stdbool.h>

#include "Utils.h"
#include "Compile.h"

//_______________________________________________________________________________________
// EEPROM routines
//_______________________________________________________________________________________
//
void EEPROM_write(unsigned int uiAddress, unsigned char ucData)
{
	while(EECR & (1<<EEWE)) wdt_reset();					// Kick the watchdog while waiting for completion of previous write
	EEAR = uiAddress;										// Set up address and data registers
	EEDR = ucData;
	EECR |= (1<<EEMWE);										// Writing is performed by writing logical one to EEMWE followed by setting EEWE within 4 clock cycles
	EECR |= (1<<EEWE);
}

unsigned char EEPROM_read(unsigned int uiAddress)
{
	while(EECR & (1<<EEWE)) wdt_reset();					// Kick the watchdog while waiting for completion of previous write
	EEAR = uiAddress;										// Set up address and data registers
	EECR |= (1<<EERE);										// Start eeprom read by writing EERE
	return EEDR;											// Return data from data register
}

// Write a string of length n to given address in EEPROM
void EEPROM_write_string(unsigned int uiAddress, char *str, int n) {
	for(int i = 0; i < n; i++) EEPROM_write(uiAddress++, str[i]);
}

// Read a string of length n from given address in EEPROM
void EEPROM_read_string(unsigned int uiAddress, char *str, int n) {
	for(int i = 0; i < n; i++) str[i] = EEPROM_read(uiAddress++);
}


#ifdef	CODE_SECTION_IIC
//_______________________________________________________________________________________
// IIC routines
//_______________________________________________________________________________________
//
// handle the TWI interrupt. Note that the interrupt flag is not cleared by hardware and must be cleared manually in the interrupt routine
ISR(TWI_vect) {
	unsigned char controlCopy =  _BV(TWEN) | _BV(TWIE) | _BV(TWINT);	// clear the action bits and acknowledge ready for next thing

#ifdef CODE_SECTION_DEBUG	
	debugPulse();
#endif /* CODE_SECTION_DEBUG */	

	switch(TWSR & TW_STATUS_MASK) {
		case TW_START :
			TWDR = TWI_address;								// Set the address and read write bit
			break;
		case TW_MT_SLA_ACK:
		case TW_MT_DATA_ACK:
			if(TWI_index < TWI_datacount) TWDR = TWI_send_data[TWI_index++];
			else {
				controlCopy |= _BV(TWSTO);					// all finished, send stop
				controlCopy &= ~_BV(TWIE);					// turn off interrupts to indicate transmission complete
			}
			break;
		case TW_MR_SLA_ACK:
			if (TWI_datacount != 1) controlCopy |= _BV(TWEA);	// If we need to receive more than one byte then make sure an ACK is sent
			break;	
		case TW_MR_DATA_ACK:
			TWI_read_data[TWI_index++] = TWDR;				// get the next byte of data
			if(TWI_index < (TWI_datacount - 1)) controlCopy |= _BV(TWEA); // Acknowledge unless we are on the last byte
			break;
		case TW_MR_DATA_NACK:
			TWI_read_data[TWI_index] = TWDR;					// get the last byte of data
			controlCopy |= _BV(TWSTO);							// all finished, send stop
			controlCopy &= ~_BV(TWIE);					// turn off interrupts to indicate transmission complete
			break;
		default:
#ifdef CODE_SECTION_DEBUG
			debugPulse();
#endif /* CODE_SECTION_DEBUG */
			controlCopy |= _BV(TWSTO);					// an error has occurred, send stop and signal error#
			controlCopy &= ~_BV(TWIE);					// turn off interrupts to indicate transmission complete
			TWI_flags |= _BV(TWI_flag_error);
		break;
	}
	TWCR = controlCopy;									// This clears the interrupt and starts the next action
}

void i2cInit() {
	TWBR = 0x00;										// run the TWI as fast as it will go
	TWSR = 0x00;										// no pre-scaler
	TWCR = _BV(TWEN);									// turn on the TWI
}

int i2cTransfer(unsigned char IIC_addr, int n, int readWrite) {
//	if((TWCR & _BV(TWIE)) != 0) return 0;				// respond with -1 if a transaction is ongoing							
	TWI_datacount = n;
	TWI_index = 0;
	TWI_address = (IIC_addr << 1) + readWrite;
	TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN) | _BV(TWIE);	// send start turn on the TWI and enable interrupts
	return TWI_address;
}
#endif /* CODE_SECTION_IIC */	

//_______________________________________________________________________________________
// Useful stuff
//_______________________________________________________________________________________
//

unsigned char BCDByte(unsigned char n) {
	return ((n >> 4) * 10 ) + (n & 0x0F);
}

#ifdef CODE_SECTION_DEBUG

//_______________________________________________________________________________________
// Debugging routines
//_______________________________________________________________________________________
//

#define		DBG_OUT		3									// PD3 

void initDebug() { DDRD |= _BV(DBG_OUT); }

void debugOn() { PORTD |= _BV(DBG_OUT); }					// turn on the debug output
	
void debugOff() { PORTD &= ~_BV(DBG_OUT); }					// turn off the debug output
	
void debugToggle() { PORTD ^= _BV(DBG_OUT); }				// toggle off the debug output
	
void debugPulse() { debugToggle(); debugToggle(); }			// make a pulse on the debug line
	
//_______________________________________________________________________________________

#endif /* CODE_SECTION_DEBUG */
