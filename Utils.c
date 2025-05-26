// Peter's utilities to drive the lock board (c) 2024

									
#include <avr/io.h>
#include <avr/wdt.h>
#include <util/twi.h>
#include <avr/interrupt.h>

#include "Utils.h"

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

//_______________________________________________________________________________________
// IIC routines
//_______________________________________________________________________________________
//
// handle the TWI interrupt. Note that the interrupt flag is not cleared by hardware and must be cleared manually in the interrupt routine
ISR(TWI_vect) {
	TWCR &= ~(_BV(TWSTO) | _BV(TWSTA));					// clear the action bits ready for next thing
	unsigned char TWI_status = TWSR & TW_STATUS_MASK;	// get the status without the prescaler bits
	
	switch(TWI_status) {
		case TW_START :
			TWDR = (TWI_address << 1) + (((TWI_flags & _BV(TWI_flag_writing)) !=0) ? TW_WRITE : TW_READ ); // Set the address and read write bit
			break;
		case TW_MT_SLA_ACK:
			break;
		case TW_MT_DATA_ACK:
			break;
		case TW_MR_SLA_ACK:
			break;	
		case TW_MR_DATA_ACK:
			break;
		case TW_MR_DATA_NACK:
			TWCR |= _BV(TWSTO);							// all finished, send stop
			TWI_flags &= ~_BV(TWI_busy);
			break;
		default:
			TWCR |= _BV(TWSTO);							// an error has occurred, send stop and signal error
			TWI_flags |= _BV(TWI_flag_error);
			TWI_flags &= ~_BV(TWI_busy);
	}
	TWCR |= _BV(TWINT);									// This clears the interrupt and starts the next action
}

void i2cInit() {
	TWBR = 0;											// run the TWI as fast as it will go
	TWCR = _BV(TWEN) | _BV(TWIE);						// turn on the TWI and enable interrupts
}

void i2cStart() {
	TWCR |= _BV(TWINT) | _BV(TWSTA);					// send start
}

int i2cWrite(unsigned char IIC_addr, int n) {
	if((TWI_flags &= _BV(TWI_busy)) != 0) return -1;	// respond with -1 if a transaction is ongoing				
	TWI_flags = _BV(TWI_flag_writing) + _BV(TWI_busy);					
	TWI_datacount = n;
	TWI_address = IIC_addr;
	i2cStart();											// initiate transfer by sending send start condition
	return 0;
}

int i2cRead(unsigned char IIC_addr, int n) {
	if((TWI_flags &= _BV(TWI_busy)) != 0) return -1;	// respond with -1 if a transaction is ongoing
	TWI_flags = _BV(TWI_busy);
	TWI_datacount = n;
	TWI_address = IIC_addr;
	i2cStart();											// initiate transfer by sending send start condition
	return 0;
}

//_______________________________________________________________________________________
// Useful stuff
//_______________________________________________________________________________________
//

unsigned char BCDByte(unsigned char n) {
	return ((n >> 4) * 10 ) + (n & 0x0F);
}

//_______________________________________________________________________________________
// Debugging routines
//_______________________________________________________________________________________
//

#define		DBG_OUT		2									// PD2 which is shared with nREDE

void debugOn() { PORTD |= _BV(DBG_OUT); }					// turn on the debug output
	
void debugOff() { PORTD &= ~_BV(DBG_OUT); }					// turn off the debug output
	
void debugToggle() { PORTD ^= _BV(DBG_OUT); }				// toggle off the debug output
	
void debugPulse() { debugToggle(); debugToggle(); }			// make a pulse on the debug line
	
//_______________________________________________________________________________________


