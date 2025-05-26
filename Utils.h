/*
 * Utils.h
 *
 * Created: 26/03/2025 18:12:55
 *  Author: peter
 */ 


#ifndef UTILS_H_
#define UTILS_H_

#define _FALSE	0
#define _TRUE	1

//_______________________________________________________________________________________
// Flags to control TWI
//_______________________________________________________________________________________
//
volatile unsigned char	TWI_flags;

#define		TWI_flag_writing		0		// set when we are transmitting
#define		TWI_flag_error			1		// set if the last TWI transaction failed
#define		TWI_busy				2		// we are doing a transfer

unsigned char	TWI_address;

volatile int	TWI_datacount;
#define		TWI_max_data			10
volatile	unsigned char	TWI_data[TWI_max_data];	// store for TWI data
//_______________________________________________________________________________________

void EEPROM_write(unsigned int uiAddress, unsigned char ucData);
unsigned char EEPROM_read(unsigned int uiAddress);
void EEPROM_write_string(unsigned int uiAddress, char *str, int n);
void EEPROM_read_string(unsigned int uiAddress, char *str, int n);

void i2cInit();
int i2cWrite(unsigned char IIC_addr, int n);
int i2cRead(unsigned char IIC_addr, int n);

unsigned char BCDByte(unsigned char n);

void debugOn();			
void debugOff();			
void debugToggle();
void debugPulse();

#endif /* UTILS_H_ */