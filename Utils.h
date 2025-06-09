/*
 * Utils.h
 *
 * Created: 26/03/2025 18:12:55
 *  Author: peter
 */ 


#ifndef UTILS_H_
#define UTILS_H_

#include <stdbool.h>
#include "Compile.h"

#ifdef	CODE_SECTION_IIC

void i2cInit();
int i2cTransfer(unsigned char IIC_addr, int n, int readWrite);

//_______________________________________________________________________________________
// Flags to control TWI
//_______________________________________________________________________________________
//
volatile unsigned char	TWI_flags;

#define		TWI_flag_error			0		// set if the last TWI transaction failed

unsigned char	TWI_address;

volatile int	TWI_datacount;
volatile int	TWI_index;
#define			TWI_max_data			4
unsigned char	TWI_send_data[TWI_max_data];	// store for TWI data
volatile unsigned char	TWI_read_data[TWI_max_data];	// store for TWI data

#endif /* CODE_SECTION_IIC */	

//_______________________________________________________________________________________

void EEPROM_write(unsigned int uiAddress, unsigned char ucData);
unsigned char EEPROM_read(unsigned int uiAddress);
void EEPROM_write_string(unsigned int uiAddress, char *str, int n);
void EEPROM_read_string(unsigned int uiAddress, char *str, int n);

//_______________________________________________________________________________________


unsigned char BCDByte(unsigned char n);


#ifdef CODE_SECTION_DEBUG
void initDebug();
void debugOn();			
void debugOff();			
void debugToggle();
void debugPulse();
#endif /* CODE_SECTION_DEBUG */

#endif /* UTILS_H_ */