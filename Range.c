/*
 * Range.c
 *
 * Created: 21/05/2025 20:06:44
 *  Author: peter
 */ 

#include "Utils.h"
#include <avr/wdt.h>

#define IIC_addr (0x29)								// I²C IIC_address of VL6180

//_______________________________________________________________________________________
// Split 16-bit register IIC_address into two bytes and write the IIC_address + data via I²C
//_______________________________________________________________________________________
//
int WriteByte(unsigned short reg, unsigned char data) {
	TWI_send_data[0] = (reg >> 8) & 0xFF;			// MSB of register IIC_address
	TWI_send_data[1] = reg & 0xFF;					// LSB of register IIC_address
	TWI_send_data[2] = data & 0xFF;
	for(int tries = 3; tries > 0 ; tries--) {
		i2cWrite(IIC_addr, 3); 
		while((TWI_flags & TWI_busy) != 0) wdt_reset();
		if ((TWI_flags & TWI_flag_error) == 0) return 0;
	}
	return -1;										 // return -1 to indicate fail after three tries
}

//_______________________________________________________________________________________
// Split 16-bit register IIC_address into two bytes and write required register IIC_address to VL6180 and read the data back
//_______________________________________________________________________________________
//
int ReadByte(unsigned short reg) {
	TWI_send_data[0] = (reg >> 8) & 0xFF;			// MSB of register IIC_address
	TWI_send_data[1] = reg & 0xFF;					// LSB of register IIC_address
	
	for(int tries = 3; tries > 0 ; tries--) {
		i2cWrite(IIC_addr, 2);
		while((TWI_flags & TWI_busy) != 0) wdt_reset();
		if ((TWI_flags & TWI_flag_error) == 0) {
			i2cRead(IIC_addr, 1);
			while((TWI_flags & TWI_busy) != 0) wdt_reset();
			if ((TWI_flags & TWI_flag_error) == 0) return 0;
		}
	}	
	return -1;										// return -1 to indicate fail after three tries	
}

//_______________________________________________________________________________________
// These are all the registers to be set on initialization
//_______________________________________________________________________________________
//
void VL6180_Init_Registers() {
	// Mandatory : private registers
	WriteByte(0x0207, 0x01);
	WriteByte(0x0208, 0x01);
	WriteByte(0x0096, 0x00);
	WriteByte(0x0097, 0xfd);
	WriteByte(0x00e3, 0x01);
	WriteByte(0x00e4, 0x03);
	WriteByte(0x00e5, 0x02);
	WriteByte(0x00e6, 0x01);
	WriteByte(0x00e7, 0x03);
	WriteByte(0x00f5, 0x02);
	WriteByte(0x00d9, 0x05);
	WriteByte(0x00db, 0xce);
	WriteByte(0x00dc, 0x03);
	WriteByte(0x00dd, 0xf8);
	WriteByte(0x009f, 0x00);
	WriteByte(0x00a3, 0x3c);
	WriteByte(0x00b7, 0x00);
	WriteByte(0x00bb, 0x3c);
	WriteByte(0x00b2, 0x09);
	WriteByte(0x00ca, 0x09);
	WriteByte(0x0198, 0x01);
	WriteByte(0x01b0, 0x17);
	WriteByte(0x01ad, 0x00);
	WriteByte(0x00ff, 0x05);
	WriteByte(0x0100, 0x05);
	WriteByte(0x0199, 0x05);
	WriteByte(0x01a6, 0x1b);
	WriteByte(0x01ac, 0x3e);
	WriteByte(0x01a7, 0x1f);
	WriteByte(0x0030, 0x00);

	// Recommended : Public registers - See data sheet for more detail
	WriteByte(0x0011, 0x10);					// Enables polling for ‘New Sample ready’ when measurement completes
	WriteByte(0x010a, 0x30);					// Set the averaging sample period (compromise between lower noise and increased execution time)
	WriteByte(0x003f, 0x46);					// Sets the light and dark gain (upper nibble). Dark gain should not be changed.
	WriteByte(0x0031, 0xFF);					// sets the # of range measurements after which auto calibration of system is performed
	WriteByte(0x0041, 0x63);					// Set ALS integration time to 100ms
	WriteByte(0x002e, 0x01);					// perform a single temperature calibration of the ranging sensor

	// Optional: Public registers - See data sheet for more detail
	WriteByte(0x001b, 0x09);					// Set default ranging inter-measurement period to 100ms
	WriteByte(0x003e, 0x31);					// Set default ALS inter-measurement period to 500ms
	WriteByte(0x0014, 0x24);					// Configures interrupt on ‘New Sample Ready threshold event’
	}

//_______________________________________________________________________________________
// load settings
//_______________________________________________________________________________________
//
int VL6180_Init() {
	i2cInit();									// prepare the IIC interface
	if(ReadByte(0x016) != 0) return -1;
	if (TWI_read_data[0] == 1){					// check to see has it be Initialized already
		VL6180_Init_Registers();		
		if(WriteByte(0x016, 0x00) != 0) return -1;	//change fresh out of set status to 0
	}
	return 0;
}

//_______________________________________________________________________________________
// Start a range measurement in single shot mode
//_______________________________________________________________________________________
//
int VL6180_Start_Range() {
	return WriteByte(0x018,0x01);
}

//_______________________________________________________________________________________
// poll for new sample ready ready
//_______________________________________________________________________________________
//
int VL6180_Poll_Range() {
	// wait for new measurement ready status
	do { ReadByte(0x04f); } while ((TWI_read_data[0] & 0x07) != 0x04);
	return 0;
}

//_______________________________________________________________________________________
// Read range result (mm)
//_______________________________________________________________________________________
//
int VL6180_Read_Range() {
	if(ReadByte(0x062) != 0) return -1;
	return TWI_read_data[0];
}

//_______________________________________________________________________________________
// clear interrupts
//_______________________________________________________________________________________
//
int VL6180_Clear_Interrupts() {
	return WriteByte(0x015,0x07);
}

