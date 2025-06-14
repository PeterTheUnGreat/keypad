/*
 * Range.h
 *
 * Created: 24/05/2025 14:58:51
 *  Author: peter
 */ 


#ifndef RANGE_H_
#define RANGE_H_

#ifdef	CODE_SECTION_RANGE

int VL6180_Init();
int VL6180_Start_Range();				// Start a range measurement in single shot mode
int VL6180_Poll_Range();				// poll for new sample ready ready
int VL6180_Read_Range();				// Read range result (mm)
int VL6180_Clear_Interrupts();			// clear interrupts

int ReadByte(unsigned short reg);
int WriteByte(unsigned short reg, unsigned char data);

#endif /* CODE_SECTION_RANGE */

#endif /* RANGE_H_ */