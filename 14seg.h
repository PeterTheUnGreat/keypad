/* headers for routines to display stuff on 14 segment display */
#ifndef FOURTEEN_SEG_H
#define FOURTEEN_SEG_H

#include <avr/io.h>

 // Write ASCII character c to digit d of display
void dispWriteChar(char c, int d, unsigned char *buf);

// write string to display (Must be a 4 char buffer)
void dispWriteStr(const char *s, unsigned char *buf);

// write byte to display (8 bits only)
void dispWriteByte(unsigned char n, unsigned char *buf);

// write double byte to display (16 bits)
void dispWriteDouble(uint16_t n, unsigned char *buf);

// write time to display hour:minute
void dispWriteTime(unsigned char hour, unsigned char minute, unsigned char *buf);

#endif 					// FOURTEEN_SEG_H