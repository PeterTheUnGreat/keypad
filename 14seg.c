// utilities to handle 14 segment display

#include "14seg.h"
#include <avr/pgmspace.h>

// this byte table is stored in program memory
const uint16_t FourteenSegmentASCII[96] PROGMEM = {
	0b000000000000000, /* (space) */
	0b100000000000110, /* ! */
	0b000001000000010, /* " */
	0b001001011001110, /* # */
	0b001001011101101, /* $ */
	0b011111111100100, /* % */
	0b000101101011001, /* & */
	0b000001000000000, /* ' */
	0b000110000000000, /* ( */
	0b010000100000000, /* ) */
	0b011111111000000, /* * */
	0b001001011000000, /* + */
	0b010000000000000, /* , */
	0b000000011000000, /* - */
	0b100000000000000, /* . */
	0b010010000000000, /* / */
	0b010010000111111, /* 0 */
	0b000010000000110, /* 1 */
	0b000000011011011, /* 2 */
	0b000000010001111, /* 3 */
	0b000000011100110, /* 4 */
	0b000100001101001, /* 5 */
	0b000000011111101, /* 6 */
	0b000000000000111, /* 7 */
	0b000000011111111, /* 8 */
	0b000000011101111, /* 9 */
	0b001001000000000, /* : */
	0b010001000000000, /* ; */
	0b000110001000000, /* < */
	0b000000011001000, /* = */
	0b010000110000000, /* > */
	0b101000010000011, /* ? */
	0b000001010111011, /* @ */
	0b000000011110111, /* A */
	0b001001010001111, /* B */
	0b000000000111001, /* C */
	0b001001000001111, /* D */
	0b000000001111001, /* E */
	0b000000001110001, /* F */
	0b000000010111101, /* G */
	0b000000011110110, /* H */
	0b001001000001001, /* I */
	0b000000000011110, /* J */
	0b000110001110000, /* K */
	0b000000000111000, /* L */
	0b000010100110110, /* M */
	0b000100100110110, /* N */
	0b000000000111111, /* O */
	0b000000011110011, /* P */
	0b000100000111111, /* Q */
	0b000100011110011, /* R */
	0b000000011101101, /* S */
	0b001001000000001, /* T */
	0b000000000111110, /* U */
	0b010010000110000, /* V */
	0b010100000110110, /* W */
	0b010110100000000, /* X */
	0b000000011101110, /* Y */
	0b010010000001001, /* Z */
	0b000000000111001, /* [ */
	0b000100100000000, /* \ */
	0b000000000001111, /* ] */
	0b010100000000000, /* ^ */
	0b000000000001000, /* _ */
	0b000000100000000, /* ` */
	0b001000001011000, /* a */
	0b000100001111000, /* b */
	0b000000011011000, /* c */
	0b010000010001110, /* d */
	0b010000001011000, /* e */
	0b001010011000000, /* f */
	0b000010010001110, /* g */
	0b001000001110000, /* h */
	0b001000000000000, /* i */
	0b010001000010000, /* j */
	0b001111000000000, /* k */
	0b000000000110000, /* l */
	0b001000011010100, /* m */
	0b001000001010000, /* n */
	0b000000011011100, /* o */
	0b000000101110000, /* p */
	0b000010010000110, /* q */
	0b000000001010000, /* r */
	0b000100010001000, /* s */
	0b000000001111000, /* t */
	0b000000000011100, /* u */
	0b010000000010000, /* v */
	0b010100000010100, /* w */
	0b010110100000000, /* x */
	0b000001010001110, /* y */
	0b010000001001000, /* z */
	0b010000101001001, /* { */
	0b001001000000000, /* | */
	0b000110010001001, /* } */
	0b010010011000000, /* ~ */
	0b000000000000000, /* (del) */
};
 
 
 // Write ASCII character c to digit d of display
void dispWriteChar(char c, int d, unsigned char *buf)
{
	uint16_t fs = pgm_read_word(&FourteenSegmentASCII[c - 32]);	// Get the 14 segment representation of the ASCII character from program memory
	buf[d * 2] = fs & 0x7F;							// Write low 7 bits into display memory
	buf[(d * 2) + 1] = fs >> 7;						// Write upper 7 bits into display memory
}

// write string to display (Must be a 4 char buffer)
void dispWriteStr(const char *s, unsigned char *buf)
{
	for (int i = 0; i <= 3; i++) dispWriteChar(s[i],i, buf);
}

// Get ASCII hex character from 16 bit number
char getHEXASCII(unsigned char n)
{
	if (n >= 10) return n - 10 + 'A';
	else return n + '0';
}

// write time to display hour:minute
void dispWriteTime(unsigned char hour, unsigned char minute, unsigned char *buf)
{
	dispWriteChar(getHEXASCII(hour / 10), 0, buf);
	dispWriteChar(getHEXASCII(hour % 10), 1, buf);
	dispWriteChar(getHEXASCII(minute / 10), 2, buf);
	dispWriteChar(getHEXASCII(minute % 10), 3, buf);
}

// write byte to display (8 bits only)
void dispWriteByte(unsigned char n, unsigned char *buf)
{
	dispWriteChar('0', 0, buf);										// pad with spaces
	dispWriteChar('x', 1, buf);
	dispWriteChar(getHEXASCII(n >> 4), 2, buf);					// write high nibble into first character
	dispWriteChar(getHEXASCII(n & 0x0F), 3, buf);				// write low nibble into first character
}

// write double byte to display (16 bits)
void dispWriteDouble(uint16_t n, unsigned char *buf)
{
	for(int i = 3; i >= 0; i--)
	{
	dispWriteChar(getHEXASCII(n & 0x0F), i, buf);				// write curreent nibble
	n >>= 4;													// prepare for next nibble
	}
}
