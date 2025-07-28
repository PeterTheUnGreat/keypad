/*
 * IO.h
 *
 * Created: 05/06/2025 16:21:33
 *  Author: peter
 */

#include "Compile.h"

#ifndef IO_H_
#define IO_H_

#ifdef CODE_SECTION_IO

void initIO();
void doIO();
int recieveIOaction(int numData, unsigned char *data);
void maintainTimeTriggers();

//_______________________________________________________________________________________
// General stuff
//_______________________________________________________________________________________
//

unsigned char	portBprevious;
unsigned char	portDprevious;
unsigned char	statusPrevious;

//_______________________________________________________________________________________
// Trigger definitions
//_______________________________________________________________________________________
//

typedef struct {
    unsigned char	outB;
    unsigned char	outD;
    unsigned char	outStatus;
    unsigned char	inB;
    unsigned char	inD;
    unsigned char	statusIn;
    unsigned char	flags;
    unsigned char	timerValue;
    unsigned char	timerActive;
} triggerStruct;

int		trigCount;						// number of triggers we have
int		EEPROMtrigCount;				// number of triggers in EEPROM

#define TRIG_MAX	10					// max number of triggers in table

#define TRIG_EDGE	0					// trigger on edge if set otherwise track state
#define TRIG_RISE	1					// trigger on rising edge if set otherwise falling
#define TRIG_CLEAR	2					// clear trigger after use if set
#define TRIG_SET	3					// Set or clear the outputs
#define TRIG_TIMER	4					// Set if timer has expired

//_______________________________________________________________________________________
// Type definitions for IO messages
//_______________________________________________________________________________________
//

#define		IO_MSG_RESET		0		// 00 - Reset all IO. (1 byte)
#define		IO_MSG_DDRD			1		// 01 - Set DDRD (5 bytes BBDD DDRs  BBDD defaults)
#define		IO_MSG_OUT			2		// 02 - Set output (5 bytes BBDDBBDD mask:output)
#define		IO_MSG_IN			3		// 03 - Read Input (1 byte) replies with 03 message with 3 bytes BBDD
#define		IO_MSG_TRIG			4		// 04 - Set trigger (7 bytes) BBDD (output) BBDDSS (source) FF (flags)

#ifdef		CODE_SECTION_DEBUG
#define		IO_MSK_B			(_BV(PB4) + _BV(PB6) + _BV(PB7))	// PortB PB4, PB6, PB7 (PB1, PB2 used by debug)
#define		DDR_MSK_B			(_BV(PB4) + _BV(PB6) + _BV(PB7))
#else
#define		IO_MSK_B			(_BV(PB1) + _BV(PB2) + _BV(PB4) + _BV(PB6) + _BV(PB7))	// PortB PB1, PB2, PB4, PB6, PB7
#define		DDR_MSK_B			(_BV(PB1) + _BV(PB4) + _BV(PB6) + _BV(PB7))	// Leave direction of PB2 alone as it will blank the display if used as an input	
#endif		/*CODE_SECTION_DEBUG*/

#ifdef		CODE_SECTION_ROTARY
#define		IO_MSK_D			(_BV(PD5) + _BV(PD6) + _BV(PD7))	// PortD PD5-PD7 (PD6 and PD7 high current open drain) PD3 and PD4 used by rotary encoder
#define		DDR_MSK_D			(_BV(PD5) + _BV(PD6) + _BV(PD7))
#else
#define		IO_MSK_D			(_BV(PD3) + _BV(PD4) + _BV(PD5) + _BV(PD6) + _BV(PD7))	// PortD PD3-PD7 (PD6 and PD7 high current open drain)
#define		DDR_MSK_D			(_BV(PD3) + _BV(PD4) + _BV(PD5) + _BV(PD6) + _BV(PD7))
#endif		/*CODE_SECTION_ROTARY*/

#endif /*CODE_SECTION_IO*/
#endif /* IO_H_ */