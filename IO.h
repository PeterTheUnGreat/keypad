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

struct Trigger {
    unsigned char	outB;
    unsigned char	outD;
    unsigned char	inB;
    unsigned char	inD;
    unsigned char	statusIn;
    unsigned char	flags;
    unsigned char	timerValue;
    unsigned char	timerActive;
};

int		trigCount;						// number of triggers we have

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

#define		IO_MSK_B			0xD6	// PortB PB1, PB2, PB4, PB6, PB7


#ifdef		CODE_SECTION_DEBUG
#define		IO_MSK_D			0xE0	// PortD PD5-PD7 (PD6 and PD7 high current open drain) PD3 and PD4 used by debug
#else
#define		IO_MSK_D			0xF8	// PortD PD3-PD7 (PD6 and PD7 high current open drain)
#endif		/*CODE_SECTION_DEBUG*/

#endif /*CODE_SECTION_IO*/
#endif /* IO_H_ */