// Peter's include file to drive the lock board (c) 2024

#ifndef 		KEYPAD_H_
#define 		KEYPAD_H_

#include		"Compile.h"

//_______________________________________________________________________________________
// General timing constants
//_______________________________________________________________________________________
//

#define F_CPU 	1000000UL			// we are running a 1MHz clock

#define			T1_1ms			125		// With a clock speed of 1MHz and prescaler of 8 give an interrupt every 1ms (0.4s per revolution 1.8deg steps with half stepping)
#define			T1_2_5ms		312		// With a clock speed of 1MHz and prescaler of 8 give an interrupt every 2.5ms (1s per revolution 1.8deg steps with half stepping)
#define			T1_2ms			250		// With a clock speed of 1MHz and prescaler of 8 give an interrupt every 2ms (0.8s per revolution 1.8deg steps with half stepping)
#define			T1_10ms			1250	// With a clock speed of 1MHz and prescaler of 8 give an interrupt every 10ms (4s per revolution 1.8deg steps with half stepping)
#define			T1_5ms			625		// With a clock speed of 1MHz and prescaler of 8 give an interrupt every 5ms (2s per revolution 1.8deg steps with half stepping)
#define			T1_100ms		12500	// With a clock speed of 1MHz and prescaler of 8 give an interrupt every 100ms (40s per revolution 1.8deg steps with half stepping)


//_______________________________________________________________________________________
// Other stuff
//_______________________________________________________________________________________
//

// Various flags to indicate stuff
volatile unsigned char	flags;

#define			FLASH_F			0			// message should be flashing
#define			KEYPAD_DIS		1			// set to a 1 if the keypad should be disabled

volatile unsigned char	holdOff;			// Counter to keep track of hold-off period

unsigned char	unitType;					// This holds a number to indicate what type of board this is
#define			TYPE_CLOCK		0
#define			TYPE_KEYPAD		1
#define			TYPE_SAFE		2
#define			TYPE_RANGE		3

volatile unsigned char statusFlags;
#define			STAT_UNLOCKED	0			// Flag to indicate correct code entered

//_______________________________________________________________________________________
// Display stuff
//_______________________________________________________________________________________
//
#define			RCLK 	0			// pin 0 of Port B latches display data
#define 		MOSI 	3
#define 		SCK 	5

unsigned char 	dispMem[8];		// reserve eight bytes for display memory
unsigned int	mem_ptr;			// pointer to display memory


//_______________________________________________________________________________________
// Keypad stuff
//_______________________________________________________________________________________
//
// Storage for button states
unsigned short	buttonState;		// Holds the current state of the buttons to be processed
unsigned short	buttonChanged;		// Holds flags for a change in button state
unsigned short 	debounceCount0;		// First set of bits of 'vertical counter'
unsigned short 	debounceCount1;		// Second set of bits of 'vertical counter'
unsigned short  sample;				// a temporary container for the values of switches collected over successive interrupts

#define			maxKeys		20		// most key presses we can hold
volatile unsigned char 	keyBuf[maxKeys];	// reserve twenty bytes to hold key presses until processed
volatile int	keyBufPtr;			// point at first key store location

//_______________________________________________________________________________________
// Stuff for key code variety
//_______________________________________________________________________________________
//
char code[5];

#ifdef  CODE_SECTION_CLOCK
//_______________________________________________________________________________________
// Stuff to do with stepper motors
//_______________________________________________________________________________________
//
// Motor B = minutes
// Motor A = hours

#define			motorAbits		0xC6	// Motor A on bits 1,2,6 and 7 of portB
#define			motorBbits		0xF0	// Motor B on bits 4,5,6 & 7 of portD (bits 6 and 7 inverted)

#define			MAZ				3		// Input from motor A zero sensor PD3
#define			MBZ				4		// Input from motor B zero sensor PB4

#define			MAX_STEP		7		// largest index into step table

unsigned char	motorAstep;				// value to hold which step we are on on motor A
unsigned char	motorBstep;				// value to hold which step we are on on motor B

#define			M_A_STEPS_REV	663		// how many steps per revolution for A and B
#define			M_A_STEPS_GAP	23		// how many steps to cross the sensing gap
#define			M_A_STEPS_HOUR	55		// how many steps per hour
#define			M_B_STEPS_REV	400

volatile unsigned short	motorAposition;			// where are we with motor A
volatile unsigned short	motorBposition;			// where are we with motor B

volatile unsigned short	motorAdesired;			// target position for motor A
volatile unsigned short	motorBdesired;			// target position for motor B

volatile unsigned char motorFlags;
#define			M_FLAG_A_REV 	0		// bits in motor flag
#define 		M_FLAG_B_REV 	1
#define			M_FLAG_A_SLOT	2
#define			M_FLAG_B_SLOT	3
#define			M_FLAG_A_MOVING	4
#define			M_FLAG_B_MOVING	5
#define			M_FLAG_A_GTR	6
#define			M_FLAG_B_GTR	7

//_______________________________________________________________________________________
// Stuff to do with clock
//_______________________________________________________________________________________
//

void StoreTimeValue(unsigned short	hourIn, unsigned short	minIn, unsigned char delayIn);

struct timeValue {
    unsigned short	Hour;
    unsigned short	Min;
    unsigned char	delay;
};

struct timeValue *timeList;
int			timeListPtrIn;
int			timeListPtrOut;

#define			MAX_TIME_LIST	10		// The most entries we can hold in the time list

unsigned short	Current_time_min;
unsigned short	Current_time_hour;
unsigned short	Current_time_tick;

unsigned short	Temp_time_min;
unsigned short	Temp_time_hour;

volatile unsigned char	clockFlags;
#define			flagTempDisplay		0	// Set if we are displaying a temporary time

volatile unsigned int	tempTimeDelay;

#define TICK_MIN	1875				// number of 32ms ticks in one minute
#endif /* CODE_SECTION_CLOCK */

//_______________________________________________________________________________________
// EEPROM allocations
//_______________________________________________________________________________________
//
#define			EEPROM_TYPE			0		// What type of hardware
#define			EEPROM_RS485_ADDR	1		// the address of this device on RS485
#define			EEPROM_CODE			2		// the code to guess (4 bytes)
#define			EEPROM_TIME			6		// store the current time (2 bytes)
#define			EEPROM_DDRB			7		// starting state for DDRB
#define			EEPROM_DDRD			8		// starting state for DDRD
#define			EEPROM_TRIG_COUNT	9		// number of trigger entries in table
#define			EEPROM_B_DEFAULT	10		// starting state for portB
#define			EEPROM_D_DEFAULT	11		// starting state for portD

#define			EEPROM_TRIG			0x100	// storage for trigger events
//				EEPROM_TRIG_END		0x1FF	// Allocate 256 bytes

//_______________________________________________________________________________________

#endif 							/* KEYPAD_H_ */

