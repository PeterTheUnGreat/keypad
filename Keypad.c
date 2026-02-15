// Peter's program to drive the lock board (c) 2024


#include "Compile.h"
#include "Keypad.h"										// this first to make sure util/delay.h picks up correct clock speed
#include <avr/io.h>
#include <util/delay_basic.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <stdlib.h>
#include <stdbool.h>
#include <util/twi.h>


#include "14seg.h"
#include "message.h"
#include "Utils.h"
#include "Menu.h"
#include "Range.h"
#include "IO.h"
#include "Telephone.h"
#include "Chess.h"
#include "NFC.h"

// This section bellow does all of the startup stuff
void setup(void) __attribute__ ((naked)) __attribute__ ((section (".init5"))); // do the setup before main called
void start_interrupts(void) __attribute__ ((naked)) __attribute__ ((section (".init8"))); // start interrupts just before main called


const menuStruct menuItems[] PROGMEM = {
    // Item Name, Digits, Type, EEPROM address, Flags
    { "TIME", 0, MENU_TYPE_NULL, 0, 0 },								// menu entry for clock - no keypad entry
    { "----", 4, MENU_TYP_CODE, 0, _BV(MENU_NUM) + _BV(MNEU_ENTRY) },	// menu entry for safe - keypad entry allowed
    { "SAFE", 0, MENU_TYPE_NULL, 0, 0 },								// menu entry for safe - no keypad entry
    { "DIST", 0, MENU_TYPE_NULL, 0, 0 },								// menu entry for range - no keypad entry
    { "CHES", 0, MENU_TYPE_NULL, 0, 0 },								// menu entry for chess - no keypad entry

    { "TEST", 0, MENU_TYP_TEST, 0, _BV(MENU_FWD) + _BV(MENU_MENU)},
    { "TYPE", 1, MENU_TYP_BYTE, EEPROM_TYPE, _BV(MENU_FWD) + _BV(MENU_BK) + _BV(MENU_MENU) + _BV(MENU_NUM)},
    { "ADDR", 3, MENU_TYP_BYTE, EEPROM_RS485_ADDR, _BV(MENU_BK) + _BV(MENU_FWD) + _BV(MENU_MENU) + _BV(MENU_NUM)},
    { "CODE", 4, MENU_TYP_BCD, EEPROM_CODE, _BV(MENU_BK) + _BV(MENU_FWD) + _BV(MENU_MENU) + _BV(MENU_NUM)},
    { "TIME", 4, MENU_TYP_TIME, EEPROM_TIME, _BV(MENU_BK) + _BV(MENU_FWD) + _BV(MENU_MENU) + _BV(MENU_NUM)},
    { "EXIT", 0, MENU_TYP_EXIT, 0 , _BV(MENU_BK) + _BV(MENU_MENU) }
};

#ifdef		CODE_SECTION_ROTARY
const safeStates safeCode[] = {
    { 10, true },
    { 15, false },
    { 20, true  },
    { 0, 0  }
};
#endif /*CODE_SECTION_ROTARY*/

#define MAX_TYPES	5		// This will determine the value that type is constrained to and also the settings menu position

const unsigned char motorAsteps[8] = { 0x82, 0x02, 0x06, 0x04, 0x44, 0x40, 0xC0, 0x80 }; // Values to half steps motor on port B
const unsigned char motorBsteps[8] = { 0x50, 0xD0, 0xF0, 0xE0, 0xA0, 0x80, 0x00, 0x40 }; // Values to half steps motor on port D

//_______________________________________________________________________________________
// Setup Stuff
//_______________________________________________________________________________________
//

void initDisplay() {
    mem_ptr	=	0;
    // Set SS as output to stop display being blanked
    DDRB |= _BV(RCLK) + _BV(MOSI) + _BV(SCK) + _BV(SS);		 	// set pins 0, 3 & 5 of port B as output pins (latch on high for display chip, MOSI and SCK)

    DDRC = 0x07;										// enable lower 3 bits of portc as outputs

#ifdef __AVR_ATmega328PB__
    SPCR0 = _BV(SPE) + _BV(MSTR) + _BV(SPIE); 			// Turn on spi in master mode with data clocked on rising edge of CLK and enable transfer complete interrupt
    TCCR0B &= ~0x07;

# ifdef CLOCK_X8
    TCCR0B |= _BV(CS01) + _BV(CS00); 					// set timer0 clock source to prescaler/64 (clock is going 8 times faster)
# else
    TCCR0B |= _BV(CS01); 								// set timer0 clock source to prescaler/8
# endif /* CLOCK_X8 */



    TIMSK0 |= _BV(TOIE0);								// enable timer 0 interrupts
#else
    SPCR = _BV(SPE) + _BV(MSTR) + _BV(SPIE); 			// Turn on spi in master mode with data clocked on rising edge of CLK and enable transfer complete interrupt
    TCCR0 &= ~0x07;
    TCCR0 |= _BV(CS01); 								// set timer0 clock source to prescaler/8
    TIMSK |= _BV(TOIE0);								// enable timer 0 interrupts
#endif

    flags &= ~_BV(FLASH_F);								// start off not flashing
    holdOff = 0;
}

void initKeyInput() {
    buttonState = 0;								// Holds the current state of the buttons to be processed
    buttonChanged = 0;								// Holds flags for a change in button state
    debounceCount0 = 0;								// First set of bits of 'vertical counter'
    debounceCount1 = 0;								// Second set of bits of 'vertical counter'
    sample = 0;										// a temporary container for the values of switches collected over successive interrupts

    keyBufPtr	= 0;								// point at first key store location
}

#ifdef		CODE_SECTION_ROTARY
void initRotary() {
    DDRD &= ~(_BV(PD3) + _BV(PD4));					// Set PD3 and PD4 as inputs
    PORTD |= (_BV(PD3) + _BV(PD4));					// Enable pull-ups

    MCUCR |= _BV(ISC11);							// INT1 on falling edge

#ifdef __AVR_ATmega328PB__
    EIMSK |= _BV(INT1);								// Enable INT1
#else
    GICR |= _BV(INT1);								// Enable INT1
#endif

    rotaryCount = 0;
    directionCount = DIRECTION_DEBOUNCE / 2;
    rotaryClockwise = true;

    safeState = 0;									// reset the state machine that unlocks the safe
}
#endif /* CODE_SECTION_ROTARY */

#ifdef CODE_SECTION_CLOCK
void initClock() {
    DDRB |= motorAbits;								// set the motor output pins for stepper A as output
    DDRD |= motorBbits;								// set the motor output pins for stepper B as output
    OCR1A = T1_2_5ms;

#ifdef __AVR_ATmega328PB__
    TIMSK1 |= _BV(OCIE1A);							// Set the timer to trigger an interrupt every 2.5ms using output compare A
#else
    TIMSK |= _BV(OCIE1A);							// Set the timer to trigger an interrupt every 2.5ms using output compare A
#endif

#ifdef CLOCK_X8
    TCCR1B = _BV(CS11) + _BV(CS10) + _BV(WGM12);	// Start timer 1 with prescaler of 64 and in CTC mode (we have sped up the clock by a factor of 8)
#else
    TCCR1B = _BV(CS11) + _BV(WGM12);				// Start timer 1 with prescaler of 8 and in CTC mode
#endif /* CLOCK_X8 */



    motorAstep = 0;									// value to hold which step we are on on motor A
    motorBstep = 0;									// value to hold which step we are on on motor B
    motorAposition	= 0;							// where are we with motor A
    motorBposition	= 0;							// where are we with motor B

    motorAdesired	= 0;							// how far to drive motor A
    motorBdesired	= 0;							// how far to drive motor B

    motorFlags = 0;

    PORTB &= ~_BV(MBZ);								// Setting this low will turn off the pullup on the zero input pins
    PORTD &= ~_BV(MAZ);

#ifdef __AVR_ATmega328PB__
    TIMSK2 |= _BV(OCIE2A);							// turn on timer 2 interrupts
    TCCR2A = _BV(WGM21);							// Start timer 2 in CTC mode

# ifdef CLOCK_X8
    TCCR2B = _BV(CS22) + _BV(CS21) + _BV(CS20);		// set a prescaler of 1024 (we have sped up the clock by a factor of 8)
    OCR2A = 250;									// This will give an interrupt every 32ms with a 8MHz clock. I.e. 1875 ticks/min
# else
    TCCR2B = _BV(CS22) + _BV(CS21);					// Set a prescaler of 256
    OCR2A = 125;									// This will give an interrupt every 32ms with a 1MHz clock. I.e. 1875 ticks/min
# endif /* CLOCK_X8 */

#else
    TCCR2 = _BV(WGM21) + _BV(CS22) + _BV(CS21);		// Start timer 2 in CTC mode with a prescaler of 256
    OCR2 = 125;										// This will give an interrupt every 32ms with a 1MHz clock. I.e. 1875 ticks/min
    TIMSK |= _BV(OCIE2);							// turn on timer 2 interrupts
#endif

    Current_time_hour = EEPROM_read(EEPROM_TIME);	// Read the stored time and check it is within range
    Current_time_min = EEPROM_read(EEPROM_TIME + 1);
    if((Current_time_hour > 11) || (Current_time_min > 59)) Current_time_hour = Current_time_min = 0;
    Current_time_tick = 0;

    clockFlags = 0;									// make sure we start of displaying normal time
    tempTimeDelay = 0;

    timeList = malloc( MAX_TIME_LIST * sizeof( struct timeValue));	// an array to hold time values
    timeListPtrIn = 0;
    timeListPtrOut = 0;
}
#endif /* CODE_SECTION_CLOCK */

// called during .init5 after variables have been initialized
void setup() {
    PORTB = 0xFF;
    PORTC = 0xFF;
    PORTD = 0xFF;										// Starting with all ports set to 0xFF will make any pins configured as inputs have pullups enabled

    wdt_enable(WDTO_2S);								// turn on watchdog and enable change of the prescaler (must be done within 4 clock cycles

//	OSCCAL = 0xB8;										// trim the internal oscillator to get timing accurate

    flags = 0;											// set all flags to 0 to start with
    statusFlags = 0;
    tempFlags = 0;
    timeOut = 0;

    initDisplay();
    initKeyInput();

    // Check if the 0 key is held after a reset - if so erase the EEPROM and set a flag for a later message
    // must be after display set up so that DDRC is set properly
    PORTC &= ~0x07;										// Set the keypad row select to 0 for the 0 key
    if ((MCUSR & _BV(EXTRF)) && !(PINC & _BV(PC3))) { // key 0 is on PC3 which will be 0 if pressed
        EEPROM_erase();
        statusFlags |= _BV(STAT_DID_RST);						// signal for a later message
    }

#ifdef		CODE_SECTION_ROTARY
    initRotary();
#endif /* CODE_SECTION_ROTARY */

#ifdef CODE_SECTION_TELEPHONE
    initTelephone();
#endif /* CODE_SECTION_TELEPHONE */

#ifdef CODE_SECTION_DEBUG
    initDebug();
#endif /* CODE_SECTION_DEBUG */
#ifdef  CODE_SECTION_IO
    initIO();
#endif /* CODE_SECTION_IO */

    initComms(EEPROM_read(EEPROM_RS485_ADDR));			// Start the RS485 comms with this units address read from EEPROM
    unitType = EEPROM_read(EEPROM_TYPE);				// Get the type of this unit from EEPROM
    if (unitType >= MAX_TYPES) unitType = TYPE_CLOCK;   // bit of a failsafe

    // Do unit specific setup
    menuItem = unitType;								// point at the correct menu

    switch(unitType) {
    case TYPE_CLOCK:
#ifdef CODE_SECTION_CLOCK
        initClock();
#endif /* CODE_SECTION_CLOCK */
        break;
    case TYPE_KEYPAD:
        EEPROM_read_string(EEPROM_CODE, code, 4);		// get the code from EEPROM
        break;
    case TYPE_SAFE:
        break;
    case TYPE_RANGE:
        break;
    case TYPE_CHESS:
        break;
    default:
        break;
    }

// If we are using the special *8 clock feature in the 328pb then turn on (THIS MUST BE DONE WITH INTERRUPTS OFF)
#ifdef CLOCK_X8
    CLKPR = _BV(CLKPCE);											// prepare to change the pre-scaler in the next 4 cycles
    CLKPR = 0;														// Set the system clock pre-scaler to 1
#endif /* CLOCK_X8 */
}

// called during .init8
void start_interrupts() {
    sei();												// turn on interrupt system
}

//_______________________________________________________________________________________
// Interrupt handlers
//_______________________________________________________________________________________
//

#ifdef CODE_SECTION_ROTARY
ISR(INT1_vect) {
    if (PIND & _BV(PD4)) {
        // clockwise
        if(directionCount < DIRECTION_DEBOUNCE) directionCount++;
        // if a change of direction is detected then set the direction to clockwise
        if((directionCount == DIRECTION_DEBOUNCE) && !rotaryClockwise) {
            rotaryClockwise = true;
            rotaryCount = 0;
            // a change of direction must reset the state machine
            safeState = 0;
        }

        if(rotaryClockwise) {
            rotaryCount++;
        } else {
            rotaryCount--;
        }

    } else {
        //anti-clockwise
        if(directionCount > 0) directionCount--;
        // if a change of direction is detected then set the direction to anti-clockwise
        if((directionCount == 0) && rotaryClockwise) {
            rotaryClockwise = false;
            rotaryCount = 0;
            // a change of direction must reset the state machine
            safeState = 0;
        }

        if(!rotaryClockwise) {
            rotaryCount++;
        } else {
            rotaryCount--;
        }
    }

    // Only take any action if the stethoscope is present
    if(statusFlags & _BV(STAT_STETH)) {
        // Have we reached the next state?
        if(rotaryCount == safeCode[safeState].steps) {
            if((rotaryClockwise && safeCode[safeState].clockwise) || (!rotaryClockwise && !safeCode[safeState].clockwise)) {
                safeState++;
                statusFlags |= _BV(STAT_PULSE); // Make a click
                // prepare for a change of direction and fudge the de-bounce counter
                if (rotaryClockwise) {
                    rotaryClockwise = false;
                    directionCount = 0;
                } else {
                    rotaryClockwise = true;
                    directionCount = DIRECTION_DEBOUNCE;
                }
                rotaryCount = 0;
                // Check to see if we have unlocked?
                if(safeCode[safeState].steps == 0) {
                    safeState = 0;
                    statusFlags |= _BV(STAT_UNLOCKED);
                    tempFlags |= _BV(TF_UNLOCKED);
                }
            }
        }
    }
}
#endif /* CODE_SECTION_ROTARY */

ISR(TIMER0_OVF_vect) {								// handle timer 0 overflow
#ifdef __AVR_ATmega328PB__
    if((flags & _BV(FLASH_F)) && (holdOff & 0x10)) SPDR0 = 0xff; // blank the display every approx. 200ms if we are flashing
    else SPDR0 = 0xff - dispMem[mem_ptr];				// write the given one to SPI
    SPCR0 |= _BV(MSTR);									// Make sure we are in master mode, just in case nSS was driven low
#else
    if((flags & _BV(FLASH_F)) && (holdOff & 0x10)) SPDR = 0xff; // blank the display every approx. 200ms if we are flashing
    else SPDR = 0xff - dispMem[mem_ptr];				// write the given one to SPI
    SPCR |= _BV(MSTR);									// Make sure we are in master mode, just in case nSS was driven low
#endif

    if(nREDE_Holdoff != 0) nREDE_Holdoff--;				// maintain the nREDE hold off timer
}

ISR(SPI_STC_handler) {

    PORTB |= _BV(RCLK);  								// set pin 0 of port B high
    PORTB &= ~_BV(RCLK); 								// set pin 0 of port B low  // latch onto display
    PORTC = (PORTC & ~0x07) | mem_ptr;					// Select the current digit

    mem_ptr++;											// Point at next half digit
    if (mem_ptr > 0x07) mem_ptr = 0;					// Make sure memory pointer never exceeds array size

    /* Read switch values
     Bits PC4, PC5 and PC6 contain the button presses in the following order

    	(mem_ptr)	PC4	PC5	PC6
    	Y3			S7	S8	S9
    	Y2			S4	S5	S6
    	Y1			S1	S2	S3
    	Y0			S0
    */
    unsigned short portCtemp = ~PINC & 0b00111000;		// extract only the lines representing buttons

    if (mem_ptr == 1) {
        sample = ((portCtemp >> 3) & 0b0000000000000001);    // The first one of the cycle clears all other bits in sample Just the one bit to go in the bottom
    }
    if (mem_ptr == 2) {
        sample |= portCtemp >> 2;
    }
    if (mem_ptr == 3) {
        sample |= portCtemp << 1;
    }
    if (mem_ptr == 4) {
        sample |= portCtemp << 4;
    }

    if(mem_ptr == 0x07) {								// only do the debounce on the 8th digit - this means all buttons will be sampled
        if(holdOff != 0) holdOff--;								// if the holdoff counter is active then decrement it
        if(timeOut != 0) timeOut--;								// if the timeOut counter is active then decrement it

#ifdef CODE_SECTION_IO
        maintainTimeTriggers();
#endif /*CODE_SECTION_IO*/

        // Switch debounce stuff here to make sure it does not upset timing make a bright spot on the display
        // Debounce switches using vertical counters counting up to 4.
        // Only debounces on switch release (assuming a 1 = switch pressed)
        unsigned short delta;

        delta = buttonState ^ sample;						// bits in delta indicate a change of state since last time
        debounceCount1 = (debounceCount1 ^ debounceCount0) & (delta & sample); // ripple the counter along (unless button not changed or it is a release)
        debounceCount0 = ~debounceCount0 & (delta & sample); // only start a de-bounce count if a button is released
        buttonChanged = (delta & ~(debounceCount0 | debounceCount1)); // bits set if a change has occurred
        buttonState ^= buttonChanged; 						// Update the stored state of the buttons only when count gets to 4)

        if(buttonChanged && (keyBufPtr < 20)) {			// only store button presses if space in buffer
            uint16_t s = buttonState & buttonChanged;				// we are only interested in presses
            for ( unsigned char i = '0'; i <= '9'; i++) {
                if (s & 0x01) {
                    keyBuf[keyBufPtr] = i;     // if button pressed then store in buffer
                    keyBufPtr++;
                }
                s >>= 1;
            }
            buttonChanged = 0;
        }
    }
}

#ifdef CODE_SECTION_CLOCK

ISR(TIMER2_COMP_handler) {								// handle timer 2 output compare interrupt
    // maintain the delay used to control the temp time if a timer is active
    if((tempTimeDelay != 0) && (tempTimeDelay != 0xFF)) {
        if (--tempTimeDelay == 0) clockFlags &= ~_BV(flagTempDisplay);
    }

    // Maintain the current time
    if(++Current_time_tick >= TICK_MIN) {
        Current_time_tick = 0;
        if(++Current_time_min >= 60) {
            Current_time_min = 0;
            if(++Current_time_hour >= 12) Current_time_hour = 0;
        }
    }
}

ISR(TIMER1_COMPA_vect) {								// handle timer 1 output compare A interrupt
    // turn stepper A if it should be moving
    if((motorFlags & (_BV(M_FLAG_A_MOVING))) != 0) {
        if((motorFlags & _BV(M_FLAG_A_REV)) == 0) {
            if(motorAstep >= MAX_STEP) motorAstep = 0;
            else motorAstep++;
            if(++motorAposition >= M_A_STEPS_REV) motorAposition = 0;
        } // if going forward inc step and wrap around
        else {
            if(motorAstep == 0) motorAstep = MAX_STEP;
            else motorAstep--;
            if(motorAposition == 0) motorAposition = (M_A_STEPS_REV - 1);
            else motorAposition--;
        } // if going backwards dec step and wrap around
        unsigned char b = PORTB & ~motorAbits;
        PORTB = b | motorAsteps[motorAstep];				// This way PORTB does not glitch if unchanged

        // if the slot is leaving the sensor as we are going forward then set motor A position as 0
        if(((PIND & _BV(MAZ)) != 0) && ((motorFlags & _BV(M_FLAG_A_SLOT)) == 0) && ((motorFlags & _BV(M_FLAG_A_REV)) == 0)) {
            if(motorAdesired > motorAposition) motorAdesired = 0;
            motorAposition = 0;
        }

        // if the slot is entering the sensor as we are going backwards then set motor A position as max
        if(((PIND & _BV(MAZ)) == 0) && ((motorFlags & _BV(M_FLAG_A_SLOT)) != 0) && ((motorFlags & _BV(M_FLAG_A_REV)) != 0)) {
            motorAposition = M_A_STEPS_REV - 1;
            if(motorAdesired >= M_A_STEPS_REV) motorAdesired = M_A_STEPS_REV - 1;
        }

        if((PIND & _BV(MAZ)) != 0) motorFlags |= _BV(M_FLAG_A_SLOT);
        else motorFlags &= ~_BV(M_FLAG_A_SLOT);
        if(motorAposition == motorAdesired) motorFlags &= ~_BV(M_FLAG_A_MOVING); // Stop moving if we have reached our destination
    }

    // turn stepper B if it should be moving
    if((motorFlags & (_BV(M_FLAG_B_MOVING))) != 0) {
        if((motorFlags & _BV(M_FLAG_B_REV)) == 0) {
            if(motorBstep >= MAX_STEP) motorBstep = 0;
            else motorBstep++;
            if(++motorBposition >= M_B_STEPS_REV) motorBposition = 0;
        } // if going forward inc step and wrap around
        else {
            if(motorBstep == 0) motorBstep = MAX_STEP;
            else motorBstep--;
            if(motorBposition == 0) motorBposition = (M_B_STEPS_REV - 1);
            else motorBposition--;
        } // if going backwards decrement step and wrap around

        unsigned char d = PORTD & ~motorBbits;
        PORTD = d | motorBsteps[motorBstep];				// This way PORTD does not glitch if unchanged

        // if the slot is leaving the sensor as we are going forward then set motor B position as 0
        if(((PINB & _BV(MBZ)) != 0) && ((motorFlags & _BV(M_FLAG_B_SLOT)) == 0) && ((motorFlags & _BV(M_FLAG_B_REV)) == 0)) {
            motorBposition = 0;
            if(motorBdesired > motorBposition) motorBdesired = 0;
        }

        // if the slot is entering the sensor as we are going backwards then set motor B position as max
        if(((PINB & _BV(MBZ)) == 0) && ((motorFlags & _BV(M_FLAG_B_SLOT)) != 0) && ((motorFlags & _BV(M_FLAG_B_REV)) != 0)) {
            motorBposition = M_B_STEPS_REV - 1;
            if(motorBdesired >= M_B_STEPS_REV) motorBdesired = M_B_STEPS_REV - 1;
        }

        if((PINB & _BV(MBZ)) != 0) motorFlags |= _BV(M_FLAG_B_SLOT);
        else motorFlags &= ~_BV(M_FLAG_B_SLOT);
        if(motorBposition == motorBdesired) motorFlags &= ~_BV(M_FLAG_B_MOVING); // Stop moving if we have reached our destination
    }
}

#endif /* CODE_SECTION_CLOCK */

//_______________________________________________________________________________________
// General stuff
//_______________________________________________________________________________________
//
// these are the things that need to be maintained all the section needs to be done all the time
void doMainStuff() {
    processReceive();
    checkMessage();
#ifdef  CODE_SECTION_IO
    doIO();
#endif /* CODE_SECTION_IO */
//	checkMsgError();
    wdt_reset();										// Kick the watchdog
}

void displayAndWaitCore(int t, unsigned char flg) {
    flags = flg;
    holdOff = t;
    while(holdOff != 0) doMainStuff();	// Wait while maintaining things that need to be maintained
}

void displayAndWait(char *str, int t, unsigned char flg) {
    dispWriteStr( str, dispMem);						// display the given string
    displayAndWaitCore(t, flg);
}

void displayDoubleAndWait(unsigned short n, int t, unsigned char flg) {
    dispWriteDouble( n, dispMem);						// display the given string
    displayAndWaitCore(t, flg);
}

void displayByteAndWait(unsigned char n, int t, unsigned char flg, char firstChar, char secondChar) {
    dispWriteByte( n, dispMem, firstChar, secondChar);						// display the given string
    displayAndWaitCore(t, flg);
}

// find and display the source of reset
void signalResetSource() {
    char chr = '?';

#ifdef __AVR_ATmega328PB__
    unsigned char source = MCUSR;						// get the reset source
    MCUSR = 0;											// clear reset sources
#else
    unsigned char source = MCUCSR;						// get the reset source
    MCUCSR = 0;											// clear reset sources
#endif

    if (source & _BV(WDRF)) chr = 'W';
    if (source & _BV(BORF)) chr = 'B';
    if (source & _BV(EXTRF)) {
        chr = 'E';    // Set to enter setup menu after external reset
        menuItem = MAX_TYPES;
    }
    if (source & _BV(PORF))  chr = 'P';		// this one last because power on also sets brownout

    displayByteAndWait(msgAddr, 64, _BV(FLASH_F), chr, '-');				// Display a message for a given time and block

    if(statusFlags & _BV(STAT_DID_RST)) displayAndWait("RST ", 64, 0);	// if we reset the EEPROM earlier - show a message
}

// execute a test routine
void doTest() {
    sendMsg('P', 0);									// Write message to the serial port
    displayAndWait("SENT", 64, _BV(FLASH_F));
}

// This is just to unmuddle the keypad on the 328pb version of the board where the rows of keys are back to front!
char mapKey(char keyPress) {
#ifdef __AVR_ATmega328PB__
    switch(keyPress) {
    case '1' :
        keyPress = '3';
        break;
    case '3' :
        keyPress = '1';
        break;
    case '4' :
        keyPress = '6';
        break;
    case '6' :
        keyPress = '4';
        break;
    case '7' :
        keyPress = '9';
        break;
    case '9' :
        keyPress = '7';
        break;
    }
#endif
    return keyPress;
}

//_______________________________________________________________________________________
// Clock specific stuff
//_______________________________________________________________________________________
//

#ifdef CODE_SECTION_CLOCK

// Work out the motor positions to show the current time
void showAnalogueTime(unsigned short hour, unsigned short min, int Obvious) {
    unsigned short temp;
    // Only set a new movement if the current one has finished
    if((motorFlags & (_BV(M_FLAG_A_MOVING) + _BV(M_FLAG_B_MOVING))) == 0) {

        unsigned short minNeed = (min * M_B_STEPS_REV) / 60;
        unsigned short hourNeed = ((hour * M_A_STEPS_REV) / 12) + ((min * M_A_STEPS_HOUR) / 60);

        cli();
        //Work out hour position
        motorAdesired = hourNeed;
        // work out minute position
        motorBdesired = minNeed;

        // Move hour hand
        if(motorAdesired != motorAposition) motorFlags |= _BV(M_FLAG_A_MOVING);
        // move minute hand
        if(motorBdesired != motorBposition) motorFlags |= _BV(M_FLAG_B_MOVING);

        motorFlags &= ~(_BV(M_FLAG_B_REV) + _BV(M_FLAG_A_REV)); // assume clockwise rotation

        // If we are being obvious then make sure each hand moves a minimum of half a revolution
        if (Obvious && ((motorFlags & _BV(M_FLAG_A_MOVING)) != 0)) {
            temp = motorAdesired;
            if(motorAposition > motorAdesired) temp += M_A_STEPS_REV;
            temp -= motorAposition;
            if(temp < (M_A_STEPS_REV / 2)) motorFlags |= _BV(M_FLAG_A_REV);
        }
        if (Obvious && ((motorFlags & _BV(M_FLAG_B_MOVING)) != 0)) {
            temp = motorBdesired;
            if(motorBposition > motorBdesired) temp += M_B_STEPS_REV;
            temp -= motorBposition;
            if(temp < (M_B_STEPS_REV / 2)) motorFlags |= _BV(M_FLAG_B_REV);
        }
        sei();
    }
}

// Run both motors until they reach the zero position
void zeroMotors() {
    // only fiddle with motor flags and drive durations with interrupts off
    cli();
    motorFlags &= ~(_BV(M_FLAG_A_REV) | ~_BV(M_FLAG_B_REV));
    motorFlags |= _BV(M_FLAG_A_MOVING) | _BV(M_FLAG_B_MOVING);
    if((PINB & _BV(MBZ)) != 0) motorFlags |= _BV(M_FLAG_B_SLOT);
    else motorFlags &= ~_BV(M_FLAG_B_SLOT);
    if((PIND & _BV(MAZ)) != 0) motorFlags |= _BV(M_FLAG_A_SLOT);
    else motorFlags &= ~_BV(M_FLAG_A_SLOT);
    // Ask the motors to drive to 0
    motorAdesired = 0;
    motorBdesired = 0;
    sei();

    dispWriteStr( "ZERO", dispMem);						// display the given string

    timeOut = timeOut_2s
              while(((motorFlags & _BV(M_FLAG_A_MOVING)) != 0) || ((motorFlags & _BV(M_FLAG_B_MOVING)) != 0) && timeOut) wdt_reset(); // Kick the watchdog
    if (!timeOut) dispWriteStr( "tOUT", dispMem);						// signal that the zeroing timed out

}

void incTimeStorePtr( int *ptr) {
    (*ptr)++;
    if(*ptr == MAX_TIME_LIST) *ptr = 0;
}

void StoreTimeValue(unsigned short	hourIn, unsigned short	minIn, unsigned char delayIn) {
    if(delayIn == 0) {						// special case if delay is 0 - cancel current delay time
        clockFlags &= ~_BV(flagTempDisplay);
        return;
    }

    timeList[timeListPtrIn].Hour = hourIn;
    timeList[timeListPtrIn].Min = minIn;
    timeList[timeListPtrIn].delay = delayIn;
    incTimeStorePtr(&timeListPtrIn);
    if(timeListPtrIn == timeListPtrOut) incTimeStorePtr(&timeListPtrOut); // If we have caught our tail then move the out pointer forward to bump one time value
}

void getTempTime() {
    if(timeListPtrOut == timeListPtrIn) return;	// Nothing to see here officer
    if(((clockFlags & _BV(flagTempDisplay)) != 0)) return; // Allow current temp time to finish
    cli();	// We are modifying a timer which is affected by interrupts
    tempTimeDelay = timeList[timeListPtrOut].delay;
    Temp_time_hour = timeList[timeListPtrOut].Hour;
    Temp_time_min = timeList[timeListPtrOut].Min;
    if (timeList[timeListPtrOut].delay != 0xFF) tempTimeDelay <<= 5; // 32 ticks is about 1 second
    clockFlags |= _BV(flagTempDisplay);
    incTimeStorePtr(&timeListPtrOut);				// mark this one as processed
    sei();
}

#endif /* CODE_SECTION_CLOCK */

//_______________________________________________________________________________________
// Main body
//_______________________________________________________________________________________
//

void getMenuItem(int mi) {
    memcpy_P(&menuLocal, &menuItems[mi], sizeof(menuStruct));
    dispWriteStr( menuLocal.name, dispMem);					// display the given string
}


void procesMenuItem() {
    char keyPress;
#ifdef CODE_SECTION_ROTARY
    char chr;
#endif /* CODE_SECTION_ROTARY */
    int	h, m;

    char str[5] = "----";

    while(1) {
        //_______________________________________________________________________________________
        // Do the stuff we have to do all of the time
        //
        // Do unit specific tasks.
        switch(unitType) {
        case TYPE_CLOCK:

#ifdef CODE_SECTION_CLOCK
            getTempTime();			// See if there is a temporary time to display
            // if we are showing a temporary time then show the temp time
            if ((clockFlags & _BV(flagTempDisplay)) == 0) showAnalogueTime(Current_time_hour, Current_time_min, false);
            else  showAnalogueTime(Temp_time_hour, Temp_time_min, true);
            if((menuLocal.flags & _BV(MENU_MENU)) == 0) {
                if ((clockFlags & _BV(flagTempDisplay)) == 0) dispWriteTime(Current_time_hour, Current_time_min, dispMem); // only display time if we are not in a menu
                else dispWriteTime(Temp_time_hour, Temp_time_min, dispMem);
            }
#endif /* CODE_SECTION_CLOCK */

            break;
        case TYPE_KEYPAD:
            EEPROM_read_string(EEPROM_CODE, code, 4);		// get the code from EEPROM
            break;
        case TYPE_SAFE:
#ifdef CODE_SECTION_ROTARY

            if(statusFlags & _BV(STAT_UNLOCKED)) {
                dispWriteStr("OPEN", dispMem);
            } else {
                switch(safeState) {
                case 1:
                    chr = '+';
                    break;
                case 2:
                    chr = 'x';
                    break;
                case 3:
                    chr = '*';
                    break;
                default:
                    chr = ' ';
                    break;
                }

                if(menuItem == TYPE_SAFE) dispWriteByte( rotaryCount, dispMem, rotaryClockwise ? 'C' : 'A', chr); // display the count only if not in menu
            }
#endif /* CODE_SECTION_ROTARY */
            break;
        case TYPE_RANGE:
#ifdef	CODE_SECTION_RANGE
            if(menuItem == TYPE_RANGE) {	// only if not in menu - keypad conflicts with IIC
                VL6180_Start_Range();					// start single range measurement
                VL6180_Poll_Range();					// poll the VL6180 till new sample ready
                unsigned short	dist = VL6180_Read_Range();	// read range result
                VL6180_Clear_Interrupts();				// clear the interrupt on VL6180

                dispWriteDecByte(dist, 'm', dispMem);	// display the distance read
            }
#endif /* CODE_SECTION_RANGE */
            break;
        case TYPE_CHESS:
#ifdef	CODE_SECTION_CHESS
            if(Chess_Poll() == 0) dispWriteByte( pn532_packetbuffer[0x09], dispMem, 'C', '-');
#endif /* CODE_SECTION_CHESS */
            break;
        default:
            break;
        }

        doMainStuff();									// do the general maintenance tasks
        //_______________________________________________________________________________________

        //_______________________________________________________________________________________
        // This is where we take different actions Dependant on menu if all digits have been entered
        if ((menuLocal.digits <= 0) && ((menuLocal.flags & _BV(MENU_NUM)) != 0)) {
            displayAndWait(str, 16, 0);			// Give time for the number to be displayed
            switch(menuLocal.type) {
            case MENU_TYP_TIME:
                m = (str[2] - '0') * 10 + (str[3] - '0');
                h = (str[0] - '0') * 10 + (str[1] - '0');
                if( (m <= 59) && (h <= 11)) {
                    EEPROM_write(menuLocal.address, h);
                    EEPROM_write(menuLocal.address + 1, m);
                    displayAndWait("GOOD", 32, 0);
                } else displayAndWait(" BAD", 32, _BV(FLASH_F));
                // Set the current time
                return;
            case MENU_TYP_BCD:
                // store 4 digits as characters in EEPROM
                EEPROM_write_string(menuLocal.address, str, 4);
                displayAndWait("GOOD", 32, 0);
                return;
            case MENU_TYP_BYTE:
                // turn 3 ASCII digits into byte and write to EEPROM
                if(str[2] == '-') str[2] = '0';
                if(str[1] == '-') str[1] = '0'; // kludge for the single digit entry
                m = ((str[1] - '0') * 100) + ((str[2] - '0') * 10) + (str[3] - '0'); // convert string into number
                if( m <= 255) {
                    EEPROM_write( menuLocal.address , (unsigned char) m);
                    displayAndWait("GOOD", 32, 0);
                } else displayAndWait(" BAD", 32, _BV(FLASH_F));
                return;
            case MENU_TYP_CODE:
                if (strcmp(str, code) == 0) {
                    statusFlags |= _BV(STAT_UNLOCKED);
                    tempFlags |= _BV(TF_UNLOCKED);
                    displayAndWait("+OK+", 64, 0);

                } else {
                    statusFlags &= ~_BV(STAT_UNLOCKED);
                    displayAndWait("FAIL", 64, _BV(FLASH_F));
                }
                return;
            default:
                // Just do nothing - this is an activity that does not accept key presses
                break;
            }
        }
        //_______________________________________________________________________________________

        if (keyBufPtr > 0) {
            cli();										// this part is not interrupt safe because a key could be pressed as we access the buffer or change the pointer
            keyBufPtr--;
            keyPress = mapKey(keyBuf[keyBufPtr]);				// get the current key press
            sei();
            if(((menuLocal.flags & _BV(MNEU_ENTRY)) == 0) && ((menuLocal.flags & _BV(MENU_MENU)) != 0)) {
                switch (keyPress) {
                case MENU_DOWN:
                    if((menuLocal.flags & _BV(MENU_FWD)) != 0) //step menu forward
                        getMenuItem(++menuItem);
                    break;
                case MENU_ENTER:
                    if(menuLocal.type == MENU_TYP_EXIT)	while(1); // If this was an exit menu entry force watchdog to restart in normal mode
                    if(menuLocal.type == MENU_TYP_TEST) {
                        doTest();    // Enter a test routine
                        return;
                    } else {
                        menuLocal.flags |= _BV(MNEU_ENTRY);				// set the menu flag to allow digit entries
                        dispWriteStr( "----", dispMem);			// display the given string
                    }
                    break;
                case MENU_UP:
                    if((menuLocal.flags & _BV(MENU_BK)) != 0)	//step menu back
                        getMenuItem(--menuItem);
                    break;
                case MENU_TEST:
                    if(menuLocal.type == MENU_TYP_TEST) {
                        doTest();    // Enter a test routine
                        return;
                    }
                    break;
                case '0':
                    break;
                default:
                    break;
                }
            } else if((menuLocal.flags & _BV(MNEU_ENTRY)) != 0) {
                // else in entry mode get key and add to number
                str[4 - menuLocal.digits--] = keyPress;				// put the key value in the code
                dispWriteStr(str, dispMem);
            }
        }
    }
}

int main (void) {
    signalResetSource();

    // Do unit specific preparation
    switch(unitType) {
    case TYPE_CLOCK:
#ifdef CODE_SECTION_CLOCK
        zeroMotors();
#endif /* CODE_SECTION_CLOCK */

        break;
    case TYPE_KEYPAD:
        break;
    case TYPE_SAFE:
        break;
#ifdef	CODE_SECTION_RANGE
    case TYPE_RANGE:
        if(VL6180_Init() != 0) displayAndWait("II2e", 32, 0); // load settings onto VL6180X
        break;
#endif /* CODE_SECTION_RANGE */
#ifdef	CODE_SECTION_CHESS
    case TYPE_CHESS:
        if(Chess_Init() != 0) displayAndWait("II2e", 32, 0); // Get set up for reading via NFC
        break;
#endif /* CODE_SECTION_CHESS */
    default:
        break;
    }

    while(1) {
        getMenuItem(menuItem);
        procesMenuItem();
    }

}
