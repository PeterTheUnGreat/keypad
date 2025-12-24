// Peter's program to drive the lock board (c) 2024


#include <avr/io.h>

#define F_CPU 	1000000UL			// we are running a 1MHz clock

// This section bellow does all of the startup stuff
void setup(void) __attribute__ ((naked)) __attribute__ ((section (".init5"))); // do the setup before main called
void start_interrupts(void) __attribute__ ((naked)) __attribute__ ((section (".init8"))); // start interrupts just before main called

//_______________________________________________________________________________________
// Setup Stuff
//_______________________________________________________________________________________
//

// called during .init5 after variables have been initialized
void setup() {
    PORTB = 0xFF;
    PORTC = 0xFF;
    PORTD = 0xFF;										// Starting with all ports set to 0xFF will make any pins configured as inputs have pullups enabled

    UBRRH = 0;											// set prescaler to fastest rate
    UBRRL = 0;

    // Set XCK (PORTD.4) as output
    DDRD |= _BV(PD4);

    // Set USART in synchronous master mode
    // UMSEL=1 ? synchronous, UCPOL=0 (rising edge leading)
    UCSRC = _BV(URSEL) | _BV(UMSEL);

    // Enable transmitter (needed for master clock generation)
    UCSRB = _BV(TXEN);
}

// called during .init8
void start_interrupts() {

}

//_______________________________________________________________________________________
// Main body
//_______________________________________________________________________________________
//

int main (void) {
    while(1) {
        // do nothing
    }

}
