#/*
 * Compile.h
 *
 * Created: 04/06/2025 13:58:32
 *  Author: peter
 */


#ifndef COMPILE_H_
#define COMPILE_H_

// This selects what parts of the code will be included for compilation

#define		CODE_SECTION_DEBUG
#define		CODE_SECTION_IIC				// Only set up to work on Atmega328PB
#define		CODE_SECTION_LOWER_CASE
//#define		CODE_SECTION_CLOCK
//#define		CODE_SECTION_RANGE				// Only set up to work on Atmega328PB
#define		CODE_SECTION_IO
//#define		CODE_SECTION_ROTARY
#define		CODE_SECTION_CHESS
#define		CODE_SECTION_NFC
#define		CODE_SECTION_SAFE
//#define		CODE_SECTION_TELEPHONE

#ifdef __AVR_ATmega328PB__
#define		CLOCK_X8						/// only on atmega328pb we can speed up the clock by a factor of 8
#endif


// Set up our own macros for interrupts
#ifdef __AVR_ATmega328PB__
#define		USART_UDRE_handler	USART0_UDRE_vect
#define		USART_RXC_handler	USART0_RX_vect
#define		USART_TXC_handler	USART0_TX_vect
#define		TIMER2_COMP_handler	TIMER2_COMPA_vect
#define		SPI_STC_handler		SPI0_STC_vect
#else
#define		USART_UDRE_handler	USART_UDRE_vect
#define		USART_RXC_handler	USART_RXC_vect
#define		USART_TXC_handler	USART_TXC_vect
#define		TIMER2_COMP_handler	TIMER2_COMP_vect
#define		SPI_STC_handler		SPI_STC_vect
#endif

#endif /* COMPILE_H_ */
