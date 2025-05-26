/*
 * Message.c
 *
 * Created: 04/02/2025 18:03:57
 *  Author: peter
 */ 

#include "message.h"
#include "Utils.h"
#include "Keypad.h"
#include "14seg.h"
#include <avr/io.h>
#include <avr/interrupt.h>

//_______________________________________________________________________________________
//Stuff to do with all messages
//_______________________________________________________________________________________
//
// Increment serial buffer pointer
// increment and make sure it doesn't go past end of buffer but wrap round to start
// This needs to be interrupt safe as the interrupt routines might get confused if the pointers are incremented but not wrapped and an interrupt occurs
void incRecPtr() {
	cli();												// disable interrupt system
	if( ++rec_ptr_out >= REC_BUFF_SIZE) rec_ptr_out = 0;
	sei();												// turn on interrupt system
}

// enable receiver and disable transmitter
void setReceive() {
	UCSRB = _BV(RXEN) + _BV(RXCIE);						// enable receiver and receive interrupts, turn transmitter off
	PORTD &= ~_BV(TX_RX_SEL);							// enable receive buffer
	msg_flags &= ~_BV(MSG_TRANSMITTING);
}

// enable transmitter and disable receiver
// must be done with interrupts off in case a transmit finishes just at this moment
void setTransmit() {
	cli();
	// if we are already transmitting then just carry on
	if((msg_flags & _BV(MSG_TRANSMITTING)) == 0) {
		msg_flags |= _BV(MSG_TRANSMITTING);
		msg_flags &= ~_BV(MSG_TX_DONE);
		UCSRB = _BV(TXEN) + _BV(UDRIE) + _BV(TXCIE);		// enable transmitter buffer empty and complete interrupts, turn receiver off
		UDR = tx_buff[tx_ptr_out++];						// pup tyhe first character in the UDR to get things going
		PORTD |= _BV(TX_RX_SEL);							// enable transmit buffer
	}
	sei();
}

//_______________________________________________________________________________________
// interrupt routines
//_______________________________________________________________________________________
//
ISR (USART_UDRE_vect)
{
	UDR = tx_buff[tx_ptr_out++];
	if(tx_ptr_out >= TX_BUFF_SIZE) tx_ptr_out = 0;
	if(tx_ptr_out == tx_ptr_in) { msg_flags |= _BV(MSG_TX_DONE); UCSRB &= ~_BV(UDRIE); } // turn off further tx empty interrupts so no new data is written into the register
}

ISR (USART_TXC_vect)
{
	if((msg_flags & _BV(MSG_TX_DONE)) != 0) setReceive();
}
 

ISR (USART_RXC_vect)
{
	rec_buff[rec_ptr_in++] = UDR;
	if(rec_ptr_in >= REC_BUFF_SIZE) rec_ptr_in = 0;	
	nREDE_Holdoff = 2;									// Prevent transmission until mater has had time to switch
}

//_______________________________________________________________________________________
// initialize the serial and message system
//_______________________________________________________________________________________
//
void initComms(int	addr) {
	msgAddr = addr;
	rec_ptr_in = 0;
	rec_ptr_out = 0;									// set up the receive buffer as empty
	tx_ptr_in = 0;
	tx_ptr_out = 0;										// set up the transmit buffer as empty
	msgState = stateStart;								// signal that we are looking for the start character
	
	DDRD |= _BV(TX_RX_SEL);								// Set data direction output
	
	// Set USART state for 9600 baud no parity one stop bits and 8 bit data
	UBRRH = (unsigned char)(BAUD_9600 >> 8);
	UBRRL = (unsigned char) BAUD_9600;
	UCSRA |= _BV(U2X);									// make sure to set the BAUD rate doubler
	UCSRC = _BV(URSEL) + _BV(UCSZ0) + _BV(UCSZ1);		// Set frame format: 8 data,  1 stop bit
	setReceive();
	
	tempCtr = 0;
}



//_______________________________________________________________________________________
//Stuff to do with the transmission of messages
//_______________________________________________________________________________________
//
// Put char in transmit buffer
void putTxChar(unsigned char chr) {
	tx_buff[tx_ptr_in++] = chr;
	if(tx_ptr_in >= TX_BUFF_SIZE) tx_ptr_in = 0;
}

// Put digit out as ASCII hex character
void putTxDigit(unsigned char chr) {
	if(chr <= 9) putTxChar(chr + '0');
	else putTxChar(chr - 10 + 'A');
}

// Put byte as two in transmit buffer
void putTxByte(unsigned char chr) {
	putTxDigit(chr >> 4);
	putTxDigit(chr & 0x0F);	
	txChecksum ^= chr;
}

// Send a message on the serial
void sendMsg(char typ, char noOfBytes) {
	txChecksum = 0;
	putTxChar(MSG_START_CHR);
	putTxChar(typ);
	putTxByte(msgAddr);
	if(typ != MSG_POLL) {
		putTxByte(noOfBytes);
		for(int i = 0; i < noOfBytes; i++) putTxByte( txData[i]);
		putTxByte(txChecksum);
	}
	putTxChar(MSG_END_CHR);
	
	// set to send, this will be set back by transmit interrupt once sending finished
	setTransmit();
}

void sendDebugMsg(unsigned short n) {
	txData[0] = n >> 8;
	txData[1] = n & 0xFF;
	sendMsg(MSG_ABOUT, 2);
}

//_______________________________________________________________________________________
//Stuff to do with the reception of messages
//_______________________________________________________________________________________
//
// Set up for receiving the message once a start character is found
void startNewMessage() {
	 msgState = stateType;
	 msg_flags &= ~(_BV(MSG_DIGIT_ONE) + _BV(MSG_DIGIT_TWO));
	 msgChecksum = 0;
}

// Character 0-9 and A-F to nibble return -1 if invalid character
signed char getHexChar(unsigned char c) {
	if((c >= '0') && (c <= '9')) return c - '0';
	if((c >= 'A') && (c <= 'F')) return c - 'A' + 0x0A; 
	return -1;
 }

// Take one character at a time from the receive buffer and using the state machine check the message
void processReceive() {
	signed char temp;
	
	if((rec_ptr_out == rec_ptr_in) || (msgState == stateDone)) return;	// Leave if there is nothing to process or if we have already found a complete message or if an error found
	unsigned char chr = rec_buff[rec_ptr_out];							// get the character to process (use an int to hold it as we are going to put other stuff in it)	
	incRecPtr();														// move pointer on to the next character	
	
	if(chr == MSG_START_CHR) { startNewMessage(); return; }				// whatever stage we are at, finding a start character initiates the processing of a new message	
	if(msgState == stateStart) return;									// If we are looking for the start character then any other character is rubbish
	
	// if we are trying to get a pair of characters for a hex digit then 
	if((msg_flags & _BV(MSG_DIGIT_ONE)) != 0) {
		temp = getHexChar(chr);
		if(temp < 0) { msgState = stateStart; return; }			// if there is an error then put us right back to waiting for the start character
		msgTemp = temp << 4;
		msg_flags |= _BV(MSG_DIGIT_TWO);
		msg_flags &= ~_BV(MSG_DIGIT_ONE);
		return;
	}
	if((msg_flags & _BV(MSG_DIGIT_TWO)) != 0) {
		if((temp = getHexChar(chr)) < 0) { msgState = stateStart; return; }
		msgTemp += temp;
		msgChecksum ^= msgTemp;
		msg_flags &= ~_BV(MSG_DIGIT_TWO);
	}

	switch (msgState){
	case stateType:
		msgType = chr;
		msg_flags |= _BV(MSG_DIGIT_ONE);
		msgState = stateAddress;
		break;
	case stateAddress:
		if(msgTemp != msgAddr) return;					 // This message is not for us
		if(msgType == MSG_POLL) msgState = stateEnd;
		else { msgState = stateLength; msg_flags |= _BV(MSG_DIGIT_ONE); }
		break;
	case stateLength:
		if(msgTemp > MAX_MSG_DATA) { msgState = stateStart; return; }
		else msgState = stateData;
		msgDataBytes = msgTemp;
		msgDataCount = 0;
		if(msgDataBytes == 0) msgState = stateChecksum;
		msg_flags |= _BV(MSG_DIGIT_ONE);
		break;
	case stateData:
		msgData[msgDataCount++] = msgTemp;
		if(msgDataCount == msgDataBytes) msgState = stateChecksum;
		msg_flags |= _BV(MSG_DIGIT_ONE);
		break;
	case stateChecksum:
//		if(msgChecksum == 0) msgState = stateEnd;
//		else msgState = stateStart;
		msgState = stateEnd;							/// Ignore checksum
		break;		
	case stateEnd:
		if(chr == MSG_END_CHR) { msgState = stateDone; }
		else msgState = stateStart;
		break;
	default:
		msgState = stateStart;
	}
}

/*
// If an error has been flagged, process it
void checkMsgError() {
	if(((msgState & stateError) == 0) || (nREDE_Holdoff != 0)) return;
	txData[0] = msgState;
	sendMsg(MSG_ERROR, 1);
	msgState = stateStart;											// Prepare to receive the next message
}
*/

void checkMessage() {
	// only process if we have found a complete message
	// Also do not process the message until we have waited for the short delay between receiving and transmitting
	if((msgState != stateDone) || (nREDE_Holdoff != 0)) return;
	switch (msgType){
	case MSG_POLL:
		sendMsg(MSG_POLL, 0);
		break;
	case MSG_SET_TIME:
		cli();
		Current_time_hour = BCDByte(msgData[0]);
		Current_time_min = BCDByte(msgData[1]);
		if(Current_time_hour > 11) Current_time_hour = 0;
		if(Current_time_min > 59) Current_time_min = 0;
		sei();
		sendMsg(MSG_POLL, 0);
		break;
	case MSG_GO_TO_TIME:
		// If the third byte of the temp time is 0 then cancel the temp time otherwise display for that many seconds (if 0xFF) then set with no timeout
		Temp_time_hour = BCDByte(msgData[0]);
		Temp_time_min = BCDByte(msgData[1]);
		if(Temp_time_hour > 11) Temp_time_hour = 0;
		if(Temp_time_min > 59) Temp_time_min = 0;
		
		cli();
		// assume 0x00 condition which clears timer and temporary display
		clockFlags &= ~(_BV(flagTempDisplay) + _BV(flagTempTimer));
		// If 0xFF then set to temp time indefinitely
		if(msgData[2] == 0xFF) clockFlags |= _BV(flagTempDisplay);
		else if(msgData[2] != 0x00)  { clockFlags |= _BV(flagTempDisplay) + _BV(flagTempTimer); tempTimeDelay = msgData[2] << 5; }	// 32 ticks is about 1 second
		sei(); 
		
		sendMsg(MSG_POLL, 0);
		break;
	case MSG_CODE:
		EEPROM_write(EEPROM_CODE, (msgData[0] >> 4) + '0');
		EEPROM_write(EEPROM_CODE + 1, (msgData[0] & 0x0F) + '0');	
		EEPROM_write(EEPROM_CODE + 2, (msgData[1] >> 4) + '0');
		EEPROM_write(EEPROM_CODE + 3, (msgData[1] & 0x0F) + '0');
		sendMsg(MSG_POLL, 0);
		break;
	case MSG_ABOUT:
		txData[0] = unitType;
		sendMsg(MSG_ABOUT, 1);					// Send an about message with one byte of data
		break;
	}
	msgState = stateStart;											// Prepare to receive the next message
}