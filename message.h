/*
 * message.h
 *
 * Created: 04/02/2025 18:02:33
 *  Author: peter
 */ 


#ifndef MESSAGE_H_
#define MESSAGE_H_

// Headers for message functions

void initComms();							// start the serial going and initialize buffer pointers
void processReceive();						// process the next character in the receive buffer
void checkMessage();						// poll the receive buffer for messages
void sendMsg(char typ, char noOfBytes);		// Write message to the serial port
//void checkMsgError();						// handle error
//_______________________________________________________________________________________
// definitions that USART
//_______________________________________________________________________________________
//
#define			BAUD_9600		12			// Value to write into UBBR with U2X bit set in UCSRA

#define			TX_RX_SEL		2			// PD2 of PORTD selects Rx when low and Tx when High

//_______________________________________________________________________________________
// definitions that control messages
//_______________________________________________________________________________________
//
#define			MAX_MSG_DATA	16			// at most 16 data bytes in a message
#define			REC_BUFF_SIZE	128			// up to 128 characters waiting to be processed
#define			TX_BUFF_SIZE	256			// up to 64 characters waiting to be processed
#define			MSG_START_CHR	'%'			// start character
#define			MSG_END_CHR		'#'			// end character
// These are the types of message
#define			MSG_POLL		'P'			// Just reply with an acknowledge message
#define			MSG_SET_TIME	'S'			// Set the actual time
#define			MSG_GO_TO_TIME	'T'			// Display a false time
#define			MSG_ABOUT		'A'			// reply with info about device
#define			MSG_CODE		'C'			// set the code to be entered
#define			MSG_ERROR		'E'			// An error has occurred
#define			MSG_IO			'I'			// Receive an IO message
#define			MSG_RESET		'X'			// Force a watchdog reset

//_______________________________________________________________________________________
//Stuff to do with all messages
//_______________________________________________________________________________________
//
int				msgAddr;					// This is the address of this unit

#define			UNIVERSAL_ADDRESS 0xFF		// an address that all units need to respond to

volatile unsigned char	nREDE_Holdoff;		// Counter to make delay between recieving and transmitting messages to allow nREDE of master to change over

unsigned char	msg_flags;
#define			MSG_DIGIT_ONE	0			// signal looking for first digit of byte
#define			MSG_DIGIT_TWO	1			// signal looking for second digit of byte
#define			MSG_TX_DONE		2			// transmit finished
#define			MSG_TRANSMITTING 3			// we are in a transmit cycle

volatile unsigned short	tempCtr;
//_______________________________________________________________________________________
//Stuff to do with the reception of messages
//_______________________________________________________________________________________
//
unsigned char	rec_buff[REC_BUFF_SIZE];	// buffer to hold received characters
volatile unsigned short	rec_ptr_in;					// in and out pointer so the buffer chases it's tail
volatile unsigned short	rec_ptr_out;

char			msgType;					// The type of the message that we have found
unsigned short	msgChecksum;				// Holds the checksum for the message
unsigned char	msgTemp;					// Holds a bytes as we get it character by character
int				msgDataBytes;				// How many data bytes this message contains
int				msgDataCount;				// A counter for the data bytes
unsigned char	msgData[MAX_MSG_DATA];		// Store for message data

// These constants tell us e=where we are in the processing of an incoming message
#define			stateStart		0
#define			stateType		1
#define			stateAddress	2
#define			stateLength		3
#define			stateData		4
#define			stateChecksum	5
#define			stateEnd		6
#define			stateDone		7

int				msgState;

/*#define			stateError		0x80
#define			ErrorAddress	0
#define			ErrorDigitOne	1
#define			ErrorCSum		2
#define			ErrorData		3
#define			ErrorTerm		4
#define			ErrorUnknown	5
#define			ErrorDigitTwo	6*/

//_______________________________________________________________________________________
//Stuff to do with the transmission of messages
//_______________________________________________________________________________________
//
unsigned char	tx_buff[TX_BUFF_SIZE];	// buffer to hold received characters
volatile unsigned short	tx_ptr_in;					// in and out pointer so the buffer chases it's tail
volatile unsigned short	tx_ptr_out;

unsigned short	txChecksum;				// Holds the checksum for the message
unsigned char	txTemp;					// Holds a bytes as we get it character by character
int				txDataBytes;				// How many data bytes this message contains
int				txDataCount;				// A counter for the data bytes
unsigned char	txData[MAX_MSG_DATA];		// Store for transmitted message data

#endif /* MESSAGE_H_ */