/*
 * NFC.c
 *
 * Created: 27/01/2026 19:47:13
 *  Author: peter
 */

#include "Compile.h"
#include <avr/wdt.h>
#include <util/twi.h>
#include "Utils.h"
#include "NFC.h"
#include "keypad.h"
#include <util/delay.h>

#ifdef CODE_SECTION_NFC

void PN532_init() {
    i2cInit();

    // Wake up the PN532
    // Sending a dummy byte triggers the chip to exit Power Down mode
    TWI_send_data[0] = 	0x55;
    TWI_send_data[1] = 	0x55;
    TWI_send_data[2] = 	0x00;
    TWI_send_data[3] = 	0x00;
    i2cTransferBlocking(PN532_I2C_ADDRESS, 4, TW_WRITE, timeOut_1s);

    // Give the chip ~20ms to stabilize after waking up
    timeOut = timeOut_100ms; // 100ms using the Utils.c timer
    while(timeOut)  wdt_reset();
}


//_____________________________________________________________________________
// Hardware layer
//_____________________________________________________________________________

// Keep polling the PN532 until the status byte is 01 = ready or we have timed out
int PN532_waitForReady() {
    timeOut = timeOut_1s;

    do {
        _delay_ms(1);
        // Just read the status byte
        i2cTransfer(PN532_I2C_ADDRESS, 1, TW_READ);
        while(((TWCR1 & _BV(TWIE)) != 0) && timeOut) wdt_reset(); // as long as interrupts are on the II2 is busy
        if (((TWI_flags & _BV(TWI_flag_error)) != 0) || (timeOut == 0)) {
            void i2cTansferStop();
            debugPulse();
            debugTrigPulse();
            if(timeOut == 0) debugTrigPulse();
            return -1;
        }
    } while (TWI_read_data[0] != 0x01);
    debugPulse();
    return 0;
}


int PN532_readAckFrame() {
    // Wait for the status to be ready
    if(PN532_waitForReady() != 0) return -1;

    // remember the status byte
    if(i2cTransferBlocking(PN532_I2C_ADDRESS, 7, TW_READ, timeOut_1s) != 0) return -1; // try a read and return error if failed

    // If we have not received an ACK then return error
    if ((TWI_read_data[1] != PN532_PREAMBLE) || (TWI_read_data[2] != PN532_STARTCODE1) || (TWI_read_data[3] != PN532_STARTCODE2) ) return -1;
    if ((TWI_read_data[4] != 0) || (TWI_read_data[5] != 0xFF) || (TWI_read_data[6] != PN532_POSTAMBLE) ) return -1;

    return 0;
}


int PN532_writeCommand(const uint8_t *body, uint8_t blen) {
    command = body[0];
    TWI_index = 0;

    TWI_send_data[TWI_index++] = PN532_PREAMBLE;
    TWI_send_data[TWI_index++] = PN532_STARTCODE1;
    TWI_send_data[TWI_index++] = PN532_STARTCODE2;

    uint8_t length = blen + 1;   // length of data field: TFI + DATA
    TWI_send_data[TWI_index++] = length;
    TWI_send_data[TWI_index++] = ~length + 1;                 // checksum of length

    TWI_send_data[TWI_index++] = PN532_HOSTTOPN532;

    uint8_t sum = PN532_HOSTTOPN532;    // sum of TFI + DATA

    for (uint8_t i = 0; i < blen; i++) {
        TWI_send_data[TWI_index++] = body[i];
        sum += body[i];
    }

    uint8_t checksum = ~sum + 1;            // checksum of TFI + DATA
    TWI_send_data[TWI_index++] = checksum;
    TWI_send_data[TWI_index++] = PN532_POSTAMBLE;

    if(i2cTransferBlocking(PN532_I2C_ADDRESS, TWI_index, TW_WRITE, timeOut_1s) == 0) return PN532_readAckFrame();
    else return -1;
}

int PN532_readResponse(uint8_t buf[], uint8_t len) {
    uint8_t length;
    uint8_t index = 1;				// Start off at 1 to avoid the status byte

    // Wait for the status to be ready
    if(PN532_waitForReady() != 0) return -1;

    // [RDY] 00 00 FF LEN LCS (TFI PD0 ... PDn) DCS 00
    // Read the the message and Signal error if read was not successful (len + 1 allows for the status byte)
    if(i2cTransferBlocking(PN532_I2C_ADDRESS, len + 1, TW_READ, timeOut_1s) != 0) return -1;

    // If we have not received the correct preamble then return error
    if ((TWI_read_data[index++] != PN532_PREAMBLE) || (TWI_read_data[index++] != PN532_STARTCODE1) || (TWI_read_data[index++] != PN532_STARTCODE2) ) return -1;

    length = TWI_read_data[index++];

    if (0 != (uint8_t)(length + TWI_read_data[index++])) return -1;   // checksum of length failed - give error

    // check the frame identifier and command echo
    uint8_t cmd = command + 1;               // command we are expecting a response from
    if ((PN532_PN532TOHOST != TWI_read_data[index++]) || ((cmd) != TWI_read_data[index++])) return -1;

    length -= 2;

    // prime checksum
    uint8_t sum = PN532_PN532TOHOST + cmd;
    for (uint8_t i = 0; i < length; i++) {
        buf[i] = TWI_read_data[index++];
        sum += buf[i];
    }

    uint8_t checksum = TWI_read_data[index++];
    if (0 != (uint8_t)(sum + checksum)) return -1;
    return 0;
}

//_____________________________________________________________________________

/**************************************************************************/
/*!
    @brief  Checks the firmware version of the PN5xx chip

	[0]	IC	The Integrated Circuit (IC) type. For the PN532, this value is usually 0x32.
	[1]	Ver	The Firmware Version number.
	[2]	Rev	The Firmware Revision number.
	[3]	Support	A bitmask indicating supported protocols (e.g., ISO14443A, ISO14443B, etc.).

*/
/**************************************************************************/
int PN532_getFirmwareVersion(void) {

    pn532_packetbuffer[0] = PN532_COMMAND_GETFIRMWAREVERSION;

    if (PN532_writeCommand(pn532_packetbuffer, 1) == -1) return -1;
    return PN532_readResponse(pn532_packetbuffer, sizeof(pn532_packetbuffer));
}


/**************************************************************************/
/*!
    @brief  Read a PN532 register.

    @param  reg  the 16-bit register address.

*/
/**************************************************************************/
int PN532_readRegister(uint16_t reg) {
    pn532_packetbuffer[0] = PN532_COMMAND_READREGISTER;
    pn532_packetbuffer[1] = (reg >> 8) & 0xFF;
    pn532_packetbuffer[2] = reg & 0xFF;

    if (PN532_writeCommand(pn532_packetbuffer, 3) == -1) return -1;
    return PN532_readResponse(pn532_packetbuffer, sizeof(pn532_packetbuffer));
}

/**************************************************************************/
/*!
    @brief  Write to a PN532 register.

    @param  reg  the 16-bit register address.
    @param  val  the 8-bit value to write.
*/
/**************************************************************************/
int PN532_writeRegister(uint16_t reg, uint8_t val) {
    pn532_packetbuffer[0] = PN532_COMMAND_WRITEREGISTER;
    pn532_packetbuffer[1] = (reg >> 8) & 0xFF;
    pn532_packetbuffer[2] = reg & 0xFF;
    pn532_packetbuffer[3] = val;


    if (PN532_writeCommand(pn532_packetbuffer, 4) == -1) return -1;
    return PN532_readResponse(pn532_packetbuffer, sizeof(pn532_packetbuffer));
}

/**************************************************************************/
/*!
    @brief  Configures the SAM (Secure Access Module)
*/
/**************************************************************************/
int PN532_SAMConfig(void) {
    pn532_packetbuffer[0] = PN532_COMMAND_SAMCONFIGURATION;
    pn532_packetbuffer[1] = 0x01; // normal mode;
    pn532_packetbuffer[2] = 0x14; // timeout 50ms * 20 = 1 second
    pn532_packetbuffer[3] = 0x01; // use IRQ pin!

    if (PN532_writeCommand(pn532_packetbuffer, 4) == -1) return -1;
    return PN532_readResponse(pn532_packetbuffer, sizeof(pn532_packetbuffer));
}

/**************************************************************************/
/*!
    Sets the MxRtyPassiveActivation uint8_t of the RFConfiguration register

    @param  maxRetries    0xFF to wait forever, 0x00..0xFE to timeout
                          after mxRetries
*/
/**************************************************************************/
int PN532_setPassiveActivationRetries(uint8_t maxRetries) {
    pn532_packetbuffer[0] = PN532_COMMAND_RFCONFIGURATION;
    pn532_packetbuffer[1] = 5;    // Config item 5 (MaxRetries)
    pn532_packetbuffer[2] = 0xFF; // MxRtyATR (default = 0xFF)
    pn532_packetbuffer[3] = 0x01; // MxRtyPSL (default = 0x01)
    pn532_packetbuffer[4] = maxRetries;

    if (PN532_writeCommand(pn532_packetbuffer, 5) == -1) return -1;
    return PN532_readResponse(pn532_packetbuffer, sizeof(pn532_packetbuffer));
}

/**************************************************************************/
/*!
    Sets the RFon/off uint8_t of the RFConfiguration register

    @param  autoRFCA    0x00 No check of the external field before
                        activation

                        0x02 Check the external field before
                        activation

    @param  rFOnOff     0x00 Switch the RF field off, 0x01 switch the RF
                        field on
*/
/**************************************************************************/

int PN532_setRFField(uint8_t autoRFCA, uint8_t rFOnOff) {
    pn532_packetbuffer[0] = PN532_COMMAND_RFCONFIGURATION;
    pn532_packetbuffer[1] = 1;
    pn532_packetbuffer[2] = 0x00 | autoRFCA | rFOnOff;

    if (PN532_writeCommand(pn532_packetbuffer, 3) == -1) return -1;
    return PN532_readResponse(pn532_packetbuffer, sizeof(pn532_packetbuffer));
}

/***** ISO14443A Commands ******/

/**************************************************************************/
/*!
    Waits for an ISO14443A target to enter the field
*/
/**************************************************************************/
int PN532_readPassiveTargetID(uint8_t cardbaudrate) {
    pn532_packetbuffer[0] = PN532_COMMAND_INLISTPASSIVETARGET;
    pn532_packetbuffer[1] = 1;  // max 1 cards at once (we can set this to 2 later)
    pn532_packetbuffer[2] = cardbaudrate;

    if (PN532_writeCommand(pn532_packetbuffer, 3) == -1) return -1;
    if (PN532_readResponse(pn532_packetbuffer, sizeof(pn532_packetbuffer)) == -1) return -1;

    // check some basic stuff
    /* ISO14443A card response should be in the following format:

      byte            Description
      -------------   ------------------------------------------
      b0              Tags Found
      b1              Tag Number (only one used in this example)
      b2..3           SENS_RES
      b4              SEL_RES
      b5              NFCID Length
      b6..NFCIDLen    NFCID
    */

    if (pn532_packetbuffer[0] != 1) return -1;			// we only want to see one card at a time
    else return 0;
}

#endif // CODE_SECTION_NFC
