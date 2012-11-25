/*
 * Copyright (c) 2011 - 2012, SimpleMesh AUTHORS
 * Eric Gnoske,
 * Colin O'Flynn,
 * Blake Leverett,
 * Rob Fries,
 * Colorado Micro Devices Inc..
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1) Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *
 *   2) Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 *   3) Neither the name of the SimpleMesh AUTHORS nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
 
#include "simplemesh-api.h"

extern void writeUserByte(uint8_t byte);
/*****************************************************************************
											Transmit side Command Functions
*****************************************************************************/
/*
	Notes:
	
	There are two ways to use the command functions!
	
	Each command function requires a pointer to a UART buffer "uartBuf" to be 
	passed in. This buffer will contain the bytes that are to be transmitted
	over the UART by the user's UART function. 
	
	In both cases, the user must supply the parameters necessary to complete the 
	command function. Also, The user must create and initilaize the UART. 
	RadioBlocks default UART settings are 115,200 BUAD, 8N1.
	
	Way #1:
	
	The user should pass in his uartTx function	as a pointer. Each command 
	function will assemble the bytes in the proper order, insert the proper 
	command ID byte, calculate the CRC and call the user's uartTx	function to 
	transmit the command to RadioBlocks.
	
	It is required that the user's UART function takes a pointer to an array
	of bytes and the number of bytes to transmit.
	
	Way #2:
	
	If the user passes a NULL pointer in place of the uartTx function pointer
	the uartTx function will not be called but the uartBuf will contain the
	appropriate byte string and crc and the user can pass that to his UART 
	transmit function at a later time. Also, each command function returns a 
	length that is the number of bytes contained in the uartBuf that must be 
	transmitted by the user's UART function.
*/
uint8_t testRequest(	void (*uartTx)(uint8_t *array, uint8_t length), 
											uint8_t* uartBuf)
{
	uint16_t crc = 0;
	
	// Build the serial string to transmit over the UART.
	uartBuf[0] = APP_UART_START_BYTE;			// Start byte
	uartBuf[1] = 1;												// Size byte
	uartBuf[2] = CMD_TX_TEST_REQUEST;			// Command ID byte
	
	// Calculate the CRC.
	crc = appCrcCcitt(1, &uartBuf[2]);
	uartBuf[3] = (uint8_t)crc;     
	uartBuf[4] = (uint8_t)(crc>>8);
	
	if(0 != uartTx)
		uartTx(uartBuf, 5);
		
	return 5;
}

/*****************************************************************************/
uint8_t resetRequest(	void (*uartTx)(uint8_t *array, uint8_t length), 
												uint8_t* uartBuf)
{
	uint16_t crc = 0;
	
	// Build the serial string to transmit over the UART.
	uartBuf[0] = APP_UART_START_BYTE;			// Start byte
	uartBuf[1] = 1;												// Size byte
	uartBuf[2] = CMD_TX_RESET_REQUEST;		// Command ID byte
	
	// Calculate the CRC.
	crc = appCrcCcitt(1, &uartBuf[2]);
	uartBuf[3] = (uint8_t)crc;
	uartBuf[4] = (uint8_t)(crc>>8); 
	
	if(0 != uartTx)
		uartTx(uartBuf, 5);
	
	return 5;
}

/*****************************************************************************/
uint8_t settingsRequest(	void (*uartTx)(uint8_t *array, uint8_t length), 
													uint8_t* uartBuf, uint8_t setting)
{
	uint16_t crc = 0;
	
	// Build the serial string to transmit over the UART.
	uartBuf[0] = APP_UART_START_BYTE;			// Start byte
	uartBuf[1] = 2;												// Size byte
	uartBuf[2] = CMD_TX_SETTINGS_REQUEST;	// Command ID byte
	uartBuf[3] = setting;
	
	// Calculate the CRC.
	crc = appCrcCcitt(2, &uartBuf[2]);
	uartBuf[4] = (uint8_t)crc;     
	uartBuf[5] = (uint8_t)(crc>>8);
	
	if(0 != uartTx)
		uartTx(uartBuf, 6);
	
	return 6;
}

/*****************************************************************************/
uint8_t configureUART(void (*uartTx)(uint8_t *array, uint8_t length), 
												uint8_t dataBits, uint8_t parity, uint8_t stopbits, 
												uint8_t baud, uint8_t* uartBuf)
{
	uint16_t crc = 0;
	
	// Build the serial string to transmit over the UART.
	uartBuf[0] = APP_UART_START_BYTE;			// Start byte
	uartBuf[1] = 5;												// Size byte
	uartBuf[2] = CMD_TX_SET_UART_MODE;		// Command ID byte

	uartBuf[3] = dataBits;
	uartBuf[4] = parity;
	uartBuf[5] = stopbits;
	uartBuf[6] = (uint8_t)baud;

	// Calculate the CRC.
	crc = appCrcCcitt(5, &uartBuf[2]);
	uartBuf[7] = (uint8_t)crc;     
	uartBuf[8] = (uint8_t)(crc>>8);
	
	if(0 != uartTx)
		uartTx(uartBuf, 9);
	
	return 9;
}

/*****************************************************************************/
uint8_t sleepRequest(	void (*uartTx)(uint8_t *array, uint8_t length), 
												uint32_t interval, uint8_t* uartBuf)
{
	uint16_t crc = 0;
	
	// Build the serial string to transmit over the UART.
	uartBuf[0] = APP_UART_START_BYTE;			// Start byte
	uartBuf[1] = 5;												// Size byte
	uartBuf[2] = CMD_TX_SLEEP_REQUEST;		// Command ID byte
	uartBuf[3] = (uint8_t)interval;
	uartBuf[4] = (uint8_t)(interval>>8);
	uartBuf[5] = (uint8_t)(interval>>16);
	uartBuf[6] = (uint8_t)(interval>>24);
	
	// Calculate the CRC.
	crc = appCrcCcitt(5, &uartBuf[2]);
	uartBuf[7] = (uint8_t)crc;     
	uartBuf[8] = (uint8_t)(crc>>8);
	
	if(0 != uartTx)
		uartTx(uartBuf, 9);
	
	return 9;
}

/*****************************************************************************/
uint8_t dataRequest(void (*uartTx)(uint8_t *array, uint8_t length), 
											uint16_t destAddr, uint8_t options, uint8_t handle, 
											uint8_t length, uint8_t* data, uint8_t* uartBuf)
{
	uint16_t crc = 0;
	uint8_t i;
	
	// Build the serial string to transmit over the UART.
	uartBuf[0] = APP_UART_START_BYTE;			// Start byte
	uartBuf[1] = 5+length;								// Size byte
	uartBuf[2] = CMD_TX_DATA_REQUEST;			// Command ID byte
	uartBuf[3] = (uint8_t)destAddr;
	uartBuf[4] = (uint8_t)(destAddr>>8);
	uartBuf[5] = options;
	uartBuf[6] = handle;
	for(i=0; i<length; i++)
		uartBuf[i+7] = data[i];
	
	// Calculate the CRC.
	crc = appCrcCcitt(5+length, &uartBuf[2]);
	uartBuf[5+length+2] = (uint8_t)crc;     
	uartBuf[5+length+3] = (uint8_t)(crc>>8);
	
	if(0 != uartTx)
		uartTx(uartBuf, (7 + length + 2));
	
	return 7 + length + 2;
}

/*****************************************************************************/
uint8_t setAddress(	void (*uartTx)(uint8_t *array, uint8_t length), 
											uint16_t address, uint8_t* uartBuf)
{
	uint16_t crc = 0;
	
	// Build the serial string to transmit over the UART.
	uartBuf[0] = APP_UART_START_BYTE;					// Start byte
	uartBuf[1] = 3;														// Size byte
	uartBuf[2] = CMD_TX_SET_ADDRESS_REQUEST;	// Command ID byte
	uartBuf[3] = (uint8_t)address;
	uartBuf[4] = (uint8_t)(address>>8);
	
	// Calculate the CRC.
	crc = appCrcCcitt(3, &uartBuf[2]);
	uartBuf[5] = (uint8_t)crc;     
	uartBuf[6] = (uint8_t)(crc>>8);
	
	if(0 != uartTx)
		uartTx(uartBuf, 7);
	
	return 7;
}

/*****************************************************************************/
uint8_t setPanid(	void (*uartTx)(uint8_t *array, uint8_t length), 
										uint16_t panid, uint8_t* uartBuf)
{
	uint16_t crc = 0;
	
	// Build the serial string to transmit over the UART.
	uartBuf[0] = APP_UART_START_BYTE;					// Start byte
	uartBuf[1] = 3;														// Size byte
	uartBuf[2] = CMD_TX_SET_PANID_REQUEST;		// Command ID byte
	uartBuf[3] = (uint8_t)panid;
	uartBuf[4] = (uint8_t)(panid>>8);
	
	// Calculate the CRC.
	crc = appCrcCcitt(3, &uartBuf[2]);
	uartBuf[5] = (uint8_t)crc;     
	uartBuf[6] = (uint8_t)(crc>>8);
	
	if(0 != uartTx)
		uartTx(uartBuf, 7);
	
	return 7;
}

/*****************************************************************************/
uint8_t setChannel(	void (*uartTx)(uint8_t *array, uint8_t length), 
											uint8_t channel, uint8_t* uartBuf)
{
	uint16_t crc = 0;
	
	// Build the serial string to transmit over the UART.
	uartBuf[0] = APP_UART_START_BYTE;					// Start byte
	uartBuf[1] = 2;														// Size byte
	uartBuf[2] = CMD_TX_SET_CHANNEL_REQUEST;	// Command ID byte
	uartBuf[3] = channel;
	
	// Calculate the CRC.
	crc = appCrcCcitt(2, &uartBuf[2]);
	uartBuf[4] = (uint8_t)crc;     
	uartBuf[5] = (uint8_t)(crc>>8);
	
	if(0 != uartTx)
		uartTx(uartBuf, 6);
	
	return 6;
}

/*****************************************************************************/
uint8_t setTRXState(void (*uartTx)(uint8_t *array, uint8_t length), 
											uint8_t state, uint8_t* uartBuf)
{
	uint16_t crc = 0;
	
	// Build the serial string to transmit over the UART.
	uartBuf[0] = APP_UART_START_BYTE;								// Start byte
	uartBuf[1] = 2;																	// Size byte
	uartBuf[2] = CMD_TX_SET_RECEIVER_STATE_REQUEST;	// Command ID byte
	uartBuf[3] = state;
	
	// Calculate the CRC.
	crc = appCrcCcitt(2, &uartBuf[2]);
	uartBuf[4] = (uint8_t)crc;     
	uartBuf[5] = (uint8_t)(crc>>8);
	
	if(0 != uartTx)
		uartTx(uartBuf, 6);
	
	return 6;
}

/*****************************************************************************/
uint8_t setTxPower(	void (*uartTx)(uint8_t *array, uint8_t length), 
											uint8_t power, uint8_t* uartBuf)
{
	uint16_t crc = 0;
	
	// Build the serial string to transmit over the UART.
	uartBuf[0] = APP_UART_START_BYTE;								// Start byte
	uartBuf[1] = 2;																	// Size byte
	uartBuf[2] = CMD_TX_SET_TRANSMIT_POWER_REQUEST;	// Command ID byte
	uartBuf[3] = power;
	
	// Calculate the CRC.
	crc = appCrcCcitt(2, &uartBuf[2]);
	uartBuf[4] = (uint8_t)crc;     
	uartBuf[5] = (uint8_t)(crc>>8);
	
	if(0 != uartTx)
		uartTx(uartBuf, 6);
	
	return 6;
}

/*****************************************************************************/
uint8_t setSecurityKey(	void (*uartTx)(uint8_t *array, uint8_t length), 
													uint8_t* key, uint8_t* uartBuf) // max 16 bytes.
{
	uint8_t i;
	uint16_t crc = 0;
	
	// Build the serial string to transmit over the UART.
	uartBuf[0] = APP_UART_START_BYTE;							// Start byte
	uartBuf[1] = 17;																// Size byte
	uartBuf[2] = CMD_TX_SET_SECURITY_KEY_REQUEST;	// Command ID byte
	for(i=0; i<16; i++)
		uartBuf[i+3] = key[i];
	
	// Calculate the CRC.
	crc = appCrcCcitt(17, &uartBuf[2]);
	uartBuf[i+2] = (uint8_t)crc;     
	uartBuf[i+3] = (uint8_t)(crc>>8);
	
	if(0 != uartTx)
		uartTx(uartBuf, 19);
	
	return 19;
}

/*****************************************************************************/
uint8_t setAckState(void (*uartTx)(uint8_t *array, uint8_t length), 
											uint8_t state, uint8_t* uartBuf)
{
	uint16_t crc = 0;
	
	// Build the serial string to transmit over the UART.
	uartBuf[0] = APP_UART_START_BYTE;						// Start byte
	uartBuf[1] = 2;															// Size byte
	uartBuf[2] = CMD_TX_SET_ACK_STATE_REQUEST;	// Command ID byte
	uartBuf[3] = state;
	
	// Calculate the CRC.
	crc = appCrcCcitt(2, &uartBuf[2]);
	uartBuf[4] = (uint8_t)crc;     
	uartBuf[5] = (uint8_t)(crc>>8);
	
	if(0 != uartTx)
		uartTx(uartBuf, 6);
	
	return 6;
}

/*****************************************************************************/
uint8_t toggleLed(void (*uartTx)(uint8_t *array, uint8_t length), 
										uint8_t state, uint8_t* uartBuf)
{
	uint16_t crc = 0;
	
	// Build the serial string to transmit over the UART.
	uartBuf[0] = APP_UART_START_BYTE;						// Start byte
	uartBuf[1] = 2;															// Size byte
	uartBuf[2] = CMD_TX_SET_LED_STATE_REQUEST;	// Command ID byte
	uartBuf[3] = state;
	
	// Calculate the CRC.
	crc = appCrcCcitt(2, &uartBuf[2]);
	uartBuf[4] = (uint8_t)crc;     
	uartBuf[5] = (uint8_t)(crc>>8);
	
	if(0 != uartTx)
		uartTx(uartBuf, 6);
	
	return 6;
}

/*****************************************************************************/
uint16_t getAddress(void (*uartTx)(uint8_t *array, uint8_t length), uint8_t* uartBuf)
{
	uint16_t crc = 0;
	
	// Build the serial string to transmit over the UART.
	uartBuf[0] = APP_UART_START_BYTE;						// Start byte
	uartBuf[1] = 1;															// Size byte
	uartBuf[2] = CMD_TX_GET_ADDRESS_REQUEST;		// Command ID byte
	
	// Calculate the CRC.
	crc = appCrcCcitt(1, &uartBuf[2]);
	uartBuf[3] = (uint8_t)crc;     
	uartBuf[4] = (uint8_t)(crc>>8);
	
	if(0 != uartTx)
		uartTx(uartBuf, 5);
	
	return 5;
}

/*****************************************************************************/
uint16_t getPanid(void (*uartTx)(uint8_t *array, uint8_t length), uint8_t* uartBuf)
{
	uint16_t crc = 0;
	
	// Build the serial string to transmit over the UART.
	uartBuf[0] = APP_UART_START_BYTE;						// Start byte
	uartBuf[1] = 1;															// Size byte
	uartBuf[2] = CMD_TX_GET_PANID_REQUEST;			// Command ID byte
	
	// Calculate the CRC.
	crc = appCrcCcitt(1, &uartBuf[2]);
	uartBuf[3] = (uint8_t)crc;     
	uartBuf[4] = (uint8_t)(crc>>8);
	
	if(0 != uartTx)
		uartTx(uartBuf, 5);
	
	return 5;	
}

/*****************************************************************************/
uint8_t getChannel(void (*uartTx)(uint8_t *array, uint8_t length), uint8_t* uartBuf)
{
	uint16_t crc = 0;
	
	// Build the serial string to transmit over the UART.
	uartBuf[0] = APP_UART_START_BYTE;						// Start byte
	uartBuf[1] = 1;															// Size byte
	uartBuf[2] = CMD_TX_GET_CHANNEL_REQUEST;		// Command ID byte
	
	// Calculate the CRC.
	crc = appCrcCcitt(1, &uartBuf[2]);
	uartBuf[3] = (uint8_t)crc;     
	uartBuf[4] = (uint8_t)(crc>>8);
	
	if(0 != uartTx)
		uartTx(uartBuf, 5);
	
	return 5;	
}

/*****************************************************************************/
uint8_t getTRXState(void (*uartTx)(uint8_t *array, uint8_t length), uint8_t* uartBuf)
{
	uint16_t crc = 0;
	
	// Build the serial string to transmit over the UART.
	uartBuf[0] = APP_UART_START_BYTE;								// Start byte
	uartBuf[1] = 1;																	// Size byte
	uartBuf[2] = CMD_TX_GET_RECEIVER_STATE_REQUEST;	// Command ID byte
	
	// Calculate the CRC.
	crc = appCrcCcitt(1, &uartBuf[2]);
	uartBuf[3] = (uint8_t)crc;     
	uartBuf[4] = (uint8_t)(crc>>8);
	
	if(0 != uartTx)
		uartTx(uartBuf, 5);
	
	return 5;	
}

/*****************************************************************************/
uint8_t getTxPower(void (*uartTx)(uint8_t *array, uint8_t length), uint8_t* uartBuf)
{
	uint16_t crc = 0;
	
	// Build the serial string to transmit over the UART.
	uartBuf[0] = APP_UART_START_BYTE;								// Start byte
	uartBuf[1] = 1;																	// Size byte
	uartBuf[2] = CMD_TX_GET_TRANSMIT_POWER_REQUEST;	// Command ID byte
	
	// Calculate the CRC.
	crc = appCrcCcitt(1, &uartBuf[2]);
	uartBuf[3] = (uint8_t)crc;     
	uartBuf[4] = (uint8_t)(crc>>8);
	
	if(0 != uartTx)
		uartTx(uartBuf, 5);
	
	return 5;	
}

/*****************************************************************************/
uint8_t getAckState(void (*uartTx)(uint8_t *array, uint8_t length), uint8_t* uartBuf)
{
	uint16_t crc = 0;
	
	// Build the serial string to transmit over the UART.
	uartBuf[0] = APP_UART_START_BYTE;						// Start byte
	uartBuf[1] = 1;															// Size byte
	uartBuf[2] = CMD_TX_GET_ACK_STATE_REQUEST;	// Command ID byte
	
	// Calculate the CRC.
	crc = appCrcCcitt(1, &uartBuf[2]);
	uartBuf[3] = (uint8_t)crc;     
	uartBuf[4] = (uint8_t)(crc>>8);
	
	if(0 != uartTx)
		uartTx(uartBuf, 5);
	
	return 5;	
}


/*****************************************************************************
										Receive side State Machine
*****************************************************************************/
/*
	This function should be called from your ISR or Polling loop.
	
	The logic is that every byte received increments the isrLen counter and
	we already know the structure of a received command frame. So, as we 
	increment through the received bytes we know which byte(s) we are 
	receiving and apply logic to switch the state. When we've received all
	the bytes the appropriate command flag is thrown which allows the function
	"processResponse" to finish the processing of the command.
*/
void parseRxBytes(uint8_t receivedData)
{
	isrLen++;
	// Just write the received byte, the bytes are processed in main.
	// DEBUG writeByte(receivedData);
	
	// Implement a mini-parser here. Every command must get acked so we
	// capture the ACK here and set a flag to alert user code.
	if(1 == isrLen)
	{
		// The first byte should be the start byte 0xab - Reset it not.
		if(APP_UART_START_BYTE != receivedData)
			isrLen = 0;;
	}
	
	if(2 == isrLen)
	{
		// The second byte is the size, save off to a global variable.
		cmdLen = receivedData;
		
		// SimpleMesh has a 113 byte useful payload:
		// 128 - 9 (MAC Header) - 6 (NWK Header) = 113
		// Reject longer frames.
		if(113 < cmdLen)
		{
			isrLen = 0;
			cmdLen = 0;
		}		
	}
	
	if(3 == isrLen)
	{
		// The third byte is the cmd id byte, save off in a global variable.
		isrCmd = receivedData;
	}
	
	// Two commands are single byte responses - i.e. no payload.
	// These are CMD_RX_TEST_RESPONSE and CMD_RX_WAKEUP_INDICATION,
	// handle them here.
	if(((4 == isrLen) || (5 == isrLen)) && ((CMD_RX_TEST_RESPONSE == isrCmd) || (CMD_RX_WAKEUP_INDICATION == isrCmd)))
	{
		// Could collect and check CRC - Left as an exercise for the user.
				
		// Reset variables used to process the command.
		if(5 == isrLen)
		{
			if((isrCmd == CMD_RX_TEST_RESPONSE))
				testCmd = 1;
			else
				wakeCmd = 1;
				
			// Reset the len.
			isrLen = 0;
		}
	}
	
	// Catch ACK command Status and CRC bytes here...
	if(((4 == isrLen) || (5 == isrLen) || (6 == isrLen)) && (CMD_RX_ACK == isrCmd))
	{		
		// Grab the ACK Status byte and place in a global.
		if(4 == isrLen)
			ackStatus = receivedData;
			
		// Could collect and check CRC - Left as an exercise for the user.			
				
		// Reset variables used to process the ACK and set the ACK flag.
		if(6 == isrLen)
		{
			isrLen = 0; //Reset this in the user ACK handler.
			ackFlag = 1;
			
			// Reset the len.
			isrLen = 0;
		}
	}
	
	// If it isn't an ACK or Test Response or Wakeup Indication, then it is another command.
	if( ((CMD_RX_ACK != isrCmd) &&
			(CMD_RX_TEST_RESPONSE != isrCmd) &&
			(CMD_RX_WAKEUP_INDICATION != isrCmd)) &&
			(isrLen > 3) )
	{
		/*	The length of a serial frame does not include:
				1. Start byte
				2. Size byte
				3. 2 CRC bytes
				
				So, the CRC bytes start at isrLen - 4... Or, cmdLen + 2.
		*/
		if(isrLen > (cmdLen+3))
		{
			// We have received all the command bytes and this should be
			// the last CRC byte.
			if(isrLen == cmdLen+4)
			{
				// We are done receiving this commands. set the appropriate flags.
				// This variable must be reset to 0 in the user function.
				cmdFlag = 1;
				
				isrLen = 0;
			}
		}
		else if(isrLen < (cmdLen+3))
		{
			// For non-ACK commands, we record the command bytes and drop the crc bytes.
			writeByte(receivedData);
		}	
	}	
}
/*
	After the ISR or the polling loop catches the serial data returned from 
	RadioBlocks it needs to be parsed. This function parses the received data.
	Remember that Test Response and Wakeup indication have no status or return 
	data. Also recall that ACK returns a one byte ack status that is contained 
	in the global variable "ackStatus" that is set in the ISR or the polling 
	loop. In the cases of all other responses or indications the functions 
	manipulate the returned variables and present them in easy to use formats 
	for the user to use.
*/
void processResponse(void)
{
	uint8_t i;
	uint8_t cmdBuf[128];
	
	while(1)
	{
			if(1 == ackFlag)
			{
				ack();
					
				// Reset ACK variables.
				ackStatus = 0;
				ackFlag = 0;
				isrLen = 0;
				break;
			}			
			
			if(1 == testCmd)
			{
				testResponse();
		
				// Reset variables here.
				testCmd = 0;
				isrLen = 0;
				cmdLen = 0;
				break;
			}
			
			if(1 == wakeCmd)
			{
				wakeUpIndication();
				
				// Reset variables here.
				wakeCmd = 0;
				isrLen = 0;
				cmdLen = 0;
				break;
			}
			
			if(1 == cmdFlag)
			{
				// Subtract one because the length includes the command id
				// which happened in the isr or polling loop.
				cmdLen -= 1;
				for(i=0; i<cmdLen; i++) 
					cmdBuf[i] = readByte();
					
				// Call the command processor here.
				switch(isrCmd)
				{
					case CMD_RX_DATA_CONFIRMATION:
						dataConfirmation(cmdBuf, cmdLen);
						break;					
					case CMD_RX_DATA_INDICATION:
						dataIndication(cmdBuf, cmdLen);
						break;					
					case CMD_RX_GET_ADDRESS_RESPONSE:
						getAddressResponse(cmdBuf, cmdLen);
						break;					
					case CMD_RX_GET_PANID_RESPONSE:
						getPanidResponse(cmdBuf, cmdLen);		
						break;					
					case CMD_RX_GET_CHANNEL_RESPONSE:
						getChannelResponse(cmdBuf, cmdLen);
						break;					
					case CMD_RX_GET_RECEIVER_STATE_RESPONSE:
						getTRXStateResponse(cmdBuf, cmdLen);
						break;					
					case CMD_RX_GET_TRANSMIT_POWER_RESPONSE:
						getTxPowerResponse(cmdBuf, cmdLen);
						break;					
					case CMD_RX_SET_ACK_STATE_RESPONSE:
						getAckStateResponse(cmdBuf, cmdLen);
						break;
					default:
						break;	
				}
				
				isrLen = 0;
				cmdLen = 0;
				cmdFlag = 0;
				break;
			}			
	}	
}


/*****************************************************************************
										Response or Indication Functions
*****************************************************************************/

/*****************************************************************************/
uint8_t ack(void)
{
	#if XPLAINED_A1_DEMO
	writeUserByte('A'); writeUserByte('C'); writeUserByte('K'); 
	writeUserByte(' '); writeUserByte('O'); writeUserByte('K'); 
	writeUserByte('\n');
	#endif
	
	/* User code here */
	return ackStatus;	// ackStatus is contained in a global, just return it.
}

/*****************************************************************************/
uint8_t testResponse(void)
{
	#if XPLAINED_A1_DEMO
	writeUserByte('T'); writeUserByte('E'); writeUserByte('S');
	writeUserByte('T'); writeUserByte(' '); writeUserByte('O');
	writeUserByte('K'); writeUserByte('\n');
	#endif
	
	// There is no status returned from this...
	
	/* User code here */
	return 0;
}

uint8_t wakeUpIndication(void)
{
	#if XPLAINED_A1_DEMO
	writeUserByte('W'); writeUserByte('A'); writeUserByte('K'); 
	writeUserByte('E'); writeUserByte('U'); writeUserByte('P');
	writeUserByte(' '); writeUserByte('O'); writeUserByte('K');
	writeUserByte('\n');
	#endif 	
	
	// There is no status returned from this...
	
	/* User code here */
	return 0;	
}

uint8_t dataConfirmation(uint8_t* data, uint8_t len)
{
	#if XPLAINED_A1_DEMO
	writeUserByte('D'); writeUserByte('A'); writeUserByte('T');
	writeUserByte('A'); writeUserByte(' '); writeUserByte('C');
	writeUserByte('O'); writeUserByte('N'); writeUserByte('F'); 
	writeUserByte('I');	writeUserByte('R'); writeUserByte('M'); 
	writeUserByte('\n');
	#endif
	
	/* User code here */
	uint8_t status = data[0];	// Status
	uint8_t handle = data[1]; // Has the handle corresponding to the Data Request
	
	return status;
}

/*****************************************************************************/
uint8_t dataIndication(uint8_t* data, uint8_t payloadLen)
{
	#if XPLAINED_A1_DEMO
	writeUserByte('D'); writeUserByte('A'); writeUserByte('T');
	writeUserByte('A'); writeUserByte(' '); writeUserByte('I');
	writeUserByte('N'); writeUserByte('D'); writeUserByte('I');
	writeUserByte('C'); writeUserByte('A'); writeUserByte('T'); 
	writeUserByte('I'); writeUserByte('O'); writeUserByte('N'); 
	writeUserByte('\n');
	#endif
		
	/* User code here */
	uint16_t address;
	uint8_t options;
	uint8_t lqi;
	uint8_t rssi;
	uint8_t payload[128];
	uint8_t i;
		
	address = data[0]<<8; // Source address
	address |= data[1];
	
	options = data[2];
	
	lqi = data[3];
	rssi = data[4];
	
	// data[5] thru data[5+payloadLen] contain the payload
	for(i=0; i<payloadLen; i++)
		payload[i] = data[5+i];
	
	return payloadLen;
}

/*****************************************************************************/
uint16_t getAddressResponse(uint8_t* data, uint8_t len)
{
	#if XPLAINED_A1_DEMO
	writeUserByte('A'); writeUserByte('D'); writeUserByte('D');
	writeUserByte('R'); writeUserByte(' '); writeUserByte('R');
	writeUserByte('E'); writeUserByte('S'); writeUserByte('P');
	writeUserByte('O'); writeUserByte('N'); writeUserByte('S'); 
	writeUserByte('E');	writeUserByte('\n');
	#endif
	
	/* User code here */
	uint16_t address;
		
	address = data[1];		// MSB
	address <<= 8;
	address |= data[0]; 	// LSB
	
	return address;
}

/*****************************************************************************/
uint16_t getPanidResponse(uint8_t* data, uint8_t len)
{
	#if XPLAINED_A1_DEMO
	writeUserByte('P'); writeUserByte('A'); writeUserByte('N');
	writeUserByte('I'); writeUserByte('D'); writeUserByte(' ');
	writeUserByte('R'); writeUserByte('E'); writeUserByte('S'); 
	writeUserByte('P'); writeUserByte('O'); writeUserByte('N');
	writeUserByte('S'); writeUserByte('E'); writeUserByte('\n');
	#endif
	
	/* User code here */
	uint16_t panid;
		
	panid = data[1];		// MSB
	panid <<= 8;
	panid |= data[0]; 	// LSB
		
	return panid;
}

/*****************************************************************************/
uint8_t getChannelResponse(uint8_t* data, uint8_t len)
{
	#if XPLAINED_A1_DEMO
	writeUserByte('C'); writeUserByte('H'); writeUserByte('A');
	writeUserByte('N'); writeUserByte('N'); writeUserByte('E');
	writeUserByte('L'); writeUserByte(' '); writeUserByte('R');
	writeUserByte('E'); writeUserByte('S'); writeUserByte('P');
	writeUserByte('O'); writeUserByte('N'); writeUserByte('S');
	writeUserByte('E'); writeUserByte('\n');
	#endif
	
	/* User code here */
	uint8_t channel = data[0];

	return channel;
}

/*****************************************************************************/
uint8_t getTRXStateResponse(uint8_t* data, uint8_t len)
{
	#if XPLAINED_A1_DEMO
	writeUserByte('T'); writeUserByte('R'); writeUserByte('X');
	writeUserByte(' '); writeUserByte('S'); writeUserByte('T');
	writeUserByte('A'); writeUserByte('T'); writeUserByte('E');
	writeUserByte(' '); writeUserByte('R');	writeUserByte('E'); 
	writeUserByte('S'); writeUserByte('P'); writeUserByte('O'); 
	writeUserByte('N'); writeUserByte('S');	writeUserByte('E'); 
	writeUserByte('\n');
	#endif
	
	/* User code here */	
	uint8_t trxState = data[0];

	return trxState;
}

/*****************************************************************************/
uint8_t getTxPowerResponse(uint8_t* data, uint8_t len)
{
	#if XPLAINED_A1_DEMO
	writeUserByte('T'); writeUserByte('X'); writeUserByte(' ');
	writeUserByte('P'); writeUserByte('O'); writeUserByte('W');
	writeUserByte('E'); writeUserByte('R'); writeUserByte(' ');
	writeUserByte('R');	writeUserByte('E'); writeUserByte('S'); 
	writeUserByte('P'); writeUserByte('O'); writeUserByte('N'); 
	writeUserByte('S');	writeUserByte('E'); writeUserByte('\n');
	#endif
			
	/* User code here */
	uint8_t txPower = data[0];

	return txPower;
}

/*****************************************************************************/
uint8_t getAckStateResponse(uint8_t* data, uint8_t len)
{
	#if XPLAINED_A1_DEMO
	writeUserByte('A'); writeUserByte('C'); writeUserByte('K');
	writeUserByte(' '); writeUserByte('S'); writeUserByte('T');
	writeUserByte('A'); writeUserByte('T'); writeUserByte('E');
	writeUserByte(' '); writeUserByte('R');	writeUserByte('E'); 
	writeUserByte('S'); writeUserByte('P');	writeUserByte('O'); 
	writeUserByte('N'); writeUserByte('S');	writeUserByte('E'); 
	writeUserByte('\n');
	#endif
	
	/* User code here */
	uint8_t ackState = data[0];
		
	return ackState;
}

/*****************************************************************************
										Helper Functions
*****************************************************************************/
uint16_t appCrcCcitt(uint8_t size, uint8_t* uartBuf)
{
  uint16_t crc = 0x1234;

  for (uint8_t i = 0; i < size; i++)
  {
    uint8_t data = uartBuf[i];

    data ^= crc & 0xff;
    data ^= data << 4;
    crc = (((uint16_t)data << 8) | ((crc >> 8) & 0xff)) ^ (uint8_t)(data >> 4) ^ ((uint16_t)data << 3);
  }

  return crc;
}

/*****************************************************************************/

// Functions for the circular UART buffer
void writeByte(uint8_t data)
{
	// Put the byte into the buffer.
	sniffBuff[start] = data;

	// Adjust the start value if it wraps around.
	if((start + 1) < CIRCSIZE)
		start += 1;
	else
		start = 0;
}

/*****************************************************************************/
uint8_t readByte(void)
{
	uint8_t data;

	data = sniffBuff[end];

	if((end + 1) < CIRCSIZE)
		end += 1;
	else
		end = 0;

	return data;
}