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

/*
	Notes:

	1.	This example changes the default clock (of 2 MHz) to a

	2.
*/


#include <avr/io.h>
#include "avr_compiler.h"
#include "simplemesh-api.h"
#include "usart_driver.h"
#include "TC_driver.h"

/*****************************************************************************
*****************************************************************************/
// Macros
// Define that selects the Usart used in example.
#define UARTC0 USARTC0
#define UARTF0 USARTF0

// Clock
#define CLKSYS_Enable( _oscSel ) ( OSC.CTRL |= (_oscSel) )
#define CLKSYS_IsReady( _oscSel ) ( OSC.STATUS & (_oscSel) )

/*
		Sets the data direction of a set of pins to output

		This macro sets the data direction of the selected port pins to output
		without altering the data direction of the other pins in that port.
		
		_port         Pointer to the PORT_t instance.
		_outputMask   A bit mask of the pins to set as output. A one in bit
		              location n will configure pin n as output.
 */
#define PORT_SetPinsAsOutput( _port, _outputMask ) ( (_port)->DIRSET = _outputMask )

/*
		brief Set the output value of a set of I/O pins to logic high.

		This macro sets the output value of a set of I/O pins to logic high.
		(Unless inverted I/O has been enabled for the pins) It does not alter the
		pins not specified in the bit mask.
	
		_port         Pointer to the PORT_t instance.
		_setMask      The bit mask of pins to set to logic high level.
 */
#define PORT_SetPins( _port, _setMask) ( (_port)->OUTSET = _setMask )

/*
		Set the output value of a set of I/O pins to logic low.
		
		This macro sets the output value of a set of I/O pins to logic low.
		(Unless inverted I/O has been enabled for the pins) It does not alter the
		pins not specified in the bit mask.
		
		_port         Pointer to the PORT_t instance.
		_clearMask    The bit mask of pins to set to logic low level.
 */
#define PORT_ClearPins( _port, _clearMask) ( (_port)->OUTCLR = _clearMask )



/*
		Toggle the output value of a set of I/O pins.
	
		This macro toggles the output value of a set of I/O pins. It does not
		alter the output value of pins not specified in the bit mask.
	
		_port         Pointer to the PORT_t instance.
		_toggleMask   The bit mask of pins to toggle.
 */
#define PORT_TogglePins( _port, _toggleMask ) ( (_port)->OUTTGL = _toggleMask )



/*
		This macro returns the current logic value of the port or virtual
	  port.
	
		This macro can also be used to access virtual ports.
	
		_port     Pointer to the PORT_t or VPORT_t instance.
		          The current logic state of the port.
 */
#define PORT_GetPortValue( _port ) ( (_port)->IN )

#define USERCIRCSIZE	256
/*****************************************************************************
*****************************************************************************/
// Prototypes
void writeUserByte(uint8_t);
uint8_t readUserByte(void);

/*****************************************************************************
*****************************************************************************/
// Globals

// Used for sending bytes from the host to the RadioBlock.
uint8_t uartBuf[134];

// Fun!
uint8_t ledFlag;

// Used for printint to USARTC0
uint8_t userBuf[USERCIRCSIZE];
uint8_t userEnd;
uint8_t userStart;

// Used for sending a "Hello!" string over the air.
uint8_t testBuf[6] = {'H', 'e', 'l', 'l', 'o', '!'};
/*****************************************************************************
*****************************************************************************/
// Functions
/*	CCP write helper function written in assembly.
 *
 *  This function is written in assembly because of the time critical
 *  operation of writing to the registers.
 *
 *  param address A pointer to the address to write to.
 *  param value   The value to put in to the register.
 */
void CCPWrite( volatile uint8_t * address, uint8_t value )
{
	volatile uint8_t * tmpAddr = address;
#ifdef RAMPZ
	RAMPZ = 0;
#endif
	asm volatile(
		"movw r30,  %0"	      "\n\t"
		"ldi  r16,  %2"	      "\n\t"
		"out   %3, r16"	      "\n\t"
		"st     Z,  %1"       "\n\t"
		:
		: "r" (tmpAddr), "r" (value), "M" (CCP_IOREG_gc), "i" (&CCP)
		: "r16", "r30", "r31"
		);
}

/*****************************************************************************/
/*	This function changes the prescaler configuration.

		Change the configuration of the three system clock
		prescaler is one single operation. The user must make sure that
		the main CPU clock does not exceed recommended limits.

		param  PSAfactor   Prescaler A division factor, OFF or 2 to 512 in
		                   powers of two.
		param  PSBCfactor  Prescaler B and C division factor, in the combination
	                     of (1,1), (1,2), (4,1) or (2,2).
 */
void CLKSYS_Prescalers_Config( CLK_PSADIV_t PSAfactor, CLK_PSBCDIV_t PSBCfactor )
{
	uint8_t PSconfig = (uint8_t) PSAfactor | PSBCfactor;
	CCPWrite( &CLK.PSCTRL, PSconfig );
}

/*****************************************************************************/
/*	This function selects the main system clock source.

		Hardware will disregard any attempts to select a clock source that is not
		enabled or not stable. If the change fails, make sure the source is ready
		and running and try again.

		param  clockSource  Clock source to use as input for the system clock
	                      prescaler block.
	
		return  Non-zero if change was successful.
 */
uint8_t CLKSYS_Main_ClockSource_Select( CLK_SCLKSEL_t clockSource )
{
	uint8_t clkCtrl = ( CLK.CTRL & ~CLK_SCLKSEL_gm ) | clockSource;
	CCPWrite( &CLK.CTRL, clkCtrl );
	clkCtrl = ( CLK.CTRL & clockSource );
	return clkCtrl;
}

/*****************************************************************************/
void clockInit(void)
{
	/* 
			Enable internal 32 MHz ring oscillator and wait until it's
			stable. Divide clock by two with the prescaler C and set the
			32 MHz ring oscillator as the main clock source.
	*/
	CLKSYS_Enable( OSC_RC32MEN_bm );
	CLKSYS_Prescalers_Config( CLK_PSADIV_1_gc, CLK_PSBCDIV_1_2_gc );
	do {} while ( CLKSYS_IsReady( OSC_RC32MRDY_bm ) == 0 );
	CLKSYS_Main_ClockSource_Select( CLK_SCLKSEL_RC32M_gc );
}

/*****************************************************************************/
void uartInit(USART_t* uart, uint16_t bsel)
{
	if(uart == &USARTC0)
	{
		// Setup UARTC0
		// PIN3 (TXD0) as output.
		PORTC.DIRSET = PIN3_bm;
		// PC2 (RXD0) as input.
		PORTC.DIRCLR = PIN2_bm;	

		// USARTC0, 8 Data bits, No Parity, 1 Stop bit.
		USART_Format_Set(uart, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);

		/* 
				Set Baudrate to 115200 bps, use the I/O clock frequency that is 32 MHz.
				Do not use the baudrate scale factor
				Baudrate select = (1/(16*(((I/O clock frequency)/Baudrate)-1)
												= 8
		*/
		USART_Baudrate_Set(uart, bsel, 0);			
	}
	else if(uart == &USARTF0)
	{
		// Setup UARTF0
		// PIN3 (TXD0) as output.
		PORTF.DIRSET = PIN3_bm;
		// PF2 (RXD0) as input.
		PORTF.DIRCLR = PIN2_bm;	

		// Enable RXC interrupt.
		USART_RxdInterruptLevel_Set(uart, USART_RXCINTLVL_LO_gc);

		// Enable PMIC interrupt level low.
		PMIC.CTRL |= PMIC_LOLVLEX_bm;

		// Don't forget to enable global interrupts in main!
		
		// USARTC0, 8 Data bits, No Parity, 1 Stop bit.
		USART_Format_Set(uart, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);

		/* 
				Set Baudrate to 115200 bps, use the I/O clock frequency that is 32 MHz.
				Do not use the baudrate scale factor
				Baudrate select = (1/(16*(((I/O clock frequency)/Baudrate)-1)
												= 8
		*/
		USART_Baudrate_Set(uart, bsel, 0);		
	}

	// Enable both RX and TX.
	USART_Rx_Enable(uart);
	USART_Tx_Enable(uart);
}

/*****************************************************************************/
void testUartTx(void)
{
	uint8_t sendData;

	// Send data from 122 down to 32 - Readable in a console.
	sendData = 122;
	while(sendData > 32) 
	{
	  // Send one char.
		do{
		/* 
			Wait until it is possible to put data into TX data register.
		  NOTE: If TXDataRegister never becomes empty this will be a DEADLOCK. 
		*/
		}while(!USART_IsTXDataRegisterEmpty(&UARTC0));
		USART_PutChar(&UARTC0, sendData);

		sendData--;
	}
}

/*****************************************************************************/
/*
		Turn on the UART that is connected to the UC3 so that a user can connect
		to a PC if that is desired.
*/
void sendUARTC0(uint8_t* dat)
{
	// Wait until it is possible to put data into TX data register.
	// NOTE: If TXDataRegister never becomes empty this will be a DEADLOCK.
		while(!USART_IsTXDataRegisterEmpty(&UARTC0));
		USART_PutChar(&UARTC0, dat);
}

/*****************************************************************************/
/*
		It is UART F0 that is connected on PORT F (connector J1 on XMEGA-A1 
		Xplained to the RadioBlock.
*/
uint8_t receiveUARTC0(void)
{
	uint8_t receivedData;

	while(!USART_IsRXComplete(&UARTC0));
	receivedData = USART_GetChar(&UARTC0);
	return receivedData;
}

/*****************************************************************************/
void sendUARTF0(uint8_t *array, uint8_t length)
{
	uint8_t i;
	
	for(i=0; i<length; i++)
	{
		// Wait until it is possible to put data into TX data register.
		// NOTE: If TXDataRegister never becomes empty this will be a DEADLOCK.
		while(!USART_IsTXDataRegisterEmpty(&UARTF0));
		USART_PutChar(&UARTF0, array[i]);
	}	
}

/*****************************************************************************/
uint8_t receiveUARTF0(void)
{
	uint8_t receivedData;

	while(!USART_IsRXComplete(&UARTF0));
	receivedData = USART_GetChar(&UARTF0);
	return receivedData;
}

/*****************************************************************************/
/*
		This example shows how to configure TCC0 for basic timer operation.

		The timer will run at the main clock frequency (32MHz) so we need
		to do a bit of math to derive the number of clock cycles to arrive
		at the desired microsecond time out.
 */
void startTimer(uint16_t microseconds)
{
		uint32_t cycles;
		
		// There are 31.25 nano seconds per clock cycle.
		// So we multiply the micro seconds by the clock period to determine
		// the number of clock cycles to achieve the microsecond timeout.
		// That number of cycles becomes our TOP value.
		// We will use 32 nano seconds to preclude floating point arithmetic.
		cycles = 32*microseconds;
		
		// Set period/TOP value.
		TCC0.CCA = cycles;
		
		// Set the CNT to 0.
		TC_SetCount(&TCC0, 0);
		
		/* Enable Input "Capture or Compare" channel A. */
		TC0_EnableCCChannels( &TCC0, TC0_CCAEN_bm );
		
		// Select clock source and start the timer.
		TC0_ConfigClockSource( &TCC0, TC_CLKSEL_DIV1_gc );
		
		// Enable CCA interrupt.
		TC0_SetCCAIntLevel( &TCC0, TC_CCAINTLVL_LO_gc );	
		PMIC.CTRL |= PMIC_LOLVLEN_bm;
}

/*****************************************************************************/
void processResponse(void)
{
	uint8_t i;
	uint8_t cmdBuf[128];
	
	while(1)
		{
			if(1 == ackFlag)
			{		
				// Call ACK processor here.	
				writeUserByte('A'); writeUserByte('C'); writeUserByte('K'); 
				writeUserByte(' '); writeUserByte('O'); writeUserByte('K'); 
				writeUserByte('\n');
				
						
				// Reset ACK variables.
				ackStatus = 0;
				ackFlag = 0;
				isrLen = 0;
				break;
			}			
			
			if(1 == testCmd)
			{
				// Process Test response here.
				writeUserByte('T'); writeUserByte('E'); writeUserByte('S');
				writeUserByte('T'); writeUserByte(' '); writeUserByte('O');
				writeUserByte('K'); writeUserByte('\n');
		
				// Reset variables here.
				testCmd = 0;
				isrLen = 0;
				cmdLen = 0;
				break;
			}
			
			if(1 == wakeCmd)
			{
				// Process Wake Up Indication here.
				writeUserByte('W'); writeUserByte('A'); writeUserByte('K'); 
				writeUserByte('E'); writeUserByte('U'); writeUserByte('P');
				writeUserByte(' '); writeUserByte('O'); writeUserByte('K');
				writeUserByte('\n');
				
				// Reset variables here.
				wakeCmd = 0;
				isrLen = 0;
				cmdLen = 0;
				break;
			}
			
			if(1 == cmdFlag)
			{
				// Subtract one because the length includes the command id.
				cmdLen -= 1;
				for(i=0; i<cmdLen; i++) 
					cmdBuf[i] = readByte();
				// Call the command processor here.
				rxState(cmdBuf, isrCmd, cmdLen);
				
				isrLen = 0;
				cmdLen = 0;
				cmdFlag = 0;
				break;
			}			
		}	
	
	while( userEnd != userStart)
	{
		sendUARTC0(readUserByte());
	}
}

/*****************************************************************************/
void timerLoop(uint8_t num)
{
	for(uint8_t i=0; i<num; i++)
	{
		startTimer(1000);
		while(!ledFlag);
		ledFlag = 0;
	}	
}


/*****************************************************************************/
int main(void)
{
	uint8_t cmdBuf[128];
	uint8_t i;


	// Init peripherals.
	// Clock is set to 32MHz.
	clockInit();
	// This UART is connected to the UC3 device and provides connectivity 
	// via USB.
	uartInit(&UARTC0, 8);	// 115,200 BAUD
	// This UART will be connected to the RadioBlocks device.
	uartInit(&UARTF0, 8); // 115,200 BAUD
	
	// Init the globals.
	isrLen = 0;
	cmdLen = 0;
	cmdFlag = 0;
	testCmd = 0;
	wakeCmd = 0;
	isrCmd = 0;
	ackStatus = 0;
	
	// Fun!
	ledFlag = 0;
	
	// These are used to help in debug. Used to print strings
	// to USARTC0 which is connected to the xplained-a1 usb
	// port through the on-board UC3.
	userEnd = 0;
	userStart = 0;
	
	// DEBUG - Create delay timer.
	//startTimer(1000); // One millisecond test.
	
	// Configure PORTA as output to measure delay timer...
	// These pins are on Xplained header J2
	PORT_SetPinsAsOutput( &PORTA, 0xFF );

	// Check UART operation
	//testUartTx();
	
	// Enable global interrupts.
	sei();

	// Create a function pointer to use with the user uart (UARTC0).
	void (*puartBuf)(uint8_t* , uint8_t);
		
	// Assign that function pointer to the send data to RadioBlocks.
	puartBuf = &sendUARTF0;

	///////////////////////	TEST CODE //////////////////
#if 0
	for(uint16_t i=0; i<CIRCSIZE; i++)
		sniffBuff[i] = 255;
	
	toggleLed(puartBuf, LED_TOGGLE, uartBuf);
	testRequest(puartBuf, uartBuf);
	setAddress(puartBuf, 0x1234, uartBuf);
	getAddress(puartBuf, uartBuf);
	sleepRequest(puartBuf, 1000, uartBuf);
	settingsRequest(puartBuf, uartBuf, RESTORE_CURRENT_SETTINGS); 
	configureUART(puartBuf, DATA_BITS_8, PARITY_NONE, STOP_BITS_1, BAUD_115200, uartBuf);
	setPanid(puartBuf, 0x5678, uartBuf); 
	getPanid(puartBuf, uartBuf); 
	setChannel(puartBuf, CHANNEL_16, uartBuf);
	getChannel(puartBuf,uartBuf);
	setTRXState(puartBuf, TX_ON, uartBuf);
	getTRXState(puartBuf, uartBuf);
	dataRequest(puartBuf, 0x0001, DATA_OPTION_NONE, 0x42, 6, testBuf, uartBuf);
	setTxPower(puartBuf, TX_POWER_2_8_DBM, uartBuf);
	getTxPower(puartBuf, uartBuf);
	//setSecurityKey(puartBuf, uint8_t* key, uartBuf); // max 16 bytes.	
#endif

	toggleLed(puartBuf, LED_TOGGLE, uartBuf);
	processResponse();

	testRequest(puartBuf, uartBuf);
	processResponse();
	processResponse();

	setAddress(puartBuf, 0x1234, uartBuf);
	processResponse();

	getAddress(puartBuf, uartBuf);
	processResponse();
	processResponse();

	setPanid(puartBuf, 0x5678, uartBuf); 
	processResponse();
	getPanid(puartBuf, uartBuf); 
	processResponse();
	processResponse();
	
	setChannel(puartBuf, CHANNEL_16, uartBuf);
	processResponse();
	getChannel(puartBuf,uartBuf);
	processResponse();
	processResponse();
                  
	setTRXState(puartBuf, TX_ON, uartBuf);
	processResponse();
	getTRXState(puartBuf, uartBuf);			
	processResponse();
	processResponse();			
	
	setTxPower(puartBuf, TX_POWER_2_8_DBM, uartBuf);
	processResponse();
	getTxPower(puartBuf, uartBuf);			                                   
	processResponse();
	processResponse();
	
	dataRequest(puartBuf, 0x0001, DATA_OPTION_NONE, 0x42, 6, testBuf, uartBuf);
	processResponse();

  while(1)
  {
		// Fun.
		toggleLed(puartBuf, LED_TOGGLE, uartBuf);
		timerLoop(100);	// WARNING, can BLOCK a loooong time.
		processResponse();
		testBuf[5]++;
		dataRequest(puartBuf, 0x0001, DATA_OPTION_NONE, 0x42, 6, testBuf, uartBuf);
		processResponse();
		
		
		
		/* USER CODE HERE! */

	}	
}

/*****************************************************************************/
/*
		Receive complete interrupt service routine.

		Receive complete interrupt service routine.
		Calls the common receive complete handler with pointer to the correct USART
		as argument.
 */

ISR(USARTF0_RXC_vect)
{
	uint8_t receivedData;

	while(!USART_IsRXComplete(&UARTF0));
	receivedData = USART_GetChar(&UARTF0);
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

/*****************************************************************************/
ISR(TCC0_CCA_vect)
{
	// DEBUG - Check timing
	PORT_TogglePins(&PORTA, 0x01);
	
	// Turn off the timer IRQ
	TC0_SetCCAIntLevel( &TCC0, TC_CCAINTLVL_OFF_gc );
	
	// Fun!
	ledFlag = 1;
}

/*****************************************************************************/

// Functions for the circular UART buffer
void writeUserByte(uint8_t data)
{
	// Put the byte into the buffer.
	userBuf[userStart] = data;

	// Adjust the start value if it wraps around.
	if((userStart + 1) < USERCIRCSIZE)
		userStart += 1;
	else
		userStart = 0;
}

/*****************************************************************************/
uint8_t readUserByte(void)
{
	uint8_t data;

	data = userBuf[userEnd];

	if((userEnd + 1) < USERCIRCSIZE)
		userEnd += 1;
	else
		userEnd = 0;

	return data;
}
