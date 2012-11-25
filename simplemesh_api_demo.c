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
#include "simplemesh-api/simplemesh-api.h"
#include "xmega_uart/usart_driver.h"
#include "xmega_timer_counter/TC_driver.h"
#include "xmega_clock/clksys_driver.h"
#include "xmega_gpio/port_driver.h"

/*****************************************************************************
*****************************************************************************/
// Macros


// Define that selects the Usart used in example.
#define UARTC0 USARTC0
#define UARTF0 USARTF0


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

// Used for printing to USARTC0
uint8_t userBuf[USERCIRCSIZE];
uint8_t userEnd;
uint8_t userStart;

// Used for sending a "Hello!" string over the air.
uint8_t testBuf[6] = {'H', 'e', 'l', 'l', 'o', '!'};
/*****************************************************************************
*****************************************************************************/
// Functions
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
void timerLoop(uint8_t num)
{
	for(uint8_t i=0; i<num; i++)
	{
		startTimer(1000);
		while(!ledFlag);
		ledFlag = 0;
	}	
}

void usartUartPrint(void)
{
	#if XPLAINED_A1_DEMO
	// Print out to the Xplained-A1 uart connected through UC3 to USB...
	while( userEnd != userStart)
	{
		sendUARTC0(readUserByte());
	}
	#endif
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
	
	// Use one of the Xplained-A1 pins. SW4 - PD4
	PORT_SetPinsAsInput( &PORTD, 0x10 );

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
	usartUartPrint();
		
	testRequest(puartBuf, uartBuf);
	processResponse();
	usartUartPrint();		
	processResponse();
	usartUartPrint();
		
	setAddress(puartBuf, 0x1234, uartBuf);
	processResponse();
	usartUartPrint();	

	getAddress(puartBuf, uartBuf);
	processResponse();
	usartUartPrint();	
	processResponse();
	usartUartPrint();

	setPanid(puartBuf, 0x5678, uartBuf); 
	processResponse();
	usartUartPrint();	
	getPanid(puartBuf, uartBuf); 
	processResponse();
	usartUartPrint();	
	processResponse();
	usartUartPrint();	
	
	setChannel(puartBuf, CHANNEL_16, uartBuf);
	processResponse();
	usartUartPrint();	
	getChannel(puartBuf,uartBuf);
	processResponse();
	usartUartPrint();	
	processResponse();
	usartUartPrint();	
                  
//	setTRXState(puartBuf, TX_ON, uartBuf);
//	processResponse();
//	getTRXState(puartBuf, uartBuf);			
//	processResponse();
//	processResponse();			
	
	setTxPower(puartBuf, TX_POWER_2_8_DBM, uartBuf);
	processResponse();
	usartUartPrint();	
	getTxPower(puartBuf, uartBuf);			                                   
	processResponse();
	usartUartPrint();	
	processResponse();
	usartUartPrint();	
	
	dataRequest(puartBuf, 0x0001, DATA_OPTION_NONE, 0x42, 6, testBuf, uartBuf);
	processResponse();
	usartUartPrint();	

	setTRXState(puartBuf, RX_ON, uartBuf);
	processResponse();
	usartUartPrint();		
	getTRXState(puartBuf, uartBuf);			
	processResponse();
	usartUartPrint();		
	processResponse();
	usartUartPrint();	
			
  while(1)
  {
		processResponse();
		usartUartPrint();			
		// Fun.
//		toggleLed(puartBuf, LED_TOGGLE, uartBuf);
//		timerLoop(100);	// WARNING, can BLOCK a loooong time.
//		processResponse();
//		testBuf[5]++;
//		setTRXState(puartBuf, TX_ON, uartBuf);
//		processResponse();
//		dataRequest(puartBuf, 0x0001, DATA_OPTION_NONE, 0x42, 6, testBuf, uartBuf);
//		processResponse();
//		setTRXState(puartBuf, RX_ON, uartBuf);
//		processResponse();
		
		
		
		/* USER CODE HERE! */

	}	
}

/*****************************************************************************/
/*
		Receive complete interrupt service routine.

		Calls the common receive complete handler with pointer to the correct USART
		as argument.
 */

ISR(USARTF0_RXC_vect)
{
	uint8_t receivedData;

	while(!USART_IsRXComplete(&UARTF0));
	receivedData = USART_GetChar(&UARTF0);
	parseRxBytes(receivedData);
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
