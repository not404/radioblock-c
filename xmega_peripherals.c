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
 





/*! \brief Set new compare value for compare channel A. ( Double buffered )
 *
 *  This macro sets a new compare value for compare channel A. The compare
 *  channel A buffer register is used, so the new period will be valid from the
 *  next update condition.
 *
 *  \param _tc               Timer/Counter 0 or 1 module instance.
 *  \param _compareValue     New compare value for compare channel A.
 */
#define TC_SetCompareA( _tc, _compareValue ) ( (_tc)->CCABUF = (_compareValue) )













/*! \brief This example shows how to configure TCC0 for basic timer operation.
 *
 *  This function implements example 1, "Basic Timer/Counter Operation" from
 *  the "Getting Started" section of application note AVR1306.
 */
void tc0Init( void )
{
	/* Set period/TOP value. */
	TC_SetPeriod( &TCC0, 0x1000 );

	/* Select clock source. */
	TC0_ConfigClockSource( &TCC0, TC_CLKSEL_DIV1_gc );

	/* Enable Input Capture channel A. */
	TC0_EnableCCChannels( &TCC0, TC0_CCAEN_bm );
}


ISR(TCC0_CCA_vect)
{
	
}