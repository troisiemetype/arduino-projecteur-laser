// Serial Due
/*
 * This class is intended to implement a serial link on the arduino Due,
 * with software flow control (Xon, Xoff) and hardware flow control (CTS, DTR)
 * Copyright (C) 2016  Pierre-Loup Martin
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#ifndef SERIAL_DUE_H
#define SERIAL_DUE_H

//include Atmel CMSIS
#include <chip.h>

#include "circularbuffer.h"

// Size of RX and TX buffers
#define RX_BUFFER_SIZE				1024
#define TX_BUFFER_SIZE				256

// For Xon flow contrl.
#define RX_FLOW_UP					62
#define RX_FLOW_DOWN				32

#define XON_SET						1
#define SET_XOFF					2
#define XOFF_SET					3
#define SET_XON						4

#define XON_CHAR					(char)17				// XON char. 0x11; ctrl+Q
#define XOFF_CHAR					(char)19				// XOFF char. 0x13; ctrl+S
#define RESET_CHAR					(char)24				// Rest char. 0x18; ctrl+X
#define NL_CHAR						(char)10				// New line char. 0x0A. \n
#define CR_CHAR						(char)13				// Carriage return char. 0x0D. \r


#define SERIAL_8N1					SerialDue::Mode_8N1
#define SERIAL_8N2					SerialDue::Mode_8N2
#define SERIAL_8E1					SerialDue::Mode_8E1
#define SERIAL_8E2					SerialDue::Mode_8E2
#define SERIAL_8O1					SerialDue::Mode_8O1
#define SERIAL_8O2					SerialDue::Mode_8O2

#define SERIAL_NO_FLOW				SerialDue::No_Flow
#define SERIAL_SOFT_FLOW			SerialDue::Soft_Flow


class SerialDue
{
public:
	enum UARTModes {
		Mode_8N1 = US_MR_CHRL_8_BIT | US_MR_NBSTOP_1_BIT | UART_MR_PAR_NO,
		Mode_8N2 = US_MR_CHRL_8_BIT | US_MR_NBSTOP_2_BIT | UART_MR_PAR_NO,
		Mode_8E1 = US_MR_CHRL_8_BIT | US_MR_NBSTOP_1_BIT | UART_MR_PAR_EVEN,
		Mode_8E2 = US_MR_CHRL_8_BIT | US_MR_NBSTOP_2_BIT | UART_MR_PAR_EVEN,
		Mode_8O1 = US_MR_CHRL_8_BIT | US_MR_NBSTOP_1_BIT | UART_MR_PAR_ODD,
		Mode_8O2 = US_MR_CHRL_8_BIT | US_MR_NBSTOP_2_BIT | UART_MR_PAR_ODD,
	};

	enum FLOWModes {
		No_Flow = 0,
		Soft_Flow = 1,
	};

	SerialDue(void);
	void begin(const uint32_t baudrate);
	void begin(const uint32_t baudrate, const UARTModes config);
	void begin(const uint32_t baudrate, const UARTModes config, const FLOWModes);
    void init(const uint32_t baudrate, const uint32_t config);
    int available(void);
    int availableForWrite(void);
    char read(void);
    void write(const char c);
	void end(void);

	void IrqHandler(void);

protected:

	uint8_t _flowMode;
	volatile uint8_t _flowState;

	CircularBuffer *_rxBuffer;
	CircularBuffer *_txBuffer;

    Uart* _pUart;
    IRQn_Type _dwIrq;
    uint32_t _dwId;


};

#endif
