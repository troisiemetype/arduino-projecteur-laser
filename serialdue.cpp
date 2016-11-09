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

 /*
  * When implementing a SerialDue object in code, the Interrupt handler must be attached:
  * void UART_Handler(void){
  *     object_name.IrqHandler();
  * }
  */

#include "serialdue.h"

SerialDue::SerialDue(){

}

// Begin default serial: given baudrate, 8N1, no flow control
void SerialDue::begin(const uint32_t baudrate){
	begin(baudrate, SERIAL_8N1);
}

// Begin serial: given baudrate, given UART mode, no flow control
void SerialDue::begin(const uint32_t baudrate, const UARTModes config){
	begin(baudrate, config, SERIAL_NO_FLOW);
}

//Begin serial: given baudrate, given UART mode, given flow control
void SerialDue::begin(const uint32_t baudrate, const UARTModes config, const FLOWModes flowMode){
    _flowMode = flowMode;
    _flowState = XON_SET;

    uint32_t modeReg = static_cast<uint32_t>(config) & 0x00000E00;
    init(baudrate, modeReg | UART_MR_CHMODE_NORMAL);
}

// Init the serial object
void SerialDue::init(const uint32_t baudrate, const uint32_t modeReg){

    _rxBuffer = new CircularBuffer(RX_BUFFER_SIZE);
    _txBuffer = new CircularBuffer(TX_BUFFER_SIZE);
    //using chip.h/sam.h/sam3x8e.h

    _pUart = UART;
    _dwIrq = UART_IRQn;
    _dwId = ID_UART;


    // Configure PMC
    pmc_enable_periph_clk(_dwIrq);

    // Disable PDC channel
    _pUart->UART_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;

    // Reset and disable receiver and transmitter
    _pUart->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX | UART_CR_RXDIS | UART_CR_TXDIS;

    // Configure mode
    _pUart->UART_MR = modeReg;

    // Configure baudrate (asynchronous, no oversampling)
    _pUart->UART_BRGR = (SystemCoreClock / baudrate) >> 4;

    // Configure interrupts
    _pUart->UART_IDR = 0xFFFFFFFF;
    _pUart->UART_IER = UART_IER_RXRDY | UART_IER_OVRE | UART_IER_FRAME;

    // Enable UART interrupt in NVIC
    NVIC_EnableIRQ(_dwIrq);

    // Enable receiver and transmitter
    _pUart->UART_CR = UART_CR_RXEN | UART_CR_TXEN;
}

int SerialDue::available(void){
    return _rxBuffer->queue();
}

int SerialDue::availableForWrite(void){
    return TX_BUFFER_SIZE - _txBuffer->queue();
}

char SerialDue::read(void){
    if (_flowMode == SERIAL_SOFT_FLOW && _rxBuffer->queue() < RX_FLOW_DOWN){
        if (_flowState == XOFF_SET){
            //Send XOFF
            _flowState = SET_XON;
            _pUart->UART_IER = UART_IER_TXRDY;
        }
    }

    return _rxBuffer->get();
}

void SerialDue::write(const char c){
    _txBuffer->set(c);
    _pUart->UART_IER = UART_IER_TXRDY;
}

// End the serial object
void SerialDue::end(void){
	//Clear buffer
	//flush
	//Disable UART interrupt
}

// Define the interrupt handler
void SerialDue::IrqHandler(void){
    // Get UART status
    uint32_t status = _pUart->UART_SR;

    // Is there data in the RX buffer?
    if ((status & UART_SR_RXRDY) == UART_SR_RXRDY){
        _rxBuffer->set(_pUart->UART_RHR);

         // Manage flow control if set
        if ((_flowMode == SERIAL_SOFT_FLOW) && (_rxBuffer->queue() > RX_FLOW_UP)){
            if (_flowState == XON_SET){
                //Send XOFF
                _flowState = SET_XOFF;
                _pUart->UART_IER = UART_IER_TXRDY;
            }
        }
    }

    // Can we send data?
    if ((status & UART_SR_TXRDY) == UART_SR_TXRDY){
        //If flow must be stopped, then send the XOFF Char
        if (_flowState == SET_XOFF){
            _pUart->UART_THR = XOFF_CHAR; 
            _flowState = XOFF_SET;

        //If flow must be enabled again, send XON.
        } else if (_flowState == SET_XON){
            _pUart->UART_THR = XON_CHAR; 
            _flowState = XON_SET;

        } else {
            if (_txBuffer->head != _txBuffer->tail){
                //Send data
                _pUart->UART_THR = _txBuffer->get();
            } else {
                // disable TX interrupt
                _pUart->UART_IDR = UART_IDR_TXRDY;
            }

        }
    }
}