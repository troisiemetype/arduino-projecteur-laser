
// Arduino laser projector
/*
 * This program is intended to control a laser projector
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
 
// serialIO.cpp
// this part of the program deals with:
// serial input: what is sent to the board, verifies that datas are correctly written, and populates the movement buffers
// serial output: send back infos about position, so the program can know how much of the pattern has already be done

/* the data comming from the computer are formated as follow:
 * I40X248Y-120L34M2S56
 * or
 * I40 X248 Y-120 L34 M2 S56
 * with I, X, Y, L, S as long, M as byte.
 * There can be a separation character between var/value pair, it will be split when parsing,
 * but every character cause loosing some speed...
 */



#include <Arduino.h>
#include "planner.h" 
#include "serialIO.h"
#include "driver.h"
#include "settings.h"
#include "system.h"

//This two defines are used to increment buffer pointer.
#define rx_incr(i)	i = (i + 1)%RX_BUFFER_SIZE
#define tx_incr(i)	i = (i + 1)%TX_BUFFER_SIZE

serialState ss;

// defining buffers for serial RX and TX.
char rx_buffer[RX_BUFFER_SIZE];
volatile char rx_head = 0;
char rx_tail = 0;

char tx_buffer[TX_BUFFER_SIZE];
char tx_head = 0;
volatile char tx_tail = 0;

char line_buffer[LINE_BUFFER_SIZE];
char line_counter = 0;

void serial_init(){
	memset(&ss, 0, sizeof(ss));												// Init the serial state struct
	ss.state = SERIAL_IDLE;

	ss.flow_state = SET_XON;												// Enables XON

	//Initialization of RX and TX buffer.
	for (int i=0; i<RX_BUFFER_SIZE; i++){
		rx_buffer[i] = 0;
	}

	for (int i=0; i<TX_BUFFER_SIZE; i++){
		tx_buffer[i] = 0;
	}

	_serial_interrupt_init();
	
//	serial_send_message(F("Liaison série initialisée."));
}

int serial_main(){

	if (ss.state == SERIAL_IDLE){
		return STATE_OK;
	}
//	_serial_append_string("serial state: ");
//	_serial_append_value(ss.state);
//	_serial_append_nl();

	if (ss.state & SERIAL_PARSE){
		return _serial_parse_data();
	}

	if (ss.state & SERIAL_RX){
		return serial_get_data();
	}
}

//This function is called from the main function.
//If there are data in the rx buffer, it copies it into a line buffer.
//That way the RX buffer is kept as empty as possible.
int serial_get_data(){

	if (rx_tail == rx_head){
		bit_false(ss.state, SERIAL_RX);
		return STATE_NO_OP;
	}

//	long debut = micros();

	//Get the current char in rx buffer
	char c = rx_buffer[rx_tail];

//	_serial_append_string("serial rx");
//	_serial_append_nl();
//	serial_send_pair("parsing", c );
	
	//If the char is end of line, set a "flag" (that is, char = 0),
	//initialize line pointer for next time, and call data parser
	if (c == '\n' || c == '\r'){
		line_buffer[line_counter] = 0;
		line_counter = 0;
		rx_incr(rx_tail);

		bit_false(ss.state, SERIAL_RX);
		bit_true(ss.state, SERIAL_PARSE);
		return STATE_OK;

	//Else chars are stored.
	} else if (c >= 'A' && c <= 'Z'){
		line_buffer[line_counter] = c;

	} else if (c >= 'a' && c <= 'z'){
		c -= 32;
		line_buffer[line_counter] = c;

	} else if (c >= '0' && c <= '9'){
		line_buffer[line_counter] = c;

	} else if (c =='-'){
		line_buffer[line_counter] = c;

	} else {
		//Every other chars are ignored.
	}

	//increment line and rx buffer pointer
	line_counter++;
	rx_incr(rx_tail);

	//Verify if we can send an XON signal to computer.
	//If yes, set a flag an enable TX interrupts.
	char queue = _serial_rx_queue();
	if ((queue < RX_FLOW_DOWN) && (ss.flow_state == XOFF_SET)){
		ss.flow_state = SET_XON;
		UCSR0B |= (1 << UDRIE0);
	}
//		_serial_append_value(micros() - debut);
//		_serial_append_nl();

	return STATE_ENTER_AGAIN;
	

}

/* this function parse the data string it receives
 * The string must formed like that:
 * I1X23Y45...
 * Each var (one letter) must be followed by its value.
 * The line read has been prepared before, stripped for wrong chars, and terminat with char 0.
 * The loop turns while there is available data to read
 * It's driven by parser states. For each state we should get precise chars.
 * If these chars are found, parser goes to the next state, and test chars again.
 * A normal loop should be:
 * SERIAL_IDLE						We have just received a new data string.
 *
 * SERIAL_PARSE_VAR 			 			looking for alpha chars a-z, A-Z.
 *
 * SERIAL_PARSE_VALUE 					looking for number 0-9, a minus sign if number is negative.
 *									else, record the pair before to parse the next one.
 * 
 * Then it calls _serial_record_values() to populates the planner buffer.
 */
int _serial_parse_data(){

	if (planner_get_available() < 1){							// Manage the planner buffer queue.
//		serial_send_message("no buffer available");
		return STATE_NO_OP;
	}

//	_serial_append_string("serial parse");
//	_serial_append_nl();
//	long debut = micros();

	bool is_neg;
	char c = 0;
	ss.parser_state = PARSE_VAR;

	while (1){

		c = line_buffer[ss.parser_count];


		if (ss.parser_state == PARSE_VAR){
			if (c >= 'A' && c <= 'Z'){
				ss.inVar = c;
				ss.inValue = 0;
				is_neg = 0;
				ss.parser_state = PARSE_VALUE;
//				_serial_append_byte(ss.inVar);
//				_serial_append_nl();
			}

			ss.parser_count++;

		} else if (ss.parser_state == PARSE_VALUE){
//				_serial_append_byte(c);
			if (c >= '0' && c <= '9'){
				ss.inValue *= 10;
				ss.inValue += (c - 48);
				ss.parser_count++;
			} else if (c =='-'){
				is_neg = 1;
				ss.parser_count++;
//				_serial_append_string("is neg");
			} else {
				if (is_neg == 1){
					ss.inValue = -ss.inValue;
				}
//				_serial_append_nl();
				_serial_record_pair();

				if (c == 0){
					ss.parser_count = 0;
					_serial_record_values();
					bit_false(ss.state, SERIAL_PARSE);
					return STATE_OK;
				}
				return STATE_ENTER_AGAIN;

			}
		}
	}
}

void _serial_record_pair(){

// Sets the right value to the right var
// Sets a flag.
// Records value.

	if (ss.inVar == 'I'){
		ss.parser_data_received |= 32;
		ss.id = ss.inValue;
//		serial_send_pair("I", ss.id);
	} else if (ss.inVar == 'X'){
		ss.parser_data_received |= 16;
		ss.posX = ss.inValue;
//		serial_send_pair("X", ss.posX);

	} else if (ss.inVar == 'Y'){
		ss.parser_data_received |= 8;
		ss.posY = ss.inValue;
//		serial_send_pair("Y", ss.posY);

	} else if (ss.inVar == 'L'){
		ss.parser_data_received |= 4;
		ss.posL = ss.inValue;
//		serial_send_pair("L", ss.posL);

	} else if (ss.inVar == 'S'){
		ss.parser_data_received |= 2;
		ss.speed = ss.inValue;
//		serial_send_pair("speed", ss.speed);

	} else if (ss.inVar == 'M'){
		ss.parser_data_received |= 1;
		ss.mode = ss.inValue;
//		serial_send_pair("mode", ss.mode);
	}
}


// Send the json string received to the planner buffer.
void _serial_record_values(){
	if (ss.parser_data_received != 0){
//		serial_send_message("populates buffer");
		planner_set_buffer(ss.id, ss.posX, ss.posY, ss.posL, ss.speed, ss.mode, ss.parser_data_received);	// Populates the buffer with the values received
	}
	ss.parser_data_received = 0;
	ss.inVar=' ';
	ss.inValue=0;
}

//This sends the go signal when Serial is able to receive
void _serial_send_go(){
//	serial_send_pair("send", 1);
	_serial_append_string("$s");
	_serial_append_nl();
}

/*
//This sends the again signal in case of lost data
void _serial_send_again(){
	serial_send_pair("send", 2);
}
*/

// This function write a pair of data to Serial, formated in json
void serial_send_pair(String name, double value){
	_serial_append_string("{\"");
	_serial_append_string(name);
	_serial_append_string("\":");
	_serial_append_value(value);
	_serial_append_string("}");
	_serial_append_nl();


}

// This function writes a simple message to Serial, formated in json
void serial_send_message(String message){
	_serial_append_string("{\"message\":\"");
	_serial_append_string(message);
	_serial_append_string("\"}");
	_serial_append_nl();
}

/*
// This function is for debugging purpose: it prints "step" on Serial. Used to "replace" code breakpoints.
void serial_step(){
	_serial_append_string("step");
	_serial_append_nl();
}
*/

// Serial interrupt init function. Set the registers according to settings values and needs.
void _serial_interrupt_init(){
//	unsigned int ubr = CLOCK_SPEED/8/BAUDRATE-1;

	unsigned int ubr = CLOCK_SPEED/(8L * BAUDRATE) - 1;
	UCSR0A |= (1 << U2X0);
//	unsigned int ubr = CLOCK_SPEED/(16L * BAUDRATE) - 1;
//	UCSR0A &= ~(1 << U2X0);

	cli();

	UBRR0H = (ubr>>8);												// Set the baudrate register, high byte first, then low.
	UBRR0L = ubr;

	// UCSR0A is not touched: it contains flags.
	UCSR0B |= (1 << RXCIE0);										// Enable RX interrupts.
//	UCSR0B |= (1 << TXCIE0);										// Enable TX interrupts.
//	UCSR0B |= (1 << UDRIE0);										// Enable empty buffer interrupt. Enable when writing data.
	UCSR0B |= (1 << RXEN0);											// Enable RX.
	UCSR0B |= (1 << TXEN0);											// Enable TX.

	sei();

}

// ISR RX interrupt.
// This populates the RX buffer with incoming bytes.
ISR(USART_RX_vect){
//	long debut = TCNT1;
	char head = rx_head;											// Copy the rx_head in a local var to preserve volatile.
	rx_buffer[head] = UDR0;											// Copy UDR0 byte to buffer queue.

	rx_incr(head);													// Increment buffer pointer.
	rx_head = head;													// update buffer pointer.

	bit_true(ss.state, SERIAL_RX);

	char queue = _serial_rx_queue();

	if ((queue > RX_FLOW_UP) && (ss.flow_state == XON_SET)){		// Test buffer size against size limit.
		ss.flow_state = SET_XOFF;									// Set the new flow control state
		UCSR0B |= (1 << UDRIE0);									// Set back UDRE ISR
	}
//	_serial_append_value(TCNT1 - debut);
//	_serial_append_nl();

}

// ISR Empty buffer interrupt.
// Sends data while their is to.
ISR(USART_UDRE_vect){

//	long debut = TCNT1;

	char tail = tx_tail;											// Temp copy to limit volatile acces.	
	// If flow control must change state, the xon/xoff state is sent before the TX buffer, that is skipped until next iteration.
	if (ss.flow_state == SET_XOFF){
		UDR0 = XOFF_CHAR;
		ss.flow_state = XOFF_SET;									// Send XOFF.
	} else if (ss.flow_state == SET_XON){
		UDR0 = XON_CHAR;											// Send XON.
		ss.flow_state = XON_SET;
	} else {
		UDR0 = tx_buffer[tail];										// Copy data buffer to TX data register.
		tx_incr(tail);												// Increment buffer tail pointer.
		tx_tail = tail;												// Copy back the temporary value to tx_tail.
	}
	if (tail == tx_head){											// if head == tail, then nothing left to send, disconnect interrupt.
		UCSR0B &= ~(1 << UDRIE0);
	}
//	_serial_append_value(TCNT1 - debut);
//	_serial_append_nl();
}


char _serial_rx_queue(){
	ss.queue = rx_head - rx_tail;
	if (ss.queue < 0){
		ss.queue += RX_BUFFER_SIZE;
	}
//	_serial_append_value(ss.queue);
//	_serial_append_nl();
	return ss.queue;
}


void _serial_append_string(String data){
	int data_length = data.length();
	for (int i=0; i<data_length; i++){
		_serial_append_byte(data.charAt(i));
	}
}
void _serial_append_value(double data){
	String data_to_send = String(data, 3);
	_serial_append_string(data_to_send);
}

void _serial_append_nl(){
	_serial_append_byte(NL_CHAR);
}

void _serial_append_byte(char data){
	while ((tx_head + 1) % TX_BUFFER_SIZE == tx_tail){
		//Room to place some calls to function like planner_plan_move()
	}
	tx_buffer[tx_head] = data;
	tx_incr(tx_head);

	UCSR0B |= (1 << UDRIE0);
}

void _serial_clear_rx_buffer(){
	rx_tail = rx_head;
}
