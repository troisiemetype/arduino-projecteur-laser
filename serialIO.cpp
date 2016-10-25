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

//This two defines are used to increment buffer pointer.
#define rx_incr(i)	i = (i + 1)%RX_BUFFER_SIZE
#define tx_incr(i)	i = (i + 1)%TX_BUFFER_SIZE

serialState ss;
long baudrate = BAUDRATE;

// defining buffers for serial RX and TX.
char rx_buffer[RX_BUFFER_SIZE];
volatile char rx_head = 0;
char rx_tail = 0;

char tx_buffer[TX_BUFFER_SIZE];
char tx_head = 0;
volatile char tx_tail = 0;

volatile bool to_read_flag = 0;

void serial_init(){
	memset(&ss, 0, sizeof(ss));												// Init the serial state struct
	ss.flow_state = SET_XON;												// Enables XON

	//Initialization of RX and TX buffer.
	for (int i=0; i<RX_BUFFER_SIZE; i++){
		rx_buffer[i] = 0;
	}

	for (int i=0; i<TX_BUFFER_SIZE; i++){
		tx_buffer[i] = 0;
	}

	_serial_interrupt_init();
	
	serial_send_message("Liaison série initialisée.");
}

bool serial_get_data(){
//	serial_xon_xoff();														// Verifies the buffer size
	ss.parser_state = PARSING_IDLE;

	if (planner_get_available() < 2){										// Manage the planner buffer queue.
//		serial_send_message("no buffer available");
		return false;
	}

	if (to_read_flag){
//		_serial_append_string("-to read-");
//		_serial_append_value(rx_head);
//		_serial_append_value(rx_tail);
		_serial_parse();
	}

	return true;

}

// This function is the entrance of a new string. It sets the parser_state for info or data
// If the new string starts by anything else than start charcaters, it's discarded.
void _serial_parse(){
	if (rx_buffer[rx_tail] == '$'){
		_serial_parse_cfg();
	} else {
		_serial_parse_data();
	}
}

/* this function parse the data string it receives
 * The string must formed like that:
 * I1X23Y45...
 * Each var (one letter) must be followed by its value.
 * The loop turns while there is available data to read
 * The function is called by the serial parser above.
 * It's driven by parser states. For each state we should get precise chars.
 * If these chars are found, parser goes to th next state, and test chars again.
 * A normal loop should be:
 * PARSING_IDLE						We have just received a new data string.
 *
 * PARSING_VAR 			 			looking for alpha chars a-z, A-Z.
 *
 * PARSING_JSON_VAR_OK 				var was OK.
 *
 * PARSING_VALUE 					looking for number 0-9, a minus sign if number is negative, or char a-z, A-Z if new pair.
 *									if char a-z, A-Z, record the pair before to parse the next one.
 *
 * On each step, the next data char is tested against what it should be. If not found, their is an error and the loop stops.
 * If a pair (var + value) is obtained, _serial_record_pair is called, which stores the value in the serial singleton and sets a flag.
 * Last, if the state is PARSING_JSON_END, then it means the parsing was without problem.
 * Then it calls _serial_record_values() to populates the planner buffer.
 */
void _serial_parse_data(){
	bool is_neg = false;
	char in_byte = 0;
	ss.parser_state = PARSING_VAR;
	while (rx_tail != rx_head){
		in_byte = rx_buffer[rx_tail];

		char queue = _serial_rx_queue();								// Get queue size.

		if ((queue < RX_FLOW_DOWN) && (ss.flow_state == XOFF_SET)){		// Test buffer size against size limit.
			ss.flow_state = SET_XON;									// Set the new flow control state
			UCSR0B |= (1 << UDRIE0);									// Set back UDRE ISR
	}


//		_serial_append_value(rx_head);
//		_serial_append_value(rx_tail);

		if (ss.parser_state == PARSING_VAR){
			if ((in_byte >= 'a' && in_byte <= 'z') || (in_byte >= 'A' && in_byte <= 'Z')){
				if (in_byte >= 'a' && in_byte <= 'z'){
					in_byte -= 32;
				}
				ss.inVar = in_byte;
				ss.inValue = 0;
				is_neg = 0;
				ss.parser_state = PARSING_VAR_OK;
//					_serial_append_string(ss.inVar);
			}
			rx_incr(rx_tail);

		} else if (ss.parser_state == PARSING_VAR_OK){
			ss.parser_state = PARSING_VALUE;

		} else if (ss.parser_state == PARSING_VALUE){
			if (in_byte >= '0' && in_byte <= '9'){
				ss.inValue *= 10;
				ss.inValue += (in_byte - 48);
				rx_incr(rx_tail);
			} else if (in_byte =='-'){
				is_neg = 1;
				rx_incr(rx_tail);
//				_serial_append_string("is neg");
			} else {
				if (is_neg == 1){
					ss.inValue = -ss.inValue;
				}
				_serial_record_pair();
//				_serial_append_value(ss.inValue);
				if ((in_byte >= 'a' && in_byte <= 'z') || (in_byte >= 'A' && in_byte <= 'Z')){
					ss.parser_state = PARSING_VAR;
//					_serial_append_string("parsing var");
				} else {
					rx_incr(rx_tail);
				}
			}

		}
	}
//	_serial_append_value(ss.id);
//	_serial_append_nl();
	_serial_record_values();
	_serial_flush_rx_buffer();
//	_serial_send_go();
	ss.parser_state = PARSING_IDLE;
	to_read_flag =0;
}

void _serial_record_pair(){

// Sets the right value to the right var
// Sets a flag.
// Records value.

	if (ss.inVar == "X"){
		ss.parser_data_received |= 16;
		ss.posX = ss.inValue;
//		serial_send_pair("X", ss.posX);

	} else if (ss.inVar == "Y"){
		ss.parser_data_received |= 8;
		ss.posY = ss.inValue;
//		serial_send_pair("Y", ss.posY);

	} else if (ss.inVar == "L"){
		ss.parser_data_received |= 4;
		ss.posL = ss.inValue;
//		serial_send_pair("L", ss.posL);

	} else if (ss.inVar == "S"){
		ss.parser_data_received |= 2;
		ss.speed = ss.inValue;
//		serial_send_pair("speed", ss.speed);

	} else if (ss.inVar == "M"){
		ss.parser_data_received |= 1;
		ss.mode = ss.inValue;
//		serial_send_pair("mode", ss.mode);
	}
}


// Send the json string received to the planner buffer.
void _serial_record_values(){
	if (ss.parser_data_received != 0){
//		serial_send_message("populates buffer");
		planner_set_buffer(ss.posX, ss.posY, ss.posL, ss.speed, ss.mode, ss.parser_data_received);	// Populates the buffer with the values received
	}
	ss.parser_data_received = 0;
	ss.inVar="";
	ss.inValue=0;
}

// Parse a cfg command.
// Empty for know, excpet that it reads data in the RX buffer. Otherwise program block.
void _serial_parse_cfg(){
	char in_byte = 0;
	while (rx_tail != rx_head){
		in_byte = rx_buffer[rx_tail];
		rx_incr(rx_tail);
	}
	_serial_send_go();
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


// This function is for debugging purpose: it prints "step" on Serial. Used to replace code breakpoints.
void serial_step(){
	_serial_append_string("step");
	_serial_append_nl();
}


// Serial interrupt init function. Set the registers according to settings values and needs.
void _serial_interrupt_init(){
	unsigned int ubr = CLOCK_SPEED/8/BAUDRATE-1;
	UCSR0A |= (1 << U2X0);
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
	char head = rx_head;											// Copy the rx_head in a local var to preserve volatile.
	rx_buffer[head] = UDR0;											// Copy UDR0 byte to buffer queue.

	if (rx_buffer[head] == NL_CHAR){								// If EOL, set a flag that serial_get_data will read.
		to_read_flag = 1;
	} else if (rx_buffer[head] == XON_CHAR){						// Implement XON/XOFF for TX?
	} else if (rx_buffer[head] == XOFF_CHAR){
	}

	rx_incr(head);													// Increment buffer pointer.
	rx_head = head;													// update buffer pointer.

	char queue = _serial_rx_queue();

	if ((queue > RX_FLOW_UP) && (ss.flow_state == XON_SET)){		// Test buffer size against size limit.
		ss.flow_state = SET_XOFF;									// Set the new flow control state
		UCSR0B |= (1 << UDRIE0);									// Set back UDRE ISR
	}

}

// ISR Empty buffer interrupt.
// Sends data while their is to.
ISR(USART_UDRE_vect){

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

void _serial_flush_rx_buffer(){
	_serial_clear_rx_buffer();
	to_read_flag = 0;

//	ss.flow_state = SET_XON;									// Set the new flow control state
//	UCSR0B |= (1 << UDRIE0);									// Set back UDRE ISR
}