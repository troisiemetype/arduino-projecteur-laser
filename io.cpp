
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

#include "io.h"

serialState ss;

SerialDue serial = SerialDue();

void UART_Handler(void){
	serial.IrqHandler();
}

CircularBuffer _line_buffer = CircularBuffer(LINE_BUFFER_SIZE);

void io_init(){
	//Init the serial state struct
	memset(&ss, 0, sizeof(ss));
	ss.state = SERIAL_IDLE;

	serial.begin(BAUDRATE, SERIAL_8N1, SERIAL_SOFT_FLOW);
	
}

//Main serial function. Empty buffer or parse a string, depending on machine state.
int io_main(){

//	io_send_message("io main");
	if (ss.state == SERIAL_IDLE){
		if (serial.available()){
			bit_true(ss.state, SERIAL_RX);
		} else {
			return STATE_OK;
		}
	}
//	_io_append_string("serial state: ");
//	_io_append_value(ss.state);
//	_io_append_nl();
	
	if (ss.state & SERIAL_PARSE){
		return _io_parse_data();
	}

	if (ss.state & SERIAL_RX){
		return _io_get_data();
	}
}

//This function is called from the main function.
//If there are data in the rx buffer, it copies it into a line buffer.
//That way the RX buffer is kept as empty as possible.
int _io_get_data(){

	if (serial.available() == 0){
		bit_false(ss.state, SERIAL_RX);
		return STATE_NO_OP;
	}

//	long debut = micros();

	//Get the current char in rx buffer
	char c = serial.read();

//	_io_append_string("serial rx");
//	_io_append_nl();
//	io_send_pair("parsing", c );
	
	//If the char is end of line, set a "flag" (that is, char = 0),
	//initialize line pointer for next time, and call data parser
	if (c == '\n' || c == '\r'){
		_line_buffer.set(0);
		bit_false(ss.state, SERIAL_RX);
		bit_true(ss.state, SERIAL_PARSE);
		return STATE_OK;

	//Else chars are stored.
	} else if (c >= 'A' && c <= 'Z'){
		_line_buffer.set(c);

	} else if (c >= 'a' && c <= 'z'){
		c -= 32;
		_line_buffer.set(c);

	} else if (c >= '0' && c <= '9'){
		_line_buffer.set(c);

	} else if (c =='-'){
		_line_buffer.set(c);

	} else {
		//Every other chars are ignored.
	}

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
 * Then it calls _io_record_values() to populates the planner buffer.
 */
int _io_parse_data(){

	if (planner_get_available() < 1){							// Manage the planner buffer queue.
//		io_send_message("no buffer available");
		return STATE_NO_OP;
	}

//	_io_append_string("serial parse");
//	_io_append_nl();
//	long debut = micros();

	bool is_neg;
	char c = 0;
	ss.parser_state = PARSE_VAR;

	while (1){

		c = _line_buffer.peek();
//		_io_append_byte(c);


		if (ss.parser_state == PARSE_VAR){
			if (c >= 'A' && c <= 'Z'){
				ss.inVar = c;
				ss.inValue = 0;
				is_neg = 0;
				ss.parser_state = PARSE_VALUE;
				_line_buffer.get();
//				_io_append_byte(ss.inVar);
//				_io_append_nl();
			}

		} else if (ss.parser_state == PARSE_VALUE){
//				_io_append_byte(c);
			if (c >= '0' && c <= '9'){
				ss.inValue *= 10;
				ss.inValue += (c - 48);
				_line_buffer.get();
			} else if (c =='-'){
				is_neg = 1;
				_line_buffer.get();
//				_io_append_string("is neg");
			} else {
				if (is_neg == 1){
					ss.inValue = -ss.inValue;
				}
//				_io_append_nl();
				_io_record_pair();

				if (c == 0){
					_line_buffer.empty();
					_io_record_values();
					bit_false(ss.state, SERIAL_PARSE);
					return STATE_OK;
				}
				return STATE_ENTER_AGAIN;

			}
		}
	}
}

void _io_record_pair(){

// Sets the right value to the right var
// Sets a flag.
// Records value.

	if (ss.inVar == 'I'){
		ss.parser_data_received |= 32;
		ss.id = ss.inValue;
//		io_send_pair("I", ss.id);
	} else if (ss.inVar == 'X'){
		ss.parser_data_received |= 16;
		ss.posX = ss.inValue;
//		io_send_pair("X", ss.posX);

	} else if (ss.inVar == 'Y'){
		ss.parser_data_received |= 8;
		ss.posY = ss.inValue;
//		io_send_pair("Y", ss.posY);

	} else if (ss.inVar == 'L'){
		ss.parser_data_received |= 4;
		ss.posL = ss.inValue;
//		io_send_pair("L", ss.posL);

	} else if (ss.inVar == 'S'){
		ss.parser_data_received |= 2;
		ss.speed = ss.inValue;
//		io_send_pair("speed", ss.speed);

	} else if (ss.inVar == 'M'){
		ss.parser_data_received |= 1;
		ss.mode = ss.inValue;
//		io_send_pair("mode", ss.mode);
	}
}


// Send the json string received to the planner buffer.
void _io_record_values(){
	if (ss.parser_data_received != 0){
//		io_send_message("populates buffer");
		planner_set_buffer(ss.id, ss.posX, ss.posY, ss.posL, ss.speed, ss.mode, ss.parser_data_received);	// Populates the buffer with the values received
	}
	ss.parser_data_received = 0;
	ss.inVar=' ';
	ss.inValue=0;
}

// This function write a pair of data to Serial, formated in json
void io_send_pair(String name, double value){
	_io_append_string("{\"");
	_io_append_string(name);
	_io_append_string("\":");
	_io_append_value(value);
	_io_append_string("}");
	_io_append_nl();
}

// This function writes a simple message to Serial, formated in json
void io_send_message(String message){
	_io_append_string("{\"message\":\"");
	_io_append_string(message);
	_io_append_string("\"}");
	_io_append_nl();
}

void io_send_value(double value){
	_io_append_string("{");
	_io_append_value(value);
	_io_append_string("}");
	_io_append_nl();
}

/*
// This function is for debugging purpose: it prints "step" on Serial. Used to "replace" code breakpoints.
void io_step(){
	_io_append_string("step");
	_io_append_nl();
}
*/

void _io_append_string(String data){
	int data_length = data.length();
	for (int i=0; i<data_length; i++){
		_io_append_byte(data.charAt(i));
	}
}
void _io_append_value(double data){
	String data_to_send = String(data, 3);
	_io_append_string(data_to_send);
}

void _io_append_nl(){
	_io_append_byte(NL_CHAR);
}

//TODO: verify if there is room in the TX buffer, and what to do if not?
void _io_append_byte(char data){
	serial.write(data);
}

