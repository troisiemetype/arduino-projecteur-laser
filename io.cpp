
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
 
// SerialUSBIO.cpp
// this part of the program deals with:
// SerialUSB input: what is sent to the board, verifies that datas are correctly written, and populates the movement buffers
// SerialUSB output: send back infos about position, so the program can know how much of the pattern has already be done

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

CircularBuffer _line_buffer = CircularBuffer(LINE_BUFFER_SIZE);

void io_init(){
	//Init the SerialUSB state struct
	memset(&ss, 0, sizeof(ss));
	ss.state = SERIAL_IDLE;

	ss.parser_state = PARSE_IDLE;

	Serial1.begin(BAUDRATE);								//Debug serial

	Serial.begin(BAUDRATE);									//Communication serial
	
	while(Serial.available()){
			Serial.read();
		}
	
}

//Main SerialUSB function. Call data parsing if needed.
int io_main(){

//	debug_send_message("io main");
	if (ss.state == SERIAL_IDLE){
		if (Serial.available()){
			bit_true(ss.state, SERIAL_RX);
		} else {
			return STATE_OK;
		}
	}
//	_debug_append_string("SerialUSB state: ");
//	_debug_append_value(ss.state);
//	_debug_append_nl();

	if (ss.state & SERIAL_RX){
		return _io_get_data();
	}
}

//This function is called from the main function.
//Initialise vars, parse data
//verifies checksum
//Send to planner when complete, or ask for data again if corrupted.
int _io_get_data(){

	if (Serial.available() == 0){
		bit_false(ss.state, SERIAL_RX);
		return STATE_NO_OP;
	}

	if (ss.parser_state == PARSE_IDLE){
//		_debug_append_string("parse header");
//		_debug_append_nl();
		byte c = Serial.read();
		ss.data_received = c;
		_debug_append_byte(c);
		ss.data_to_read = 1;
		ss.posX = 0;
		ss.posY = 0;
		ss.posL = 0;
		ss.speed = 0;
		ss.mode = 0;
		ss.checksum = c;

		if (ss.data_received & FLAG_X){
			ss.data_to_read += 2;
		}

		if (ss.data_received & FLAG_Y){
			ss.data_to_read += 2;
		}

		if (ss.data_received & FLAG_L){
			ss.data_to_read += 1;
		}

		if (ss.data_received & FLAG_SPEED){
			ss.data_to_read += 2;
		}

		if (ss.data_received & FLAG_MODE){
			ss.data_to_read += 1;
		}

		ss.parser_state = PARSE_HEADER;

		return STATE_ENTER_AGAIN;

	} else if (ss.parser_state == PARSE_HEADER){

		if (Serial.available() < ss.data_to_read){
//			_debug_append_string("waiting for data");
//			_debug_append_nl();
			return STATE_ENTER_AGAIN;
		}

		if (ss.data_received & FLAG_X){
			_io_parse_int(&ss.posX);
//			_debug_append_string("parse X");
//			_debug_append_nl();
		}

		if (ss.data_received & FLAG_Y){
			_io_parse_int(&ss.posY);
//			_debug_append_string("parse Y");
//			_debug_append_nl();
		}

		if (ss.data_received & FLAG_L){
			_io_parse_char(&ss.posL);
//			_debug_append_string("parse L");
//			_debug_append_nl();
		}

		if (ss.data_received & FLAG_SPEED){
			_io_parse_int(&ss.speed);
//			_debug_append_string("parse speed");
//			_debug_append_nl();
		}

		if (ss.data_received & FLAG_MODE){
			_io_parse_char(&ss.mode);
//			_debug_append_string("parse mode");
//			_debug_append_nl();
		}

		byte c = Serial.read();
		_debug_append_byte(c);
		_debug_append_byte(ss.checksum);
		while(Serial.available()){
//			_debug_append_string("empty serial");
//			_debug_append_nl();
			Serial.read();
		}

		if (c != ss.checksum){
			ss.parser_state = PARSE_IDLE;
			_debug_append_byte(IO_SEND_AGAIN);
			return STATE_OK;

		} else {
			ss.parser_state = PARSE_RECORD;

			if (planner_get_available() < 1){							// Manage the planner buffer queue.
//				io_send_message("no buffer available");
				return STATE_ENTER_AGAIN;
			}
		}
	}

	if (ss.parser_state == PARSE_RECORD){
		_debug_append_byte(IO_OK);
		_io_record_values();

		ss.parser_state = PARSE_IDLE;

	}

}

byte _io_parse_char(byte* value){
	byte c = Serial.read();
	_debug_append_byte(c);
	*value = c;
	ss.checksum += (byte)c;
	return c;
}

int _io_parse_int(int* value){
	byte c = Serial.read();
	_debug_append_byte(c);
	*value = c << 8;
	ss.checksum += (byte)c;
	c = Serial.read();
	_debug_append_byte(c);
	*value += c;
	ss.checksum += (byte)c;
	return *value;
}

// Send the coordinates received to the planner buffer.
void _io_record_values(){
	if (ss.data_received != 0){
//		debug_send_message("populates buffer");
		planner_set_buffer(ss.id, ss.posX, ss.posY, ss.posL, ss.speed, ss.mode, ss.data_received);	// Populates the buffer with the values received
	}
}

// This function write a pair of data to SerialUSB, formated in json
void io_send_pair(String name, double value){
	_io_append_string("{\"");
	_io_append_string(name);
	_io_append_string("\":");
	_io_append_value(value);
	_io_append_string("}");
	_io_append_nl();
}

// This function writes a simple message to SerialUSB, formated in json
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
	Serial.write(data);
}





// This function write a pair of data to SerialUSB, formated in json
void debug_send_pair(String name, double value){
	_debug_append_string("{\"");
	_debug_append_string(name);
	_debug_append_string("\":");
	_debug_append_value(value);
	_debug_append_string("}");
	_debug_append_nl();
}

// This function writes a simple message to SerialUSB, formated in json
void debug_send_message(String message){
	_debug_append_string("{\"message\":\"");
	_debug_append_string(message);
	_debug_append_string("\"}");
	_debug_append_nl();
}

void debug_send_value(double value){
	_debug_append_string("{");
	_debug_append_value(value);
	_debug_append_string("}");
	_debug_append_nl();
}

void _debug_append_string(String data){
	int data_length = data.length();
	for (int i=0; i<data_length; i++){
		_debug_append_byte(data.charAt(i));
	}
}
void _debug_append_value(double data){
	String data_to_send = String(data, 3);
	_debug_append_string(data_to_send);
}

void _debug_append_nl(){
	_debug_append_byte(NL_CHAR);
}

//TODO: verify if there is room in the TX buffer, and what to do if not?
void _debug_append_byte(char data){
	Serial1.write(data);
}

