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

char line_buffer[LINE_BUFFER_SIZE];
char line_counter = 0;

void serial_init(){
	Serial.begin(115200);
	ss.serial_state = 0;
//	serial_send_message(F("Liaison série initialisée."));
}

void serial_main(){
	switch (ss.serial_state){
		case SERIAL_IDLE:
			serial_get_data();
			break;
		case SERIAL_EMPTY_RX:
			serial_get_data();
			break;
		case SERIAL_PARSE:
			_serial_parse_data();
			break;
		default:
			break;
	}

}

//This function is called from the main function.
//If there are data in the rx buffer, it copies it into a line buffer.
//That way the RX buffer is kept as empty as possible.
bool serial_get_data(){
	//while there is data in the RX buffer.
	while (Serial.available()){
		ss.serial_state = SERIAL_EMPTY_RX;

		//Get the current char in rx buffer
		char c = Serial.read();

//		serial_send_pair("parsing", c );
		
		//If the char is end of line, set a "flag" (that is, char = 0),
		//initialize line pointer for next time, and call data parser
		if (c == '\n' || c == '\r'){
			line_buffer[line_counter] = 0;
			line_counter = 0;
//			Serial.println("new line");
			ss.serial_state = SERIAL_PARSE;
			return true;
		//Else chars are stored.
		} else if (c >= 'A' && c <= 'Z'){
			line_buffer[line_counter] = c;
//			Serial.print("char: ");
//			Serial.write(c);

		} else if (c >= 'a' && c <= 'z'){
			c -= 32;
			line_buffer[line_counter] = c;
//			Serial.print("char: ");
//			_serial_append_byte(c);
//			_serial_append_nl();

		} else if (c >= '0' && c <= '9'){
			line_buffer[line_counter] = c;
//			Serial.print("char: ");
//			Serial.write(c);
//			_serial_append_nl();

		} else if (c =='-'){
			line_buffer[line_counter] = c;
//			Serial.print("char: ");
//			_serial_append_byte(c);
//			_serial_append_nl();

		} else {
//			Serial.print("discard char: ");
//			_serial_append_byte(c);
//			_serial_append_nl();
			//Every other chars are ignored.
		}

		//increment line and rx buffer pointer
		line_counter++;
	}

	return false;

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
void _serial_parse_data(){

	if (planner_get_available() < 1){										// Manage the planner buffer queue.
//		serial_send_message("no buffer available");
		return;
	}

	bool is_neg;
	char c = 1;
	char line_counter = 0;
	ss.serial_state = SERIAL_PARSE_VAR;

	while (c != 0){

		c = line_buffer[line_counter];

		if (ss.serial_state == SERIAL_PARSE_VAR){
			if (c >= 'A' && c <= 'Z'){
				ss.inVar = c;
				ss.inValue = 0;
				is_neg = 0;
				ss.serial_state = SERIAL_PARSE_VALUE;
//				_serial_append_byte(ss.inVar);
//				_serial_append_nl();
			}

			line_counter++;

		} else if (ss.serial_state == SERIAL_PARSE_VALUE){
			if (c >= '0' && c <= '9'){
				ss.inValue *= 10;
				ss.inValue += (c - 48);
				line_counter++;
			} else if (c =='-'){
				is_neg = 1;
				line_counter++;
//				Serial.print("is neg");
			} else {
				if (is_neg == 1){
					ss.inValue = -ss.inValue;
				}
				_serial_record_pair();
				ss.serial_state = SERIAL_PARSE_VAR;
//				Serial.print(ss.inValue);
			}

		}
	}

//	_serial_append_nl();
	_serial_record_values();
	ss.serial_state = SERIAL_IDLE;
	_serial_send_go();
//	to_read_flag =0;
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
	Serial.println("$s");
}

/*
//This sends the again signal in case of lost data
void _serial_send_again(){
	serial_send_pair("send", 2);
}
*/

// This function write a pair of data to Serial, formated in json
void serial_send_pair(String name, double value){
	Serial.print("{\"");
	Serial.print(name);
	Serial.print("\":");
	Serial.print(value);
	Serial.print("}");


}

// This function writes a simple message to Serial, formated in json
void serial_send_message(String message){
	Serial.print("{\"message\":\"");
	Serial.print(message);
	Serial.print("\"}");
}
