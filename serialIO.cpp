// Arduino laser projector

// serialIO.cpp
// this part of the program deals with:
// serial input: what is sent to the board, verifies that datas are correctly written, and populates the movement buffers
// serial output: send back infos about position, so the program can know how much of the pattern has already be done

/* the data comming from the computer are formated as Json strings, like {"X":value, "Y":value, "L":value, mode:value}
 * with X, Y, L as int, mode as byte
 */



#include <Arduino.h>
#include "planner.h"
#include "serialIO.h"
#include "driver.h"
#include "settings.h"

byte _parser_state = 0;														// Stores the last operation the parser has done
byte _parser_data_received = 0;												// Stores the values that have been received
																			// Each set bit is a received value, e.g. 1010 is X & L set, Y & mode not

void serial_init(){
	Serial.begin(115200, SERIAL_CONFIG);
	Serial.setTimeout(1);
	serial_send_message("liaison série initialisée");
}

/* this function just look at the beggining of a json string
 * 
 */
void serial_get_data(){
	byte inByte = Serial.read();
	if (planner_get_available() < 4){
		serial_send_message("no buffer available");
		return;
	}

	if (inByte == '{'){
		delay(10);															// waits a bit for the following data to come
		_serial_parser();													// calls the parser
	} else if (inByte == '$'){
		// empty for now
		Serial.println(planner_get_available());
	} else if (inByte == '!'){
		// empty for now
	} else if (inByte == '~'){
		// Empty for now
 	} else if (inByte == '%'){
 		// empty for now
 	}

}

/* this function parse the json string it receives
 * The json string must formed like that:
 * {"var1":value1,"var2":value2,...,"varN":valueN}
 * The loop turns while there is available data to read
 * - First it looks for incomming char.
 * - If it's an openning brace, it's the start of a json string, so it loops again
 * - If it's a comma, there should be a new var/value pair, so it loops again
 * - If it's a closing brace, it ends the loop to compute values
 * - If it's a quote mark, that is the start of a var/value pair, as long as there is data available
 * - If it finds one, it then reads the name of the var until the next quote mark.
 * - After the value, it looks for a colon that should separate th name of the var from its value.
 *   If there is no, it returns the function with an error
 * - The following value should be a number, so it test if it's one. If it's not, it tests for a minus sign. Else it returns with an error
 * - If it's a number (positive or negative) it records it as the current value
 * - Then it tests the name of the var received against what it should be.
 * - If ok, it records the value and sets a flag so it's known that the value has been set.
 * - If one or several flags have been set, it sends the data to the planner, that populate a new buffer
 */
byte _serial_parser(){
	String inVar;															// declaring vars. inVar records the name of the var passed trough JSON
	byte inByte = 0;														// The byte read
	int inValue = 0;														// The value read

	_parser_data_received = 0;												// Keeps track of which data have been received

	int id = 0;																	// Stores the values received
	int posX = 0;
	int posY = 0;
	int posL = 0;
	int speed = 0;
	byte mode = 0;

	while (Serial.available()){												// If data available
		inByte = Serial.read();


		if (inByte == '{'){													// Look for an openning brace for the beginning of the json string
			_parser_state = PARSING_START_JSON;
			continue;
		} else if (inByte == '"'){											// Look for a quote mark (start of a var/value pair)...
			_parser_state = PARSING_VAR;									// Sets the parser status
			inVar = Serial.readStringUntil('"');							// records the name of the var sent
		} else if (inByte == ','){													// If it's a comma, loop again to a new var/value pair
			_parser_state = PARSING_START_JSON;
			continue;
		} else if( inByte == '}'){											// Else if it's a closing brace, end of the JSON string
			_parser_state = PARSING_END_JSON;								// Update status
			break;															// And quit while loop to store the received data
		} else {
			continue;
		}

		if (!Serial.read() == ':'){											// Look for a colon after the var, else stop the loop
			_parser_state = PARSING_PAIR_ERROR;
			return _parser_state;
		}

		if ((Serial.peek() < '0' || Serial.peek() > '9')){					// verifies that the following byte is a number
			if (Serial.peek() !='-'){										// It also can be a minux sign for negative number
				_parser_state = PARSING_PAIR_ERROR;							// sets statuts, then continue to next value
				return _parser_state;
			}
		}

		_parser_state = PARSING_VALUE;										// Once the var has been recorded, one must find its value. Sets status
		inValue = Serial.parseInt();										// Recording the value

		if (inVar == "ID"){													// Sets the right value to the right var
			_parser_data_received |= 32;
			id = inValue;
			serial_send_pair("ID", id);
		} else if (inVar == "X"){
			_parser_data_received |= 16;
			posX = inValue;
			serial_send_pair("X", posX);
		} else if (inVar == "Y"){
			_parser_data_received |= 8;
			posY = inValue;
			serial_send_pair("Y", posY);
		} else if (inVar == "L"){
			_parser_data_received |= 4;
			posL = inValue;
			serial_send_pair("L", posL);
		} else if (inVar == "speed"){
			_parser_data_received |= 2;
			speed = inValue;
			serial_send_pair("speed", speed);
		} else if (inVar == "mode"){
			_parser_data_received |= 1;
			mode = inValue;
			serial_send_pair("mode", mode);
		} else {
			continue;
		}
	}

	if (_parser_data_received != 0){
		serial_send_message("populates buffer");
		planner_set_buffer(id, posX, posY, posL, speed, mode, _parser_data_received);	// Populates the buffer with the values received
	}
	return _parser_state = PARSING_OK;
}

// This function write a pair of data to Serial, formated in json
void serial_send_pair(String name, double value){
	Serial.print("{\"");
	Serial.print(name);
	Serial.print("\":");
	Serial.print(value);
	Serial.print("}");
	Serial.println();
}

// This function writes a simple message to Serial, formated in json
void serial_send_message(String message){
	Serial.print("{\"message\":\"");
	Serial.print(message);
	Serial.print("\"}");
	Serial.println();
}

// This function is for debugging purpose: it prints "step" on Serial. Used to replace breakpoints.
void serial_step(){
	Serial.println("step");
}

// This function send the current position
void serial_send_position(){
	moveBuffer * bf = planner_get_run_buffer();
//	serial_send_pair("pos X = ", bf->nowX);
//	serial_send_pair("pos Y = ", bf->nowY);
//	serial_send_pair("pos L = ", bf->nowL);

}