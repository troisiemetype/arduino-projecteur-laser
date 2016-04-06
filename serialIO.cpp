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
#include "settings.h"

byte _parserState = PARSER_IDLE;

void serial_init(){
	Serial.begin(115200, SERIAL_CONFIG);
	Serial.setTimeout(100);
}

// this function just look at the beggining of a json string
void serial_getData(){
	if (Serial.read() == '{'){												// Waits for the start of a JSON string
		_parserState = PARSER_START_JSON;									// sets the status of the parser
		delay(10);															// waits a bit for the following data to come
		_serial_parser();													// calls the parser
	} else {
		_parserState = PARSER_IDLE;
	}

}

// this function parse the json string it receives
void _serial_parser(){
	String inVar;															// declaring vars. inVar records the name of the var passed trough JSON
	byte inByte = 0;														// The byte read
	int inValue = 0;														// the value read
	while (Serial.available()){												// If data available

		if (Serial.read() == '"'){											// Look for a quote mark (start of a var/value pair)...
			_parserState = PARSING_VAR;										// Sets the parser status
			inVar = Serial.readStringUntil('"');							// records the name of the var sent
		} else {															// ...else go on
			continue;
		}

		if (!Serial.read() == ':'){											// Look for a colon after the var, else stop the loop
			_parserState = PARSING_PAIR_ERROR;
			continue;
		}
		_parserState = PARSING_VALUE;										// Once the var has been recorded, one must find its value. Sets status
		inValue = Serial.parseInt();										// Recording the value
		inByte = Serial.read();												// looking at the following byte

		if (inVar == "X"){													// Sets the right value to the right var
			Serial.print("X = ");
			Serial.println(inValue);
		} else if (inVar == "Y"){
			Serial.print("Y = ");
			Serial.println(inValue);
		} else if (inVar == "L"){
			Serial.print("L = ");
			Serial.println(inValue);
		} else if (inVar == "mode"){
			Serial.print("mode = ");
			Serial.println(inValue);
		} else {
			continue;
		}

		if (inByte == ','){													// If it's a comma, loop again to a new var/value pair
			continue;
		} else if( inByte == '{'){											// Else if it's a closing brace, end of the JSON string
			_parserState = PARSER_END_JSON;									// Update status
			return;															// And quit function
		}
	}
}