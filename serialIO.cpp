// Arduino laser projector

// serialIO.cpp
// this part of the program deals with:
// serial input: what is sent to the board, verifies that datas are correctly written, and populates the movement buffers
// serial output: send back infos about position, so the program can know how much of the pattern has already be done

/* the data comming from the computer are formated as Json strings, like {"ID":value, "X":value, "Y":value, "L":value, mode:value}
 * with X, Y, L as int, mode as byte
 */



#include <Arduino.h>
#include "planner.h"
#include "serialIO.h"
#include "driver.h"
#include "settings.h"

serialState ss;
long baudrate = BAUDRATE;

void serial_init(){
	memset(&ss, 0, sizeof(ss));												// Init the serial state struct
	ss.xon_state = 1;														// Enables XON
	Serial.begin(baudrate, SERIAL_CONFIG);
	Serial.setTimeout(0.01);
	serial_send_message("Liaison série initialisée.");
}

/* this function looks at the next char in the serial buffer, the ncalls the right function.
 * 
 */
void serial_get_data(){
//	serial_xon_xoff();														// Verifies the buffer size

	if (planner_get_available() < 2){										// Manage the planner buffer queue.
//		serial_send_message("no buffer available");
		return;
	}

	int inByte = Serial.read();												// Read the next byte in the serial buffer
	if (inByte == -1){
		return;
	}
//	serial_send_pair("char", inByte);

	if ((inByte == NL_CHAR) || (inByte == CR_CHAR)){						// If end of line, then parser wait for a string start
/*		if ((ss.parser_state != PARSING_JSON_END) &&
		(ss.parser_state != PARSING_CFG_END)){								// If the last string was not parsed till the end
//			serial_send_message("Erreur fin de ligne");
			serial_send_pair("erreur",ss.parser_state);
			serial_send_again();
		}
*/		ss.parser_state = PARSING_IDLE;
		return;
	}

	switch (ss.parser_state){												// Switch between the diferent possible states
		case PARSING_IDLE:													// Default state: all commands done, waiting for new
			serial_parse_input(inByte);
			break;
		case PARSING_JSON_START:											// Beggining a json string, waiting for a quote
			serial_parse_json_start(inByte);
			break;
		case PARSING_JSON_VAR:												// Having a quote, waiting for var name
			serial_parse_json_var(inByte);
			break;
		case PARSING_JSON_VALUE:											// having var name, waiting for a value
			serial_parse_json_value(inByte);
			break;
		case PARSING_JSON_PAIR:												// Having value, waiting for new pair or end of json
			serial_parse_json_pair(inByte);
			break;
		case PARSING_JSON_END:												// End of the json string, send values to planner
			serial_record_values();
			break;
		case PARSING_CFG_START:
			break;
		case PARSING_CFG_VAR:
			break;
		default:
//			serial_send_again();
			serial_send_pair("erreur",ss.parser_state);
			ss.parser_state = PARSING_IDLE;
			serial_send_again();
			break;
	}

}

// This function is the entrance of a new string. It sets the parser_state for json or info
void serial_parse_input(int inByte){
	switch (inByte){
		case char('{'):
			ss.parser_state++;
			break;
		case char('$'):
			ss.parser_state = PARSING_CFG_START;
			serial_parse_cfg();
			break;
		default:
//			serial_send_message("Erreur parse input");
			ss.parser_state = PARSING_ERROR_INPUT;
			break;
	}

}

// This function waits for the quote that means the start of a name value
void serial_parse_json_start(int inByte){
//	serial_send_message("parse json");
	ss.inVar="";
	ss.inValue=0;
	if (inByte == '"'){
		ss.parser_state++;
	} else {
		ss.parser_state = PARSING_JSON_ERROR_START;
//		serial_send_message("Erreur parse json start");
	}
}

// This function records the var name until it finds a second quote
void serial_parse_json_var(int inByte){
//	serial_send_message("parse var");
	if (inByte == '"'){
		ss.parser_state++;
	} else {
		ss.inVar += char(inByte);
	}
}

// This function parse the value of the pair
// Once done, it records this value temporarly in the serialstate singleton. If there is no problem, it will later be sent to the planner
void serial_parse_json_value(int inByte){
	delay(1);
//	serial_send_message(ss.inVar);
//	serial_send_message("parse value");
	if (inByte ==':'){
		if ((Serial.peek() < '0' || Serial.peek() > '9')){					// verifies that the following byte is a number
			if (Serial.peek() !='-'){										// It also can be a minux sign for negative number
				ss.parser_state = PARSING_JSON_ERROR_VALUE;						// sets statuts, then continue to next value
//				serial_send_message("Erreur parse json start");
				return;
			}
		}

		ss.parser_state++;
		ss.inValue = Serial.parseInt();

		if (ss.inVar == "ID"){												// Sets the right value to the right var
			ss.parser_data_received |= 32;
			ss.id = ss.inValue;
//			serial_send_pair("ID", ss.id);

		} else if (ss.inVar == "X"){
			ss.parser_data_received |= 16;
			ss.posX = ss.inValue;
//			serial_send_pair("X", ss.posX);

		} else if (ss.inVar == "Y"){
			ss.parser_data_received |= 8;
			ss.posY = ss.inValue;
//			serial_send_pair("Y", ss.posY);

		} else if (ss.inVar == "L"){
			ss.parser_data_received |= 4;
			ss.posL = ss.inValue;
//			serial_send_pair("L", ss.posL);

		} else if (ss.inVar == "speed"){
			ss.parser_data_received |= 2;
			ss.speed = ss.inValue;
//			serial_send_pair("speed", ss.speed);

		} else if (ss.inVar == "mode"){
			ss.parser_data_received |= 1;
			ss.mode = ss.inValue;
//			serial_send_pair("mode", ss.mode);
		}

	} else {
		ss.parser_state = PARSING_IDLE;
	}
}

// This function, once a pair has been received, waits for a new pair (comma) or for the end of json string (clising brace)
void serial_parse_json_pair(int inByte){
//	serial_send_message("parse pair ok");
	switch (inByte){
		case char(','):
//			serial_send_message("new pair");
			ss.parser_state = PARSING_JSON_START;
			break;
		case char('}'):
//			serial_send_message("end of string");
			ss.parser_state = PARSING_JSON_END;
			serial_record_values();
			break;
		default:
			ss.parser_state = PARSING_JSON_ERROR_PAIR;
//			serial_send_message("Erreur json pair");
			break;
	}

}

// This function sends the json string received to the planner buffer
void serial_record_values(){
	serial_send_go();
	if (ss.parser_data_received != 0){
//		serial_send_message("populates buffer");
		planner_set_buffer(ss.id, ss.posX, ss.posY, ss.posL, ss.speed, ss.mode, ss.parser_data_received);	// Populates the buffer with the values received
	}
	ss.parser_data_received = 0;
	ss.inVar="";
	ss.inValue=0;
}

void serial_parse_cfg(){
	delay(1);
	String cfg_item = Serial.readStringUntil(':');
	long cfg_value = Serial.parseInt();
	serial_send_pair(cfg_item, cfg_value);
	if(cfg_item == "baudrate"){
		baudrate = cfg_value;
		serial_send_message("baudrate updated");
		Serial.end();
		serial_init();
	} else if (cfg_item = "size"){
//		driver_set_size(cfg_value);
		serial_send_message("size updated");
	}
	ss.parser_state = PARSING_CFG_END;
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

// This function looks at how the serial buffer is full. It sends the xon/xoff chars according to that.
// It's called by the serial_get_data function, that is itself called by the main loop.
 /*
byte serial_xon_xoff(){
	ss.available = Serial.available();											// Look at how much bytes are in the buffer
	if (ss.available > 40 && ss.xon_state == XON_SET){							// If more then a given amount, send XOFF (if not already sent)
		ss.xon_state = XOFF_SET;
		Serial.write(XOFF_CHAR);
//		serial_send_message("XOFF");
	} else if (ss.available < 20 && ss.xon_state == XOFF_SET){					//If bellow a certain amount, send XON again (if not already)
		ss.xon_state = XON_SET;
		Serial.write(XON_CHAR);
//		serial_send_message("XON");
	}

	return ss.available;
}
*/

//This sends the go signal when Serial is able to receive
void serial_send_go(){
	serial_send_pair("send", 1);
}

//This sends the again signal in case of lost data
void serial_send_again(){
	serial_send_pair("send", 2);
}


// This function is called on each main loop, to send back datas to the python program
void serial_write_data(){
	driverState * ds = driver_get_ds();											// Get the driver singleton.
	if (ds->percent_flag == 0){
		return;
	}
	moveBuffer *bf = planner_get_run_buffer();									// Get the current move buffer.

	Serial.print("{\"ID\":");
	Serial.print(bf->id);
	Serial.print("}");
	Serial.println();

	ds->percent_flag = 0;
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

// This function is for debugging purpose: it prints "step" on Serial. Used to replace code breakpoints.
void serial_step(){
	Serial.println("step");
}

// This function send the current position
void serial_send_position(){
	driverState * ds = driver_get_ds();
	serial_send_pair("pos_x", ds->now[0]);
	serial_send_pair("pos_y", ds->now[1]);
	serial_send_pair("pos_l", ds->now[2]);
}