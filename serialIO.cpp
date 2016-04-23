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

#define rx_incr(i)	i = (i + 1)%RX_BUFFER_SIZE
#define tx_incr(i)	i = (i + 1)%TX_BUFFER_SIZE

serialState ss;
long baudrate = BAUDRATE;

// defining buffers for serial RX and TX.
volatile char rx_buffer[RX_BUFFER_SIZE];
volatile byte rx_head = 0;
volatile byte rx_tail = 0;

volatile char tx_buffer[TX_BUFFER_SIZE];
volatile byte tx_head = 0;
volatile byte tx_tail = 0;

volatile bool to_read_flag = 0;
volatile bool to_write_flag = 0;
byte rx_data_length = 0;


void serial_init(){
	memset(&ss, 0, sizeof(ss));												// Init the serial state struct
	ss.xon_state = 1;														// Enables XON
	_serial_interrupt_init();
	_serial_append_string("Serial initialisé.");

//	serial_send_message("liaison série initialisée");
}

void serial_get_data(){
//	serial_xon_xoff();														// Verifies the buffer size
	if (planner_get_available() < 2){										// Manage the planner buffer queue.
//		serial_send_message("no buffer available");
		return;
	}
	if (to_read_flag){
		serial_parse();
	}

}

// This function is the entrance of a new string. It sets the parser_state for json or info
void serial_parse(){
	rx_data_length = rx_head - rx_tail;
	if (rx_data_length < 0) rx_data_length += 64;

	if (rx_buffer[rx_tail] == '{'){
		serial_parse_json();
	}else if (rx_buffer[rx_tail] == '$'){
		serial_parse_cfg();
	}
}

/* this function parse the json string it receives
 * The json string must formed like that:
 * {"var1":value1,"var2":value2,...,"varN":valueN}, without spaces between values.
 * The loop turns while there is available data to read
 * - First it looks for incomming char.
 * - If it's an openning brace, it's the start of a json string, so it loops again
 * - If it's a comma, there should be a new var/value pair, so it loops again
 * - If it's a closing brace, it ends the loop to compute values
 * - If it's a quote mark, that is the start of a var/value pair, as long as there is data available
 * - If it finds one, it then reads the name of the var until the next quote mark.
 * - After the value, it looks for a colon that should separate th name of the var from its value.
 *   If there is no, it returns the function with an error
 * - The following value should be a number, so it test if it's one. If it's not, it tests for a minus sign. Else it loops.
 * - If it's a number (positive or negative) it records it as the current value
 * - Then it tests the name of the var received against what it should be.
 * - If ok, it records the value and sets a flag so it's known that the value has been set.
 * - If one or several flags have been set, it sends the data to the planner, that populate a new buffer
 */
void serial_parse_json(){
	bool is_neg = false;
	byte in_byte = 0;
	while (rx_tail != rx_head){
		in_byte = rx_buffer[rx_tail];

		if (in_byte == '{'){
			ss.parser_state = PARSING_JSON_START;
			_serial_append_string("PARSING_JSON_START");
			rx_incr(rx_tail);
			continue;
		}

		if (in_byte == '"'){
			if (ss.parser_state == PARSING_JSON_START){
				ss.parser_state = PARSING_JSON_VAR;
				rx_incr(rx_tail);
				_serial_append_string("PARSING_JSON_VAR");
				continue;
			} else if (ss.parser_state == PARSING_JSON_VAR){
				ss.parser_state = PARSING_JSON_VAR_OK;
				_serial_append_string("PARSING_JSON_VAR_OK");
			} else {
				ss.parser_state == PARSING_JSON_ERROR_VAR;
				_serial_append_string("PARSING_JSON_VAR_ERROR");
			}
		}

		if (in_byte == ':'){
			if (ss.parser_state == PARSING_JSON_VAR_OK){
				ss.parser_state == PARSING_JSON_VALUE;
				_serial_append_string("PARSING_JSON_VALUE");
				is_neg = false;
				rx_incr(rx_tail);
				continue;
			} else {
				ss.parser_state = PARSING_JSON_ERROR_PAIR;
				_serial_append_string("PARSING_JSON_PAIR");
			}
		}

		if (in_byte == ','){
			ss.parser_state == PARSING_JSON_START;
			_serial_append_string("PRASING_JSON_START");
		}

		if (in_byte == '}'){
			ss.parser_state == PARSING_JSON_END;
			_serial_append_string("PARSING_JSON_END");
			_serial_clear_rx_buffer();
			break;
		}

		if (ss.parser_state == PARSING_JSON_VAR){
			ss.inVar += in_byte;
		}

		if (ss.parser_state == PARSING_JSON_VALUE){
			if (in_byte < '0' || in_byte >'9'){
				if (in_byte == '-'){
					is_neg = true;
				}
				ss.parser_state = PARSING_JSON_VALUE_OK;
			} else {
				ss.inValue *=10;
				ss.inValue += in_byte;
			}
		}

		if (ss.parser_state == PARSING_JSON_VALUE_OK){

			if (ss.inVar == "ID"){												// Sets the right value to the right var
				ss.parser_data_received |= 32;
				ss.id = ss.inValue;
				_serial_append_string("ID");
//				serial_send_pair("ID", ss.id);

			} else if (ss.inVar == "X"){
				ss.parser_data_received |= 16;
				ss.posX = ss.inValue;
				_serial_append_string("X");
//				serial_send_pair("X", ss.posX);

			} else if (ss.inVar == "Y"){
				ss.parser_data_received |= 8;
				ss.posY = ss.inValue;
				_serial_append_string("Y");
//				serial_send_pair("Y", ss.posY);

			} else if (ss.inVar == "L"){
				ss.parser_data_received |= 4;
				ss.posL = ss.inValue;
				_serial_append_string("L");
//				serial_send_pair("L", ss.posL);

			} else if (ss.inVar == "speed"){
				ss.parser_data_received |= 2;
				ss.speed = ss.inValue;
				_serial_append_string("speed");
//				serial_send_pair("speed", ss.speed);

			} else if (ss.inVar == "mode"){
				ss.parser_data_received |= 1;
				ss.mode = ss.inValue;
				_serial_append_string("mode");
//				serial_send_pair("mode", ss.mode);
			}
		}
		rx_incr(rx_tail);
	}
}


// Send the json string received to the planner buffer.
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

// Parse a cfg command.
void serial_parse_cfg(){

}



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

	ds->percent_flag = 0;
}

// This function write a pair of data to Serial, formated in json
void serial_send_pair(String name, double value){

}

// This function writes a simple message to Serial, formated in json
void serial_send_message(String message){
}

// This function is for debugging purpose: it prints "step" on Serial. Used to replace code breakpoints.
void serial_step(){
}

// This function send the current position
void serial_send_position(){
	driverState * ds = driver_get_ds();
}


// Serila interrupt init function. Set the registers according to settings values and needs.
void _serial_interrupt_init(){
	unsigned int ubr = CLOCK_SPEED/16/(BAUDRATE - 1);
//	Serial.print(tra, DEC);
	cli();

	UBRR0H = (ubr>>8);												// Set the baudrate register, high first, then low.
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
	byte head = rx_head;
	rx_buffer[head] = UDR0;

	if (rx_buffer[head] == NL_CHAR){
		to_read_flag = 1;
	}

	rx_incr(head);
	rx_head = head;

}

// ISR Empty buffer interrupt.
// Sends data while their is to.
ISR(USART_UDRE_vect){
	byte tail = tx_tail;										// Temp copy to limit volatile acces.
	UDR0 = tx_buffer[tail];											// Copy data buffer to TX data register.
	tx_incr(tail);													// Increment buffer tail pointer.
	tx_tail = tail;												// Copy back the temporary value to tx_tail.

	if (tail == tx_head){										// if head == tail, then nothing left to send, disconenct interrupt.
		UCSR0B &= ~(1 << UDRIE0);
	}
}

void _serial_append_string(String data){
	int data_length = data.length();

	for (int i=0; i<data_length; i++){
		tx_buffer[tx_head] = data.charAt(i);
		tx_incr(tx_head);
	}
	_serial_append_byte(NL_CHAR);
}
void _serial_append_value(long data){
	String data_to_send = String(data);
	_serial_append_string(data_to_send);
}

void _serial_append_byte(char data){

	tx_buffer[tx_head] = data;
	tx_incr(tx_head);

	UCSR0B |= (1 << UDRIE0);
}

void _serial_clear_rx_buffer(){
	rx_tail = rx_head;
}