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
char rx_buffer[RX_BUFFER_SIZE];
volatile char rx_head = 0;
char rx_tail = 0;

char tx_buffer[TX_BUFFER_SIZE];
char tx_head = 0;
volatile char tx_tail = 0;

bool to_read_flag = 0;
bool to_write_flag = 0;

void serial_init(){
	memset(&ss, 0, sizeof(ss));												// Init the serial state struct
	ss.xon_state = 1;														// Enables XON
	_serial_interrupt_init();

	serial_send_message("Liaison série initialisée.");
}

void serial_get_data(){
//	serial_xon_xoff();														// Verifies the buffer size
	ss.parser_state = PARSING_IDLE;

	if (planner_get_available() < 2){										// Manage the planner buffer queue.
//		serial_send_message("no buffer available");
		return;
	}
	if (to_read_flag){
//		_serial_append_value(rx_head);
//		_serial_append_value(rx_tail);
		_serial_parse();
	}

}

// This function is the entrance of a new string. It sets the parser_state for json or info
// If the new string starts by anything else than start charcaters, it's discarded.
void _serial_parse(){
	if (rx_buffer[rx_tail] == '{'){
		_serial_parse_json();
	} else if (rx_buffer[rx_tail] == '$'){
		_serial_parse_cfg();
	} else {
		_serial_flush_rx_buffer();
	}
}

/* this function parse the json string it receives
 * The json string must formed like that:
 * {"var1":value1,"var2":value2,...,"varN":valueN}, without spaces between values.
 * The loop turns while there is available data to read
 * The function is called b ythe serial parser above, when a "{" is detected.
 * It's driven by parser states. For each state we should get precise chars.
 * If these chars are found, parser goes to th next state, and test chars again.
 * A normal loop should be:
 * PARSING_IDLE						We have just ereceived a new data string. Waiting for opening brace
 *
 * PARSING_JSON_START 				looking for quote mark to start parsing var name
 *
 * PARSING_JSON_VAR 				looking for alpha chars a-z, A-Z, or quote mark to end parsing var name.
 *
 * PARSING_JSON_VAR_OK 				looking for semicolon.
 *
 * PARSING_JSON_VALUE_START			looking for minus sign or number 0-9.
 *
 * PARSING_JSON_VALUE 				looking for number 0-9, a comma to start a new var/value pair, or a closing brace to end json string.
 *
 * possible loop to PARSING_JSON_START for new var/value pair.
 *
 * PARSING_JSON_END					set up rx_buffer and state for next data string.
 *
 * On each step, the next data char is tested against what it should be. If not found, their is an error and the loop stops.
 * On each end of the loop, the buffer pointer rx_tail is incremented for the next iteration.
 * If a pair (var + value) is obtained, _serial_record_pair is called, whiwh stores the value in the serial singleton and sets a flag.
 * Last, if the state is PARSING_JSON_END, then it means the parsing was without problem.
 * Then it calls _serial_record_values() to populates the planner buffer.
 */
void _serial_parse_json(){
	bool is_neg = false;
	char in_byte = 0;
	while (rx_tail != rx_head){
		in_byte = rx_buffer[rx_tail];
//		_serial_append_value(rx_head);
//		_serial_append_value(rx_tail);

		if (ss.parser_state == PARSING_IDLE){
			if (in_byte == '{'){
				ss.parser_state = PARSING_JSON_START;
//				_serial_append_string("json start");

			} else {
				ss.parser_state = PARSING_JSON_ERROR_START;
				_serial_flush_rx_buffer();
//				_serial_append_string("json error start");
				break;
			}

		} else if (ss.parser_state == PARSING_JSON_START){
			if (in_byte == '"'){
				ss.parser_state = PARSING_JSON_VAR;
				ss.inVar = "";
//				_serial_append_string("json var");

			} else {
				ss.parser_state = PARSING_JSON_ERROR_VAR;
				_serial_flush_rx_buffer();
//				_serial_append_string("json error var");
				break;
			}

		} else if (ss.parser_state == PARSING_JSON_VAR){
			if ((in_byte >= 'a' && in_byte <= 'z') || (in_byte >= 'A' && in_byte <= 'Z')){
				ss.inVar += in_byte;
//				_serial_append_string(ss.inVar);

			} else if (in_byte = '"'){
				ss.parser_state = PARSING_JSON_VAR_OK;
//				_serial_append_string("json var ok");

			} else {
				ss.parser_state = PARSING_JSON_ERROR_VAR;
				_serial_flush_rx_buffer();
//				_serial_append_string("json error var");
				break;
			}
			
		} else if (ss.parser_state == PARSING_JSON_VAR_OK){
			if (in_byte == ':'){
				ss.parser_state = PARSING_JSON_VALUE_START;
//				_serial_append_string("json value start");
				ss.inValue = 0;
				is_neg = 0;

			} else {
				ss.parser_state = PARSING_JSON_ERROR_PAIR;
				_serial_flush_rx_buffer();
//				_serial_append_string("json error pair");
				break;
			}
			
		} else if (ss.parser_state == PARSING_JSON_VALUE_START){
			if (in_byte == '-'){
				is_neg = 1;
//				_serial_append_string("is neg");
			} else if (in_byte >= '0' && in_byte <= '9'){
				ss.inValue *= 10;
				ss.inValue += (in_byte - 48);
//				_serial_append_value(ss.inValue);
			} else {
				ss.parser_state = PARSING_JSON_ERROR_VALUE;
				_serial_flush_rx_buffer();
//				_serial_append_string("json error value start");
				break;
			}
			ss.parser_state = PARSING_JSON_VALUE;

		} else if (ss.parser_state == PARSING_JSON_VALUE){
			if (in_byte >= '0' && in_byte <= '9'){
				ss.inValue *= 10;
				ss.inValue += (in_byte - 48);
//				_serial_append_value(ss.inValue);

			} else if (in_byte == ','){
				ss.parser_state = PARSING_JSON_START;
				if (is_neg == 1){
					ss.inValue = -ss.inValue;
				}
				_serial_record_pair();
//				_serial_append_string("json start");

			} else if (in_byte == '}'){
				ss.parser_state = PARSING_JSON_END;
				if (is_neg == 1){
					ss.inValue = -ss.inValue;
				}
				_serial_record_pair();
//				_serial_append_string("json end");

			} else {
				ss.parser_state = PARSING_JSON_ERROR_VALUE;
				_serial_flush_rx_buffer();
//				_serial_append_string("json error value");
				break;
			}
			
		} else if (ss.parser_state == PARSING_JSON_END){
			_serial_record_values();
			rx_tail = rx_head;
			ss.parser_state = PARSING_IDLE;
			break;
		} else {
			_serial_flush_rx_buffer();
			break;
		}

		rx_incr(rx_tail);
	}
	ss.parser_state = PARSING_IDLE;
	to_read_flag =0;
}

void _serial_record_pair(){

	if (ss.inVar == "ID"){												// Sets the right value to the right var
		ss.parser_data_received |= 32;									// Sets a flag.
		ss.id = ss.inValue;												// Records value.
		serial_send_pair("ID", ss.id);

	} else if (ss.inVar == "X"){
		ss.parser_data_received |= 16;
		ss.posX = ss.inValue;
		serial_send_pair("X", ss.posX);

	} else if (ss.inVar == "Y"){
		ss.parser_data_received |= 8;
		ss.posY = ss.inValue;
		serial_send_pair("Y", ss.posY);

	} else if (ss.inVar == "L"){
		ss.parser_data_received |= 4;
		ss.posL = ss.inValue;
		serial_send_pair("L", ss.posL);

	} else if (ss.inVar == "speed"){
		ss.parser_data_received |= 2;
		ss.speed = ss.inValue;
		serial_send_pair("speed", ss.speed);

	} else if (ss.inVar == "mode"){
		ss.parser_data_received |= 1;
		ss.mode = ss.inValue;
		serial_send_pair("mode", ss.mode);
	}
}


// Send the json string received to the planner buffer.
void _serial_record_values(){
	_serial_send_go();
	if (ss.parser_data_received != 0){
//		serial_send_message("populates buffer");
		planner_set_buffer(ss.id, ss.posX, ss.posY, ss.posL, ss.speed, ss.mode, ss.parser_data_received);	// Populates the buffer with the values received
	}
	ss.parser_data_received = 0;
	ss.inVar="";
	ss.inValue=0;
}

// Parse a cfg command.
void _serial_parse_cfg(){

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
void _serial_send_go(){
	serial_send_pair("send", 1);
}

//This sends the again signal in case of lost data
void _serial_send_again(){
	serial_send_pair("send", 2);
}

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
	_serial_append_string("{\"message\":");
	_serial_append_string(message);
	_serial_append_string("}");
	_serial_append_nl();
}

// This function is for debugging purpose: it prints "step" on Serial. Used to replace code breakpoints.
void serial_step(){
	_serial_append_string("step");
	_serial_append_nl();
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
	char head = rx_head;
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
	char tail = tx_tail;											// Temp copy to limit volatile acces.
	UDR0 = tx_buffer[tail];											// Copy data buffer to TX data register.
	tx_incr(tail);													// Increment buffer tail pointer.
	tx_tail = tail;													// Copy back the temporary value to tx_tail.

	if (tail == tx_head){											// if head == tail, then nothing left to send, disconenct interrupt.
		UCSR0B &= ~(1 << UDRIE0);
	}
}

void _serial_append_string(String data){
	int data_length = data.length();
	for (int i=0; i<data_length; i++){
		_serial_append_byte(data.charAt(i));
	}
}
void _serial_append_value(long data){
	String data_to_send = String(data);
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

}