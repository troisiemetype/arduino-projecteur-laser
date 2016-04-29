// Arduino laser projector

// serialIO.h
// this part of the program deals with:
// serial input: what is sent to the board, verifies that datas are correctly written, and populates the movement buffers
// erial output: send back infos about position, so the program can know how much of the pattern has already be done

#ifndef SERIALIO_H
#define SERIALIO_H

#define RX_BUFFER_SIZE				128
#define TX_BUFFER_SIZE				128

/* For Xon flow. Doesn't work.
#define RX_FLOW_UP					64
#define RX_FLOW_DOWN				16

#define SET_XON						1
#define XON_SET						2
#define SET_XOFF					3
#define XOFF_SET					4
*/
#define XON_CHAR					17				// XON char. 0x11; ctrl+Q
#define XOFF_CHAR					19				// XOFF char. 0x13; ctrl+S
#define RESET_CHAR					24				// Rest char. 0x18; ctrl+X
#define NL_CHAR						10				// New line char. 0x0A. \n
#define CR_CHAR						13				// Carriage return char. 0x0D. \r


//define parser states / error
#define PARSING_IDLE				0
#define PARSING_JSON_START			1
#define PARSING_JSON_VAR			2
#define PARSING_JSON_VAR_OK			3
#define PARSING_JSON_SEMI			4
#define PARSING_JSON_VALUE_START	5
#define PARSING_JSON_VALUE			6
#define PARSING_JSON_VALUE_OK		7
#define PARSING_JSON_PAIR			8
#define PARSING_JSON_END			9
#define PARSING_ERROR_INPUT			10
#define PARSING_JSON_ERROR_START	11
#define PARSING_JSON_ERROR_VAR		12
#define PARSING_JSON_ERROR_VALUE	13
#define PARSING_JSON_ERROR_PAIR		14
#define PARSING_JSON_ERROR			15

#define PARSING_CFG_START			20
#define PARSING_CFG_VAR				21
#define PARSING_CFG_VALUE			22
#define PARSING_CFG_END				23

// serial singleton. contains value defining the parser state,
// stores values that have been parsed and not recorded yet.
struct serialState{
	char parser_state;								// Stocks the parser state. Used for knowing what to parse. See #defines above.
	char parser_data_received;						// Data parsed mask. Set bit per bit, a bit vor a value.
	String inVar;									// Stocks temporary var names before values are parsed and record.
	long inValue;									// Stocks value, before they are recorded.

//	volatile char queue;							// Stocks the rx queue size when _serial_rx_queue is called.

	long id;										// Stocks the vales received, if parsing has been successfull.
	long posX;
	long posY;
	long posL;
	long speed;
	char mode;

//	volatile char flow_state;						// Stocks the state of flow control. See #defines above for states.
};

void serial_init();
void serial_get_data();
void _serial_parse();
void _serial_parse_json();
void _serial_record_pair();
void _serial_record_values();
void _serial_parse_cfg();
void _serial_send_go();
//void _serial_send_again();
void serial_send_pair(String text, double value);
void serial_send_message(String message);
void serial_percent(long id);
void serial_step();
void _serial_interrupt_init();
//char _serial_rx_queue();
void _serial_append_string(String data);
void _serial_append_value(long value);
void _serial_append_nl();
void _serial_append_byte(char data);
void _serial_clear_rx_buffer();
void _serial_flush_rx_buffer();

#endif