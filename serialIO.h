// Arduino laser projector

// serialIO.h
// this part of the program deals with:
// serial input: what is sent to the board, verifies that datas are correctly written, and populates the movement buffers
// erial output: send back infos about position, so the program can know how much of the pattern has already be done

#ifndef SERIALIO_H
#define SERIALIO_H

#define RX_BUFFER_SIZE				128
#define TX_BUFFER_SIZE				64

#define XON_SET 			1
#define XOFF_SET			0

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

struct serialState{
	char parser_state;
	char parser_data_received;
	String inVar;
	long inValue;

	char available;

	long id;
	long posX;
	long posY;
	long posL;
	long speed;
	char mode;

	boolean xon_state;
};

void serial_init();
void serial_get_data();
void _serial_parse();
void _serial_parse_json();
void _serial_record_pair();
void _serial_record_values();
void _serial_parse_cfg();
byte _serial_xon_xoff();
void _serial_send_go();
void _serial_send_again();
void serial_write_data();
void serial_send_pair(String text, double value);
void serial_send_message(String message);
void serial_step();
void serial_send_position();
void _serial_interrupt_init();
byte _serial_rx_incr(byte index);
byte _serial_tx_incr(byte index);
void _serial_append_string(String data);
void _serial_append_value(long value);
void _serial_append_byte(char data);
void _serial_clear_rx_buffer();
void _serial_flush_rx_buffer();

#endif