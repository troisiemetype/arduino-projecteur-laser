// Arduino laser projector

// serialIO.h
// this part of the program deals with:
// serial input: what is sent to the board, verifies that datas are correctly written, and populates the movement buffers
// erial output: send back infos about position, so the program can know how much of the pattern has already be done

#ifndef SERIALIO_H
#define SERIALIO_H

#define RX_BUFFER_SIZE				64
#define TX_BUFFER_SIZE				64

#define XON_SET 			1
#define XOFF_SET			0

//define parser states / error
#define PARSING_IDLE				0
#define PARSING_JSON_START			1
#define PARSING_JSON_VAR			2
#define PARSING_JSON_VAR_OK			3
#define PARSING_JSON_VALUE			4
#define PARSING_JSON_VALUE_OK		5
#define PARSING_JSON_PAIR			6
#define PARSING_JSON_END			7
#define PARSING_ERROR_INPUT			8
#define PARSING_JSON_ERROR_START	9
#define PARSING_JSON_ERROR_VAR		10
#define PARSING_JSON_ERROR_VALUE	11
#define PARSING_JSON_ERROR_PAIR		12
#define PARSING_JSON_ERROR			13

#define PARSING_CFG_START			14
#define PARSING_CFG_VAR				15
#define PARSING_CFG_VALUE			16
#define PARSING_CFG_END				17

struct serialState{
	byte parser_state;
	byte parser_data_received;
	String inVar;
	int inValue;

	byte available;

	long id;
	int posX;
	int posY;
	int posL;
	int speed;
	byte mode;

	boolean xon_state;
};

void serial_init();
void serial_get_data();
void serial_parse();
void serial_parse_json();
void serial_record_values();
void serial_parse_cfg();
byte serial_xon_xoff();
void serial_send_go();
void serial_send_again();
void serial_write_data();
void serial_send_pair(String text, double value);
void serial_send_message(String message);
void serial_step();
void serial_send_position();
void _serial_interrupt_init();
byte _serial_rx_incr(byte index);
byte _serial_tx_incr(byte index);
void _serial_append_string(String data);
void _serial_append_byte(char data);
void _serial_clear_rx_buffer();

#endif