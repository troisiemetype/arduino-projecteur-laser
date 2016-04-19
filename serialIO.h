// Arduino laser projector

// serialIO.h
// this part of the program deals with:
// serial input: what is sent to the board, verifies that datas are correctly written, and populates the movement buffers
// erial output: send back infos about position, so the program can know how much of the pattern has already be done

#ifndef SERIALIO_H
#define SERIALIO_H

#define XON_SET 			1
#define XOFF_SET			0

//define parser states / error
#define PARSING_IDLE				0
#define PARSING_JSON_START			1
#define PARSING_JSON_VAR			2
#define PARSING_JSON_VALUE			3
#define PARSING_JSON_PAIR			4
#define PARSING_JSON_END			5
#define PARSING_JSON_ERROR			6
#define PARSING_INFO_START			7
#define PARSING_INFO_VAR			8

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
void serial_parse_input(int inByte);
void serial_parse_json_start(int inByte);
void serial_parse_json_var(int inByte);
void serial_parse_json_value(int inByte);
void serial_parse_json_pair(int inByte);
void serial_record_values();
byte serial_xon_xoff();
void serial_write_data();
void serial_send_pair(String text, double value);
void serial_send_message(String message);
void serial_step();
void serial_send_position();

#endif