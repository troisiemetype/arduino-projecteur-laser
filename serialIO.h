// Arduino laser projector

// serialIO.h
// this part of the program deals with:
// serial input: what is sent to the board, verifies that datas are correctly written, and populates the movement buffers
// erial output: send back infos about position, so the program can know how much of the pattern has already be done

#ifndef SERIALIO_H
#define SERIALIO_H

//define parser states / error
#define PARSING_OK			0
#define PARSING_START_JSON	1
#define PARSING_END_JSON	2
#define PARSING_VAR			3
#define PARSING_VALUE		4
#define PARSING_PAIR_OK		5
#define PARSING_VAR_ERROR   6
#define PARSING_VALUE_ERROR 7
#define PARSING_PAIR_ERROR	8
#define PARSING_ERROR		9


void serial_init();
void serial_get_data();
byte _serial_parser();
void serial_send_pair(String text, double value);
void serial_send_message(String message);
void serial_step();
void serial_send_position();

#endif