// Arduino laser projector

// serialIO.h
// this part of the program deals with:
// serial input: what is sent to the board, verifies that datas are correctly written, and populates the movement buffers
// erial output: send back infos about position, so the program can know how much of the pattern has already be done

#ifndef SERIALIO_H
#define SERIALIO_H

//define parser states
#define PARSER_IDLE		0
#define PARSER_START_JSON	1
#define PARSER_END_JSON		2
#define PARSING_VAR			3
#define PARSING_VALUE		4
#define PARSING_VAR_ERROR   5
#define PARSING_VALUE_ERROR 6
#define PARSING_PAIR_ERROR	7


void serial_init();
void serial_getData();
void _serial_parser();


#endif