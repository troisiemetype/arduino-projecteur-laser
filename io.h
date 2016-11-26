// Arduino laser projector
/*
 * This program is intended to control a laser projector
 * Copyright (C) 2016  Pierre-Loup Martin
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
 
// serialIO.h
// this part of the program deals with:
// serial input: what is sent to the board, verifies that datas are correctly written, and populates the movement buffers
// serial output: send back infos about position, so the program can know how much of the pattern has already be done


#ifndef IO_H
#define IO_H

#include <Arduino.h>
#include "debug.h"
#include "planner.h"
#include "driver.h"
#include "settings.h"
#include "system.h"

#define SERIAL_WATCHDOG				256

#define IO_OK						0
#define IO_SEND_AGAIN 				1

//define parser states / error
#define PARSE_IDLE					0
#define PARSE_HEADER				1
#define PARSE_POSX					2
#define PARSE_POSY					3
#define PARSE_POSL					4
#define PARSE_SPEED					5
#define PARSE_MODE					6
#define PARSE_CHECKSUM				7
#define PARSE_RECORD				8

#define PARSE_VAR_OK				2
#define PARSE_VALUE					3
#define PARSE_VALUE_OK				4
#define PARSE_PAIR					5
#define ERROR_INPUT					6
#define ERROR_START					7
#define ERROR_VAR					8
#define ERROR_VALUE					9
#define ERROR_PAIR					10
#define ERROR						11

#define CFG_START					20
#define CFG_VAR						21
#define CFG_VALUE					22
#define CFG_END						23

#define XON_CHAR					(char)17				// XON char. 0x11; ctrl+Q
#define XOFF_CHAR					(char)19				// XOFF char. 0x13; ctrl+S
#define RESET_CHAR					(char)24				// Rest char. 0x18; ctrl+X
#define NL_CHAR						(char)10				// New line char. 0x0A. \n
#define CR_CHAR						(char)13				// Carriage return char. 0x0D. \r


// serial singleton. contains value defining the parser state,
// stores values that have been parsed and not recorded yet.
struct serialState{
	int state;

	int parser_state;

	byte serial_watchdog;
	byte data_to_read;
	byte data_received;
	byte checksum;
	byte inVar;										// Stocks temporary var names before values are parsed and record.
	long inValue;									// Stocks value, before they are recorded.

	int id;										// Stocks the values received, if parsing has been successfull.
	int posX;
	int posY;
	byte posL;
	int speed;
	byte mode;
};

void io_init();
int io_main();
int _io_get_data();
byte _io_parse_char();
int _io_parse_int();
void _io_record_values();

void io_send_pair(String text, double value);
void io_send_message(String message);
void io_send_value(double value);

void _io_append_string(String data);
void _io_append_value(double value);
void _io_append_nl();
void _io_append_byte(char data);

#endif