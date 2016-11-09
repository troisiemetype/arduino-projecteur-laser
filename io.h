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
#include "circularbuffer.h"
#include "planner.h" 
#include "serialdue.h"
#include "driver.h"
#include "settings.h"
#include "system.h"

#define LINE_BUFFER_SIZE			64

//define parser states / error
#define PARSE_IDLE			0
#define PARSE_VAR			1
#define PARSE_VAR_OK		2
#define PARSE_VALUE			3
#define PARSE_VALUE_OK		4
#define PARSE_PAIR			5
#define ERROR_INPUT			6
#define ERROR_START			7
#define ERROR_VAR			8
#define ERROR_VALUE			9
#define ERROR_PAIR			10
#define ERROR				11

#define CFG_START			20
#define CFG_VAR				21
#define CFG_VALUE			22
#define CFG_END				23

// serial singleton. contains value defining the parser state,
// stores values that have been parsed and not recorded yet.
struct serialState{
	int state;

	int parser_state;

	int parser_data_received;
	char inVar;										// Stocks temporary var names before values are parsed and record.
	long inValue;									// Stocks value, before they are recorded.

	long id;										// Stocks the vales received, if parsing has been successfull.
	int posX;
	int posY;
	int posL;
	long speed;
	char mode;
};

void io_init();
int io_main();
int _io_get_data();
int _io_parse_data();
void _io_record_pair();
void _io_record_values();
void _io_send_go();
//void _io_send_again();
void io_send_pair(String text, double value);
void io_send_message(String message);
void io_percent(long id);
void io_step();
void _io_interrupt_init();
char _io_rx_queue();
void _io_append_string(String data);
void _io_append_value(double value);
void _io_append_nl();
void _io_append_byte(char data);
void _io_clear_rx_buffer();

#endif