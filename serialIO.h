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

#ifndef SERIALIO_H
#define SERIALIO_H

#define RX_BUFFER_SIZE				128
#define TX_BUFFER_SIZE				64

#define LINE_BUFFER_SIZE			30

// For Xon flow.
#define RX_FLOW_UP					64
#define RX_FLOW_DOWN				32

#define SET_XON						1
#define XON_SET						2
#define SET_XOFF					3
#define XOFF_SET					4

#define XON_CHAR					17				// XON char. 0x11; ctrl+Q
#define XOFF_CHAR					19				// XOFF char. 0x13; ctrl+S
#define RESET_CHAR					24				// Rest char. 0x18; ctrl+X
#define NL_CHAR						10				// New line char. 0x0A. \n
#define CR_CHAR						13				// Carriage return char. 0x0D. \r


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
	volatile int state;

	int parser_state;
	char parser_count;

	int parser_data_received;
	char inVar;										// Stocks temporary var names before values are parsed and record.
	long inValue;									// Stocks value, before they are recorded.

	volatile char queue;							// Stocks the rx queue size when _serial_rx_queue is called.

	long id;										// Stocks the vales received, if parsing has been successfull.
	int posX;
	int posY;
	int posL;
	long speed;
	char mode;

	volatile char flow_state;						// Stocks the state of flow control. See #defines above for states.
};

void serial_init();
int serial_main();
int serial_get_data();
int _serial_parse_data();
void _serial_record_pair();
void _serial_record_values();
void _serial_send_go();
//void _serial_send_again();
void serial_send_pair(String text, double value);
void serial_send_message(String message);
void serial_percent(long id);
void serial_step();
void _serial_interrupt_init();
char _serial_rx_queue();
void _serial_append_string(String data);
void _serial_append_value(double value);
void _serial_append_nl();
void _serial_append_byte(char data);
void _serial_clear_rx_buffer();

#endif