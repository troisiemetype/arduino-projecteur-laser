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
// erial output: send back infos about position, so the program can know how much of the pattern has already be done

#ifndef SERIALIO_H
#define SERIALIO_H

#define RX_BUFFER_SIZE				128
#define TX_BUFFER_SIZE				128

// For Xon flow.
#define RX_FLOW_UP					96
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
#define PARSING_IDLE				0
#define PARSING_VAR					1
#define PARSING_VAR_OK				2
#define PARSING_VALUE				3
#define PARSING_VALUE_OK			4
#define PARSING_PAIR				5
#define PARSING_ERROR_INPUT			6
#define PARSING_ERROR_START			7
#define PARSING_ERROR_VAR			8
#define PARSING_ERROR_VALUE			9
#define PARSING_ERROR_PAIR			10
#define PARSING_ERROR				11

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

	volatile char queue;							// Stocks the rx queue size when _serial_rx_queue is called.

//	long id;										// Stocks the vales received, if parsing has been successfull.
	long posX;
	long posY;
	long posL;
	long speed;
	char mode;

	volatile char flow_state;						// Stocks the state of flow control. See #defines above for states.
};

void serial_init();
void serial_get_data();
void _serial_parse();
void _serial_parse_data();
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
char _serial_rx_queue();
void _serial_append_string(String data);
void _serial_append_value(double value);
void _serial_append_nl();
void _serial_append_byte(char data);
void _serial_clear_rx_buffer();
void _serial_flush_rx_buffer();

#endif