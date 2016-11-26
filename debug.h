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


#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>
#include "system.h"

#define XON_CHAR					(char)17				// XON char. 0x11; ctrl+Q
#define XOFF_CHAR					(char)19				// XOFF char. 0x13; ctrl+S
#define RESET_CHAR					(char)24				// Rest char. 0x18; ctrl+X
#define NL_CHAR						(char)10				// New line char. 0x0A. \n
#define CR_CHAR						(char)13				// Carriage return char. 0x0D. \r

void debug_init();
void debug_pair(String text, double value);
void debug_message(String message);
void debug_value(double value);
void debug_append_string(String data);
void debug_append_value(double value);
void debug_append_nl();
void debug_append_byte(char data);

#endif