
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
 
// debug.cpp
// this part of the program gives method for debuging on the Due Serial1.
// That way debug communication are separate from program communication,
// and can be easily de-included when needed

#include "debug.h"


void debug_init(){

	Serial1.begin(BAUDRATE);								//Debug serial
	
}

// This function write a pair of data to Serial1, formated in json
void debug_pair(String name, double value){
	debug_append_string(name);
	debug_append_string(" :");
	debug_append_value(value);
	debug_append_nl();
}

// This function writes a simple message to Serial1, formated in json
void debug_message(String message){
	debug_append_string(message);
	debug_append_nl();
}

void debug_value(double value){
	debug_append_value(value);
	debug_append_nl();
}


// This function is for debugging purpose: it prints "step" on Serial1. Used to "replace" code breakpoints.
void debug_step(){
	debug_append_string("step");
	debug_append_nl();
}


void debug_append_string(String data){
	int data_length = data.length();
	for (int i=0; i<data_length; i++){
		debug_append_byte(data.charAt(i));
	}
}
void debug_append_value(double data){
	String data_to_send = String(data, 3);
	debug_append_string(data_to_send);
}

void debug_append_nl(){
	debug_append_byte(NL_CHAR);
}

void debug_append_byte(char data){
	Serial1.write(data);
}
