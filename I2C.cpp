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
 
// I2C.cpp
// This part of the program sends and receives datas trough I2C.

/*
 * Init function just initialize the I2C link. The DAC address and others setting bytes are given by I2C.h.
 * I2C_write sends new data for a channel (X, Y or L) to the DAC board.
 * Then I2C_update can update all the channels at once.
 */

#include <Arduino.h>

#include "I2C.h"
#include "settings.h"
#include "serialIO.h"
#include <Wire.h>

char _I2C_address;
char _commandByte;
char _addressByte;

//this function initialize the DAC
void I2C_init(){

	Wire.begin();												// Opens the connection
	_I2C_address = I2C_ADDR;									// Sets the addres of the DAC
	_I2C_address = _I2C_address >> 1;							// Shift the adress 1 bit right to be in write mode (never need read mode)

/*	_commandByte = AD5665_POWER << 3;							// Sets the power mode
	Wire.beginTransmission(_I2C_address);
	Wire.write(_commandByte);												
	Wire.write(0);
	Wire.write(3);
	Wire.endTransmission();
*/
	_commandByte = AD5665_INTERNAL_REF << 3;							// Sets the power mode
	Wire.beginTransmission(_I2C_address);
	Wire.write(_commandByte);												
	Wire.write(0);
	Wire.write(1);
	Wire.endTransmission();

//	I2C_write('X', 0x8000);										// and all axes at mid course
//	I2C_write('Y', 0xFFFF);										// Not needed: DAC initialize mid-pos with POR.
	I2C_update();

	serial_send_message("Liaison I2C initialisée.");



}

// This writes a new value to one channel of the DAC.
void I2C_write(char axe, int pos){

	switch (axe){
		case 'X':
			_addressByte = AD5665_DAC_A;
			break;
		case 'Y':
			_addressByte = AD5665_DAC_B;
			break;
		case 'L':
			_addressByte = AD5665_DAC_C;
			break;
		default:
			return;
	}

	_commandByte = (AD5665_WRITE_N << 3);							// sets the command byte and bitshift it
	_commandByte |= _addressByte;									// Adds the address byte
	_serial_append_value(_commandByte);
	_serial_append_nl();

	Wire.beginTransmission(_I2C_address);							// Start the transmission to the I2C slave. Send address + write
	Wire.write(_commandByte);										// sends the command byte and the address byte
	Wire.write(pos >> 8);
	Wire.write(pos);
	Wire.endTransmission();
}

// This updates all the DAC channels at once.
void I2C_update(){
	_commandByte = (AD5665_UPDATE_N << 3);
	_commandByte |= AD5665_DAC_ALL;
	Wire.beginTransmission(_I2C_address);							// Start the transmission to the I2C slave. Send address + write
	Wire.write(_commandByte);										// sends the command byte and the address byte
	Wire.write(0xFF);
	Wire.write(0xFF);
	Wire.endTransmission();
}
