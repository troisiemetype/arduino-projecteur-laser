// Arduino laser projector

// I2C.cpp
// This part of the program sends and receives datas trough I2C.

#include <Arduino.h>

#include "I2C.h"
#include "settings.h"
#include "serialIO.h"
#include <Wire.h>

byte _I2C_address;
byte _commandByte;
byte _addressByte;

//this function initialize the DAC
void I2C_init(){

	Wire.begin();												// Opens the connection
	_I2C_address = I2C_ADDR;									// Sets the addres of the DAC
	_I2C_address = _I2C_address << 1;							// Shift the adress 1bit left to be in write mode (never need read mode)

	_commandByte = AD5665_POWER << 3;							// Sets the power mode
	Wire.beginTransmission(_I2C_address);
	Wire.write(_commandByte);												
	Wire.write(0);
	Wire.write(7);
	Wire.endTransmission();

	I2C_write('L', 0x00);										// sets the laser to 0
	I2C_write('X', 0x80);										// and all axes at mid course
	I2C_write('Y', 0x80);
	I2C_update();

//	serial_send_message("liaison I2C initialisée");



}

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

	_commandByte = AD5665_WRITE_N << 3;								// sets the command byte and bitshift it
	_commandByte += _addressByte;									// Adds the address byte

	Wire.beginTransmission(_I2C_address);							// Start the transmission to the I2C slave. Send address + write
	Wire.write(_commandByte);										// sends the command byte and the address byte
	Wire.write(pos >> 4);
	Wire.write(pos);
	Wire.endTransmission();
}

void I2C_update(){
	_commandByte = AD5665_UPDATE_N << 3;
	_commandByte += AD5665_DAC_ALL;
	Wire.beginTransmission(_I2C_address);							// Start the transmission to the I2C slave. Send address + write
	Wire.write(_commandByte);										// sends the command byte and the address byte
	Wire.write(0);
	Wire.write(0);
	Wire.endTransmission();
}
