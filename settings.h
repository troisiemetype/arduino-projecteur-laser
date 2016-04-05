// Arduino laser projector

// settings.h
// This part of the program stores the settings used by the other parts.
// It also stores the #defines

#ifndef SETTINGS_H
#define SETTINGS_H

#define SERIAL_CONFIG			"SERIAL_8N1"			// See http://www.arduino.cc/en/Reference/Serial for possible values

#define X_I2C_ADDR				0x62					// These addreses can changed accroding to the hardware DAC you use
#define Y_I2C_ADDR				0x63
#define L_I2C_ADDR				0x64

#define I2C_ADDR				0x10					// This adress is for use with a AD5665

#define DAC_SIZE				12						// Number of bit the DAC uses

#endif