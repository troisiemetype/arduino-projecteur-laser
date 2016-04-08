// Arduino laser projector

// settings.h
// This part of the program stores the settings used by the other parts.
// It also stores the #defines

#ifndef SETTINGS_H
#define SETTINGS_H

#define SERIAL_CONFIG			SERIAL_8N1				// See http://www.arduino.cc/en/Reference/Serial for possible values

#define X_I2C_ADDR				0x62					// These addreses can changed accroding to the hardware DAC you use
#define Y_I2C_ADDR				0x63
#define L_I2C_ADDR				0x64

#define I2C_ADDR				0x10					// This adress is for use with a AD5665 DAC

#define DAC_SIZE				12						// Number of bit the DAC uses

#define DEFAULT_SPEED			100						// Default speed of the projecteur

#define CLOCK_SPEED				16000000				// Frequency of the board
#define ISR_FREQUENCY			200						// Frequency of the interrupt

#define MOVE_CARTESIAN			0						// Used to test the move type in the driver ISR
#define MOVE_POLAR				1

#define Z_DISTANCE				1000					// Distance from projector to wall, in mm

#endif