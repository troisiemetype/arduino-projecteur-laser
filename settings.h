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

// settings.h
// This part of the program stores the settings used by the other parts.
// It also stores the #defines

#ifndef SETTINGS_H
#define SETTINGS_H

//#define LANGUAGE				FRANCAIS

// Serial settings
#define BAUDRATE				115200					// Defines the baudrate used for the serial

// I2C settings
#define X_I2C_ADDR				0x62					// These addresses can changed accroding to the hardware DAC you use
#define Y_I2C_ADDR				0x63
#define L_I2C_ADDR				0x64

#define I2C_ADDR				0x20					// This adress is for use with a AD5665 DAC in adress mode

#define DAC_SIZE				16						// Number of bit the DAC uses

#define DEFAULT_SPEED			1000					// Default speed of the projecteur. Galvo steps/seconde. Used to avoid division / 0.

#define CLOCK_SPEED				16000000				// Frequency of the board
#define ISR_FREQUENCY			10000					// Frequency of the interrupt
#define BEAT_FREQUENCY			0.5						// Frequency of the heartbeat

#define WATCHDOG_TIMER			100						// Number of ISR interrupts whitout move before to stop the laser

/*
#define MOVE_CARTESIAN			0						// Used to test the move type in the driver ISR
#define MOVE_POLAR				1
*/

#define Z_DISTANCE				1000					// Default distance from projector to wall, in mm

#endif