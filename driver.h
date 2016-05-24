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
 
// driver.h
/* This part of the program sends at a regular interval the positions to the galvos
   It fires an interrupt on a known interval,
   then at each interrupt it computes the new position value using what has been set in the buffer.
   Finally, these new values are sent to the external DAC trough I2C.
*/

#ifndef DRIVER_H
#define DRIVER_H

// This structure stores the state of the Driver
struct driverState{
	volatile double now[3];											// Stores the current position
	volatile double previous[3];									// Stores the previous position

	int watchdog;													// looks after the move, to stop the laser if there's no.

	unsigned int beat_count;										// Stores the number of overflow to give an heartbeat.
	int beat_max;													// Current value, is set ot one of the two above by driver.
	int beat_max_idle;												// Stores the value the led state should be changed when idle.
	int beat_max_driving;											// Ditto computing and sending data to galvo drivers.

	int zDistance;													// Stores the distance from projector to wall

	volatile char moving;											// Knows if it's moving or not

	volatile bool update;
};

void driver_init();
void driver_interrupt_init();
void driver_update_pos();
driverState * driver_get_ds();
boolean driver_is_moving();
volatile double * driver_get_position();

#endif