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

#define DRIVER_OFFSET		32768

#define DRIVER_POOL_SIZE	12

// This structure stores the state of the Driver
struct driverState{
	//Stores the current poition.
	//Stores the previous prosition. Used to know if there is move.
	double now[3];
	double previous[3];

	//Vars for the heartbeat led. It counts the ISR interrupts,
	//and the current and maxe values for both modes.
	volatile unsigned int beat_count;
	int beat_max;
	int beat_max_idle;
	int beat_max_driving;

	//Stores if the laser is moving
	bool moving;

	bool update;

	//debug: get the ISR time length
	volatile long isrLength;
};

//This buffer stores the computed position of the driver,
//and a pointer to next and previous buffers;.
struct driverBuffer{
	struct driverBuffer *pv;
	struct driverBuffer *nx;

	double pos[3];
};

//This is the buffer ring for position buffers.
struct driverBufferPool{
	struct driverBuffer *run;
	struct driverBuffer *queue;

	byte available;

	driverBuffer pool[DRIVER_POOL_SIZE];
};

void driver_init();
void driver_init_buffer();
void driver_interrupt_init();
void driver_heartbeat();
void driver_plan_pos();
void driver_update_pos();
void driver_laser();
double * driver_get_position();

#endif