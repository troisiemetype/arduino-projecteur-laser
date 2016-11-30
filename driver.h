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

#include <Arduino.h>
#include <chip.h>
#include <math.h>

#include "driver.h"
#include "planner.h"
#include "I2C.h"
#include "io.h"
#include "settings.h"
#include "system.h"


#define TIMER_N 			TC0
#define TIMER_CHANNEL		0
#define TIMER_IRQ			TC0_IRQn

#define DRIVER_OFFSET		32768

#define DRIVER_POOL_SIZE	64

// This structure stores the state of the Driver
struct driverState{
	//Stores driver state.
	volatile int state;

	//Store the driver state
	//Stores the previous prosition. Used to know if there is move.
	//test fixpoint
	long now[3];
	long previous[3];
	int pause;

	//Vars for the heartbeat led. It counts the ISR interrupts,
	//and the current and max values for both modes.
	volatile bool ledState;
	volatile unsigned int beat_count;
	volatile int beat_max;
	int beat_max_idle;
	int beat_max_driving;

	//Stores if the laser is moving
	bool moving;

	//debug: get the ISR time length
	volatile long isrLength;
};

//This buffer stores the computed position of the driver,
//and a pointer to next and previous buffers;.
struct driverBuffer{
	struct driverBuffer *pv;
	struct driverBuffer *nx;

	long pos[3];

	int pause;
};

//This is the buffer ring for position buffers.
struct driverBufferPool{
	struct driverBuffer *run;
	struct driverBuffer *queue;

	byte available;

	driverBuffer pool[DRIVER_POOL_SIZE];
};

void driver_init(void);
void _driver_buffer_init(void);
void _driver_timer_init(void);
void _driver_pwm_init(void);
void driver_heartbeat(void);
int driver_main(void);
int _driver_plan_pos(void);
int _driver_update_pos(void);
void _driver_laser(void);
long * driver_get_position(void);

#endif