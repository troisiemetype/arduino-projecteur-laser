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
 
// planner.h
// This part of the program reads each point that is received, buffers it and calculate every point of the route.
// Once all is done, buffers are used by the driver program to control the galvos



#ifndef PLANNER_H
#define PLANNER_H

#include <Arduino.h>

#include "debug.h"
#include "planner.h"
#include "io.h"
#include "settings.h"
#include "driver.h"
#include "system.h"

#define BUFFER_POOL_SIZE 64

struct plannerState{
	int state;
};

// this structure stores each point sent by computer to the program
struct plannerBuffer {
	struct plannerBuffer *pv;						// Stores the address of the previous buffer
	struct plannerBuffer *nx;						// Stores the address of the next buffer

	bool active;									// Remember if the buffer is active, i.e. if it has been set or if it's empty
	bool compute;									// Rememeber if the planner has already calulated the deltas and increments

	long id;
	long pos[3];									// The position we must go to
	long speed;										// Ditto speed. Applies to the curent movement. Galvo increments/second.
	byte mode;										// Stores the current mode: 0 = fast movement, 1 = fast + tempo, 2 = calibrated movement

	//Test fixpoint
	long current[3];								// Stores the instant position

	unsigned long deltaTotal;						// The delta between the start and the final position.
	long delta[3];													

	long steps;										// Stores the number ot steps for this move (== (delta / speed) * ISR_FREQUENCY)
	volatile long nowSteps;

	long incr[3];									// The increment it must goes forward on each step
};

// this structure stores the states of the buffers
struct plannerBufferPool {
	struct plannerBuffer *write;					// Stores the address of the write buffer, i.e. where to write incomming data
	struct plannerBuffer *queue;					// Stores the addres of the queue buffer, i.e. the last one added to the queue
	struct plannerBuffer *run;						// Stores the address of the run buffer, i.e. which one is currently executing

	byte available;									// How much buffers are available for write

	bool computed;

	plannerBuffer pool[BUFFER_POOL_SIZE];			// Hold the place for every buffer
};

void planner_init();
void planner_init_buffer();
int planner_main();
byte planner_get_available();
bool planner_computed();
plannerBuffer* planner_get_run_buffer();
void planner_set_next_buffer(byte buffer);
void planner_set_buffer(long I, long posX, long posY, long posL, long speed, byte mode, byte set);
void planner_free_buffer(plannerBuffer * bf);
int planner_plan_move();

#endif