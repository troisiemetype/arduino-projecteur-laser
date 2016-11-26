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
 
// planner.cpp
// This part of the program reads each point that is received, buffers it and calculate every point of the route.
// Once all is done, buffers are used by the driver program to control the galvos

#include "planner.h"

plannerState ps;
plannerBufferPool pbp;

// initialisation function.
/* Init the buffers in memory
 */
void planner_init(){

	ps.state = PLANNER_IDLE;
	planner_init_buffer();

//	io_send_message(F("Planner initialisé."));
	
}

//init the buffers in memory
void planner_init_buffer(){
	plannerBuffer *pv;															// creates a pointer to buffer

	memset(&pbp, 0, sizeof(pbp));											// Clears the buffer pool

	pbp.write = &pbp.pool[0];												// sets pointers to write on the first item in the pool
	pbp.run = &pbp.pool[0];													// Ditto run
	pbp.queue = &pbp.pool[0];												// Ditto queue
	pbp.available = BUFFER_POOL_SIZE;										// Sets the number of available buffers

	pv = &pbp.pool[BUFFER_POOL_SIZE-1];										// Sets the pv pointer on the last buffer of the pool

	for (byte i=0; i<BUFFER_POOL_SIZE; i++){								// For each buffer in the pool,
		pbp.pool[i].pv = pv;												// Sets a pointer to the previous buffer
		if (i<BUFFER_POOL_SIZE-1){											// And to the next buffer								
			pbp.pool[i].nx = &pbp.pool[i+1];								// pointer is to the next buffer
		} else {															// If last iteration, next buffer is #0
			pbp.pool[i].nx = &pbp.pool[0];
		}
		pv = &pbp.pool[i];													// Prepare next iteration: current buffer is the previous of the next.
	}
}

int planner_main(){
//	io_send_message("planner main");
	if (ps.state == PLANNER_IDLE){
		return STATE_OK;
	}
//	_io_append_string("planner state: ");
//	_io_append_value(ps.state);
//	_io_append_nl();

	if (ps.state & PLANNER_COMPUTE_BUF){
		return planner_plan_move();
	}
}

// This just gives how much buffers are available
byte planner_get_available(){
	return pbp.available;
}

bool planner_computed(){
	return pbp.computed;
}

// This just gives a pointer to the run buffer
plannerBuffer* planner_get_run_buffer(){
	return pbp.run;
}

// This function updates the buffer pool with the next buffer
void planner_set_next_buffer(byte buffer){
	switch(buffer){
		case 0:																// write buffer
			pbp.write = pbp.write->nx;
			break;
		case 1:																// queue buffer
			pbp.queue = pbp.queue->nx;
			break;
		case 2:																// run buffer
			pbp.run = pbp.run->nx;
			break;
		default:
			break;
	}

}

// This sets up a buffer with the data received from Serial
/* It first verifies for each parameter if it has been set or if we copy those of the previous buffer
 * then it copies these parameters to the buffer
 * last, it steps up the write pointer and decrease the available counter
 */
void planner_set_buffer(long id, long posX, long posY, long posL, long speed, byte mode, byte set){

//	long debut = micros();

	plannerBuffer *bf = pbp.write;
	plannerBuffer *pv = bf->pv;

	// Here we want to know which is the start position of this move.
	// If the previous buffer is populated, the start position is the end position of this buffer, so we copy it
	// If the previous buffer is empty, then we have to use the current position, that is stored in the driverState struct
	if (pv->active == 0){													// verifies how is the buffer queue
		long * position = driver_get_position();					// Get the driver state
		for (int i=0; i<3; i++){
			// If previous buffer is empty, set current[] as the current position
			bf->current[i] = (double)position[i];
		}
//		io_send_pair("pos X départ arreté = ", bf->current[0]);
	} else {
		// If previous buffer is populated, current[] will be the previous output
		for (int i=0; i<3; i++){
			bf->current[i] = pv->pos[i];
		}
//		io_send_pair("pos X départ mouvement = ", bf->current[0]);
	}

	posX *= 256;
	posY *= 256;
	posL *= 256;
	speed *= 256;

	// These tests verify if the value has been sent by the computer.
	// A value that hasn't been sent by the computer program should be copy for the previous position, i.e.:
	// driver state position if the move is stopped
	// previous buffer end position if it's populated
	// So we just copy the values that have been set above
	if (!(set & FLAG_X)){
		posX = bf->current[0];
	}
	if (!(set & FLAG_Y)){
		posY = bf->current[1];
	}
	if (!(set & FLAG_L)){
		posL = bf->current[2];
	}
	if (!(set & FLAG_SPEED)){
		speed = pv->speed;
	}
	if (!(set & FLAG_MODE)){
		mode = pv->mode;
	}
	debug_value(posX);
	debug_value(posY);
	debug_append_nl();


	bf->id = id;
	bf->pos[0] = posX;
	bf->pos[1] = posY;
	bf->pos[2] = posL;
	bf->speed = speed;
	bf->mode = mode;
	// Compute = 0 says that the buffer needs to be planned
	//bf->compute = 0;
	// active = 1 tells that values have been loaded in the buffer
	bf->active = 1;

	// steps up the write pointer.
	planner_set_next_buffer(0);
	pbp.available--;

	bit_true(ps.state, PLANNER_COMPUTE_BUF);

//	_debug_append_string("planner populated");
//	_debug_append_nl();

//	_io_append_value(micros() - debut);
//	_io_append_nl();

}

// This functions frees a buffer (e.g., that has been run)
/* It does so by copying pv and nx pointers, do a memset (populates buffer with zeros) and copy back pv and nx pointers
 * It then increase the available counter
 */
void planner_free_buffer(plannerBuffer* bf){
	plannerBuffer *pv;
	plannerBuffer *nx;

	pv = bf->pv;
	nx = bf->nx;

	memset(bf, 0, sizeof(*bf));

	bf->pv = pv;
	bf->nx = nx;

	pbp.available++;

	if (pbp.available == BUFFER_POOL_SIZE){
		pbp.computed = 0;
	}

}

// This function plans the move for a buffer
/* It first get the pointer to the queue buffer
 */
int planner_plan_move(){

//	long debut = micros();

	plannerBuffer *bf = pbp.queue;
//	io_send_pair("bf->active", bf->active);
	//Verify if the queue buffer has to be computed.
	if (bf->compute == 1 || bf->active == 0){								// If compute == 1, the buffer has already been computed
		bit_false(ps.state, PLANNER_COMPUTE_BUF);
		if(pbp.available >= BUFFER_POOL_SIZE){
				//If no computing because of empty buffer pool, nothing
				return STATE_NO_OP;															// If Compute == 0, the buffer is empty
			} else {
				//If no computing because eveything has already been computed, driver needs compute.
				return STATE_ENTER_AGAIN;
			} 
	}

	if (bf->mode != 0){														// Look at the type of move: O is fast (placement), else is calibrated
		for (int i=0; i<3; i++){											// Compute the delta between previous and goal position
			bf->delta[i] = bf->pos[i] - bf->current[i];
		}
//	_io_append_value(bf->delta[0]);
//	_io_append_nl();
		bf->deltaTotal = sqrt(pow(bf->delta[0], 2) + pow(bf->delta[1], 2));	// Compute the length of the route
//		io_send_pair("delta", bf->deltaTotal);

		if (bf->speed == 0){												// If speed is equal to zero, sets the default speed
			bf->speed = DEFAULT_SPEED;
		}

		bf->steps = abs((double)bf->deltaTotal / (double)bf->speed) * ISR_FREQUENCY;		// Compute the number of steps according to the route, speed and ISR
//		io_send_pair("steps", bf->steps);
//	_io_append_value(bf->steps);
//	_io_append_nl();

		if (bf->steps == 0){												// Steps cannot be 0. If it is, sets it to 1
			bf->steps = 1;
		}

		for (int i = 0; i<3; i++){											// Compute the increment for each axe
			bf->incr[i] = bf->delta[i] / bf->steps;
		}
//	_io_append_value(bf->incr[0]);
//	_io_append_nl();
	} else {
		//If the move type is fast move, the increment equals the new pos.
		bf->steps = 1;
		for (int i = 0; i<3; i++){
			bf->incr[i] = bf->pos[i] - bf->current[i];
		}
	}
/*	io_send_pair("posX",bf->pos[0]);
	io_send_pair("posY",bf->pos[1]);
	io_send_pair("posL",bf->pos[2]);
	_io_append_nl();
	io_send_pair("delta total",bf->deltaTotal);
	io_send_pair("deltaX",bf->delta[0]);
	io_send_pair("deltaY",bf->delta[1]);
	io_send_pair("deltaL",bf->delta[2]);
	io_send_pair("steps", bf->steps);
	_io_append_nl();
	io_send_pair("incrX",bf->incr[0]);
	io_send_pair("incrY",bf->incr[1]);
	io_send_pair("incrL",bf->incr[2]);
	io_send_pair("mode", bf->mode);
	_io_append_nl();
	_io_append_nl();
*/
	pbp.computed = 1;
	bf->compute = 1;														// The buffer is marked as having been compute
	planner_set_next_buffer(1);												// The queue index is step up

//	_io_append_string("planner plan");
//	_io_append_nl();
//	_io_append_value(micros() - debut);
//	_io_append_nl();
	
	if (planner_get_available() > 1){
		return STATE_OK;
	}
}