// Arduino laser projector

// planner.cpp
// This part of the program reads each point that is received, buffers it and calculate every point of the route.
// Once all is done, buffers are used by the driver program to control the galvos

#include <Arduino.h>

#include "planner.h"
#include "serialIO.h"
#include "settings.h"
#include "driver.h"

moveBufferPool mbp;

// initialisation function.
/* Init the buffers in memory
 */
void planner_init(){
	planner_init_buffer();
	serial_send_message("planner initialisé");
	
}

//init the buffers in memory
void planner_init_buffer(){
	moveBuffer *pv;															// creates a pointer to buffer

	memset(&mbp, 0, sizeof(mbp));											// Clears the buffer pool

	mbp.write = &mbp.pool[0];												// sets pointers to write on the first item in the pool
	mbp.run = &mbp.pool[0];													// Ditto run
	mbp.queue = &mbp.pool[0];												// Ditto queue
	mbp.available = BUFFER_POOL_SIZE;										// Sets the number of available buffers

	pv = &mbp.pool[BUFFER_POOL_SIZE-1];										// Sets the pv pointer on the last buffer of the pool

	for (byte i=0; i<BUFFER_POOL_SIZE; i++){								// For each buffer in the pool,
		mbp.pool[i].pv = pv;												// Sets a pointer to the previous buffer
		if (i<BUFFER_POOL_SIZE-1){											// And to the next buffer								
			mbp.pool[i].nx = &mbp.pool[i+1];								// pointer is to the next buffer
		} else {															// If last iteration, next buffer is #0
			mbp.pool[i].nx = &mbp.pool[0];
		}
		pv = &mbp.pool[i];													// Prepare next iteration: current buffer is the previous of the next.
	}
}

// This just gives how much buffers are available
byte planner_get_available(){
	return mbp.available;
}

// This just gives a pointer to the run buffer
moveBuffer* planner_get_run_buffer(){
	return mbp.run;
}

// This function updates the buffer pool with the next buffer
void planner_set_next_buffer(byte buffer){
	switch(buffer){
		case 0:																// write buffer
			mbp.write = mbp.write->nx;
			break;
		case 1:																// queue buffer
			mbp.queue = mbp.queue->nx;
			break;
		case 2:																// run buffer
			mbp.run = mbp.run->nx;
			break;
		default:
			break;
	}

}

// This sets up a buffer with the data received from Serial
/* It first verifies for each parameter if it has been set or if we copy those of the previous buffer
 * then it coppies these parameters to the buffer
 * last, it steps up the write pointer and decrease the available counter
 */
void planner_set_buffer(int id, int posX, int posY, int posL, int speed, byte mode, byte set){
	moveBuffer *bf = mbp.write;
	moveBuffer *pv = bf->pv;

	// Here we want to know which is the start position of this move.
	// If the previous buffer is populated, the start position is the end position of this buffer, so we copy it
	// If the previous buffer is empty, then we have to use the current position, that is stored in the driverState struct
	if (pv->active == 0){													// verifies how is the buffer queue
		volatile double * position = driver_get_position();					// Get the driver state
		for (int i=0; i<3; i++){
			bf->now[i] = position[i];										// If previous buffer is empty, set now[] as the current position
		}
	} else {																// If previous buffer is populated, now[] will be the previous output
		for (int i=0; i<3; i++){
			bf->now[i] = pv->pos[i];
		}
	}

	// These tests verifies if the value has been sent by the computer.
	// A value that hasn't been sent by the computer program should be copy for the previous position, i.e.:
	// driver state position is the move is stopped
	// previous buffer end position if it's populated
	// So we just copy the values that have been set above
	if (!(set & 16)){
		posX = bf->now[0];
	}
	if (!(set & 8)){
		posY = bf->now[1];
	}
	if (!(set & 4)){
		posL = bf->now[2];
	}
	if (!(set & 2)){
		speed = pv->speed;
	}
	if (!(set & 1)){
		mode = pv->mode;
	}

	bf->active = 1;															// active = 1 tells that values have been load in the buffer
	bf->id = id;															// Copies the values to the buffer
	bf->pos[0] = posX;
	bf->pos[1] = posY;
	bf->pos[2] = posL;
	bf->speed = speed;
	bf->mode = mode;
	bf->compute = 0;														// Compute = 0 says that the buffer needs to be planned

	// steps up the write pointer.
	planner_set_next_buffer(0);
//	mbp.write = bf->nx;
	mbp.available--;

}

// This functions frees a buffer (e.g., that has been run)
/* It does so by copying pv and nx pointers, do a memset (populates buffer with zeros) and copy back pv and nx pointers
 * It then increase the available counter
 */
void planner_free_buffer(moveBuffer*bf){
	moveBuffer *pv;
	moveBuffer *nx;

	pv = bf->pv;
	nx = bf->nx;

	memset(bf, 0, sizeof(*bf));

	bf->pv = pv;
	bf->nx = nx;

	mbp.available++;

}

// This function plans the move for a buffer
/* It first get the pointer to the queue buffer
 */
void planner_plan_move(){
	moveBuffer *bf = mbp.queue;
	if (bf->compute == 1 || bf->active == 0){								// If compute == 1, the buffer has already been computed
		return;																// If Compute == 0, the buffer is empty
	}

	if (bf->mode != 0){														// Look at the type of move: O is fast (placement), else is calibrated
		for (int i=0; i<3; i++){											// Compute the delta between previous and goal position
			bf->delta[i] = bf->pos[i] - bf->now[i];
		}
		bf->deltaTotal = sqrt(pow(bf->delta[0], 2) + pow(bf->delta[1], 2));	// Compute the length of the route
//		serial_send_pair("delta", bf->deltaTotal);

		if (bf->speed == 0){												// If speed is equal to zero, sets the default speed
			bf->speed = DEFAULT_SPEED;
		}

		bf->steps = abs(bf->deltaTotal / bf->speed) * ISR_FREQUENCY;		// Compute the number of steps according to the route, speed and ISR
//		serial_send_pair("steps", bf->steps);

		if (bf->steps == 0){												// Steps cannot be 0. If it is, sets it to 1
			bf->steps = 1;
		}

		for (int i = 0; i<3; i++){											// Compute the increment for each axe
			bf->incr[i] = (double)bf->delta[i] / (double)bf->steps;
		}
//		serial_send_pair("incrX",bf->incr[0]);
//		serial_send_pair("incrY",bf->incr[1]);
//		serial_send_pair("incrL",bf->incr[2]);
	}

	bf->compute = 1;														// The buffer is marked as having been compute
	mbp.queue = bf->nx;														// The queue index is step up
}