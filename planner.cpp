// Arduino laser projector

// planner.cpp
// This part of the program reads each point that is received, buffers it and calculate every point of the route.
// Once all is done, buffers are used by the driver program to control the galvos

#include <Arduino.h>
//#include <math.h>

#include "planner.h"
#include "serialIO.h"
#include "settings.h"

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

// This sets up a buffer with the data received from Serial
/* It first verifies for each parameter if it has been set or if we copy those of the previous buffer
 * then it coppies these parameters to the buffer
 * last, it steps up the write pointer and decrease the available counter
 */
void planner_set_buffer(int id, int posX, int posY, int posL, int speed, byte mode, byte set){
	moveBuffer *bf = mbp.write;

	//These tests verifies if the value has been sent by the computer.
	if (!(set & 32)){
		id = bf->pv->id;
	}
	if (!(set & 16)){
		posX = bf->pv->posX;
	}
	if (!(set & 8)){
		posY = bf->pv->posY;
	}
	if (!(set & 4)){
		posL = bf->pv->posL;
	}
	if (!(set & 2)){
		speed = bf->pv->speed;
	}
	if (!(set & 1)){
		mode = bf->pv->mode;
	}

	bf->active = 1;															// active = 1 tells that values have been load in the buffer
	bf->id = id;															// Copies the values to the buffer
	bf->posX = posX;
	bf->posY = posY;
	bf->posL = posL;
	bf->speed = speed;
	bf->mode = mode;
	bf->compute = 0;														// Compute = 0 says that the buffer needs to be planned

	// steps up the write pointer.
	mbp.write = bf->nx;
	mbp.available--;

}

// This functions frees a buffer (e.g., that has been run)
/* It does so by copying pv and nx pointers, do a memset (populates buffer with zeros) and copy back pv and nx pointers
 * It then increase the available counter
 */
void planner_free_buffer(moveBuffer *bf){
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
	moveBuffer *pv = bf->pv;
	if (bf->compute == 1 || bf->active == 0){								// If compute == 1, the buffer has already been computed
		return;																// If Compute == 0, the buffer is empty
	}

	if (bf->mode == 1){														// Look at the type of move: O is fast (placement), 1 is calibrated
		bf->deltaX = bf->posX - pv->posX;									// Compute the delta between previous and goal position
		bf->deltaY = bf->posY - pv->posY;
		bf->deltaL = bf->posL - pv->posL;
		bf->delta = sqrt(pow(bf->deltaX, 2) + pow(bf->deltaY, 2));			// Compute the length of the route
		serial_send_pair("delta", bf->delta);

		if (bf->speed == 0){												// If speed is equal to zero, sets the default speed
			bf->speed = DEFAULT_SPEED;
		}

		bf->steps = abs(bf->delta / bf->speed) * ISR_FREQUENCY;				// Compute the number of steps according to the route, speed and ISR
		serial_send_pair("steps", bf->steps);

		if (bf->steps == 0){												// Steps cannot be 0 If it is, sets it to 1
			bf->steps = 1;
		}

		bf->incrX = (double)bf->deltaX / (double)bf->steps;					// Compute the increment for each axe
		bf->incrY = (double)bf->deltaY / (double)bf->steps;
		bf->incrL = (double)bf->deltaL / (double)bf->steps;
		serial_send_pair("incrX",bf->incrX);
		serial_send_pair("incrY",bf->incrY);
		serial_send_pair("incrL",bf->incrL);
	}

	bf->compute = 1;														// The buffer is marked as having been compute
	mbp.queue = bf->nx;														// The queue index is step up
}