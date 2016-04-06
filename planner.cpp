// Arduino laser projector

// planner.cpp
// This part of the program reads each point that is received, buffers it and calculate every point of the route.
// Once all is done, buffers are used by the driver program to control the galvos

#include <Arduino.h>

#include "planner.h"

moveBufferPool_t mbp;

void planner_init(){
	planner_init_buffer();
	
}

void planner_init_buffer(){
	moveBuffer_t *pv;

	memset(&mbp, 0, sizeof(mbp));

	mbp.queue = &mbp.pool[0];
	mbp.write = &mbp.pool[0];
	mbp.run = &mbp.pool[0];
	mbp.available = BUFFER_POOL_SIZE;

	pv = &mbp.pool[BUFFER_POOL_SIZE-1];

	for (byte i=0; i<BUFFER_POOL_SIZE; i++){
		mbp.pool[i].pv = pv;
		if (i<BUFFER_POOL_SIZE-1){
			mbp.pool[i].nx = &mbp.pool[i+1];
		} else {
			mbp.pool[i].nx = &mbp.pool[0];
		}
		pv = &mbp.pool[i];
	}
}

byte planner_get_available(){
	return mbp.available;
}

void planner_set_buffer(int x, int y, int l, byte mode){
	moveBuffer_t *bf = mbp.queue;
	mbp.write = mbp.queue;
	bf->posX = x;
	bf->posY = y;
	bf->posL = l;
	bf->mode = mode;

	mbp.queue = bf->nx;

}