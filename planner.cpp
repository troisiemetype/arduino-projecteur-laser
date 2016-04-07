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

void planner_set_buffer(int posX, int posY, int posL, byte mode, byte set){
	moveBuffer_t *bf = mbp.write;

	if (!(set &= 0x1000)){
		posX = bf->pv->posX;
	}
	if (!(set &= 0x0100)){
		posY = bf->pv->posY;
	}
	if (!(set &= 0x0010)){
		posL = bf->pv->posL;
	}
	if (!(set &= 0x0001)){
		mode = bf->pv->mode;
	}

	bf->active = 1;
	bf->posX = posX;
	bf->posY = posY;
	bf->posL = posL;
	bf->mode = mode;

	mbp.write = bf->nx;
	mbp.available--;

}

void planner_free_buffer(moveBuffer_t *bf){
	moveBuffer_t *pv;
	moveBuffer_t *nx;

	pv = bf->pv;
	nx = bf->nx;

	memset(bf, 0, sizeof(*bf));

	bf->pv = pv;
	bf->nx = nx;

	mbp.available++;

}

