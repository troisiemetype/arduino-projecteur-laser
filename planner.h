// Arduino laser projector

// planner.h
// This part of the program reads each point that is received, buffers it and calculate every point of the route.
// Once all is done, buffers are used by the driver program to control the galvos

#ifndef PLANNER_H
#define PLANNER_H

#define BUFFER_POOL_SIZE 10

// this structure stores each point sent by computer to the program
typedef struct moveBuffer {
	struct moveBuffer *pv;										// Stores the address of the previous buffer
	struct moveBuffer *nx;										// Stores the address of the next buffer

	byte active;												// remember if the buffer is active, i.e. if it has been set or if it's empty

	int posX;													// The X position we must go to
	int posY;													// Ditto Y
	int posL;													// Ditto Laser (compute as if it were a position)
	byte mode;													// Stores the current mode: 0 = fast movement, 1 = calibrated movement

	int deltaX;													// The delta between the start and the final position
	int deltaY;
	int deltaL;

	float incrX;												// The increment it must goes forward on each step
	float incrY;
	float incrL;

	int percent;												// percent of the move that has already be done

} moveBuffer_t;

// this structure stores the states of the buffers
typedef struct moveBufferPool {
	struct moveBuffer *queue;									// Stores the address of the queue buffer, i.e. where to write next incoming data
	struct moveBuffer *write;									// Stores the address of the write buffer, i.e. where to write
	struct moveBuffer *run;										// Stores the address of the run buffer, i.e. which one is currently executing

	byte available;												// How much buffers are available for write

	moveBuffer_t pool[BUFFER_POOL_SIZE];						// Hold the place for every buffer
} moveBufferPool_t;

void planner_init();
void planner_init_buffer();
byte planner_get_available();
void planner_set_buffer(int x, int y, int l, byte mode);

#endif