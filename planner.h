// Arduino laser projector

// planner.h
// This part of the program reads each point that is received, buffers it and calculate every point of the route.
// Once all is done, buffers are used by the driver program to control the galvos

#ifndef PLANNER_H
#define PLANNER_H

#define BUFFER_POOL_SIZE 12

// this structure stores each point sent by computer to the program
struct moveBuffer {
	struct moveBuffer *pv;										// Stores the address of the previous buffer
	struct moveBuffer *nx;										// Stores the address of the next buffer

	int id;														// Stores the ID of this move, sent by the program

	byte active;												// remember if the buffer is active, i.e. if it has been set or if it's empty
	byte compute;												// Rememeber if the planner has already calulated the deltas and increments

	int pos[3];													// The X position we must go to
	int speed;													// Ditto speed. Applies to the curent movement. mm/s.
	byte mode;													// Stores the current mode: 0 = fast movement, else = calibrated movement

	double now[3];												// Stores the instant position

	double posA;												// Stores the position it must go
	double posB;

	double deltaTotal;												// The delta between the start and the final position
	int delta[3];													

	long steps;													// Stores the number ot steps for this move (== (delta / speed) * ISR_FREQUENCY)
	long nowSteps;

	double incr[3];												// The increment it must goes forward on each step
};

// this structure stores the states of the buffers
struct moveBufferPool {
	struct moveBuffer *write;									// Stores the address of the write buffer, i.e. where to write incomming data
	struct moveBuffer *queue;									// Stores the addres of the queue buffer, i.e. the last one added to the queue
	struct moveBuffer *run;										// Stores the address of the run buffer, i.e. which one is currently executing

	byte available;												// How much buffers are available for write

	moveBuffer pool[BUFFER_POOL_SIZE];						// Hold the place for every buffer
};

void planner_init();
void planner_init_buffer();
byte planner_get_available();
moveBuffer* planner_get_run_buffer();
void planner_set_next_buffer(byte buffer);
void planner_set_buffer(int id, int posX, int posY, int posL, int speed, byte mode, byte set);
void planner_free_buffer(moveBuffer * bf);
void planner_plan_move();


#endif