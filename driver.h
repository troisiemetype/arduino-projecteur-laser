// Arduino laser projector

// driver.h
/* This part of the program sends at a regular interval the positions to the galvos
   It fires an interrupt on a known interval,
   then at each interrupt it computes the new position value using what has been set in the buffer.
   Finally, these new values are sent to the external DAC trough I2C.
*/

#ifndef DRIVER_H
#define DRIVER_H

// This structure stores the state of the Driver
struct driverState{
	volatile double now[3];											// Stores the current position
	volatile double previous[3];									// Stores the previous position

	int watchdog;													// lokks after the move, to stop the laser if there's no.

	int zDistance;													// Stores the distance from projector to wall

	volatile byte moving;											// Knows if it's moving or not

	volatile boolean move_flag;												//Sets each time a move is compute, so serial knows if this move has been sent

};
void driver_init();
void driver_interrupt_init();
ISR(TIMER1_COMPA_vect);
driverState * driver_get_ds();
boolean driver_is_moving();
volatile double * driver_get_position();

#endif