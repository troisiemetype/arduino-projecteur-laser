// Arduino laser projector

// driver.cpp
/* This part of the program sends at a regular interval the positions to the galvos
   It fires an interrupt on a known interval,
   then at each interrupt it computes the new position value using what has been set in the buffer.
   Finally, these new values are sent to the external DAC trough I2C.
*/

/* There are three timer on the Atmel 328p the uno/nano is build on: TIMER0, TIMER1, TIMER2
 * TIMER0 is a 8 bits timer. Arduino uses it for delay(), millis() and micros() functions
 * TIMER1 is a 16 bits timer, Arduino uses it for the Servo library
 * TIMER2 is a 8 bits timer, Arduino uses it for tone()
 * We used the TIMER 1 for the interrupt routine of the driver, as it's 16 bits (and so it's more "smooother" to use, and we don't care the Servo library for the projector
 */

#include <Arduino.h>
#include <math.h>

#include "driver.h"
#include "planner.h"
#include "I2C.h"
#include "serialIO.h"
#include "settings.h"

driverState ds;

void driver_init(){
	memset(&ds, 0, sizeof(ds));										// Init the driver state with 0
	ds.zDistance = Z_DISTANCE;
	driver_interrupt_init();
	PORTB &= ~(1 << PB5);											// Configure pin 13 (led)
	DDRB |= (1 << PB5);
	serial_send_message("Driver initialisé.");

}

// This function set up the TIMER 1
/* Is sets up the TIMER 1 according to the settings (IC frequency and wanted update move frequency)
 * The Timer is set to CTC, that is generating an interrupt, and sets to 0 on compare match.
 * It can switch between no prescalling and a 1/8 prescalle, so the update frequency can be set between 16MHz and 30Hz
 * This could largely cover all case.
 * The channel B can be used for another feature if needed
 */
void driver_interrupt_init(){
	cli();															// Cancel interrupts during set up
	TCCR1A = 0;														// Initialisatin registers
	TCCR1B = 0;
	TCNT1 = 0;

	long isr_time = CLOCK_SPEED / ISR_FREQUENCY;

	if (isr_time > 0xffff){											// verifies the prescalling factor
		OCR1A = isr_time / 8;										// Prescale
		TCCR1B |= (1 << CS11);
	} else {
		OCR1A = isr_time;											// No prescale
		TCCR1B |= (1 << CS10);
	}

	TCCR1B |= (1 << WGM12);											// Set TIMER 1 on CTC mode
	TIMSK1 |= (1 << OCIE1A);										// Enables output compare A

	sei();															// Enable interrupt again

}

// The ISR drives the position calculation in realtime
ISR(TIMER1_COMPA_vect){
	moveBuffer *bf = planner_get_run_buffer();						// Get a pointer to the run buffer

	// Verifies that there is movement
	if ((ds.now[0] == ds.previous[0]) && (ds.now[1] == ds.previous[1])){
		ds.moving = 0;												// records the current state
		ds.watchdog++;												// The watchdog increments if there is no move
		PORTB &= ~(1 << PB5);										// Shut led when no move

	} else {
		ds.moving = 1;
		ds.watchdog = 0;											// It's set back to 0 each time there's a move
		PORTB |= (1 << PB5);										// Lit led when moving.



	}
	if (ds.watchdog > WATCHDOG_TIMER){								// If it overflows the limit value, the laser is cut
		ds.now[2] = 0;												// It's a security feature for it doesn't burn anything by staying immobile
	}

	for (int i=0; i<3; i++){										// records the last position before to update it
		ds.previous[i] = ds.now[i];
	}


	if (bf->active == 0 || bf->compute == 0){						// If the run buffer is not set or compute, escape the interrupt routine
		return;
	}

	/*
	if (bf->id == ds.percent_incr){									// Set a flag for sending percent each time a new percent has been insolate
		ds.percent_incr += ds.percent;
		ds.percent_flag == 1;
	}
	*/

	for (int i=0; i<3; i++){										// compute each of the axis (X, Y and laser)
		bf->now[i] += (double)bf->incr[i];							// compute the new position with the older one and the increment
		ds.now[i] = bf->now[i];										// Records the new position
	}

	if (bf->nowSteps >= bf->steps-1){								// If the number of steps of this move has been reach,

		for (int i=0; i<3; i++){									// Copy the goal position to the driverState
			ds.now[i] = bf->pos[i];									// Otherwise the coordinate approx could lead to drift
		}
		planner_set_next_buffer(2);									// Set the next run buffer
		planner_free_buffer(bf);									// Frees the buffer
	} else {
		bf->nowSteps++;
	}
}

driverState * driver_get_ds(){
	return &ds;
}

boolean driver_is_moving(){
	return ds.moving;
}

/*
void driver_set_size(long size){
	ds.size = size;
	ds.percent_incr = size/100;
	ds.percent = ds.percent_incr;
}
*/
volatile double * driver_get_position(){
	return ds.now;
}