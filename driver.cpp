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
 * We used the TIMER 1 for the interrupt routine of the driver, as it's 16 bits, and we don't care the Servo library for the projector
 */

#include <Arduino.h>

#include "driver.h"
#include "planner.h"
#include "I2C.h"
#include "serialIO.h"
#include "settings.h"

double now[3];													// Stores the current position

void driver_init(){
	driver_interrupt_init();
	serial_send_message("driver initialisé");

}

// This function set up the TIMER 1
void driver_interrupt_init(){
	cli();														// Cancel interrupts during set up
	TCCR1A = 0;													// Initialisatin registers
	TCCR1B = 0;
	TCNT1 = 0;

	long isr_time = CLOCK_SPEED / ISR_FREQUENCY;

	if (isr_time > 0xffff){										// verifies the prescalling factor
		OCR1A = isr_time / 8;									// Prescale
		TCCR1B |= (1 << CS11);
	} else {
		OCR1A = isr_time;										// No prescale
		TCCR1B |= (1 << CS10);
	}

	TCCR1B |= (1 << WGM12);										// Set TIMER 1 on CTC mode
	TIMSK1 |= (1 << OCIE1A);									// Enables output compare A

	sei();														// Enable interrupt again

}

// The ISR drives the position calculation in realtime
ISR(TIMER1_COMPA_vect){
	moveBuffer *bf = planner_get_run_buffer();
	if (bf->active == 0){
		return;
	}
	for (int i=0; i<3; i++){
		now[i] = bf->now[i];
		bf->now[i] = (double)now[i] + (double)bf->incr[i];
	}
	
	if (bf->nowSteps >= bf->steps){

		planner_free_buffer(bf);
	} else {
		bf->nowSteps++;
	}


}