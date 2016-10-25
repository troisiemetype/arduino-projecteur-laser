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
 
// driver.cpp
/* This part of the program compute the position at a regular interval, and send it to galvos.
   It fires an interrupt on a known interval,
   then at each interrupt it computes the new position value using what has been set in the buffer.
   Finally, these new values are sent to the external DAC trough I2C.
*/

/* There are three timer on the Atmel 328p the uno/nano is build on: TIMER0, TIMER1, TIMER2
 * TIMER0 is a 8 bits timer. Arduino uses it for delay(), millis() and micros() functions
 * TIMER1 is a 16 bits timer, Arduino uses it for the Servo library
 * TIMER2 is a 8 bits timer, Arduino uses it for tone()
 * We used the TIMER 1 for the interrupt routine of the driver, as it's 16 bits
 * (and so it's more "smooother" to use, and we don't care the Servo library for the projector)
 * We also use TIMER 2 to generate the PWM output of the laser intensity.
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

	ds.beat_count = 0;												// heartbeat values init.
	ds.beat_max_idle = ISR_FREQUENCY / BEAT_FREQUENCY;				// Heartbeat duration.
	ds.beat_max_driving = ds.beat_max_idle / 4;

	ds.beat_max = ds.beat_max_idle;

	DDRB |= (1 << PB3) | (1 << PB5);								// Set pins 11 (laser) and 13 (led) as outputs
	PORTB &= ~(1 << PB5);											// Unset pin 13
	
	driver_interrupt_init();

	serial_send_message("Driver initialisé.");

}

// This function set up the TIMER 1 and TIMER 2
/* Is sets up the TIMER 1 according to the settings (IC frequency and wanted update move frequency)
 * The Timer is set to CTC, that is generating an interrupt, and sets to 0 on compare match.
 * It can switch between no prescalling and a 1/8 prescalle, so the update frequency can be set between 16MHz and 30Hz
 * This could largely cover all case.
 *
 * 
 */
void driver_interrupt_init(){
	cli();															// Cancel interrupts during set up

	// Setup for TIMER 1
	TCCR1A = 0;														// Initialisation registers
	TCCR1B = 0;
	TCNT1 = 0;

	long isr_time = CLOCK_SPEED / ISR_FREQUENCY;


	if (isr_time > 0xffff){											// verifies the prescalling factor
		OCR1A = isr_time / 8;										// Prescale
		TCCR1B |= (1 << CS11);
		ds.beat_max_idle /= 8;
		ds.beat_max_driving /= 8;
	} else {
		OCR1A = isr_time;											// No prescale
		TCCR1B |= (1 << CS10);
	}

	TCCR1B |= (1 << WGM12);											// Set TIMER 1 on CTC mode
	TIMSK1 |= (1 << OCIE1A);										// Enables output compare A

	// Setup for TIMER 2
	
	TCCR2A = 0;														// Initialisation registers
	TCCR2B = 0;
	TCNT2 = 0;
	OCR2A = 0;
// Problem with fast PWM: When OCR2A is set to 0, there is a narrow spike at the timer overflow,
// so the output is never totally shut, that causes the laser to be always on.
//	TCCR2A |= (1 << COM2A1) | (1 << WGM21) | (1 << WGM20);			// Clear OC2A on compare match, fast PWM
//	TCCR2B |= (1 << CS22) | (1 << CS21);							// prescaller 256
	TCCR2A |= (1 << COM2A1) | (1 << WGM20);							// Clear OC2A on up-counting compare match, phase correct PWM
	TCCR2B |= (1 << CS21);											// no prescaller
	

	sei();															// Enable interrupt again

}

// The ISR sets a flag when position needs to be updated.
ISR(TIMER1_COMPA_vect){
	// Debug: ISR time mesure
	long debut = TCNT1;

	// Heartbeat calculation.
	ds.beat_count++;

	// Verifies that there is movement
	if ((ds.now[0] == ds.previous[0]) && (ds.now[1] == ds.previous[1])){
		ds.moving = 0;												// records the current state
		ds.watchdog++;												// The watchdog increments if there is no move

	} else {
		ds.laser_enable = 1;
		ds.moving = 1;
		ds.watchdog = 0;											// It's set back to 0 each time there's a move.
	}

	if (ds.watchdog > WATCHDOG_TIMER && ds.laser_enable == 1){								// If it overflows the limit value, the laser is cut
//		ds.now[2] = 0;												// It's a security feature for it doesn't burn anything by staying immobile
		ds.laser_enable = 0;
		ds.update = 1;
	}

	ds.update = 1;

	//debug: ISR time mesure
	long fin = TCNT1;

	ds.isrLength = fin - debut;

//	_serial_append_value(micros()-debut);
//	_serial_append_nl();
}

void driver_heartbeat(){
	if (ds.beat_count >= ds.beat_max){
		ds.beat_count = 0;
		bool state = (PORTB & (1 << PB5));
		if (state == 1){
			PORTB &= ~(1 << PB5);
		} else {
			PORTB |= (1 << PB5);
		}
	}

	if (ds.moving){
		ds.beat_max = ds.beat_max_driving;							// Led blink faster when moving.
	} else {
		ds.beat_max = ds.beat_max_idle;								// Led blink slow when idle.
	}

}

bool driver_prepare_pos(){
	if (!ds.compute){
		return 0;
	}
	//Start of the ISR block
	moveBuffer *bf = planner_get_run_buffer();						// Get a pointer to the run buffer

	for (int i=0; i<3; i++){										// records the last position before to update it
		ds.previous[i] = ds.now[i];
	}


	if (bf->active == 0 || bf->compute == 0){						// If the run buffer is not set or compute, escape the interrupt routine
		return;
	}

	for (int i=0; i<3; i++){										// compute each of the axis (X, Y and laser)
		bf->now[i] += bf->incr[i];									// compute the new position with the older one and the increment
		ds.now[i] = bf->now[i];										// Records the new position
	}

	ds.update = 1;													// Set a flag to update pos. I2C cannot be called from an ISR.
	
	if (bf->nowSteps >= bf->steps-1){								// If the number of steps of this move has been reach,

		for (int i=0; i<3; i++){									// Copy the goal position to the driverState
			ds.now[i] = bf->pos[i];									// Otherwise the coordinate approx could lead to drift
		}
		planner_set_next_buffer(2);									// Set the next run buffer
		planner_free_buffer(bf);									// Frees the buffer
	} else {
		bf->nowSteps++;
	}
	//End of the ISR block
	ds.compute = 0;
	return 1;
}

bool driver_update_pos(){
	if (ds.update == 0 || ds.compute == 1){
		return 0;
	}
	serial_send_pair("ticks ISR", ds.isrLength);
	//test X value for modification.
	if (ds.now[0] != ds.previous[0]){
		unsigned int pos = ds.now[0] + DRIVER_OFFSET;
		I2C_write('X', pos);
	}
	//test Y value for modification.
	if (ds.now[1] != ds.previous[1]){
		unsigned int pos = ds.now[1] + DRIVER_OFFSET;
		I2C_write('Y', pos);
	}
	I2C_update();
	ds.update = 0;
	ds.compute = 1;

	if(ds.watchdog > WATCHDOG_TIMER){
		OCR2A = 0;
	} else {
		OCR2A = ds.now[2];
	}

	return 1;
//	OCR2A = 10;												// Temp value for laser testing

}

driverState * driver_get_ds(){
	return &ds;
}

boolean driver_is_moving(){
	return ds.moving;
}

volatile double * driver_get_position(){
	return ds.now;
}