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
#include "system.h"

driverState ds;

driverBufferPool dbp;

void driver_init(){
	memset(&ds, 0, sizeof(ds));										// Init the driver state with 0

	ds.state = DRIVER_IDLE;

	ds.beat_count = 0;												// heartbeat values init.
	ds.beat_max_idle = ISR_FREQUENCY / BEAT_FREQUENCY;				// Heartbeat duration.
	ds.beat_max_driving = ds.beat_max_idle / 4;

	ds.beat_max = ds.beat_max_idle;

	DDRB |= (1 << PB3) | (1 << PB5);								// Set pins 11 (laser) and 13 (led) as outputs
	PORTB &= ~(1 << PB5);											// Unset pin 13
	
	driver_init_buffer();
	driver_interrupt_init();

//	serial_send_message(F("Driver initialisé."));

}

void driver_init_buffer(){
	driverBuffer *pv;															// creates a pointer to buffer

	memset(&dbp, 0, sizeof(dbp));											// Clears the buffer pool

	dbp.run = &dbp.pool[0];												// sets pointers to write on the first item in the pool
	dbp.queue = &dbp.pool[0];												// sets pointers to write on the first item in the pool
	dbp.available = DRIVER_POOL_SIZE;										// Sets the number of available buffers

	pv = &dbp.pool[DRIVER_POOL_SIZE-1];										// Sets the pv pointer on the last buffer of the pool

	for (byte i=0; i<DRIVER_POOL_SIZE; i++){								// For each buffer in the pool,
		dbp.pool[i].pv = pv;												// Sets a pointer to the previous buffer
		if (i<DRIVER_POOL_SIZE-1){											// And to the next buffer								
			dbp.pool[i].nx = &dbp.pool[i+1];								// pointer is to the next buffer
		} else {															// If last iteration, next buffer is #0
			dbp.pool[i].nx = &dbp.pool[0];
		}
		pv = &dbp.pool[i];													// Prepare next iteration: current buffer is the previous of the next.
	}

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
//	long debut = TCNT1;

	// Heartbeat calculation.
	ds.beat_count++;

	bit_true(ds.state, DRIVER_UPDATE_POS);

	//debug: ISR time mesure
//	ds.isrLength = TCNT1 - debut;

}


int driver_main(){
	if (ds.state == DRIVER_IDLE){
		return STATE_OK;
	}

	if (planner_computed()){
		bit_true(ds.state, DRIVER_COMPUTE_BUF);
	}
	if (ds.state & DRIVER_UPDATE_POS){
		int state = driver_update_pos();
		driver_laser();
		return state;
	}

	if (ds.state & DRIVER_COMPUTE_BUF){
		return driver_plan_pos();
	}

}

int driver_plan_pos(){
	if (dbp.available < 1){
		return STATE_NO_OP;
	}
//	long debut = micros();
	//Create pointers to the current planner buffer.
	plannerBuffer *bf = planner_get_run_buffer();

	//Verify there is a move to compute in the planner, else return.
	if (bf->active == 0 || bf->compute == 0){
		bit_false(ds.state, DRIVER_COMPUTE_BUF);
		return STATE_NO_OP;
	}
//	_serial_append_string("driver plan");
//	_serial_append_nl();

//	serial_send_message("driver plan");

	//Get pointer to the driver buffer
	driverBuffer *db = dbp.queue;

	//compute each of the axis.
	for (int i=0; i<3; i++){	
		//Compute the new position, then record it.
		bf->current[i] += bf->incr[i];
		db->pos[i] = bf->current[i];
	}

	//If the max number of steps for this move has been reach, 
	if (bf->nowSteps >= bf->steps-1){
		//Copy the goal position instead of the compute position, to minimize drift due to roudings.
		for (int i=0; i<3; i++){
			db->pos[i] = bf->pos[i];
		}
		//Set the planner buffer to next, then free it.
		planner_set_next_buffer(2);
		planner_free_buffer(bf);

	} else {
		bf->nowSteps++;
	}

	//prepare the next movement.
	dbp.queue = db->nx;
	dbp.available --;

//	_serial_append_string("driver plan");
//	_serial_append_nl();
//	_serial_append_value(micros() - debut);
//	_serial_append_nl();

	if (dbp.available < DRIVER_POOL_SIZE){
		bit_true(ds.state, DRIVER_COMPUTE_BUF);
		return STATE_OK;
	} else {
		bit_false(ds.state, DRIVER_COMPUTE_BUF);
		return STATE_OK;
	}

}

int driver_update_pos(){

//	long debut = micros();
	// Verifies that there is movement
	if ((ds.now[0] == ds.previous[0]) && (ds.now[1] == ds.previous[1])){
		ds.moving = 0;
	}
	//Record the last sent position before to update.
	for (int i=0; i<3; i++){
		ds.previous[i] = ds.now[i];
	}

	//Debug: display the length of the driver isr.
	//Check to uncomment mesurings in the ISR, and var def in driver.h
//	serial_send_pair("ticks ISR:", ds.isrLength);


	//Verifies there are positions to update.
	if (dbp.available >= DRIVER_POOL_SIZE){
		bit_false(ds.state, DRIVER_UPDATE_POS);
//		_serial_append_string("update quit");
//		_serial_append_nl();
		return STATE_NO_OP;
	}

//	_serial_append_string("driver update pos");
//	_serial_append_nl();

	//Get the current run buffer
	driverBuffer *db = dbp.run;

	//Send the new values to the I2C
	if (db->pos[0] != ds.previous[0]){
		ds.moving = 1;
		unsigned int pos = db->pos[0] + DRIVER_OFFSET;
		I2C_write('X', pos);
	}
 
	if (db->pos[1] != ds.previous[1]){
		ds.moving = 1;
		unsigned int pos = db->pos[1] + DRIVER_OFFSET;
		I2C_write('Y', pos);
	}

	I2C_update();

	//Record the new current position.
	for (int i=0; i<3; i++){
		ds.now[i] = db->pos[i];
	}

	//Prepare the next pos.
	dbp.run = db->nx;
	dbp.available++;
//	_serial_append_string("driver update");
//	_serial_append_nl();
//	_serial_append_value(micros() - debut);
//	_serial_append_nl();

	bit_false(ds.state, DRIVER_UPDATE_POS);
	return STATE_OK;

}

//update the laser output.
//Out of the update function because of the need to cut it when not moving.
void driver_laser(){
	if(ds.moving){
//		_serial_append_string("laser");
//		_serial_append_nl();
		OCR2A = ds.now[2];
	} else {
		OCR2A = 0;
	}
}

double * driver_get_position(){
	return ds.now;
}

// set or unset the led.
void driver_heartbeat(){
//	long debut = micros();
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

//	_serial_append_value(micros() - debut);
//	_serial_append_nl();

}
