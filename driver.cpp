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

#include "driver.h"

driverState ds;

driverBufferPool dbp;

void driver_init(){
	memset(&ds, 0, sizeof(ds));										// Init the driver state with 0

	ds.state = DRIVER_IDLE;

	ds.beat_count = 0;												// heartbeat values init.
	ds.beat_max_idle = ISR_FREQUENCY / 2 / BEAT_FREQUENCY;				// Heartbeat duration.
	ds.beat_max_driving = ds.beat_max_idle / 4;

	ds.beat_max = ds.beat_max_idle;

	ds.ledState = 0;

//TODO: find the right port for laser PWM.
//	DDRB |= (1 << PB3) | (1 << PB5);								// Set pins 11 (laser) and 13 (led) as outputs
//	PORTB &= ~(1 << PB5);											// Unset pin 13
	pinMode(LED, OUTPUT);
	
	_driver_buffer_init();
	_driver_timer_init();
	_driver_pwm_init();

//	io_send_message(F("Driver initialisé."));

}

void _driver_buffer_init(){
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

// This function set up the timer for main interrupt
/* Is sets up the TIMER 1 according to the settings (IC frequency and wanted update move frequency)
 * The Timer is set to CTC, that is generating an interrupt, and sets to 0 on compare match.
 * It can switch between no prescalling and a 1/8 prescalle, so the update frequency can be set between 16MHz and 30Hz
 * This could largely cover all case.
 *
 * 
 */
void _driver_timer_init(){

	pmc_set_writeprotect(false);
	pmc_enable_periph_clk(TIMER_IRQ);

	TC_Configure(TIMER_N, TIMER_CHANNEL,
		TC_CMR_WAVE |
		TC_CMR_WAVSEL_UP_RC |
		TC_CMR_TCCLKS_TIMER_CLOCK1);
	uint32_t rc = CLOCK_FREQUENCY / 2 / ISR_FREQUENCY;
	TC_SetRC(TIMER_N, TIMER_CHANNEL, rc);

	TIMER_N->TC_CHANNEL[TIMER_CHANNEL].TC_IDR = ~0;
	TIMER_N->TC_CHANNEL[TIMER_CHANNEL].TC_IER = TC_IER_CPCS;

	TC_Start(TIMER_N, TIMER_CHANNEL);

	NVIC_EnableIRQ(TIMER_IRQ);
}

void _driver_pwm_init(){

	pmc_set_writeprotect(false);
	pmc_enable_periph_clk(PWM_IRQn);
//	pmc_enable_periph_clk(PIOC);

	PWMC_ConfigureClocks(0, 0, CLOCK_FREQUENCY);

	//Set pin 9 for the laser
	PIO_Configure(PIOC, PIO_PERIPH_B, PIO_PC21B_PWML4, PIO_DEFAULT);

	//Configure PWM, channel 4, prescaller 8
	//(values bellow a 8 prescaller lead to incomplete burning: laser driver is not fast enough)
	PWMC_ConfigureChannel(PWM, 4, PWM_CMR_CPRE_MCK_DIV_8, 0, 0);
	//Configure for counting up to 255, init at 0
	PWMC_SetPeriod(PWM, 4, 255);
	PWMC_SetDutyCycle(PWM, 4, 0);

	PWMC_EnableChannel(PWM, 4);
}

// The ISR sets a flag when position needs to be updated.
void TC0_Handler(void){

	TC_GetStatus(TIMER_N, TIMER_CHANNEL);

	//Increment beatCount, update led if needed.
	ds.beat_count++;
	if (ds.beat_count >= ds.beat_max){
		ds.beat_count = 0;
		digitalWrite(LED, ds.ledState = !ds.ledState);

	}


	bit_true(ds.state, DRIVER_UPDATE_POS);
}


int driver_main(){
//	debug_message("driver main");
	if (ds.state == DRIVER_IDLE){
		return STATE_OK;
	}

	if (planner_computed()){
		bit_true(ds.state, DRIVER_COMPUTE_BUF);
	}

	if (ds.state & DRIVER_UPDATE_POS){
		int state = _driver_update_pos();
		_driver_laser();
		return state;
	}

	if (ds.state & DRIVER_COMPUTE_BUF){
		return _driver_plan_pos();
	}

}

int _driver_plan_pos(){
	if (dbp.available < 1){
		return STATE_NO_OP;
	}
//	long debut = micros();
	//Get pointer to the current planner buffer.
	plannerBuffer *bf = planner_get_run_buffer();

	//Verify there is a move to compute in the planner, else return.
	if (bf->active == 0 || bf->compute == 0){
		bit_false(ds.state, DRIVER_COMPUTE_BUF);
		return STATE_NO_OP;
	}
//	debug_pair("plan move", bf->id);

//	debug_message("driver plan");

	//Get pointer to the driver buffer
	driverBuffer *db = dbp.queue;

	//compute each of the axis.
	for (int i=0; i<3; i++){	
		//Compute the new position, then record it.
		bf->current[i] += bf->incr[i];
		db->pos[i] = bf->current[i];
	}

	if (bf->mode == 1){
		db->pause = bf->speed;
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
//		debug_value(db->pos[0] >> 8);

	} else {
		bf->nowSteps++;
	}

	//prepare the next movement.
	dbp.queue = db->nx;
	dbp.available --;

//	debug_message("driver plan");
//	debug_value(micros() - debut);

	if (dbp.available < DRIVER_POOL_SIZE){
		bit_true(ds.state, DRIVER_COMPUTE_BUF);
		return STATE_OK;
	} else {
		bit_false(ds.state, DRIVER_COMPUTE_BUF);
		return STATE_OK;
	}

}

int _driver_update_pos(){

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
//	debug_pair("ticks ISR:", ds.isrLength);

	//Cut the laser before to update pos.
//	PWMC_SetDutyCycle(PWM, 4, 0);

	I2C_update();

	//Verifies there are positions to update.
	if (dbp.available >= DRIVER_POOL_SIZE){
		bit_false(ds.state, DRIVER_UPDATE_POS);
//		debug_message("update quit");
		return STATE_NO_OP;
	}

//	debug_message("driver update pos");

	//Get the current run buffer
	driverBuffer *db = dbp.run;

	//Send the new values to the I2C
	if (db->pos[0] != ds.previous[0]){
		ds.moving = 1;
		int pos = (db->pos[0]);
		if(INVERT_X){pos = -pos;}
		pos += DRIVER_OFFSET;
		I2C_write('X', pos);
	}
 
	if (db->pos[1] != ds.previous[1]){
		ds.moving = 1;
		int pos = (db->pos[1]);
		if(INVERT_Y){pos = -pos;}
		pos += DRIVER_OFFSET;
		I2C_write('Y', pos);
	}


	//Record the new current position.
	for (int i=0; i<3; i++){
		ds.now[i] = db->pos[i];
	}

	//Prepare the next pos.
	if (ds.pause >= db->pause){
		db->pause = 0;
		ds.pause = 0;
		dbp.run = db->nx;
		dbp.available++;
	} else {
		ds.pause++;
	}
//	debug_message("driver update");
//	debug_value(micros() - debut);

	bit_false(ds.state, DRIVER_UPDATE_POS);
	return STATE_OK;

}

//update the laser output.
//Out of the update function because of the need to cut it when not moving.
void _driver_laser(){
//	if(dbp.available >= DRIVER_POOL_SIZE){
	if(!ds.moving && ds.pause == 0){
		PWMC_SetDutyCycle(PWM, 4, 0);
	} else {
		PWMC_SetDutyCycle(PWM, 4, ds.now[2]);
	}
}

long * driver_get_position(){
	return ds.now;
}

// set or unset the led.
void driver_heartbeat(){
//	long debut = micros();
	if (ds.moving){
		ds.beat_max = ds.beat_max_driving;							// Led blink faster when moving.
	} else {
		ds.beat_max = ds.beat_max_idle;								// Led blink slow when idle.
	}

//	debug_value(micros() - debut);
}
