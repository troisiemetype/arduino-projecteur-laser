// Arduino laser projector

/* This program is intended to control a laser projector
 * The projector is controled by positions sent by a programm sent via UART link
 */
 /*
  * This is the main program that runs the projector.
  * It does the initial setup by calling the dedicated sub-program initialize functions,
  * then runs the main loop, calling each part one after the other.
  * 
  * The program is composed of these modules: 
  * Serial: Set and use the serial UART data flow, get data from PC.
  * Planner: populates buffer with coordinates from the data received.
  * Driver: Use an interrupt to update position on a regular basis.
  * I2C. Transmit position from the driver to DAC trough I2C.
  * Settings. Store general settings. Settings that are dedicated to a sub_program are stored in thier respectives header files.
  */


//includes
#include "settings.h"
#include "text_fr.h"
#include "serialIO.h"
#include "I2C.h"
#include "planner.h"
#include "driver.h"

long temps = 0;
long temps_prec = 0;

void setup(){

	//calling the init functions of all program parts
	serial_init();
	I2C_init();
	planner_init();
	driver_init();

	serial_send_message("Projecteur initialisé.");
}

void loop(){
	//Look for available data
	serial_get_data();
	//Populates buffer if needed.
	planner_plan_move();
	/*
	temps = micros();
	_serial_append_value(temps - temps_prec);
	_serial_append_nl();
	temps_prec = temps;
	*/
	
}