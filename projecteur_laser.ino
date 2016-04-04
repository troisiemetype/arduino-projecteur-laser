// Arduino laser projector

/* This program is intended to control a laser projector
 * The projector is controled by positions sent by a programm sent via UART link
 */
 /*
  * This is the main program that runs the projector.
  * It does the initial setup, then runs the main loop, calling each part one after the other
  */


//includes

#include "planner.h"
#include "driver.h"
#include "I2C.h"
#include "serialIO.h"
#include "settings.h"

void setup(){

	//calling the init functions of all program parts
	serial_init();
	I2C_init();
	planner_init();
	driver_init();

	#define DISPATCH(func) if (func == 0) return

}

void loop(){
	
}