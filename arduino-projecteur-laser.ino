// Arduino laser projector

/* This program is intended to control a laser projector
 * The projector is controled by positions sent by a programm sent via UART link
 */
 /*
  * This is the main program that runs the projector.
  * It does the initial setup, then runs the main loop, calling each part one after the other
  */


//includes
#include "settings.h"
#include "text_fr.h"
#include "serialIO.h"
#include "I2C.h"
#include "planner.h"
#include "driver.h"

long temps;
long temps_prec;

void setup(){

	//calling the init functions of all program parts
	serial_init();
	I2C_init();
	planner_init();
	driver_init();

	serial_send_message("Projecteur initialisé.");
}

#define DISPATCH(func) if (func == 0) return

void loop(){
	serial_get_data();
//	serial_write_data();
	planner_plan_move();
/*	temps = micros();
	serial_send_pair("boucle", temps - temps_prec);
	temps_prec = temps;
	serial_send_pair("TCNT1", TCNT1);
	*/
}