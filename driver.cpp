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
#include "I2C.h"
#include "serialIO.h"

void driver_init(){
	serial_send_message("driver initialisé");

}

void driver_interrupt_init(){

}