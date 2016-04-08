// Arduino laser projector

// driver.h
/* This part of the program sends at a regular interval the positions to the galvos
   It fires an interrupt on a known interval,
   then at each interrupt it computes the new position value using what has been set in the buffer.
   Finally, these new values are sent to the external DAC trough I2C.
*/

#ifndef DRIVER_H
#define DRIVER_H

void driver_init();
void driver_interrupt_init();
ISR(TIMER1_COMPA_vect);
#endif