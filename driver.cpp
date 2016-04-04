// Arduino laser projector

// driver.cpp
/* This part of the program sends at a regular interval the positions to the galvos
   It fires an interrupt on a known interval,
   then at each interrupt it computes the new position value using what has been set in the buffer.
   Finally, these new values are sent to the external DAC trough I2C.
*/

#include <Arduino.h>

#include "driver.h"
#include "I2C.h"

void driver_init(){
	
}