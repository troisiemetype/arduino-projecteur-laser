// Arduino laser projector

// serialIO.cpp
// this part of the program deals with:
// serial input: what is sent to the board, verifies that datas are correctly written, and populates the movement buffers
// serial output: send back infos about position, so the program can know how much of the pattern has already be done


#include <Arduino.h>
#include "planner.h"
#include "serialIO.h"
#include "settings.h"

void serial_init(){
	Serial.begin(115200, SERIAL_CONFIG);
}