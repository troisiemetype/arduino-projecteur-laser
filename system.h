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
 
// system.h
// this part of the program deals with:
// Real time processing, and prioriry of events and functions.
// 



#ifndef SYSTEM_H
#define SYSTEM_H


#include <Arduino.h>

#include "driver.h"
#include "planner.h"
#include "I2C.h"
#include "io.h"
#include "settings.h"
#include "system.h"



#define bit(n)						(1 << n)

#define bit_istrue(x, mask)			((x & mask) != 0)
#define bit_isfalse(x, mask)		((x & mask) == 0)			
#define bit_true(x, mask)			(x) |= (mask)
#define bit_false(x, mask)			(x) &= ~(mask)

#define SYSTEM_IDLE					0

#define STATE_OK					0				//Function returns after doing its job
#define STATE_ENTER_AGAIN			1				//Function needs to enter again
#define STATE_NO_OP					2				//Function didn't do any work

#define DRIVER_IDLE					0				//Default state of the driver. Nothing to do.
#define DRIVER_UPDATE_POS			bit(0)			//Position update needed
#define DRIVER_COMPUTE_BUF			bit(1)			//Compute positions buffer

#define PLANNER_IDLE				0				//Default state. Nothing to do.
#define PLANNER_COMPUTE_BUF			bit(0)			//Buffer parsing needed.

#define SERIAL_IDLE					0
#define SERIAL_RX					bit(0)
#define SERIAL_PARSE				bit(1)

void system_init();
void system_main();
#endif