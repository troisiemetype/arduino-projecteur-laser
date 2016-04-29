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
 
// I2C.h
// This part of the program sends and receives datas trough I2C.
// Above setting sare intended for an AD5665R 16 bits 4 channels DAC.

#ifndef I2C_H
#define I2C_H

//define AD5665 commands
#define AD5665_WRITE_N			0x0
#define AD5665_UPDATE_N			0x1
#define AD5665_WRITE_N_UPDATE_A	0x2
#define AD5665_WRITE_N_UPDATE_N	0x3
#define AD5665_POWER			0x4
#define AD5665_RESET			0x5
#define AD5665_LDAC_SETUP		0x6
#define AD5665_INTERNAL_REF		0x7


//define AD5665 adresses
#define AD5665_DAC_A 	0x0
#define AD5665_DAC_B	0x1
#define AD5665_DAC_C	0x2
#define AD5665_DAC_D	0x3
#define AD5665_DAC_ALL	0x7


void I2C_init();
void I2C_write(char axe, int pos);
void I2C_update();

#endif