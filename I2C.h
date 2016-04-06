// Arduino laser projector

// I2C.h
// This part of the program sends and receives datas trough I2C.

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