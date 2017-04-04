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

/* This program is intended to control a laser projector
 */

/*
 * Pin mapping:
 * Screen is connected on SDA and SCL pins, that are A4 and A5, respectively (pins 18 and 19)
 * Encoder is connected to pins 2 and 3, but doesn't use INT0 and INT1 interrupts.
 * Laser intensity is connected on pin 11.
 * It's driven by the TIMER2 channel A PWM, in fast PWM mode
 * (Non-onverting mode causes the output to be glimpsing when timer overflows,
 * causing laser to be always on.)
 */
//includes
#include <Adafruit_GFX.h>
#include <gfxfont.h>
#include <Adafruit_SSD1306.h>

#include <Fonts/FreeSerifItalic18pt7b.h>

Adafruit_SSD1306 display;

//Var declaration
byte pwmValue = 0;
byte pwmDisplay = 0;
bool light = 1;

int read = 0;
int previousRead = 0;

int channelA = 0;
int channelB = 0;
int prevChannelA = 0;
int prevChannelB = 0;


//setting up the program.
void setup(){

	//Frist thing to do is to set the PWM pin as output, and setting it to 0.
	//The laser can be hasardous, so it must be shut everytime we don't need it.
	//Plus, being on it could cause unwanted insolating on a picture.
	//In the main program, the direct access to register are preferred to speed up things.
	//Set pin 11 (port B3) as output
	DDRB |= (1 << 3);
	//Set it to 0;
	PORTB &= ~(0 << 3);

	//Set pin A0 and A1 as input
	DDRC &= ~((1 << 0) | (1 << 1));

	//Initialise PWM
	pwmInit();

	display.begin();
	display.clearDisplay();
	display.setTextColor(WHITE);
	display.setFont(&FreeSerifItalic18pt7b);
	display.setCursor(0, 25);
	display.print("L: ");
	display.print(pwmDisplay);
	display.print('%');
	display.display();

//	Serial.begin(115200);
}

void loop(){
	//update laser state.
	if(light){
		OCR2A = pwmValue;
	} else {
		OCR2A = 0;
	}

//	channelA = analogRead(A1);
//	Serial.println(channelA);

	//read the values from audio input.
	//abort loop if we need to cut the laser.

	//Read the value from pot.
	read = analogRead(A0);
	//read encoder
	if(read != previousRead){
		//If pot has been moved, change pwm value.
		previousRead = read;
		pwmValue = read / 4;
		pwmDisplay = (read) / 10.24;

		display.clearDisplay();
		display.setCursor(0, 25);
		display.print("L: ");
		display.print(pwmDisplay);
		display.print('%');
		display.display();
	//	Serial.println(read);
	//	Serial.println(pwmDisplay);
	//	Serial.println(pwmValue);
	//	Serial.println();
	}	
}

void pwmInit(){
	//cancel interrupts when setting up the timer.
	cli();

	//First set all registers to 0;
	TCCR2A = 0;
	TCCR2B = 0;
	TCNT2 = 0;
	OCR2A = 0;

	//Set PWM in phase correct mode, no prescaller, clear on up-couting, set on down-counting.
	TCCR2A |= (1 << COM2A1) | (1 << WGM21) | (1 << WGM20);		//Clear OC2A up-counting; fast PWM.
	TCCR2B |= (1 << CS20);										//No prescaller.

	//Enable interrupt again.
	sei();
}