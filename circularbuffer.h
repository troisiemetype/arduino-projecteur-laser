// Ring buffer
/*
 * This class is a circular buffer, that can be used for TX/RX, for example..
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

 #ifndef CIRCULAR_BUFFER_H
 #define CIRCULAR_BUFFER_H

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

class CircularBuffer{
public:
	CircularBuffer(uint16_t size);
	~CircularBuffer(void);
	char peek(void);
	void set(char c);
	char get(void);
	void increment(volatile uint16_t *);
	void empty(void);
	uint16_t queue(void);

public:
	volatile uint16_t head;
	volatile uint16_t tail;

protected:
	char *_buffer;
	uint16_t _size;

};

 

 #endif