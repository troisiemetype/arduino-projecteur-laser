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

 #include "circularbuffer.h"

CircularBuffer::CircularBuffer(uint16_t bufferSize){
	_size = bufferSize;
	_buffer = (char*)malloc(_size);
	CircularBuffer::empty();
 }

CircularBuffer::~CircularBuffer(void){
	if(_buffer){
		free(_buffer);
	}
}

char CircularBuffer::peek(void){
	char c = _buffer[tail];
	return c;
}

void CircularBuffer::set(char c){
	_buffer[head] = c;
	CircularBuffer::increment(&head);

}

char CircularBuffer::get(void){
	char c = _buffer[tail];
	CircularBuffer::increment(&tail);
	return c;

}

void CircularBuffer::increment(volatile uint16_t* value){
	*value = (*value + 1)%_size;
}

void CircularBuffer::empty(void){
 	memset(_buffer, 0, _size);
 	head = 0;
 	tail = 0;
}

uint16_t CircularBuffer::queue(void){
	int16_t queue = head - tail;
	if (queue < 0){
		queue += _size;
	}
	return queue;

}
