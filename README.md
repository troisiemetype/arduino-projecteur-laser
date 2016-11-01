===============
Laser projector
===============

This program is intended to run a laser projector (like those used in music show) to expose photographic processes like cyanotype, dichromate gum, etc.
This laser projector features an arduino board, a power supply, two galvos and their drivers (X and Y), a laser and its driver.

This part of the project works on an Uno board. A this board's power is not enough to run the program, further developpment will use an Arduino Due. That said, the Uno can process the data with some limitations.

Here is how it works:

An image is opened by the program running on the computer. See the C-projecteur-laser for this program or python-projecteur-laser for the first version, now not supported.

That image is processed, and a set of coordinates is generated, to be sent to the Arduino.

When a set of coordinate (i.e. a position (X and Y), a laser intensity, a move mode and according speed) is received, the serial part of the program copies these raw coordinates into a buffer. Then this buffer is parsed and coordinates are copied to the planner buffer.

Once a planner buffer is populated, coordinates are processed. The delta between actual position and goal position is found, and given the move type and the intended move speed, an increment for each axis (X, Y and laser intensity) is computed and the number of increment needed is calculated too.

Once done, this buffer is available for the driver to use it. There is a driver buffer that prepares future positions. The TIMER ISR sets a flag, and on each tick the driver sends a new coordinate to the DAC, trough TWI link.

The functions are called by the main loop, with a priority. First is called the driver update function, that verifies if the current position needs an update.
Then the driver planner computes if needed new future positions.
Then the planner computes positions received from serial and not processed yet.
Finally, the serial empties the RX buffer, and then parses new incoming sets of data.

Each of this functions returns a state. If the functions has done nothing the following one is called (i.e. if the planner had nothing to update, the driver planner is called). If it has done seomthing the main loop starts again from beginning, and skip the following calls to function.

here is the limitation with Uno: it is not fast enough to run the main loop at a sufiscient speed, so updating coordinates in mode 0 (a coordinate = a position) is ok, but mode 1 (a coordinate = a route between two positions, with a move speed) is not.