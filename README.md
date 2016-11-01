===============
Laser projector
===============

This program is intended to run a laser projector (like those used in music show) to expose photographic processes like cyanotype, dichromate gum, etc.
This laser projector features an arduino board, a power supply, two galvos and their drivers (X and Y), a laser and its driver.

The program works for now, look at the Uno branch to get a working solution on the Arduino Uno board. It's not powerfull enough to completely work: coordinates can pe processed, but there is not enough rpocessor power to compute and update routes.

For that reason the program will now be developped to run on an Arduino Due.

Here is how it works:
* A program running on a computer computes a set of cordinates (X, Y, laser intensity, and a move speed) for each pixel of the picture. The positions are computed given the size intended, the distance between projector and image, intended speed. The cartsian positions are tranformed into polar positions, as laser go trough mirrors.
* These coordinates are sent to the arduino trough serial link (via USB).
* The arduino records each new coordinate in a buffer.
* It computes every position betwen the current position and the place to go (for X, Y and laser power).
* These position are sent to a routine run by an ISR (timer) that fire on a regular basis.
* The ISR sends, at every step, a new coordinate to the galvos and laser driver.
* These coordinates are sent trough UART (I2C) to DACs, to transform binary data into variable tension.
* Finally, the arduino sends back to the computer some datas about position.

Diving a bit more into code organisation, there are three main parts:
* Serial deals with incomming and outgoing data.
* Planner records position, and compute deltas between positions.
* Driver compute instant positions, and update them when the ISR fires.

The main loop is a hierachical state machine, that calls the functions with priority order. If a function process some data, the main loop is started again from beginning, else it goes to the following one, that is procesed the same way.

Here are the three main calls, and what they do.
* Driver_main() verifies if there were an ISR flag set. If yes, it updates the new position, then returns (and so the main loop starts again). If no, it processes the next position so as it will be ready when update is nedded. It then returns, asking the function to be enterred again on next main loop iteration.
* Planner_main() verifies if there are new coordinates that have been sent from serial, and not processed yet. If yes it's processed, then returns to the main loop, asking to enter again if there are other buffer to process.
* Serial_main() verifies if there are data in the RX buffer. If yes they are copied into a temporary buffer before to be parsed, so that the RX buffer stays as empty as possible. Then the temporary buffer is parsed, and the coordinates sent to the planner buffer.