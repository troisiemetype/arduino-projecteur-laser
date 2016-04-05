===============
Laser projector
===============

This program is intended to run a laser projector (like those used in music show) to expose photographic processes like cyanotype, dichromate gum, etc.
This laser projector features an arduino board, a power supply, two galvos and their drivers (X and Y), a laser and its driver.

It's at the very beggining of its developpement, so for now there is absolutely nothing that works.

Here is how the workflow should be:
* A program (to be written) transforms pictures into routes
* These routes are sent to the arduino trough serial link (via USB)
* The arduino records each new coordinate in a circular buffer
* It computes every position betwen the current position and the place to go (for X, Y and laser power)
* It also converts the cartesian positions that are sent into polar positions
* These position are sent to a routine run by an ISR (timer) that fire on a regular basis, according to the speed we want
* The ISR sends, at every step, a new coordinate to the galvos and laser driver.
* These coordinates are sent trough UART (I2C) to DACs, to transform binary data into variable tension
* Finally, the arduino sends back to the computer some datas about position