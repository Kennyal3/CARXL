# CARXL
PID control (minus the "I") for steering assembly that uses a  potentiometer and DC motor.

OS: Ubuntu 16.04
Arduino 1.8 .ino.
Runs on Teensy 3.5.
Additional libraries required.
Uses Joy topic from ROS Kinetic. The Teensy subcribes to the ROS topic "/joy" via "rosserial_python serial_node.py".

Hardware:
Logitech USB game controller (joypad)
Sabertooth 2x25 Motor Driver (using packetized serial mode)
Teensy 3.5 micro controller
A big RC car named CARXL that can seat a small 31.75 kg human... or monkey. Or possibly a chilled, well behaved medium sized dog. Whatever your preference may be. 

The steering machanism is rack & pinion, 24-26 V DC motor attached to a 1 K Ohm variable resistor (pot).
The pot is connected to the Teensy... 10bit ADC, 3.6 V with pull-up enable. Pull up is probably not neccessary but certainly doesn't hurt and may even help a little with signal bounce or filtering.

Known Isues: There is a slight delay, roughly 250 ms from the gamepad to the motor. Not too bad when it comes to driving the vehicle but would be nice to have a quicker response. Attempted to increase the baud rate. Setting the Sabertooth baud rate to anything other than 9600 and it stops working. Further research or advice is needed. Other potential issue could be the amount of over head from joy to motor. I think there is a way to time the signal from start to end but I'm not entirely sure how to do that. Any advice or suggestions are welcome. I am an undergrad EE and this has been an enormous learning process.
