# MobaTools
Arduino library for model railroaders ( and maybe for others too ;)

The library uses Timer1 for all classes. There is no special function that has to be
called in the loop frequently. You can even use the delay() function in the loop while
servos and steppers are moving.

It contains 4 classes:

Servo8: 
Can control up to 16 servos. Compatible with arduino servo lib, but allows to control 
the speed of the servo.

Stepper4: 
A class to control unipolar stepper motors. The arduino sketch is not blocked while 
the stepper is moving. After setting a reference point, the stepper can be positioned 
absolutely just like a servo. But without the angle limitation of 0--180 degrees.

Softled: 
Allows easy softon / softoff of an led.

Eggtimer: 
Allows easy nonblocking timedelays in sketches.
