MobaTools
=========
### Arduino library for model railroaders ( and maybe for others too 😉 )

**Latest changes:**

| Version |  Release Date  | Description
| ------- |  ------------  | -----------
| v1.1.0beta  | not released yet | acceleration/deceleration for steppermotors is now possible


## Functionality

The lib contains 4 classes:

### Servo8: 
Can control up to 16 servos. Compatible with arduino servo lib, but allows to control 
the speed of the servo.

#### Stepper4: 
A class to control unipolar stepper motors. The arduino sketch is not blocked while 
the stepper is moving. After setting a reference point, the stepper can be positioned 
absolutely just like a servo. But without the angle limitation of 0--180 degrees.
V1.1: Ramps can be defined to accelerate/decelerate the Stepper.

#### Softled: 
Allows easy softon / softoff of leds. It works on all digital outputs.

#### Eggtimer: 
Allows easy nonblocking timedelays in sketches.

#### Additional Info:
The library uses Timer1 for all classes. V1.0: from this version on, timer 3 is used instead of timer 1 if available.
On the STM32F1 platform, timer 4 is used.

There is no special function that has to be
called in the loop frequently. You can even use the delay() function in the loop while
servos and steppers are moving.
