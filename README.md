MobaTools
=========
### Arduino library for model railroaders ( and maybe for others too :) )

**Latest changes:**

| Version |  Release Date  | Description
| ------- |  ------------  | -----------
| 2.0.0 | 2019-12-?? | managing an enable pin for steppers is possible, ESP8266 supported
| | | new method 'getSpeedSteps' returns actual speed
| 1.1.4 | 2019-09-25 | check for setting speed to 0 ( this is not allowed, will be changed to '1' internally )
| | | fix error when setting targetposition repetitive very fast
| | | allow higher steprates (up to 2500 steps/sec)at the expense of higher relative jitter at these rates
| | | ( absolute jitter is 200µs )
| 1.1.3 | 2019-08-22 | fix errors when converting angle to microseconds and vice versa in servo class
| | | fix errors when converting angle to steps and vice versa in stepper class
| | | no more warnings
| 1.1.2 | 2019-08-03 | fix error, when only servo objects are defined (sketch crashed), 2 more Stepper examples
| 1.1.1 | 2019-07-29 | acceleration/deceleration for steppermotors is now possible
| | | optimized flash usage when only part of the functionality is used

### Functionality

The lib contains 4 classes:

#### MoToServo: 
Can control up to 16 servos. Compatible with arduino servo lib, but allows to control 
the speed of the servo.

#### MoToStepper: 
A class to control stepper motors. The arduino sketch is not blocked while 
the stepper is moving. After setting a reference point, the stepper can be positioned 
absolutely just like a servo. But without the angle limitation of 0--180 degrees.
V1.1: Ramps can be defined to accelerate/decelerate the Stepper.

#### MoToSoftled: 
Allows easy softon / softoff of leds. It works on all digital outputs.

#### MoToTimer: 
Allows easy nonblocking timedelays in sketches. You don't have to bother with millis() directly

#### Additional Info:
The library uses Timer1 for all classes. V1.0: from this version on, timer 3 is used instead of timer 1 if available.
On the STM32F1 platform, timer 4 is used.
With ESP8266 the core waveform creating fuctions, and IO-interrupts are used.

There is no special function that has to be
called in the loop frequently. You can even use the delay() function in the loop while
servos and steppers are moving.
