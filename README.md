MobaTools
=========
### Arduino library for model railroaders ( and maybe for others too 😉 )
This library contains functionality
- to control up to 16 servos with speed control
- to control up to 6 stepper motors with accelerating and decelerating
- to softly turn leds on and off ( bulb simulation )
- to implement time functions without delay.
- to debounce and evaluate up to 32 buttons/switches

### Installation

Released versions can be installed by means of the library manager in arduino IDE

A documentation file in [german](MobaTools-23-de.pdf) and [english](MobaTools-23-en.pdf) is provided.

**Latest changes:**

| Version |  Release Date  | Description
| ------- |  ------------  | -----------
| 2.3.0 | 2020-07-31| MoToButtons: The longpress event is already triggered when the time for longpress expires, not when the button is released
| | | New class MoToTimebase to create a trigger in regular intervals
| | | When setting the zero point, steps per rotation can be changed also.
| | | fix error in rotate(0) - did not stop under special circumstances
| 2.2.0 | 2020-02-23| MoToButtons: new method 'clicked' to recognize single and double clicks.
| | | MoToButtons: if all buttons/switches are simply connected to input pins, MoToButtons can read them alone without a callback function.
| 2.1.1 | 2020-02-08| Fix error in MoToServo: stopped working with more than one Servo
| 2.1.0 | 2020-02-05| new class 'MoToButtons' to manage multiple buttons and switches ( up to 32 in one instance )
| | | MoToTimer: new method 'expired', which returns a 'true' only with the first call after the timer has expired
| 2.0.0 | 2020-01-13| managing an enable pin for steppers is possible
| | | new method 'getSpeedSteps' returns actual speed
| | | ESP8266 is now supported ( with limitations regarding stepper mode: only step/dir is possible )
| | | classnames have changed ( the old names can still be used for compatibility, but should not be used in new sketches)
| 1.1.5 | 2019-12-29 | Servo8: fix timing error if first and second write after attach are too close together
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

### Classes

#### MoToServo: 
Can control up to 16 servos. Compatible with arduino servo lib, but allows to control 
the speed of the servo.

#### MoToStepper: 
A class to control stepper motors. The arduino sketch is not blocked while 
the stepper is moving. After setting a reference point, the stepper can be positioned 
absolutely just like a servo. But without the angle limitation of 0--180 degrees.
V1.1: Ramps can be defined to accelerate/decelerate the Stepper.

#### MoToSoftLed: 
Allows easy softon / softoff of leds. It works on all digital outputs.

#### MoToTimer: 
Allows easy nonblocking timedelays in sketches. You don't have to bother with millis() directly

#### MoToButtons: 
Manage up to 32 buttons and switches with debounce and event handling (pressed, released, short press, long press ) in one instance. The buttons/switches are read in via a user callback function. This enables matrix arrangements and e.g. I2C port expander to be used.


#### Additional Info:
Apart from class MoToButtons, there is no special function that has to be called in the loop frequently. You can even use the delay() function in the loop while servos and steppers are moving.

The library uses Timer1 for all classes (AVR). V1.0: from this version on, timer 3 is used instead of timer 1 if available.
On the STM32F1 platform, timer 4 is used.
MoToButtons does not use any timer und should be compatible with all plattforms.

With ESP8266 waveform creating fuctions, and IO-interrupts are used. Because the core functions could not be used for this purpose, the integrated functions tone(), analogWrite() and servo() cannot be used together with MobaTools.
To overcome this problem, there exists an additional class exclusively for the ESP8266 platform:
#### MoToPwm ( only ESP8266 ):
Contains Methods to create pwm and tone outputs.


