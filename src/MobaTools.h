#ifndef MOBATOOLS_H
#define MOBATOOLS_H
/*
  MobaTools.h - a library for model railroaders
  Author: fpm, fpm-gh@mnet-mail.de
  Copyright (c) 2021 All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  MobaTools V2.4.1
   
  History:
  V2.4.1 11-2021
     - fix typo ( arduino.h -> Arduino.h ). This created an error on linux.
     - some fixes in MoToButtons and documentation
  V2.4.0 05-2021
     - ESP32 prozessors are supported
     - some ATtiny are supported ( needs a 16bit-timer1 amd a SPI or USI hardware )
     - Step-tuning for 32Bit prozessors ( except ESP8266 ) for higher steprates
     - more examples
  V2.3.1 11-2020
     - Fix error with doSteps(0) and no ramp. Motor did not stop
  V2.3 07-2020
     - New class MoToTimebase to create events in regular intervals
     - MoToButton: The longpress event is already triggered when the time for longpress expires, not when the button is released
     - MoToStepper: steps per rotation can be changed together with setting a new reference point.
     
  V2.2 03-2020
  V2.1 02-2020
      - new class 'MoToButtons' to manage up to 32 buttons/switches
        ( debounce, events 'pressed', 'released', 'short press' and 'long press'
      - MoToTimer: new method 'expired', which is only 'true' at the first call 
        after timer expiry.
  V2.0 01-2020
      - classnames changed ( the old names can still be used for compatibility, 
        but should not be used in new sketches)
      - ESP8266 is supported
      - it is possible to define an enable pin for steppers. This is active
        only when the stepper is moving.
      - new method 'getSpeedSteps' returns actual speed
  V1.1.5 12-2019
      - Servo: fix error when 1st and 2nd write ( after attach ) are too close together
  V1.1.4 09-2019
      - speed = 0 is not allowsed( it is set to 1 internally )
	  - fix error when repeatedly setting target position very fast
	  - allow higher steprates up to 2500 steps/sec.
	    ( relative jitter increases with higher rates, abs. jitter is 200µs )
	  - typo corrected in MoToBase.h 
  V1.1.3 08-2019
      - no more warnings when compiling
      - fix error (overflow) when converting angle to steps
      - fix error when converting angle to microseconds in servo class
      - reworked softleds. Risetimes unitl 10 sec are possible.
  V1.1.2 08-2019
      - fix error when only servo objects are defined ( sketch crashed )
      - two more stepper examples ( thanks to 'agmue' from german arduino.cc forum )
      - detach() configures used pins back to INPUT
  V1.1 07-2019
      - stepper now supports ramps (accelerating, decelerating )
      - stepper speed has better resolution with high steprates
      - split source into several fuction-specific .cpp files. This
        saves flash space if only part of the functionality is used.
  V1.0  11-2017 Use of Timer 3 if available ( on AtMega32u4 and AtMega2560 )
  V0.9  03-2017
        Better resolution for the 'speed' servo-paramter (programm starts in compatibility mode)
        outputs for softleds can be inverted
        MobaTools run on STM32F1 platform
        
  V0.8 02-2017
        Enable Softleds an all digital outputs
  V0.7 01-2017
		Allow nested Interrupts with the servos. This allows more precise other
        interrupts e.g. for NmraDCC Library.
		A4988 stepper driver IC is supported (needs only 2 ports: step and direction)

   
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/



// defines that may be changed by the user

    
// default CYCLETIME is processordependent, change only if you know what you are doing ).
#ifdef  ARDUINO_ARCH_ESP8266 ///////////////////////////////////////////////////////////
#define CYCLETIME       60      // Min. irq-periode in us ( ESP-default is 60 )
#define MIN_STEP_CYCLE  2       // Minimum number of cycles per step. 
#define MAX_GPIO        10      // max number of usable gpios
// at max 10 gpio's can be used at an ESP12: gpio 0,1,2,3,4,5,12,13,14,15
// gpio 6-10 is internally used for flash
// gpio16 has no interrupt capability ( but can be used as dir-pin for a stepper)

#elif defined ARDUINO_ARCH_STM32F1 /////////////////////////////////////////////////////
#define MIN_STEP_CYCLE  25   // Minimum number of µsec  per step 

#elif defined ARDUINO_ARCH_STM32F4 /////////////////////////////////////////////////////
#define MIN_STEP_CYCLE  20   // Minimum number of µsec  per step 

#elif defined ARDUINO_ARCH_ESP32 ///////////////////////////////////////////////////////
#define USE_VSPI                // default is HSPI ( for SPI-Stepper )
#define MIN_STEP_CYCLE 20       // Minimum number of µsec  per Step

#elif defined ARDUINO_ARCH_AVR ////////////////////////////////////////////////////////
//#define NO_TIMER3             // never use Timer 3
#define CYCLETIME       200     // Min. irq-periode in us ( default is 200 ), 
#define MIN_STEP_CYCLE  2       // Minimum number of cycles per step. 
#define FASTSPI                 // only for devices with USI Interface ( instead of SPI HW )
                                // if defined SPI clock ist CPU clock / 2
                                // if not defined, SPI clock ist CPU clock / 4
//#define USI_SS  7               // defines the SS - Pin with USI-SPI-Stepper
                                // if not defined the core-default (SS) is used
#else ///////////////////////////////////////////////////////////////////////////////////
    #error Processor not supported
#endif //////////////////////////////////////////////////////////////////////////////////

// stepper related defines
#define MAX_STEPPER     6       // 
#define DEF_SPEEDSTEPS  3000    // default speed after attach
#define DEF_RAMP        0       // default ramp after attach 
#define RAMPOFFSET      16      // startvalue of rampcounter

// servo related defines
#if defined ARDUINO_ARCH_ESP32 || defined ARDUINO_ARCH_ESP8266 
#define MINPULSEWIDTH   550     // there is no general limit on ESP
#define MAXPULSEWIDTH   2600    // there is no general limit on ESP
#else // AVR and STM32
#define MINPULSEWIDTH   700     // don't make it shorter than 700
#define MAXPULSEWIDTH   2300    // don't make it longer than 2300
#endif

// softled related defines
#define LED_DEFAULT_RISETIME   50

//  !!!!!!!!!!!!  Don't change anything after tis line !!!!!!!!!!!!!!!!!!!!
 

#include <utilities/MoToBase.h>
#include <utilities/MoToStepper.h>
#include <utilities/MoToServo.h>
#include <utilities/MoToSoftled.h>
#include <utilities/MoToPwm.h>
#ifdef ARCHITECT_INCLUDE
#include ARCHITECT_INCLUDE
#endif
#ifndef INTERNALUSE
#include <MoToButtons.h>
#include <MoToTimer.h>
#endif

#ifdef debug
#include <utilities/MoToDbg.h>
#endif
#endif

