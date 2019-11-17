#ifndef MOBATOOLS_H
#define MOBATOOLS_H
/*
  MobaTools.h - a library for model railroaders
  Author: fpm, fpm-gh@mnet-mail.de
  Copyright (c) 2019 All right reserved.

  MobaTools V1.2
   (C) 08-2019 fpm fpm@mnet-online.de
   
  History:
  V1.2 08-2019
      - it is possible to define an enable pin for steppers. This is active
        only when the stepper is moving.
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

// stepper related defines
#define MAX_STEPPER     6       // 
#define DEF_SPEEDSTEPS  3000    // default speed after attach
#define DEF_RAMP        0       // default ramp after attach     
// default CYCLETIME is processordependent, change only if you know what you are doing ).
#ifdef  ESP8266
#define CYCLETIME       80      // Min. irq-periode in us ( ESP-default is 80 )
#define MAX_GPIO        10      // max number of usable gpios
// at max 10 gpio's can be used at an ESP12: gpio 0,1,2,3,4,5,12,13,14,15
// gpio 6-10 is internally used für flash
// gpio16 has no interrupt capability ( but can be used as dir-pin for a stepper)

#elif defined ( STM32F1 )
#define CYCLETIME       100     // Min. irq-periode in us ( STM-default is 100 )
#else
#define CYCLETIME       200     // Min. irq-periode in us ( default is 200 ), 
#endif
#define MIN_STEP_CYCLE  2       // Minimum number of cycles per step. 
#define RAMPOFFSET      16      // startvalue of rampcounter

// servo related defines
// #define FIXED_POSITION_SERVO_PULSES  // does not work yet - leave commented out
#define MINPULSEWIDTH   700     // don't make it shorter than 700
#define MAXPULSEWIDTH   2300    // don't make it longer than 2300

// softled related defines
#define LED_DEFAULT_RISETIME   50


#include <utilities/MoToBase.h>
#include <utilities/MoToStepper.h>
#include <utilities/MoToServo.h>
#include <utilities/MoToSoftled.h>

#endif

