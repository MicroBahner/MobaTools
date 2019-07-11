#ifndef MOBATOOLS_H
#define MOBATOOLS_H
/*
  MobaTools.h - a library for model railroaders
  Author: fpm, fpm@mnet-mail.de
  Copyright (c) 2019 All right reserved.

  MobaTools V1.0
   (C) 02-2019 fpm fpm@mnet-online.de
   
  History:
  V1.1 06-2019
        stepper now supports ramps (accelerating, decelerating )
        stepper speed has better resolution with high steprates
        split source into several fuction-specific .cpp files
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

  Library to drive the Stepper Motor 28BYJ-48
  connected to SPI (MOSI,CLK,SS) Interface via shift register
  or 4 pins directly
   
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

/*  08-02-17    start of implementing STM32 support
    03-02-17 / V0.8  Softleds now working on all digital outputs
    02-11-16 / (V0.7) Updating Stepper driver:
   - stepper motor can be connected ba means of a4988 stepper motor driver IC
     this uses only 2 pins: STEP and DIRECTION
*/

// This global include activates all features of MobaTools
#include <MoToStepper.h>
#include <MoToServo.h>
#include <MoToSoftled.h>

#endif