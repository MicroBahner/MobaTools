/*
 Arduino.h - Main include file for the Arduino SDK
 Copyright (c) 2005-2013 Arduino Team.  All right reserved.

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
 */

#ifndef MAIN_ESP32_MT_TIMER_H_
#define MAIN_ESP32_MT_TIMER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "esp32-hal.h"
#include "freertos/FreeRTOS.h"

struct hw_timer_s;
typedef struct hw_timer_s hw_timer_t;

hw_timer_t * timerBeginMoTo(uint8_t timer, uint16_t divider, bool countUp);


void timerAttachInterruptMoTo(hw_timer_t *timer, void (*fn)(void), bool edge);
void timerAlarmEnableMoTo(hw_timer_t *timer);
void timerAlarmWriteMoTo(hw_timer_t *timer, uint64_t interruptAt, bool autoreload);



#ifdef __cplusplus
}
#endif

#endif /* MAIN_ESP32_HAL_TIMER_H_ */
