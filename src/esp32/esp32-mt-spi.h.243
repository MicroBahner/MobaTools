// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// MobaTools Version
#ifndef MAIN_ESP32_MT_SPI_H_
#define MAIN_ESP32_MT_SPI_H_   // This too supresses the hal-Version in MobaTools files

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define SPI_HAS_TRANSACTION

#define FSPI  1 //SPI bus attached to the flash (can use the same data lines but different SS)
#define HSPI  2 //SPI bus normally mapped to pins 12 - 15, but can be matrixed to any pins
#define VSPI  3 //SPI bus normally attached to pins 5, 18, 19 and 23, but can be matrixed to any pins

// This defines are not representing the real Divider of the ESP32
// the Defines match to an AVR Arduino on 16MHz for better compatibility
#define SPI_CLOCK_DIV2    0x00101001 //8 MHz
#define SPI_CLOCK_DIV4    0x00241001 //4 MHz
#define SPI_CLOCK_DIV8    0x004c1001 //2 MHz
#define SPI_CLOCK_DIV16   0x009c1001 //1 MHz
#define SPI_CLOCK_DIV32   0x013c1001 //500 KHz
#define SPI_CLOCK_DIV64   0x027c1001 //250 KHz
#define SPI_CLOCK_DIV128  0x04fc1001 //125 KHz

#define SPI_MODE0 0
#define SPI_MODE1 1
#define SPI_MODE2 2
#define SPI_MODE3 3

#define SPI_CS0 0
#define SPI_CS1 1
#define SPI_CS2 2
#define SPI_CS_MASK_ALL 0x7

#define SPI_LSBFIRST 0
#define SPI_MSBFIRST 1

struct spi_struct_t;
typedef struct spi_struct_t spi_t;

spi_t * spiStartBusMoTo(uint8_t spi_num, uint32_t freq, uint8_t dataMode, uint8_t bitOrder);
//void spiStopBus(spi_t * spi);

//Attach/Detach Signal Pins
void spiAttachMOSIMoTo(spi_t * spi, int8_t mosi);
void spiAttachSCKMoTo(spi_t * spi, int8_t sck);
void spiAttachSSMoTo(spi_t * spi, uint8_t cs_num, int8_t ss);
void spiWriteShortNLMoTo(spi_t * spi, uint16_t data);
void spiSSEnableMoTo(spi_t * spi);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_ESP32_HAL_SPI_H_ */
