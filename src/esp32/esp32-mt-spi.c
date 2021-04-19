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
#ifdef ARDUINO_ARCH_ESP32

#include "esp32-mt-spi.h"              // MobaTools Version
#include "esp32-hal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "rom/ets_sys.h"
#include "esp_attr.h"
#include "esp_intr.h"
#include "rom/gpio.h"
#include "soc/spi_reg.h"
#include "soc/spi_struct.h"
#include "soc/io_mux_reg.h"
#include "soc/gpio_sig_map.h"
#include "soc/dport_reg.h"
#include "soc/rtc.h"

#define SPI_CLK_IDX(p)  ((p==0)?SPICLK_OUT_IDX:((p==1)?SPICLK_OUT_IDX:((p==2)?HSPICLK_OUT_IDX:((p==3)?VSPICLK_OUT_IDX:0))))
#define SPI_MISO_IDX(p) ((p==0)?SPIQ_OUT_IDX:((p==1)?SPIQ_OUT_IDX:((p==2)?HSPIQ_OUT_IDX:((p==3)?VSPIQ_OUT_IDX:0))))
#define SPI_MOSI_IDX(p) ((p==0)?SPID_IN_IDX:((p==1)?SPID_IN_IDX:((p==2)?HSPID_IN_IDX:((p==3)?VSPID_IN_IDX:0))))

#define SPI_SPI_SS_IDX(n)   ((n==0)?SPICS0_OUT_IDX:((n==1)?SPICS1_OUT_IDX:((n==2)?SPICS2_OUT_IDX:SPICS0_OUT_IDX)))
#define SPI_HSPI_SS_IDX(n)   ((n==0)?HSPICS0_OUT_IDX:((n==1)?HSPICS1_OUT_IDX:((n==2)?HSPICS2_OUT_IDX:HSPICS0_OUT_IDX)))
#define SPI_VSPI_SS_IDX(n)   ((n==0)?VSPICS0_OUT_IDX:((n==1)?VSPICS1_OUT_IDX:((n==2)?VSPICS2_OUT_IDX:VSPICS0_OUT_IDX)))
#define SPI_SS_IDX(p, n)   ((p==0)?SPI_SPI_SS_IDX(n):((p==1)?SPI_SPI_SS_IDX(n):((p==2)?SPI_HSPI_SS_IDX(n):((p==3)?SPI_VSPI_SS_IDX(n):0))))

#define SPI_INUM(u)        (2)
#define SPI_INTR_SOURCE(u) ((u==0)?ETS_SPI0_INTR_SOURCE:((u==1)?ETS_SPI1_INTR_SOURCE:((u==2)?ETS_SPI2_INTR_SOURCE:((p==3)?ETS_SPI3_INTR_SOURCE:0))))

struct spi_struct_t {
    spi_dev_t * dev;
#if !CONFIG_DISABLE_HAL_LOCKS
    xSemaphoreHandle lock;
#endif
    uint8_t num;
};

#if CONFIG_DISABLE_HAL_LOCKS
#define SPI_MUTEX_LOCK()
#define SPI_MUTEX_UNLOCK()

static spi_t _spi_bus_array[4] = {
    {(volatile spi_dev_t *)(DR_REG_SPI0_BASE), 0},
    {(volatile spi_dev_t *)(DR_REG_SPI1_BASE), 1},
    {(volatile spi_dev_t *)(DR_REG_SPI2_BASE), 2},
    {(volatile spi_dev_t *)(DR_REG_SPI3_BASE), 3}
};
#else
#define SPI_MUTEX_LOCK()    do {} while (xSemaphoreTake(spi->lock, portMAX_DELAY) != pdPASS)
#define SPI_MUTEX_UNLOCK()  xSemaphoreGive(spi->lock)

static spi_t _spi_bus_array[4] = {
    {(volatile spi_dev_t *)(DR_REG_SPI0_BASE), NULL, 0},
    {(volatile spi_dev_t *)(DR_REG_SPI1_BASE), NULL, 1},
    {(volatile spi_dev_t *)(DR_REG_SPI2_BASE), NULL, 2},
    {(volatile spi_dev_t *)(DR_REG_SPI3_BASE), NULL, 3}
};
#endif

/*
 * Clock Calculators
 *
 * */

typedef union {
    uint32_t value;
    struct {
            uint32_t clkcnt_l:       6;                     /*it must be equal to spi_clkcnt_N.*/
            uint32_t clkcnt_h:       6;                     /*it must be floor((spi_clkcnt_N+1)/2-1).*/
            uint32_t clkcnt_n:       6;                     /*it is the divider of spi_clk. So spi_clk frequency is system/(spi_clkdiv_pre+1)/(spi_clkcnt_N+1)*/
            uint32_t clkdiv_pre:    13;                     /*it is pre-divider of spi_clk.*/
            uint32_t clk_equ_sysclk: 1;                     /*1: spi_clk is eqaul to system 0: spi_clk is divided from system clock.*/
    };
} spiClk_t;


#define ClkRegToFreq(reg) (apb_freq / (((reg)->clkdiv_pre + 1) * ((reg)->clkcnt_n + 1)))

static uint32_t _spiFrequencyToClockDiv(uint32_t freq)
{
    uint32_t apb_freq = getApbFrequency();

    if(freq >= apb_freq) {
        return SPI_CLK_EQU_SYSCLK;
    }

    const spiClk_t minFreqReg = { 0x7FFFF000 };
    uint32_t minFreq = ClkRegToFreq((spiClk_t*) &minFreqReg);
    if(freq < minFreq) {
        return minFreqReg.value;
    }

    uint8_t calN = 1;
    spiClk_t bestReg = { 0 };
    int32_t bestFreq = 0;

    while(calN <= 0x3F) {
        spiClk_t reg = { 0 };
        int32_t calFreq;
        int32_t calPre;
        int8_t calPreVari = -2;

        reg.clkcnt_n = calN;

        while(calPreVari++ <= 1) {
            calPre = (((apb_freq / (reg.clkcnt_n + 1)) / freq) - 1) + calPreVari;
            if(calPre > 0x1FFF) {
                reg.clkdiv_pre = 0x1FFF;
            } else if(calPre <= 0) {
                reg.clkdiv_pre = 0;
            } else {
                reg.clkdiv_pre = calPre;
            }
            reg.clkcnt_l = ((reg.clkcnt_n + 1) / 2);
            calFreq = ClkRegToFreq(&reg);
            if(calFreq == (int32_t) freq) {
                memcpy(&bestReg, &reg, sizeof(bestReg));
                break;
            } else if(calFreq < (int32_t) freq) {
                if(abs(freq - calFreq) < abs(freq - bestFreq)) {
                    bestFreq = calFreq;
                    memcpy(&bestReg, &reg, sizeof(bestReg));
                }
            }
        }
        if(calFreq == (int32_t) freq) {
            break;
        }
        calN++;
    }
    return bestReg.value;
}

void spiAttachSCKMoTo(spi_t * spi, int8_t sck)
{
    if(!spi) {
        return;
    }
    if(sck < 0) {
        if(spi->num == HSPI) {
            sck = 14;
        } else if(spi->num == VSPI) {
            sck = 18;
        } else {
            sck = 6;
        }
    }
    pinMode(sck, OUTPUT);
    pinMatrixOutAttach(sck, SPI_CLK_IDX(spi->num), false, false);
}


void spiAttachMOSIMoTo(spi_t * spi, int8_t mosi)
{
    if(!spi) {
        return;
    }
    if(mosi < 0) {
        if(spi->num == HSPI) {
            mosi = 13;
        } else if(spi->num == VSPI) {
            mosi = 23;
        } else {
            mosi = 8;
        }
    }
    pinMode(mosi, OUTPUT);
    pinMatrixOutAttach(mosi, SPI_MOSI_IDX(spi->num), false, false);
}

static void _spiEnableSSPins(spi_t * spi, uint8_t cs_mask)
{
    if(!spi) {
        return;
    }
    SPI_MUTEX_LOCK();
    spi->dev->pin.val &= ~(cs_mask & SPI_CS_MASK_ALL);
    SPI_MUTEX_UNLOCK();
}


void spiAttachSSMoTo(spi_t * spi, uint8_t cs_num, int8_t ss)
{
    if(!spi) {
        return;
    }
    if(cs_num > 2) {
        return;
    }
    if(ss < 0) {
        cs_num = 0;
        if(spi->num == HSPI) {
            ss = 15;
        } else if(spi->num == VSPI) {
            ss = 5;
        } else {
            ss = 11;
        }
    }
    pinMode(ss, OUTPUT);
    pinMatrixOutAttach(ss, SPI_SS_IDX(spi->num, cs_num), false, false);
    _spiEnableSSPins(spi, (1 << cs_num));
}

void spiSSEnableMoTo(spi_t * spi)
{
    if(!spi) {
        return;
    }
    SPI_MUTEX_LOCK();
    spi->dev->user.cs_setup = 1;
    spi->dev->user.cs_hold = 1;
    SPI_MUTEX_UNLOCK();
}

static void _spiSetClockDiv(spi_t * spi, uint32_t clockDiv)
{
    if(!spi) {
        return;
    }
    SPI_MUTEX_LOCK();
    spi->dev->clock.val = clockDiv;
    SPI_MUTEX_UNLOCK();
}

static void _spiSetDataMode(spi_t * spi, uint8_t dataMode)
{
    if(!spi) {
        return;
    }
    SPI_MUTEX_LOCK();
    switch (dataMode) {
    case SPI_MODE1:
        spi->dev->pin.ck_idle_edge = 0;
        spi->dev->user.ck_out_edge = 1;
        break;
    case SPI_MODE2:
        spi->dev->pin.ck_idle_edge = 1;
        spi->dev->user.ck_out_edge = 1;
        break;
    case SPI_MODE3:
        spi->dev->pin.ck_idle_edge = 1;
        spi->dev->user.ck_out_edge = 0;
        break;
    case SPI_MODE0:
    default:
        spi->dev->pin.ck_idle_edge = 0;
        spi->dev->user.ck_out_edge = 0;
        break;
    }
    SPI_MUTEX_UNLOCK();
}

static void _spiSetBitOrder(spi_t * spi, uint8_t bitOrder)
{
    if(!spi) {
        return;
    }
    SPI_MUTEX_LOCK();
    if (SPI_MSBFIRST == bitOrder) {
        spi->dev->ctrl.wr_bit_order = 0;
        spi->dev->ctrl.rd_bit_order = 0;
    } else if (SPI_LSBFIRST == bitOrder) {
        spi->dev->ctrl.wr_bit_order = 1;
        spi->dev->ctrl.rd_bit_order = 1;
    }
    SPI_MUTEX_UNLOCK();
}

static void _on_apb_change(void * arg, apb_change_ev_t ev_type, uint32_t old_apb, uint32_t new_apb)
{
    spi_t * spi = (spi_t *)arg;
    if(ev_type == APB_BEFORE_CHANGE){
        SPI_MUTEX_LOCK();
        while(spi->dev->cmd.usr);
    } else {
        spi->dev->clock.val = _spiFrequencyToClockDiv(old_apb / ((spi->dev->clock.clkdiv_pre + 1) * (spi->dev->clock.clkcnt_n + 1)));
        SPI_MUTEX_UNLOCK();
    }
}

static void _spiStopBus(spi_t * spi)
{
    if(!spi) {
        return;
    }
    SPI_MUTEX_LOCK();
    spi->dev->slave.trans_done = 0;
    spi->dev->slave.slave_mode = 0;
    spi->dev->pin.val = 0;
    spi->dev->user.val = 0;
    spi->dev->user1.val = 0;
    spi->dev->ctrl.val = 0;
    spi->dev->ctrl1.val = 0;
    spi->dev->ctrl2.val = 0;
    spi->dev->clock.val = 0;
    SPI_MUTEX_UNLOCK();
    removeApbChangeCallback(spi, _on_apb_change);
}

spi_t * spiStartBusMoTo(uint8_t spi_num, uint32_t clockDiv, uint8_t dataMode, uint8_t bitOrder)
{
    if(spi_num > 3){
        return NULL;
    }

    spi_t * spi = &_spi_bus_array[spi_num];

#if !CONFIG_DISABLE_HAL_LOCKS
    if(spi->lock == NULL){
        spi->lock = xSemaphoreCreateMutex();
        if(spi->lock == NULL) {
            return NULL;
        }
    }
#endif

    if(spi_num == HSPI) {
        DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_SPI_CLK_EN);
        DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_SPI_RST);
    } else if(spi_num == VSPI) {
        DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_SPI_CLK_EN_2);
        DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_SPI_RST_2);
    } else {
        DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_SPI_CLK_EN_1);
        DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_SPI_RST_1);
    }

    _spiStopBus(spi);
    _spiSetDataMode(spi, dataMode);
    _spiSetBitOrder(spi, bitOrder);
    _spiSetClockDiv(spi, clockDiv);

    SPI_MUTEX_LOCK();
    spi->dev->user.usr_mosi = 1;
    spi->dev->user.usr_miso = 1;
    spi->dev->user.doutdin = 1;

    int i;
    for(i=0; i<16; i++) {
        spi->dev->data_buf[i] = 0x00000000;
    }
    SPI_MUTEX_UNLOCK();

    addApbChangeCallback(spi, _on_apb_change);
    return spi;
}


/*
 * Manual Lock Management
 * */

#define MSB_16_SET(var, val) { (var) = (((val) & 0xFF00) >> 8) | (((val) & 0xFF) << 8); }
void IRAM_ATTR spiWriteShortNLMoTo(spi_t * spi, uint16_t data)
{
    if(!spi) {
        return;
    }
    if(!spi->dev->ctrl.wr_bit_order){
        MSB_16_SET(data, data);
    }
    spi->dev->mosi_dlen.usr_mosi_dbitlen = 15;
    spi->dev->miso_dlen.usr_miso_dbitlen = 0;
    spi->dev->data_buf[0] = data;
    spi->dev->cmd.usr = 1;
    //while(spi->dev->cmd.usr); //MobaTools; kein Warten auf Übertragungsende
}




#endif
