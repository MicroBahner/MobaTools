// STM32F1 HW-spcific Functions
#ifdef ARDUINO_ARCH_STM32F1
#define bool int
#include <stm32f1/drivers.h>

#warning "HW specfic - STM32F1 ---"

#ifdef USE_SPI2
void __irq_spi2(void) {// STM32
    static int rxData;
    rxData = spi_rx_reg(SPI2);            // Get dummy data (Clear RXNE-Flag)
    digitalWrite(BOARD_SPI2_NSS_PIN,HIGH);
}
#else
void __irq_spi1(void) {// STM32
    rxData = spi_rx_reg(SPI1);            // Get dummy data (Clear RXNE-Flag)
    digitalWrite(BOARD_SPI1_NSS_PIN,HIGH);
}
#endif

#endif
