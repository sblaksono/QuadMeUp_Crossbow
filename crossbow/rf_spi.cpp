#include <avr/io.h>

#include "rf_spi.h"
#include "board.h"

//---------------------------
// AVR SPI functions
//---------------------------

void rf_spi_disable()
{
    SPCR &= ~(1<<SPE);
}

//----------------------

void rf_spi_enable_master_mode()
{
    RF_SS_ON();
    SET_RF_SS_IS_OUTPUT();
    SPCR = (1<<SPE) | (1<<MSTR);
    SET_RF_XCK_IS_OUTPUT();
    SET_RF_MOSI_IS_OUTPUT();
}

//----------------------

uint8_t rf_spi_xfer(uint8_t data)
{
    SPDR = data;
    asm volatile("nop");
    while (!(SPSR & (1<<SPIF)));
    return SPDR;
}

//----------------------

void rf_spi_buffer_xfer(void *buf, uint8_t count)
{
    if (count == 0) return;
    uint8_t *p = (uint8_t *)buf;
    SPDR = *p;
    while (--count > 0) {
      uint8_t out = *(p + 1);
      while (!(SPSR & _BV(SPIF))) ;
      uint8_t in = SPDR;
      SPDR = out;
      *p++ = in;
    }
    while (!(SPSR & _BV(SPIF))) ;
    *p = SPDR;
}

//----------------------

uint8_t interruptMask = 0;
uint8_t interruptSave = 0;

void rf_spi_using_interrupt()
{
    interruptMask |= LORA_DIO0_INT_MASK; 
}

void rf_spi_no_interrupt()
{
    interruptMask &= ~LORA_DIO0_INT_MASK;   
}

void rf_spi_transaction_begin()
{
    interruptSave = SPI_AVR_EIMSK;
    SPI_AVR_EIMSK &= ~interruptMask;  
}

void rf_spi_transaction_end()
{
    SPI_AVR_EIMSK = interruptSave;  
}
