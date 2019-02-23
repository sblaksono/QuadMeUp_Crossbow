#ifndef RF_SPI_H
#define RF_SPI_H

#include <stdint.h>

void rf_spi_disable();
void rf_spi_enable_master_mode();
uint8_t rf_spi_xfer(uint8_t data);
void rf_spi_buffer_xfer(void *buf, uint8_t count);
void rf_spi_using_interrupt();
void rf_spi_no_interrupt();
void rf_spi_transaction_begin();
void rf_spi_transaction_end();

#endif // _RF_SPI_H_
