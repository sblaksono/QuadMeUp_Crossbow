/*
  This file was derived from Arduino LoRa library originally licensed as MIT
  https://github.com/sandeepmistry/arduino-LoRa
*/

#include <avr/interrupt.h>
#include <avr/io.h>

#include "lora.h"
#include "config.h"
#include "board.h"
#include "rf_spi.h"

// registers
#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42

// modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06

// PA config
#define PA_BOOST                 0x80

// IRQ masks
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40

#define MAX_PKT_LENGTH           255

int _frequency = 0;
int _packetIndex = 0;
int _implicitHeaderMode = 0;
void (*_onReceive)(int) = NULL;


uint8_t singleTransfer(uint8_t address, uint8_t value)
{
  uint8_t response;

  RF_CS_LORA_ACTIVE();

  rf_spi_transaction_begin();
  rf_spi_xfer(address);
  response = rf_spi_xfer(value);
  rf_spi_transaction_end();

  RF_CS_LORA_INACTIVE();

  return response;
}

uint8_t readRegister(uint8_t address)
{
  return singleTransfer(address & 0x7f, 0x00);
}

void writeRegister(uint8_t address, uint8_t value)
{
  singleTransfer(address | 0x80, value);
}

void bufferTransfer(uint8_t address, uint8_t buffer[], uint8_t size) {
  uint8_t response;

  RF_CS_LORA_ACTIVE();

  rf_spi_transaction_begin();
  rf_spi_xfer(address);
  rf_spi_buffer_xfer(buffer, size);
  rf_spi_transaction_end();

  RF_CS_LORA_INACTIVE();
}

void writeRegister(uint8_t address, uint8_t buffer[], uint8_t size)
{
  bufferTransfer(address | 0x80, buffer, size);
}

void readRegister(uint8_t address, uint8_t buffer[], uint8_t size)
{
  bufferTransfer(address & 0x7f, buffer, size);
}

void explicitHeaderMode()
{
  _implicitHeaderMode = 0;

  writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) & 0xfe);
}

void implicitHeaderMode()
{
  _implicitHeaderMode = 1;

  writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) | 0x01);
}

int LoRa_beginPacket(int implicitHeader)
{
  // put in standby mode
  LoRa_idle();

  if (implicitHeader) {
    implicitHeaderMode();
  } else {
    explicitHeaderMode();
  }

  // reset FIFO address and paload length
  writeRegister(REG_FIFO_ADDR_PTR, 0);
  writeRegister(REG_PAYLOAD_LENGTH, 0);

  return 1;
}

int LoRa_endPacket()
{
  // put in TX mode
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

  // wait for TX done
  while((readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0);

  // clear IRQ's
  writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);

  return 1;
}

//https://github.com/sandeepmistry/arduino-LoRa/pull/62/files
void LoRa_endPacketAsync()
{
  // put in TX mode
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

  // apparently this grace time is required for the radio
  delayMicroseconds(150);
}

//https://github.com/sandeepmistry/arduino-LoRa/pull/62/files
bool LoRa_isTransmitting()
{
  if ((readRegister(REG_OP_MODE) & MODE_TX) == MODE_TX)
    return true;

  if (!(readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0)
    // clear IRQ's
    writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);

  return false;
}

int LoRa_parsePacket(int size)
{
  int packetLength = 0;
  int irqFlags = readRegister(REG_IRQ_FLAGS);

  if (size > 0) {
    implicitHeaderMode();

    writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
  } else {
    explicitHeaderMode();
  }

  // clear IRQ's
  writeRegister(REG_IRQ_FLAGS, irqFlags);

  if ((irqFlags & IRQ_RX_DONE_MASK) && (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
    // received a packet
    _packetIndex = 0;

    // read packet length
    if (_implicitHeaderMode) {
      packetLength = readRegister(REG_PAYLOAD_LENGTH);
    } else {
      packetLength = readRegister(REG_RX_NB_BYTES);
    }

    // set FIFO address to current RX address
    writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));

    // put in standby mode
    LoRa_idle();
  } else if (readRegister(REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)) {
    // not currently in RX mode

    // reset FIFO address
    writeRegister(REG_FIFO_ADDR_PTR, 0);

    // put in single RX mode
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
  }

  return packetLength;
}

int LoRa_packetRssi()
{
  return (readRegister(REG_PKT_RSSI_VALUE) - (_frequency < RADIO_FREQUENCY_MIN ? 164 : 157));
}

float LoRa_packetSnr()
{
  return ((int8_t)readRegister(REG_PKT_SNR_VALUE)) * 0.25;
}

void LoRa_write(uint8_t data)
{
  int currentLength = readRegister(REG_PAYLOAD_LENGTH);

  writeRegister(REG_FIFO, data);

  // update length
  writeRegister(REG_PAYLOAD_LENGTH, currentLength + 1);
}

void LoRa_write(uint8_t buffer[], uint8_t size)
{
  int currentLength = readRegister(REG_PAYLOAD_LENGTH);

  // check size
  if ((currentLength + size) > MAX_PKT_LENGTH) {
    size = MAX_PKT_LENGTH - currentLength;
  }

  writeRegister(REG_FIFO, buffer, size);

  // update length
  writeRegister(REG_PAYLOAD_LENGTH, currentLength + size);
}

int LoRa_available()
{
  return (readRegister(REG_RX_NB_BYTES) - _packetIndex);
}

int LoRa_fastRead() {
  _packetIndex++;

  return readRegister(REG_FIFO);
}

void LoRa_read(uint8_t buffer[], uint8_t size) {
  _packetIndex += size;
  return readRegister(REG_FIFO, buffer, size);
}

int LoRa_read()
{
  if (!LoRa_available()) {
    return -1;
  }

  return LoRa_fastRead();
}

ISR(LORA_DIO0_INT_VECT)
{
  int irqFlags = readRegister(REG_IRQ_FLAGS);

  // clear IRQ's
  writeRegister(REG_IRQ_FLAGS, irqFlags);

  if ((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
    // received a packet
    _packetIndex = 0;

    // read packet length
    int packetLength = _implicitHeaderMode ? readRegister(REG_PAYLOAD_LENGTH) : readRegister(REG_RX_NB_BYTES);

    // set FIFO address to current RX address
    writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));

    if (_onReceive) {
      _onReceive(packetLength);
    }

    // reset FIFO address
    writeRegister(REG_FIFO_ADDR_PTR, 0);
  }
}

void LoRa_onReceive(void(*callback)(int))
{
  _onReceive = callback;

  if (callback) {
    SET_LORA_DIO0_PIN_MODE();

    writeRegister(REG_DIO_MAPPING_1, 0x00);
    rf_spi_using_interrupt();
    ENABLE_LORA_DIO0_INT();
  } else {
    DISABLE_LORA_DIO0_INT();
    rf_spi_no_interrupt();
  }
}

void LoRa_receive(int size)
{
  if (size > 0) {
    implicitHeaderMode();

    writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
  } else {
    explicitHeaderMode();
  }

  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

void LoRa_idle()
{
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void LoRa_sleep()
{
  writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void LoRa_setTxPower(int level, int outputPin)
{
  if (PA_OUTPUT_RFO_PIN == outputPin) {
    // RFO
    if (level < 0) {
      level = 0;
    } else if (level > 14) {
      level = 14;
    }

    writeRegister(REG_PA_CONFIG, 0x70 | level);
  } else {
    // PA BOOST
    if (level < 2) {
      level = 2;
    } else if (level > 17) {
      level = 17;
    }

    writeRegister(REG_PA_CONFIG, PA_BOOST | (level - 2));
  }
}

void LoRa_setFrequency(long frequency)
{
  _frequency = frequency;

  uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

  writeRegister(REG_FRF_MSB, (uint8_t)(frf >> 16));
  writeRegister(REG_FRF_MID, (uint8_t)(frf >> 8));
  writeRegister(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

void LoRa_setSpreadingFactor(int sf)
{
  if (sf < 6) {
    sf = 6;
  } else if (sf > 12) {
    sf = 12;
  }

  if (sf == 6) {
    writeRegister(REG_DETECTION_OPTIMIZE, 0xc5);
    writeRegister(REG_DETECTION_THRESHOLD, 0x0c);
  } else {
    writeRegister(REG_DETECTION_OPTIMIZE, 0xc3);
    writeRegister(REG_DETECTION_THRESHOLD, 0x0a);
  }

  writeRegister(REG_MODEM_CONFIG_2, (readRegister(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
}

void LoRa_setSignalBandwidth(long sbw)
{
  int bw;

  if (sbw <= 7.8E3) {
    bw = 0;
  } else if (sbw <= 10.4E3) {
    bw = 1;
  } else if (sbw <= 15.6E3) {
    bw = 2;
  } else if (sbw <= 20.8E3) {
    bw = 3;
  } else if (sbw <= 31.25E3) {
    bw = 4;
  } else if (sbw <= 41.7E3) {
    bw = 5;
  } else if (sbw <= 62.5E3) {
    bw = 6;
  } else if (sbw <= 125E3) {
    bw = 7;
  } else if (sbw <= 250E3) {
    bw = 8;
  } else /*if (sbw <= 250E3)*/ {
    bw = 9;
  }

  writeRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
}

void LoRa_setCodingRate4(int denominator)
{
  if (denominator < 5) {
    denominator = 5;
  } else if (denominator > 8) {
    denominator = 8;
  }

  int cr = denominator - 4;

  writeRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

void LoRa_setPreambleLength(long length)
{
  writeRegister(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
  writeRegister(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

void LoRa_setSyncWord(int sw)
{
  writeRegister(REG_SYNC_WORD, sw);
}

void LoRa_enableCrc()
{
  writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) | 0x04);
}

void LoRa_disableCrc()
{
  writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) & 0xfb);
}

uint8_t LoRa_random()
{
  return readRegister(REG_RSSI_WIDEBAND);
}

int LoRa_begin(long frequency)
{
  // setup pins
  SET_LORA_SS_PIN_MODE();
  SET_LORA_RST_PIN_MODE();

  // perform reset
  LORA_RST_LOW();
  delay(10);
  LORA_RST_HIGH();
  delay(10);

  // set SS high
  RF_CS_LORA_INACTIVE();

  // start SPI
  rf_spi_enable_master_mode();

  // check version
  uint8_t version = readRegister(REG_VERSION);
  if (version != 0x12) {
    return 0;
  }

  // put in sleep mode
  LoRa_sleep();

  // set frequency
  LoRa_setFrequency(frequency);

  // set base addresses
  writeRegister(REG_FIFO_TX_BASE_ADDR, 0);
  writeRegister(REG_FIFO_RX_BASE_ADDR, 0);

  // set LNA boost
  writeRegister(REG_LNA, readRegister(REG_LNA) | 0x03);

  // set auto AGC
  writeRegister(REG_MODEM_CONFIG_3, 0x04);

  // set output power to 17 dBm
  LoRa_setTxPower(17);

  // put in standby mode
  LoRa_idle();

  return 1;
}

void LoRa_end()
{
  // put in sleep mode
  LoRa_sleep();

  // stop SPI
  rf_spi_disable();
}
