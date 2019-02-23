/*
  This file was derived from Arduino LoRa library originally licensed as MIT
  https://github.com/sandeepmistry/arduino-LoRa
*/

#ifndef LORA_H
#define LORA_H

#include <SPI.h>

#define LORA_DEFAULT_SS_PIN    10
#define LORA_DEFAULT_RESET_PIN 9
#define LORA_DEFAULT_DIO0_PIN  2

#define PA_OUTPUT_RFO_PIN      0
#define PA_OUTPUT_PA_BOOST_PIN 1

/*
class LoRaClass {
public:
  LoRaClass();
*/
int LoRa_begin(long frequency);
void LoRa_end();

int LoRa_beginPacket(int implicitHeader = false);
int LoRa_endPacket();

void LoRa_endPacketAsync();
bool LoRa_isTransmitting();

int LoRa_parsePacket(int size = 0);
int LoRa_packetRssi();
float LoRa_packetSnr();

void LoRa_write(uint8_t byte);
void LoRa_write(uint8_t buffer[], size_t size);
int LoRa_available();
int LoRa_read();
int LoRa_fastRead();
void LoRa_read(uint8_t buffer[], uint8_t size);

void LoRa_onReceive(void(*callback)(int));

void LoRa_receive(int size = 0);
void LoRa_idle();
void LoRa_sleep();

void LoRa_setTxPower(int level, int outputPin = PA_OUTPUT_PA_BOOST_PIN);
void LoRa_setFrequency(long frequency);
void LoRa_setSpreadingFactor(int sf);
void LoRa_setSignalBandwidth(long sbw);
void LoRa_setCodingRate4(int denominator);
void LoRa_setPreambleLength(long length);
void LoRa_setSyncWord(int sw);
void LoRa_enableCrc();
void LoRa_disableCrc();

byte LoRa_random();

void LoRa_setPins(int ss = LORA_DEFAULT_SS_PIN, int reset = LORA_DEFAULT_RESET_PIN, int dio0 = LORA_DEFAULT_DIO0_PIN);
void LoRa_setSPIFrequency(uint32_t frequency);

void LoRa_dumpRegisters(Stream& out);

#endif
