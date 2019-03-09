/*
PPMReader library based on https://github.com/Hasi123/read-any-ppm
Works with Servo library
Paweł (DzikuVx) Spychalski 2017
https://quadmeup.com
License: GNU GPL v3
*/

#ifndef PPMREADER_H
#define PPMREADER_H

#include "config.h"

#if defined(DEVICE_MODE_TX) && defined(FEATURE_TX_INPUT_PPM)

#include "Arduino.h"
#include "tx_input.h"

#define PPMREADER_PPM_CHANNEL_COUNT 10

class PPMReader : public TxInput
{
  public:
    PPMReader(bool useTimer);
    void loop(void);
    void start(void);
    void stop(void);
    bool isReceiving(void);
    void (* setRcChannelCallback)(uint8_t channel, int value, int offset);
  private:
    int _pin;
    int _interrupt;
};

#endif

#endif
