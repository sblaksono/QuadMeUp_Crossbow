/*
PPMReader library based on https://github.com/Hasi123/read-any-ppm
Works with Servo library
Paweł (DzikuVx) Spychalski 2017
https://quadmeup.com
License: GNU GPL v3
*/

#ifndef PPMREADER_H
#define PPMREADER_H

#if defined(DEVICE_MODE_TX) && defined(FEATURE_TX_INPUT_PPM)

#include "Arduino.h"

#define PPMREADER_PMM_CHANNEL_COUNT 10

class PPMReader
{
  public:
    PPMReader(bool useTimer);
    void loop(void);
    volatile static int ppm[PPMREADER_PMM_CHANNEL_COUNT];
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
