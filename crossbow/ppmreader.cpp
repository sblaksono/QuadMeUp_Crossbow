/*
PPMReader library based on https://github.com/Hasi123/read-any-ppm
Works with Servo library
Paweł (DzikuVx) Spychalski 2017
https://quadmeup.com
License: GNU GPL v3
*/

#include "config.h"
#include "ppmreader.h"
#include "board.h"

#if defined(DEVICE_MODE_TX) && defined(FEATURE_TX_INPUT_PPM)

#define CPU_SPEED_MULTIPLIER (F_CPU/8000000)
#define NO_UPDATE_THRESHOLD 500 //if no update in this number of ms, raise alarm

volatile int ppm[PPMREADER_PPM_CHANNEL_COUNT];
volatile bool ppmReaderUseTimer = false;
volatile uint32_t lastPacketUpdate = 0; 

PPMReader::PPMReader(bool useTimer)
{
    ppmReaderUseTimer = useTimer;

    SET_PPM_INPUT_PIN_MODE();
    ENABLE_PPM_INPUT_INT();
}

void PPMReader::loop(void)
{
    for (int i = 0; i < PPMREADER_PPM_CHANNEL_COUNT; i++) {
      setRcChannelCallback(i, ppm[i], 0);
    }
}

bool PPMReader::isReceiving(void) {
    if (millis() - lastPacketUpdate > NO_UPDATE_THRESHOLD) {
        return false;
    } else {
        return true;
    }
}

void PPMReader::start(void) {
    ENABLE_PPM_INPUT_INT();
}

void PPMReader::stop(void) {
    DISABLE_PPM_INPUT_INT();
}

ISR(PPM_INPUT_INT_VECT)
{
    static unsigned int pulse;
    static unsigned long counter;
    static byte channel;
    static unsigned long previousCounter = 0;
    static unsigned long currentMicros = 0;
    int tmpVal;

    if (ppmReaderUseTimer) {
        counter = TCNT1 * CPU_SPEED_MULTIPLIER;
        TCNT1 = 0;
    } else {
        currentMicros = micros();
        counter = currentMicros - previousCounter;
        previousCounter = currentMicros;
    }

    if (counter < 510) { //must be a pulse
        pulse = counter;
    }
    else if (counter > 1910)
    { //sync
        channel = 0;
        lastPacketUpdate = millis();
    }
    else
    { //servo values between 810 and 2210 will end up here
        tmpVal = counter + pulse;
        if (tmpVal > 810 && tmpVal < 2210) {
            ppm[channel] = tmpVal;
        }
        channel++;
    }
}

#endif
