#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

/*
 *  Hardware type. Available types:
 *  ARDUINO_AVR_FEATHER32U4
 *  ARDUINO_PRO_MICRO
 *  ARDUINO_PRO_MINI
 *
 *  Leave commented for autodetect
 */
//#define ARDUINO_AVR_FEATHER32U4
//#define ARDUINO_PRO_MICRO
#define ARDUINO_PRO_MINI
/*
 * TX or RX mode for hardware. Available types:
 * DEVICE_MODE_TX
 * DEVICE_MODE_RX
 */
//#define DEVICE_MODE_TX
#define DEVICE_MODE_RX

//#define FEATURE_TX_OLED
//#define FORCE_TX_WITHOUT_INPUT

/*
 * Default mode of TX data input: SBUS
 * Possible values:
 * FEATURE_TX_INPUT_PPM
 * FEATURE_TX_INPUT_SBUS
 */
#define FEATURE_TX_INPUT_PPM
//#define FEATURE_TX_INPUT_SBUS

/*
 * Default mode of RX data output
 */
#define FEATURE_RX_OUTPUT_SBUS
//#define FEATURE_RX_OUTPUT_PPM

/*
 * Other features
 */
#define FEATURE_BIND_BUTTON
//#define FEATURE_TX_BUZZER

//#define DEBUG_SERIAL
// #define DEBUG_PING_PONG
// #define DEBUG_LED

#define RADIO_FREQUENCY_BIND 915000000
#define RADIO_CHANNEL_WIDTH_BIND 125000

#define RADIO_FREQUENCY_MIN 915000000
#define RADIO_FREQUENCY_MAX 917000000
#define RADIO_FREQUENCY_RANGE (RADIO_FREQUENCY_MAX-RADIO_FREQUENCY_MIN)
#define RADIO_CHANNEL_WIDTH 250000
#define RADIO_CHANNEL_COUNT ((RADIO_FREQUENCY_RANGE/RADIO_CHANNEL_WIDTH) + 1)
#define RADIO_HOP_OFFSET 5

//#define PPM_FRAME_LENGTH 30500  //set the PPM frame length in microseconds (1ms = 1000µs)
//#define PPM_PULSE_LENGTH 300  //set the pulse length
//#define PPM_OUTPUT_MULTIPLIER 1 //1 for 8MHz RX, 2 for 16MHz RX
//#define PPM_SIGNAL_POSITIVE_STATE 1  //set polarity of the pulses: 1 is positive, 0 is negative
//#define PPM_OUTPUT_PIN 10  //set PPM signal output pin on the arduino


#endif
