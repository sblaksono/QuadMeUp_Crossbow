/*
 *  Board definitions
 *
 */

#ifndef _BOARD_H_
#define _BOARD_H_

#include <Arduino.h>
#include <avr/io.h>

#include "config.h"

// Port Pin Bitmaps for general use.

#ifndef PIN0_bm
#define PIN0_bm  0x01
#endif
#ifndef PIN1_bm
#define PIN1_bm  0x02
#endif
#ifndef PIN2_bm
#define PIN2_bm  0x04
#endif
#ifndef PIN3_bm
#define PIN3_bm  0x08
#endif
#ifndef PIN4_bm
#define PIN4_bm  0x10
#endif
#ifndef PIN5_bm
#define PIN5_bm  0x20
#endif
#ifndef PIN6_bm
#define PIN6_bm  0x40
#endif
#ifndef PIN7_bm
#define PIN7_bm  0x80
#endif

#ifdef ARDUINO_AVR_FEATHER32U4

    #define LORA_SS_PIN     8
    #define LORA_RST_PIN    4
    #define LORA_DI0_PIN    7

    #define BUTTON_0_PIN    9
    #define BUTTON_1_PIN    10

    #define SET_STATUS_LED_PIN_MODE()   pinMode(LED_BUILTIN, OUTPUT)
    #define STATUS_LED_ON()             digitalWrite(LED_BUILTIN, HIGH)
    #define STATUS_LED_OFF()            digitalWrite(LED_BUILTIN, LOW)
    #define STATUS_LED_TOGGLE()         digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN))

#elif defined(ARDUINO_PRO_MICRO)

    #define LORA_SS_PIN     10
    #define LORA_RST_PIN    4
    #define LORA_DI0_PIN    3

    #define BUTTON_0_PIN    9
    #define BUTTON_1_PIN    10

    #define OUT_D_STATUS_LED            PIN5_bm   // pin D5/30 - TX LED
    #define SET_STATUS_LED_PIN_MODE()   DDRD |= (OUT_D_STATUS_LED)
    #define STATUS_LED_ON()            PORTD &= ~(OUT_D_STATUS_LED)
    #define STATUS_LED_OFF()             PORTD |= (OUT_D_STATUS_LED)
    #define IS_STATUS_LED_OFF()          (PIND & (OUT_D_STATUS_LED))
    #define STATUS_LED_TOGGLE()         { IS_STATUS_LED_OFF() ? STATUS_LED_ON() : STATUS_LED_OFF(); }

#elif defined(ARDUINO_SAMD_FEATHER_M0)

    #define LORA_SS_PIN     8
    #define LORA_RST_PIN    4
    #define LORA_DI0_PIN    3

    #define BUTTON_0_PIN    9 //Please verify
    #define BUTTON_1_PIN    10 //Please verify

#else
    #error please select hardware
#endif

#define RX_ADC_PIN_1 A0
#define RX_ADC_PIN_2 A1
#define RX_ADC_PIN_3 A2

#define TX_BUZZER_PIN A5



#endif // _BOARD_H_
