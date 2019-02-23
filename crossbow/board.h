/*
 *  Board definitions
 *
 */

#ifndef _BOARD_H_
#define _BOARD_H_

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

    //#define LORA_SS_PIN     10
    //#define LORA_RST_PIN    4
    //#define LORA_DI0_PIN    3
    
    #define OUT_B_LORA_SS               PIN6_bm   // pin PB6/10
    #define SET_LORA_SS_PIN_MODE()      DDRB |= (OUT_B_LORA_SS)
    #define LORA_SS_HIGH()              PORTB |= (OUT_B_LORA_SS)
    #define LORA_SS_LOW()               PORTB &= ~(OUT_B_LORA_SS)

    #define OUT_D_LORA_RST              PIN4_bm   // pin PD4/4
    #define SET_LORA_RST_PIN_MODE()     DDRD |= (OUT_D_LORA_RST)
    #define LORA_RST_HIGH()             PORTD |= (OUT_D_LORA_RST)
    #define LORA_RST_LOW()              PORTD &= ~(OUT_D_LORA_RST)

    #define LORA_DIO0_INT0              // using INT0
    #define LORA_DIO0_INT_MASK          (1<<INT0)
    #define IN_D_LORA_DIO0              PIN3_bm   // pin PD3/3
    #define SET_LORA_DIO0_PIN_MODE()    DDRD &= ~(IN_D_LORA_DIO0)
    #define LORA_DIO0_PULLUP()          PORTD |= IN_D_LORA_DIO0;

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

// SPI port

#if defined(EIMSK)
  #define SPI_AVR_EIMSK  EIMSK
#elif defined(GICR)
  #define SPI_AVR_EIMSK  GICR
#elif defined(GIMSK)
  #define SPI_AVR_EIMSK  GIMSK
#endif

// 168 and 328 Arduinos
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega168P__) || defined(__AVR_ATmega328P__)

  #define OUT_B_RF_SS                 PIN2_bm
  #define SET_RF_SS_IS_OUTPUT()       DDRB |= (OUT_B_RF_SS)
  #define RF_SS_ON()                  PORTB |= (OUT_B_RF_SS)
  #define RF_SS_OFF()                 PORTB &= ~(OUT_B_RF_SS)
  #define OUT_B_RF_XCK                PIN5_bm
  #define SET_RF_XCK_IS_OUTPUT()      DDRB |= (OUT_B_RF_XCK)
  #define RF_XCK_ON()                 PORTB |= (OUT_B_RF_XCK)
  #define RF_XCK_OFF()                PORTB &= ~(OUT_B_RF_XCK)
  #define OUT_B_RF_MOSI               PIN3_bm
  #define SET_RF_MOSI_IS_INPUT()      DDRB &= ~(OUT_B_RF_MOSI)
  #define SET_RF_MOSI_IS_OUTPUT()     DDRB |= (OUT_B_RF_MOSI)
  #define IS_RF_MOSI_ON               (PINB & (OUT_B_RF_MOSI))

  #define ENABLE_INT0_RISING()        { EIMSK |= PIN0_bm; EICRA |= (PIN1_bm | PIN0_bm); }
  #define DISABLE_INT0()              EIMSK &= ~PIN0_bm
  #define ENABLE_INT1_RISING()        { EIMSK |= PIN1_bm; EICRA |= (PIN3_bm | PIN2_bm); }
  #define DISABLE_INT1()              EIMSK &= ~PIN1_bm

// 32U4
#elif defined(__AVR_ATmega32U4__)

  #define OUT_B_RF_SS                 PIN0_bm
  #define SET_RF_SS_IS_OUTPUT()       DDRB |= (OUT_B_RF_SS)
  #define RF_SS_ON()                  PORTB |= (OUT_B_RF_SS)
  #define RF_SS_OFF()                 PORTB &= ~(OUT_B_RF_SS)
  #define OUT_B_RF_XCK                PIN1_bm
  #define SET_RF_XCK_IS_OUTPUT()      DDRB |= (OUT_B_RF_XCK)
  #define RF_XCK_ON()                 PORTB |= (OUT_B_RF_XCK)
  #define RF_XCK_OFF()                PORTB &= ~(OUT_B_RF_XCK)
  #define OUT_B_RF_MOSI               PIN2_bm
  #define SET_RF_MOSI_IS_INPUT()      DDRB &= ~(OUT_B_RF_MOSI)
  #define SET_RF_MOSI_IS_OUTPUT()     DDRB |= (OUT_B_RF_MOSI)
  #define IS_RF_MOSI_ON               (PINB & (OUT_B_RF_MOSI))

#if defined(LORA_DIO0_INT0)
  #define ENABLE_LORA_INT_RISING()        { EICRA = (EICRA & ~((1<<ISC00) | (1<<ISC01))) | (3 << ISC00); EIMSK |= (1<<INT0); }
  #define DISABLE_LORA_INT()              { EIMSK &= ~(1<<INT0); }
#elif defined(LORA_DIO0_INT1)
  #define ENABLE_LORA_INT_RISING()        { EICRA = (EICRA & ~((1<<ISC10) | (1<<ISC11))) | (3 << ISC10); EIMSK |= (1<<INT1); }
  #define DISABLE_LORA_INT()              { EIMSK &= ~(1<<INT1) }
#elif defined(LORA_DIO0_INT6)
  #define ENABLE_LORA_INT_RISING()        { EICRB = (EICRB & ~((1<<ISC60) | (1<<ISC61))) | (3 << ISC60); EIMSK |= (1<<INT6); }
  #define DISABLE_LORA_INT()              { EIMSK &= ~(1<<INT6) }
#endif

#endif


#define RX_ADC_PIN_1 A0
#define RX_ADC_PIN_2 A1
#define RX_ADC_PIN_3 A2

#define TX_BUZZER_PIN A5

#define PPM_INPUT_PIN       0 // Has to be one of Interrupt pins



#endif // _BOARD_H_
