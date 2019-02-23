#ifndef CROSSBOW_H
#define CROSSBOW_H

#include "qsp.h"
#include "variables.h"
#include "config.h"

extern volatile uint8_t RadioNode_radioState;
extern uint8_t RadioNode_rssi;
extern uint8_t RadioNode_snr;
extern uint32_t RadioNode_loraBandwidth;
extern uint8_t RadioNode_loraSpreadingFactor;
extern uint8_t RadioNode_loraCodingRate;
extern uint8_t RadioNode_loraTxPower; // Defines output power of TX, defined in dBm range from 2-17

enum deviceStates {
    DEVICE_STATE_OK,
    DEVICE_STATE_FAILSAFE,
    DEVICE_STATE_UNDETERMINED
};

enum platformConfigMemoryLayout {
    EEPROM_ADDRESS_BIND_KEY_SEEDED = 0x00,
    EEPROM_ADDRESS_BIND_0,
    EEPROM_ADDRESS_BIND_1,
    EEPROM_ADDRESS_BIND_2,
    EEPROM_ADDRESS_BIND_3,
    PLATFORM_CONFIG_LAST_BYTE
};

#define PLATFORM_TOTAL_CHANNEL_COUNT 11 //Including RSSI channel and other
#define PLATFORM_CHANNEL_COUNT 10
#define PLATFORM_DEFAULT_CHANNEL_VALUE 1000

void PlatformNode_enterBindMode(void);
void PlatformNode_leaveBindMode(void);

extern uint8_t PlatformNode_platformState;
extern bool PlatformNode_isBindMode;

void Crossbow_setup();
void Crossbow_loop();

//extern RadioNode radioNode;
//extern PlatformNode platformNode;


#endif
