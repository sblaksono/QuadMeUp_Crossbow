#include "config.h"
#include "board.h"
#include "lora.h"
#include "qsp.h"
#include "crossbow.h"

#define SBUS_UPDATE_RATE 15 //ms
#define SBUS_PACKET_LENGTH 25

#define RC_CHANNEL_MIN 990
#define RC_CHANNEL_MAX 2010

#define RX_TASK_HEALTH 200 //5Hz should be enough
#define RSSI_CHANNEL 11

#define MIN_PACKET_SIZE 3 //Min theorethical size of valid packet
#define MAX_PACKET_SIZE 20 //Max theorethical size of valid packet

#define NO_DATA_TO_READ -1

/*
 * Main defines for device working in TX mode
 */
#ifdef DEVICE_MODE_TX

#ifdef FEATURE_TX_INPUT_PPM

  #include "ppmreader.h"
  PPMReader txInput(false);

#elif defined(FEATURE_TX_INPUT_SBUS)

  #include "sbus.h"
  SbusInput txInput(SBUS_SERIAL);

#else
  #error please select tx input source
#endif

#include "txbuzzer.h"

BuzzerState_t buzzer;

#ifdef FEATURE_TX_OLED
#include "tx_oled.h"
TxOled oled;
#include "tactile.h"
Tactile button0(BUTTON_0_PIN);
Tactile button1(BUTTON_1_PIN);
#endif

#endif

/*
 * Main defines for device working in RX mode
 */
#ifdef DEVICE_MODE_RX

#ifdef FEATURE_RX_OUTPUT_SBUS

#include "sbus.h"

uint32_t sbusTime = 0;
uint8_t sbusPacket[SBUS_PACKET_LENGTH] = {0};

#endif

uint32_t lastRxStateTaskTime = 0;

#endif


#if defined(ARDUINO_AVR_FEATHER32U4) || defined(ARDUINO_PRO_MICRO) || defined(ARDUINO_PRO_MINI)
#include <EEPROM.h>
#elif defined(ARDUINO_SAMD_FEATHER_M0)
// Include EEPROM-like API for FlashStorage
#include <FlashAsEEPROM.h>
#endif

volatile int8_t bytesToRead = -1;

uint8_t lastReceivedChannel = 0;
uint8_t failedDwellsCount = 0;

bool canTransmit = false;
uint8_t _channel = 0;
uint32_t _channelEntryMillis = 0;
uint32_t nextTxCheckMillis = 0;

volatile uint8_t RadioNode_radioState = RADIO_STATE_RX;
uint8_t RadioNode_rssi = 0;
uint8_t RadioNode_snr = 0;
uint32_t RadioNode_loraBandwidth = 250000;
uint8_t RadioNode_loraSpreadingFactor = 7;
uint8_t RadioNode_loraCodingRate = 6;
uint8_t RadioNode_loraTxPower = 17; // Defines output power of TX, defined in dBm range from 2-17

uint8_t bindKey[4];
uint32_t nextLedUpdate = 0;
uint8_t PlatformNode_platformState = DEVICE_STATE_UNDETERMINED;
bool PlatformNode_isBindMode = false;
uint32_t bindModeExitMillis;
uint32_t bindModeChangeMillis;

volatile int _channels[PLATFORM_TOTAL_CHANNEL_COUNT];

int8_t txSendSequence[16] = {
    QSP_FRAME_PING,
    QSP_FRAME_RC_DATA,
    QSP_FRAME_RC_DATA,
    QSP_FRAME_RC_DATA,
    QSP_FRAME_RC_DATA,
    QSP_FRAME_RC_DATA,
    QSP_FRAME_RC_DATA,
    QSP_FRAME_RC_DATA,
    QSP_FRAME_RC_DATA,
    QSP_FRAME_RC_DATA,
    QSP_FRAME_RC_DATA,
    QSP_FRAME_RC_DATA,
    QSP_FRAME_RC_DATA,
    QSP_FRAME_RC_DATA,
    QSP_FRAME_RC_DATA,
    QSP_FRAME_RC_DATA
};

int8_t rxSendSequence[16] = {
    QSP_FRAME_RX_HEALTH,
    -1,
    -1,
    -1,
    QSP_FRAME_RX_HEALTH,
    -1,
    -1,
    -1,
    QSP_FRAME_RX_HEALTH,
    -1,
    -1,
    -1,
    QSP_FRAME_RX_HEALTH,
    -1,
    -1,
    -1
};

uint32_t getFrequencyForChannel(uint8_t channel) {
    return RADIO_FREQUENCY_MIN + (RADIO_CHANNEL_WIDTH * channel);
}

uint8_t getNextChannel(uint8_t channel) {
    return (channel + RADIO_HOP_OFFSET) % RADIO_CHANNEL_COUNT;
}

uint8_t getPrevChannel(uint8_t channel) {
    return (RADIO_CHANNEL_COUNT + channel - RADIO_HOP_OFFSET) % RADIO_CHANNEL_COUNT;
}

void readRssi(void)
{
    RadioNode_rssi = 164 - constrain(LoRa_packetRssi() * -1, 0, 164);
}

void readSnr(void)
{
    RadioNode_snr = (uint8_t) constrain(LoRa_packetSnr(), 0, 255);
}

uint8_t getChannel(void) {
    return _channel;
}

uint32_t getChannelEntryMillis(void) {
    return _channelEntryMillis;
}

void RadioNode_set(
    uint8_t power,
    long bandwidth,
    uint8_t spreadingFactor,
    uint8_t codingRate,
    long frequency
) {
    LoRa_sleep();

    LoRa_setTxPower(power);
    LoRa_setSignalBandwidth(bandwidth);
    LoRa_setCodingRate4(codingRate);
    LoRa_setFrequency(frequency);

    LoRa_idle();
}

void RadioNode_reset(void) {
    RadioNode_set(
        RadioNode_loraTxPower,
        RadioNode_loraBandwidth,
        RadioNode_loraSpreadingFactor,
        RadioNode_loraCodingRate,
        getFrequencyForChannel(getChannel())
    );
}

void RadioNode_init(void(*callback)(int)) {

    if (!LoRa_begin(getFrequencyForChannel(getChannel()))) {
        while (true);
    }

    RadioNode_reset();
    LoRa_enableCrc();

    //Setup ISR callback and start receiving
    LoRa_onReceive(callback);
    LoRa_receive();
    RadioNode_radioState = RADIO_STATE_RX;
}

void readAndDecode(
    QspConfiguration_t *qsp,
    RxDeviceState_t *rxDeviceState,
    TxDeviceState_t *txDeviceState,
    uint8_t bindKey[]
) {
    uint8_t tmpBuffer[MAX_PACKET_SIZE];
    /*
     * There is data to be read from radio!
     */
    if (bytesToRead != NO_DATA_TO_READ) {
        LoRa_read(tmpBuffer, bytesToRead);

        for (int i = 0; i < bytesToRead; i++) {
            qspDecodeIncomingFrame(qsp, tmpBuffer[i], rxDeviceState, txDeviceState, bindKey);
        }

        //After reading, flush radio buffer, we have no need for whatever might be over there
        LoRa_sleep();
        LoRa_receive();

        RadioNode_radioState = RADIO_STATE_RX;
        bytesToRead = NO_DATA_TO_READ;
    }
}

void hopFrequency(bool forward, uint8_t fromChannel, uint32_t timestamp) {
    _channelEntryMillis = timestamp;

    if (forward) {
        _channel = getNextChannel(fromChannel);
    } else {
        _channel = getPrevChannel(fromChannel);
    }

    // And set hardware
    LoRa_sleep();
    LoRa_setFrequency(
        getFrequencyForChannel(_channel)
    );
    LoRa_idle();
}

void handleChannelDwell(void) {
    //In the beginning just keep jumping forward and try to resync over lost single frames
    if (failedDwellsCount < 6 && getChannelEntryMillis() + RX_CHANNEL_DWELL_TIME < millis()) {
        failedDwellsCount++;
        hopFrequency(true, getChannel(), getChannelEntryMillis() + RX_CHANNEL_DWELL_TIME);
        LoRa_receive();
    }

    // If we are loosing more frames, start jumping in the opposite direction since probably we are completely out of sync now
    if (failedDwellsCount >= 6 && getChannelEntryMillis() + (RX_CHANNEL_DWELL_TIME * 5) < millis()) {
        hopFrequency(false, getChannel(), getChannelEntryMillis() + RX_CHANNEL_DWELL_TIME); //Start jumping in opposite direction to resync
        LoRa_receive();
    }
}

void handleTxDoneState(bool hop) {
    uint32_t currentMillis = millis();

    if (
        currentMillis > nextTxCheckMillis &&
        RadioNode_radioState == RADIO_STATE_TX &&
        !LoRa_isTransmitting()
    ) {

        /*
         * In case of TX module, hop right now
         */
        if (hop) {
            hopFrequency(true, getChannel(), currentMillis);
        }

        LoRa_receive();
        RadioNode_radioState = RADIO_STATE_RX;
        nextTxCheckMillis = currentMillis + 1; //We check of TX done every 1ms
    }
}

void handleTx(QspConfiguration_t *qsp, uint8_t bindKey[]) {

    if (!canTransmit) {
        return;
    }

    uint8_t size;
    uint8_t tmpBuffer[MAX_PACKET_SIZE];

    LoRa_beginPacket();
    //Prepare packet
    qspEncodeFrame(qsp, tmpBuffer, &size, getChannel(), bindKey);
    //Sent it to radio in one SPI transaction
    LoRa_write(tmpBuffer, size);
    LoRa_endPacketAsync();

    //Set state to be able to detect the moment when TX is done
    RadioNode_radioState = RADIO_STATE_TX;
}



void PlatformNode_init(void) {
    for (uint8_t i = 0; i < PLATFORM_TOTAL_CHANNEL_COUNT; i++) {
        _channels[i] = PLATFORM_DEFAULT_CHANNEL_VALUE;
    }
}

/**
 * Return true if new bind key was generated
 */
void seed(void) {
    uint8_t val;

    val = EEPROM.read(EEPROM_ADDRESS_BIND_KEY_SEEDED);

    if (val != 0xf1) {
        EEPROM.write(EEPROM_ADDRESS_BIND_0, random(1, 255)); //Yes, from 1 to 254
        EEPROM.write(EEPROM_ADDRESS_BIND_1, random(1, 255)); //Yes, from 1 to 254
        EEPROM.write(EEPROM_ADDRESS_BIND_2, random(1, 255)); //Yes, from 1 to 254
        EEPROM.write(EEPROM_ADDRESS_BIND_3, random(1, 255)); //Yes, from 1 to 254
        EEPROM.write(EEPROM_ADDRESS_BIND_KEY_SEEDED, 0xf1);
        #ifdef ARDUINO_SAMD_FEATHER_M0
        EEPROM.commit();
        #endif
    }
}

void loadBindKey(uint8_t key[]) {
    key[0] = EEPROM.read(EEPROM_ADDRESS_BIND_0);
    key[1] = EEPROM.read(EEPROM_ADDRESS_BIND_1);
    key[2] = EEPROM.read(EEPROM_ADDRESS_BIND_2);
    key[3] = EEPROM.read(EEPROM_ADDRESS_BIND_3);
}

void saveBindKey(uint8_t key[]) {
    EEPROM.write(EEPROM_ADDRESS_BIND_0, key[0]);
    EEPROM.write(EEPROM_ADDRESS_BIND_1, key[1]);
    EEPROM.write(EEPROM_ADDRESS_BIND_2, key[2]);
    EEPROM.write(EEPROM_ADDRESS_BIND_3, key[3]);
    EEPROM.write(EEPROM_ADDRESS_BIND_KEY_SEEDED, 0xf1);
    #ifdef ARDUINO_SAMD_FEATHER_M0
    EEPROM.commit();
    #endif
}

int getRcChannel(uint8_t channel) {
    if (channel < PLATFORM_TOTAL_CHANNEL_COUNT) {
        return _channels[channel];
    } else {
        return PLATFORM_DEFAULT_CHANNEL_VALUE;
    }
}

void setRcChannel(uint8_t channel, int value, int offset) {
    if (channel < PLATFORM_TOTAL_CHANNEL_COUNT) {
        _channels[channel] = value + offset;
    }
}

void PlatformNode_enterBindMode(void) {
    PlatformNode_isBindMode = true;

    // Set temporary bind mode
    bindKey[0] = 0xf1;
    bindKey[1] = 0x1e;
    bindKey[2] = 0x07;
    bindKey[3] = 0x42;

    RadioNode_set(
        0, // Minimum power
        RADIO_CHANNEL_WIDTH_BIND, // 125kHz bandwidth
        6, // low spreading factor, we do not need high RX sensitivity
        5, // same for coding rate
        RADIO_FREQUENCY_BIND //Fixed frequency while binding
    );
    bindModeExitMillis = millis() + 1000; //This happens only on RX
}

void PlatformNode_leaveBindMode(void) {
    PlatformNode_isBindMode = false;
    loadBindKey(bindKey);
    RadioNode_reset();
}




/*
 * Start of QSP protocol implementation
 */
QspConfiguration_t qsp = {};
RxDeviceState_t rxDeviceState = {};
TxDeviceState_t txDeviceState = {};

void onQspSuccess(QspConfiguration_t *qsp, TxDeviceState_t *txDeviceState, RxDeviceState_t *rxDeviceState, uint8_t receivedChannel) {
    //If recide received a valid frame, that means it can start to talk
    lastReceivedChannel = receivedChannel;

    //RX can start transmitting only when an least one frame has been receiveds
    canTransmit = true;

    readRssi();
    readSnr();

    /*
     * RX module hops to next channel after frame has been received
     */
#ifdef DEVICE_MODE_RX
    if (!PlatformNode_isBindMode) {
        //We do not hop frequency in bind mode!
        hopFrequency(true, lastReceivedChannel, millis());
        failedDwellsCount = 0; // We received a frame, so we can just reset this counter
        LoRa_receive(); //Put radio back into receive mode
    }
#endif

    //Store the last timestamp when frame was received
    if (qsp->frameId < QSP_FRAME_COUNT) {
        qsp->lastFrameReceivedAt[qsp->frameId] = millis();
    }
    qsp->anyFrameRecivedAt = millis();
    switch (qsp->frameId) {
        case QSP_FRAME_RC_DATA:
            qspDecodeRcDataFrame(qsp, rxDeviceState);
            break;

        case QSP_FRAME_RX_HEALTH:
            decodeRxHealthPayload(qsp, rxDeviceState);
            break;

        case QSP_FRAME_PING:
            qsp->forcePongFrame = true;
            break;

        case QSP_FRAME_BIND:
#ifdef DEVICE_MODE_RX
            if (PlatformNode_isBindMode) {
                bindKey[0] = qsp->payload[0];
                bindKey[1] = qsp->payload[1];
                bindKey[2] = qsp->payload[2];
                bindKey[3] = qsp->payload[3];

                saveBindKey(bindKey);
                PlatformNode_leaveBindMode();
            }
#endif
            break;

        case QSP_FRAME_PONG:
            txDeviceState->roundtrip = qsp->payload[0];
            txDeviceState->roundtrip += (uint32_t) qsp->payload[1] << 8;
            txDeviceState->roundtrip += (uint32_t) qsp->payload[2] << 16;
            txDeviceState->roundtrip += (uint32_t) qsp->payload[3] << 24;

            txDeviceState->roundtrip = (micros() - txDeviceState->roundtrip) / 1000;
            break;

        default:
            //Unknown frame
            //TODO do something in this case
            break;
    }

    qsp->transmitWindowOpen = true;
}

void onQspFailure(QspConfiguration_t *qsp, TxDeviceState_t *txDeviceState, RxDeviceState_t *rxDeviceState) {

}

void Crossbow_onReceive(int packetSize)
{
    /*
     * We can start reading only when radio is not reading.
     * If not reading, then we might start
     */
    if (bytesToRead == NO_DATA_TO_READ) {
        if (packetSize >= MIN_PACKET_SIZE && packetSize <= MAX_PACKET_SIZE) {
            //We have a packet candidate that might contain a valid QSP packet
            bytesToRead = packetSize;
        } else {
            /*
            That packet was not very interesting, just flush it, we have no use
            */
            LoRa_sleep();
            LoRa_receive();
            RadioNode_radioState = RADIO_STATE_RX;
        }
    }
}

void Crossbow_setup()
{

    PlatformNode_init();

    qsp.onSuccessCallback = onQspSuccess;
    qsp.onFailureCallback = onQspFailure;
    qsp.rcChannelGetCallback = getRcChannel;
    qsp.setRcChannelCallback = setRcChannel;

#ifdef DEVICE_MODE_RX
    PlatformNode_platformState = DEVICE_STATE_FAILSAFE;
#else
    PlatformNode_platformState = DEVICE_STATE_OK;

    txInput.setRcChannelCallback = setRcChannel;

#endif

    RadioNode_init(Crossbow_onReceive);

#ifdef FEATURE_BIND_BUTTON
    SET_BIND_BUTTON_PIN_MODE();
    BIND_BUTTON_PULLUP();
    delay(10);
#endif

#ifdef DEVICE_MODE_RX

    pinMode(RX_ADC_PIN_1, INPUT);
    pinMode(RX_ADC_PIN_2, INPUT);
    pinMode(RX_ADC_PIN_3, INPUT);

#ifdef FEATURE_RX_OUTPUT_SBUS
    /*
     * Prepare Serial1 for S.Bus processing
     */
    SBUS_SERIAL_BEGIN(100000, SERIAL_8E2);
#endif

#ifdef FEATURE_BIND_BUTTON
    if (IS_BIND_BUTTON_HIGH()) {
        PlatformNode_leaveBindMode();
    }
    else {
        PlatformNode_enterBindMode();
    }
#else    
    PlatformNode_enterBindMode();
#endif    
    LoRa_receive(); //TODO this probably should be moved somewhere....
#endif

#ifdef DEVICE_MODE_TX

    randomSeed(analogRead(A4));
    seed();

#ifdef FEATURE_TX_OLED
    oled.init();
    oled.page(TX_PAGE_INIT);
    /*
     * Buttons on TX module
     */
    button0.start();
    button1.start();
#endif

    /*
     * TX should start talking imediately after power up
     */
    canTransmit = true;

#ifdef FEATURE_TX_BUZZER
    pinMode(TX_BUZZER_PIN, OUTPUT);

    //Play single tune to indicate power up
    buzzerSingleMode(BUZZER_MODE_CHIRP, &buzzer);
#endif

    /*
     * Prepare Serial1 for S.Bus processing
     */
    txInput.start();

    loadBindKey(bindKey);

#endif

    INIT_LEDS();

}

uint8_t currentSequenceIndex = 0;
#define TRANSMIT_SEQUENCE_COUNT 16

#ifdef DEVICE_MODE_RX

void updateRxDeviceState(RxDeviceState_t *rxDeviceState) {
    rxDeviceState->rxVoltage = map(analogRead(RX_ADC_PIN_1), 0, 1024, 0, 255);
    rxDeviceState->a1Voltage = map(analogRead(RX_ADC_PIN_2), 0, 1024, 0, 255);
    rxDeviceState->a2Voltage = map(analogRead(RX_ADC_PIN_3), 0, 1024, 0, 255);
}

int8_t getFrameToTransmit(QspConfiguration_t *qsp) {

    if (qsp->forcePongFrame) {
        qsp->forcePongFrame = false;
        return QSP_FRAME_PONG;
    }

    int8_t retVal = rxSendSequence[currentSequenceIndex];

    currentSequenceIndex++;
    if (currentSequenceIndex >= TRANSMIT_SEQUENCE_COUNT) {
        currentSequenceIndex = 0;
    }

    return retVal;
}

#endif

#ifdef DEVICE_MODE_TX
int8_t getFrameToTransmit(QspConfiguration_t *qsp) {

    if (PlatformNode_isBindMode) {
        return QSP_FRAME_BIND;
    }

    int8_t retVal = txSendSequence[currentSequenceIndex];

    currentSequenceIndex++;
    if (currentSequenceIndex >= TRANSMIT_SEQUENCE_COUNT) {
        currentSequenceIndex = 0;
    }

    return retVal;
}
#endif

/*
 *
 * Main loop starts here!
 *
 */
void Crossbow_loop()
{

    static uint32_t nextKey = millis();

    uint32_t currentMillis = millis();

#ifdef DEVICE_MODE_RX

#ifndef FEATURE_BIND_BUTTON
    //Make sure to leave bind mode when binding is done
    if (PlatformNode_isBindMode && millis() > bindModeExitMillis) {
        PlatformNode_leaveBindMode();
    }
#endif

    /*
     * This routine handles resync of TX/RX while hoppping frequencies
     * When not in bind mode. Bind mode is single frequency operation
     */
    if (!PlatformNode_isBindMode) {
        handleChannelDwell();
    }

    /*
     * Detect the moment when radio module stopped transmittig and put it
     * back in to receive state
     */
    handleTxDoneState(false);
#else

#ifdef FEATURE_BIND_BUTTON
    if (IS_BIND_BUTTON_HIGH()) {
        bindModeChangeMillis = millis() + 3000;      
    }
    else {
        if (millis() > bindModeChangeMillis) {
            if (PlatformNode_isBindMode) {
                PlatformNode_leaveBindMode();
            }
            else {
                PlatformNode_enterBindMode();
            }
            bindModeChangeMillis = millis() + 3000;      
        }      
    }
#endif

#ifdef FEATURE_TX_OLED
    //Process buttons
    button0.loop();
    button1.loop();

    oled.loop();
#endif

#ifdef FEATURE_TX_INPUT_SBUS
    txInput.recoverStuckFrames();

    /*
     * If we are not receiving SBUS frames from radio, try to restart serial
     */
    static uint32_t serialRestartMillis = 0;

    /*
     * Final guard for SBUS input. If there is no input, try to restart serial port
     */
    if (!txInput.isReceiving() && serialRestartMillis + 100 < currentMillis) {
        txInput.restart();
        serialRestartMillis = currentMillis;
    }
#endif

    handleTxDoneState(!PlatformNode_isBindMode);
#endif

    readAndDecode(
        &qsp,
        &rxDeviceState,
        &txDeviceState,
        bindKey
    );

    bool transmitPayload = false;

    /*
     * Watchdog for frame decoding stuck somewhere in the middle of a process
     */
    if (
        qsp.protocolState != QSP_STATE_IDLE &&
        qsp.frameDecodingStartedAt + QSP_MAX_FRAME_DECODE_TIME < currentMillis
    ) {
        qsp.protocolState = QSP_STATE_IDLE;
    }

#ifdef DEVICE_MODE_TX

    txInput.loop();

    if (
        RadioNode_radioState == RADIO_STATE_RX &&
        qsp.protocolState == QSP_STATE_IDLE &&
        qsp.lastTxSlotTimestamp + TX_TRANSMIT_SLOT_RATE < currentMillis
    ) {

        int8_t frameToSend = getFrameToTransmit(&qsp);

    #ifndef FORCE_TX_WITHOUT_INPUT
        /*
         * If module is not receiving data from radio, do not send RC DATA
         * This is the only way to trigger failsafe in that case
         */
        if (frameToSend == QSP_FRAME_RC_DATA && !txInput.isReceiving()) {
            frameToSend = -1;
        }
    #endif

        if (frameToSend > -1) {

            qsp.frameToSend = frameToSend;
            qspClearPayload(&qsp);

            switch (qsp.frameToSend) {
                case QSP_FRAME_PING:
                    encodePingPayload(&qsp, micros());
                    break;

                case QSP_FRAME_RC_DATA:
                    encodeRcDataPayload(&qsp, PLATFORM_CHANNEL_COUNT);
                    break;

                case QSP_FRAME_BIND:

                    /*
                     * Key to be transmitted is stored in EEPROM
                     * During binding different key is used
                     */
                    uint8_t key[4];
                    loadBindKey(key);

                    encodeBindPayload(&qsp, key);
                    break;
            }

            transmitPayload = true;
        }

        qsp.lastTxSlotTimestamp = currentMillis;
    }

#endif

#ifdef DEVICE_MODE_RX
    /*
     * This routine updates RX device state and updates one of radio channels with RSSI value
     */
    if (lastRxStateTaskTime + RX_TASK_HEALTH < currentMillis) {
        lastRxStateTaskTime = currentMillis;
        updateRxDeviceState(&rxDeviceState);

        uint8_t output = constrain(RadioNode_rssi - 40, 0, 100);

        rxDeviceState.indicatedRssi = (output * 10) + 1000;
    }

    /*
     * Main routine to answer to TX module
     */
    if (qsp.transmitWindowOpen && qsp.protocolState == QSP_STATE_IDLE) {
        qsp.transmitWindowOpen = false;

        int8_t frameToSend = getFrameToTransmit(&qsp);
        if (frameToSend > -1) {
            qsp.frameToSend = frameToSend;

            if (frameToSend != QSP_FRAME_PONG) {
                qspClearPayload(&qsp);
            }
            switch (qsp.frameToSend) {
                case QSP_FRAME_PONG:
                    /*
                     * Pong frame just responses with received payload
                     */
                    break;

                case QSP_FRAME_RX_HEALTH:
                    encodeRxHealthPayload(
                        &qsp,
                        &rxDeviceState,
                        RadioNode_rssi,
                        RadioNode_snr,
                        (PlatformNode_platformState == DEVICE_STATE_FAILSAFE)
                    );
                    break;
            }

            transmitPayload = true;
        }

    }

#ifdef FEATURE_RX_OUTPUT_SBUS
    if (currentMillis > sbusTime) {
        setRcChannel(RSSI_CHANNEL - 1, rxDeviceState.indicatedRssi, 0);
        sbusPreparePacket(sbusPacket, false, (PlatformNode_platformState == DEVICE_STATE_FAILSAFE), getRcChannel);
        SBUS_SERIAL_WRITE(sbusPacket, SBUS_PACKET_LENGTH);
        sbusTime = currentMillis + SBUS_UPDATE_RATE;
    }
#endif

    if (qsp.lastFrameReceivedAt[QSP_FRAME_RC_DATA] + RX_FAILSAFE_DELAY < currentMillis) {
        PlatformNode_platformState = DEVICE_STATE_FAILSAFE;
        rxDeviceState.indicatedRssi = 0;
        RadioNode_rssi = 0;
    } else {
        PlatformNode_platformState = DEVICE_STATE_OK;
    }

#endif

    if (transmitPayload)
    {
        handleTx(&qsp, bindKey);
    }

#ifdef DEVICE_MODE_TX

#ifdef FEATURE_TX_BUZZER
    buzzerProcess(TX_BUZZER_PIN, currentMillis, &buzzer);
#endif

    // This routing enables when TX starts to receive signal from RX for a first time or after
    // failsafe
    if (txDeviceState.isReceiving == false && qsp.anyFrameRecivedAt != 0) {
        //TX module started to receive data
#ifdef FEATURE_TX_BUZZER
        buzzerSingleMode(BUZZER_MODE_DOUBLE_CHIRP, &buzzer);
#endif
        txDeviceState.isReceiving = true;
        PlatformNode_platformState = DEVICE_STATE_OK;
    }

    //Here we detect failsafe state on TX module
    if (
        txDeviceState.isReceiving &&
        qsp.anyFrameRecivedAt + TX_FAILSAFE_DELAY < currentMillis
    ) {
        txDeviceState.isReceiving = false;
        rxDeviceState.a1Voltage = 0;
        rxDeviceState.a2Voltage = 0;
        rxDeviceState.rxVoltage = 0;
        rxDeviceState.rssi = 0;
        rxDeviceState.snr = 0;
        rxDeviceState.flags = 0;
        txDeviceState.roundtrip = 0;
        PlatformNode_platformState = DEVICE_STATE_FAILSAFE;
        qsp.anyFrameRecivedAt = 0;
    }

    //FIXME rxDeviceState should be resetted also in RC_HEALT frame is not received in a long period

#ifdef FEATURE_TX_BUZZER
    //Handle audible alarms
    if (PlatformNode_platformState == DEVICE_STATE_FAILSAFE) {
        //Failsafe detected by TX
        buzzerContinousMode(BUZZER_MODE_SLOW_BEEP, &buzzer);
    } else if (txDeviceState.isReceiving && (rxDeviceState.flags & 0x1) == 1) {
        //Failsafe reported by RX module
        buzzerContinousMode(BUZZER_MODE_SLOW_BEEP, &buzzer);
    } else if (txDeviceState.isReceiving && RadioNode_rssi < 45) {
        buzzerContinousMode(BUZZER_MODE_DOUBLE_CHIRP, &buzzer); // RSSI below 45dB // Critical state
    } else if (txDeviceState.isReceiving && RadioNode_rssi < 55) {
        buzzerContinousMode(BUZZER_MODE_CHIRP, &buzzer); // RSSI below 55dB // Warning state
    } else {
        buzzerContinousMode(BUZZER_MODE_OFF, &buzzer);
    }
#endif

#endif

}


void Crossbow_updateLeds()
{
    uint32_t currentMillis = millis();

    /*
     * Handle LED updates
     */
    if (nextLedUpdate < currentMillis) {
#ifdef DEVICE_MODE_TX
        if (PlatformNode_isBindMode) {
            nextLedUpdate = currentMillis + 50;
            STATUS_LED_TOGGLE();
        } else {
            if (txDeviceState.isReceiving) {
                STATUS_LED_TOGGLE();
                nextLedUpdate = currentMillis + 300;
            } else if (txInput.isReceiving()) {
                STATUS_LED_TOGGLE();
                nextLedUpdate = currentMillis + 100;
            } else {
                STATUS_LED_ON();
                nextLedUpdate = currentMillis + 200;
            }
        }
#else
        if (PlatformNode_isBindMode) {
            nextLedUpdate = currentMillis + 50;
            STATUS_LED_TOGGLE();
        } else {
            nextLedUpdate = currentMillis + 200;
            if (PlatformNode_platformState == DEVICE_STATE_FAILSAFE) {
                STATUS_LED_ON();
            } else {
                STATUS_LED_TOGGLE();
            }
        }
#endif
    }


}
