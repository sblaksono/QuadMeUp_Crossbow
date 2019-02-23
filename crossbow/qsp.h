#ifndef QSP_H
#define QSP_H

#include <stdint.h>

#include "config.h"

#define QSP_PAYLOAD_LENGTH 32

#define QSP_MAX_FRAME_DECODE_TIME 10 //max time that frame can be decoded in ms

#define QSP_FRAME_RC_DATA 0x0
#define QSP_FRAME_RX_HEALTH 0x1
#define QSP_FRAME_GET_RX_CONFIG 0x2
#define QSP_FRAME_RX_CONFIG 0x3
#define QSP_FRAME_SET_RX_CONFIG 0x4
#define QSP_FRAME_PING 0x5
#define QSP_FRAME_PONG 0x6
#define QSP_FRAME_BIND 0x7
#define QSP_FRAME_COUNT 0x8

struct TxDeviceState_t {
    uint8_t flags = 0;
    uint32_t roundtrip = 0;
    bool isReceiving = false; //Indicates that TX module is receiving frames from RX module
};

struct RxDeviceState_t {
    uint8_t rssi = 0;
    uint8_t snr = 0;
    uint8_t rxVoltage = 0;
    uint8_t a1Voltage = 0;
    uint8_t a2Voltage = 0;
    uint8_t flags = 0;
    int16_t indicatedRssi = 0;
};

static const uint8_t qspFrameLengths[QSP_FRAME_COUNT] = {
    9, //QSP_FRAME_RC_DATA
    6, //QSP_FRAME_RX_HEALTH
    0, //QSP_FRAME_GET_RX_CONFIG -> Not used
    0, //QSP_FRAME_RX_CONFIG -> Not used
    0, //QSP_FRAME_SET_RX_CONFIG -> Not used
    4, //QSP_FRAME_PING
    4, //QSP_FRAME_PONG
    4  //QSP_FRAME_BIND
};

enum dataStates {
    QSP_STATE_IDLE,
    QSP_STATE_FRAME_TYPE_RECEIVED,
    QSP_STATE_PAYLOAD_RECEIVED,
    QSP_STATE_CRC_RECEIVED
};

struct QspConfiguration_t {
    uint8_t protocolState = QSP_STATE_IDLE;
    uint8_t crc = 0;
    uint8_t payload[QSP_PAYLOAD_LENGTH] = {0};
    uint8_t payloadLength = 0;
    uint8_t frameToSend = 0;
    uint8_t frameId = 0;
    uint32_t lastFrameReceivedAt[QSP_FRAME_COUNT] = {0};
    uint32_t anyFrameRecivedAt = 0;
    void (* onSuccessCallback)(QspConfiguration_t*, TxDeviceState_t*, RxDeviceState_t*, uint8_t receivedChannel);
    void (* onFailureCallback)(QspConfiguration_t*, TxDeviceState_t*, RxDeviceState_t*);
    int (* rcChannelGetCallback)(uint8_t);
    void (* setRcChannelCallback)(uint8_t channel, int value, int offset);
    bool forcePongFrame = false;
    uint32_t frameDecodingStartedAt = 0;
    uint32_t lastTxSlotTimestamp = 0;
    bool transmitWindowOpen = false;
};

void qspDecodeRcDataFrame(QspConfiguration_t *qsp, RxDeviceState_t *rxDeviceSate);
void decodeRxHealthPayload(QspConfiguration_t *qsp, RxDeviceState_t *rxDeviceState);

uint8_t get10bitHighShift(uint8_t channel);
uint8_t get10bitLowShift(uint8_t channel);
void qspComputeCrc(QspConfiguration_t *qsp, uint8_t dataByte);
void encodeRxHealthPayload(QspConfiguration_t *qsp, RxDeviceState_t *rxDeviceState, uint8_t rssi, uint8_t snr, bool isFailsafe);
void encodeRcDataPayload(QspConfiguration_t *qsp, uint8_t noOfChannels);
void qspDecodeIncomingFrame(
    QspConfiguration_t *qsp, 
    uint8_t incomingByte, 
    RxDeviceState_t *rxDeviceState,
    TxDeviceState_t *txDeviceState,
    uint8_t bindKey[]
);
void qspClearPayload(QspConfiguration_t *qsp);
void qspEncodeFrame(QspConfiguration_t *qsp, uint8_t buffer[], uint8_t *size, uint8_t radioChannel, uint8_t bindKey[]);

void encodePingPayload(QspConfiguration_t *qsp, uint32_t currentMicros);
void encodeBindPayload(QspConfiguration_t *qsp, uint8_t bindKey[]);

#endif
