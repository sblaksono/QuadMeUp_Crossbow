#include "radio_node.h"
#include "lora.h"

uint32_t getFrequencyForChannel(uint8_t channel) {
    return RADIO_FREQUENCY_MIN + (RADIO_CHANNEL_WIDTH * channel);
}

uint8_t getNextChannel(uint8_t channel) {
    return (channel + RADIO_HOP_OFFSET) % RADIO_CHANNEL_COUNT;
}

uint8_t getPrevChannel(uint8_t channel) {
    return (RADIO_CHANNEL_COUNT + channel - RADIO_HOP_OFFSET) % RADIO_CHANNEL_COUNT;
}

RadioNode::RadioNode(void) {

}

void RadioNode::reset(void) {
    set(
        loraTxPower, 
        loraBandwidth, 
        loraSpreadingFactor, 
        loraCodingRate, 
        getFrequencyForChannel(getChannel())
    );
}

void RadioNode::init(uint8_t ss, uint8_t rst, uint8_t di0, void(*callback)(int)) {
    /*
     * Setup hardware
     */
    LoRa_setPins(
        ss,
        rst,
        di0
    );

    if (!LoRa_begin(getFrequencyForChannel(getChannel()))) {
        while (true);
    }

    reset();
    LoRa_enableCrc();

    //Setup ISR callback and start receiving
    LoRa_onReceive(callback);
    LoRa_receive();
    radioState = RADIO_STATE_RX;
}

void RadioNode::readRssi(void)
{
    rssi = 164 - constrain(LoRa_packetRssi() * -1, 0, 164);
}

void RadioNode::readSnr(void)
{
    snr = (uint8_t) constrain(LoRa_packetSnr(), 0, 255);
}

uint8_t RadioNode::getChannel(void) {
    return _channel;
}

uint32_t RadioNode::getChannelEntryMillis(void) {
    return _channelEntryMillis;
}

void RadioNode::readAndDecode(
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

        radioState = RADIO_STATE_RX;
        bytesToRead = NO_DATA_TO_READ;
    }
}

void RadioNode::hopFrequency(bool forward, uint8_t fromChannel, uint32_t timestamp) {
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

void RadioNode::handleChannelDwell(void) {
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

void RadioNode::handleTxDoneState(bool hop) {
    uint32_t currentMillis = millis();
    
    if (
        currentMillis > nextTxCheckMillis &&
        radioState == RADIO_STATE_TX &&
        !LoRa_isTransmitting()
    ) {

        /*
         * In case of TX module, hop right now
         */
        if (hop) {
            hopFrequency(true, getChannel(), currentMillis);
        }

        LoRa_receive();
        radioState = RADIO_STATE_RX;
        nextTxCheckMillis = currentMillis + 1; //We check of TX done every 1ms
    }
}

void RadioNode::handleTx(QspConfiguration_t *qsp, uint8_t bindKey[]) {

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
    radioState = RADIO_STATE_TX;
}

void RadioNode::set(
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
