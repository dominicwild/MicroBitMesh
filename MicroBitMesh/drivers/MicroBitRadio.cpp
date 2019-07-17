/*
The MIT License (MIT)

Copyright (c) 2016 British Broadcasting Corporation.
This software is provided by Lancaster University by arrangement with the BBC.

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
*/

#include "MicroBitConfig.h"
#include "MicroBitRadio.h"
#include "MicroBitComponent.h"
#include "EventModel.h"
#include "MicroBitDevice.h"
#include "ErrorNo.h"
#include "MicroBitFiber.h"
#include "MicroBitBLEManager.h"
#include "nrf51.h"
#include "MicroBitSystemTimer.h"

/**
  * Provides a simple broadcast radio abstraction, built upon the raw nrf51822 RADIO module.
  *
  * The nrf51822 RADIO module supports a number of proprietary modes of operation in addition to the typical BLE usage.
  * This class uses one of these modes to enable simple, point to multipoint communication directly between micro:bits.
  *
  * TODO: The protocols implemented here do not currently perform any significant form of energy management,
  * which means that they will consume far more energy than their BLE equivalent. Later versions of the protocol
  * should look to address this through energy efficient broadcast techniques / sleep scheduling. In particular, the GLOSSY
  * approach to efficient rebroadcast and network synchronisation would likely provide an effective future step.
  *
  * TODO: Meshing should also be considered - again a GLOSSY approach may be effective here, and highly complementary to
  * the master/slave arachitecture of BLE.
  *
  * TODO: This implementation may only operated whilst the BLE stack is disabled. The nrf51822 provides a timeslot API to allow
  * BLE to cohabit with other protocols. Future work to allow this colocation would be benefical, and would also allow for the
  * creation of wireless BLE bridges.
  *
  * NOTE: This API does not contain any form of encryption, authentication or authorisation. Its purpose is solely for use as a
  * teaching aid to demonstrate how simple communications operates, and to provide a sandpit through which learning can take place.
  * For serious applications, BLE should be considered a substantially more secure alternative.
  */

MicroBitRadio* MicroBitRadio::instance = NULL;

//in micro seconds
#define WAIT_OFF_RX 2
#define WAIT_ON_TX 133

void (*tic_)(void) = NULL;
void (*toc_)(void) = NULL;
void (*output)(char*) = NULL;
void (*printInt)(uint8_t) = NULL;
void (*printInt64)(uint64_t) = NULL;
void (*printChar)(char c) = NULL;
void (*printBuf)(FrameBuffer*) = NULL;

#define DUMMY 55


extern "C" void RADIO_IRQHandler(void) {
    if (NRF_RADIO->EVENTS_READY) {
        NRF_RADIO->EVENTS_READY = 0;
        // Start listening and wait for the END event
        NRF_RADIO->TASKS_START = 1;
    }
    if (NRF_RADIO->EVENTS_END) {
        NRF_RADIO->EVENTS_END = 0;
        if (NRF_RADIO->CRCSTATUS == 1) {
            
            int sample = (int) NRF_RADIO->RSSISAMPLE;
            FrameBuffer* buffer = MicroBitRadio::instance->getRxBuf();

            uint64_t before = system_timer_current_time_us();
            if (MicroBitRadio::instance->gotJam) { //While jammed
                if (buffer->protocol == MICROBIT_MESH_JAM_DATAGRAM) { //Only want to flood jam packets
                    MicroBitRadio::instance->flood();
                } 
            } else {
                if (!(buffer->protocol == MICROBIT_MESH_JAM_DATAGRAM)) { //Dummy "if" to maintain determinism
                    MicroBitRadio::instance->flood();
                }
            }

            MicroBitRadio::instance->gotPacket = true;
            if (buffer->protocol == MICROBIT_MESH_SYNC_DATAGRAM) {
                MicroBitRadio::instance->processSync();
            }

            if (!MicroBitRadio::instance->timeSet) {
                uint64_t diff = system_timer_current_time_us() - before;
                NRF_TIMER0->CC[0] = MicroBitRadio::instance->untilSleep - MicroBitRadio::instance->getRxBuf()->relayCounter * (diff);
                NRF_TIMER0->TASKS_CLEAR = 1;
                NRF_TIMER0->TASKS_START = 1;
                MicroBitRadio::instance->timeSet = true;
            }

            if (buffer->protocol == MICROBIT_MESH_JAM_DATAGRAM && !MicroBitRadio::instance->gotJam) { //Only run this code once per glossy round.
                MicroBitRadio::instance->gotJam = true;
                MicroBitRadio::instance->n = 1; //Already flooded one jam packet
            } else if (buffer->protocol == MICROBIT_MESH_PROTOCOL_DATAGRAM) {
                if (MicroBitRadio::instance->gotMeshPacket) { //If we got another mesh packet to compare to
                    if (MicroBitRadio::instance->lastPacket->seqNum == MicroBitRadio::instance->getRxBuf()->sequenceNumber) { //Compare seqNums
                        for (int i = 0; i < ADDRESS_LENGTH; i++) { //Compare sender names, if differ, send jam packet.
                            if (MicroBitRadio::instance->lastPacket->senderName[i] != MicroBitRadio::instance->getRxBuf()->senderName[i]) {
                                MicroBitRadio::instance->sendJamPacket();
                                break;
                            }
                        }
                        MicroBitRadio::instance->gotRepeat = true; //For confidence packet has flooded network
                    } else {
                        MicroBitRadio::instance->sendJamPacket();
                    }
                } else { //If no mesh packet to compare to, log it and update state to show we have one.
                    MicroBitRadio::instance->gotMeshPacket = true;
                    MicroBitRadio::instance->logPacket(MicroBitRadio::instance->getRxBuf());
                }
            }

        } else {
            MicroBitRadio::instance->setRSSI(0);
        }
        // Start listening and wait for the END event
        NRF_RADIO->TASKS_START = 1;
    }
}

void MicroBitRadio::logPacket(FrameBuffer* buffer){
    memcpy(lastPacket->senderName, buffer->senderName, ADDRESS_LENGTH);
    lastPacket->seqNum = buffer->sequenceNumber;
}

void MicroBitRadio::sendJamPacket(void){
    memcpy(this->jamBuf->senderName, this->radioName, ADDRESS_LENGTH);
    transmit(this->jamBuf);
    MicroBitRadio::instance->gotJam = true;
    MicroBitRadio::instance->n = 1;
}

void MicroBitRadio::processSync(void) {
    SyncBuffer* buffer = (SyncBuffer*) NRF_RADIO->PACKETPTR;
    this->collisions = buffer->collisions;
    this->untilSleep = buffer->untilSleep;
    this->inGlossyRound = true;
}

bool MicroBitRadio::intendedRecipient() {
    FrameBuffer* buffer = MicroBitRadio::instance->getRxBuf();
    ManagedString broadcast("AllBit");
    uint8_t broadcastPtr[6];
    broadcast.toIntArray(broadcastPtr);
    int match;
    if(this->addressEquals(this->radioName,buffer->senderName)){    //If we were the sender, we don't want our own packet
        return false;
    }
    if(this->addressEquals(this->radioName,buffer->recieverName)|| this->addressEquals(this->radioName,broadcastPtr)){
        return true;
    }

    for (int i = 0; i < 6; i++) {       //If it's the broadcast address, we take the packet
        if (this->broadcast[i] != buffer->recieverName[i]) {
            break;
        } else {
            match++;
        }
    }
    if (match == 6) {
        return true;
    }

    for (int i = 0; i < 6; i++) {
        if (this->radioName[i] != buffer->recieverName[i]) {    //If our name is the intended receiver then we take it.
            return false;
        }
    }
    return true;
}
//(DIAMETER + 2)*SEND_TIME = enough time to flood one packet around the entire network
void MicroBitRadio::sendSyncPacket(void){           
    int floodTime = (DIAMETER + 2*N - 1)*SEND_TIME;
    this->untilSleep = (3*floodTime + (1 << (this->collisions+1))*floodTime); 
    SyncBuffer* buf = new SyncBuffer(); 
    buf->collisions = this->collisions;
    buf->untilSleep = this->untilSleep;
    buf->relayCounter = 0;
    buf->version = 0;
    ManagedString broadcast("AllBit");
    broadcast.toIntArray(buf->recieverName);
    memcpy(buf->senderName, this->radioName, ADDRESS_LENGTH);
    buf->protocol = MICROBIT_MESH_SYNC_DATAGRAM;
    buf->group = 0;
    buf->length = SYNC_PACKET_SIZE;
    transmit(buf);
    NRF_TIMER0->CC[0] = this->untilSleep; //- (system_timer_current_time_us() - before);
    NRF_TIMER0->TASKS_CLEAR = 1;
    NRF_TIMER0->TASKS_START = 1;
    this->timeSet = true;
    this->syncN++;
    delete buf;
}

void MicroBitRadio::setInitiator(bool state){
    this->initiator = state;
}

bool MicroBitRadio::addressEquals(uint8_t* address1, uint8_t* address2) {
    for (int i = 0; i < 6; i++) {
        if (address1[i] != address2[i]) {
            return false;
        }
    }
    return true;
}

int MicroBitRadio::flood(void) {
    if (MicroBitRadio::instance->getRxBuf()->protocol == MICROBIT_MESH_SYNC_DATAGRAM) {
        if (syncN++ < N) {
            FrameBuffer* buffer = MicroBitRadio::instance->getRxBuf();
            buffer->relayCounter++;
            transmit(buffer);
        }
    } else {
        if (n++ < N) {
            FrameBuffer* buffer = MicroBitRadio::instance->getRxBuf();
            buffer->relayCounter++;
            transmit(buffer);
        }
    }
    return MICROBIT_OK;
}

void MicroBitRadio::setName(ManagedString name) {
    name.toIntArray(this->radioName);
}

/**
  * Constructor.
  *
  * Initialise the MicroBitRadio.
  *
  * @note This class is demand activated, as a result most resources are only
  *       committed if send/recv or event registrations calls are made.
  */
MicroBitRadio::MicroBitRadio(uint16_t id) : datagram(*this), event (*this)
{
    
    this->id = id;
    this->status = 0;
	this->group = MICROBIT_RADIO_DEFAULT_GROUP;
	this->queueDepth = 0;
    this->rssi = 0;
    this->rxQueue = NULL;
    this->rxBuf = NULL;

    uint8_t padding[6]{0, 0, 0, 0, 0, 0};
    this->initiator = false;
    this->lastPacket = new PacketLog();
    memcpy(this->lastPacket->senderName, padding, ADDRESS_LENGTH);
    this->lastPacket->seqNum = 255;

    ManagedString broadcast("AllBit");  //Pre-allocate parameters of the jam packet
    this->jamBuf = new FrameBuffer();
    this->jamBuf->protocol = MICROBIT_MESH_JAM_DATAGRAM;
    this->jamBuf->group = 0;
    broadcast.toIntArray(this->jamBuf->recieverName);
    this->jamBuf->length = 13;

    instance = this;
}

/**
  * Change the output power level of the transmitter to the given value.
  *
  * @param power a value in the range 0..7, where 0 is the lowest power and 7 is the highest.
  *
  * @return MICROBIT_OK on success, or MICROBIT_INVALID_PARAMETER if the value is out of range.
  */
int MicroBitRadio::setTransmitPower(int power)
{
    if (power < 0 || power >= MICROBIT_BLE_POWER_LEVELS)
        return MICROBIT_INVALID_PARAMETER;

    NRF_RADIO->TXPOWER = (uint32_t)MICROBIT_BLE_POWER_LEVEL[power];

    return MICROBIT_OK;
}

/**
  * Change the transmission and reception band of the radio to the given channel
  *
  * @param band a frequency band in the range 0 - 100. Each step is 1MHz wide, based at 2400MHz.
  *
  * @return MICROBIT_OK on success, or MICROBIT_INVALID_PARAMETER if the value is out of range,
  *         or MICROBIT_NOT_SUPPORTED if the BLE stack is running.
  */
int MicroBitRadio::setFrequencyBand(int band)
{
    if (ble_running())
        return MICROBIT_NOT_SUPPORTED;

    if (band < 0 || band > 100)
        return MICROBIT_INVALID_PARAMETER;

    NRF_RADIO->FREQUENCY = (uint32_t)band;

    return MICROBIT_OK;
}

/**
  * Retrieve a pointer to the currently allocated receive buffer. This is the area of memory
  * actively being used by the radio hardware to store incoming data.
  *
  * @return a pointer to the current receive buffer.
  */
FrameBuffer* MicroBitRadio::getRxBuf()
{
    return rxBuf;
}

/**
  * Attempt to queue a buffer received by the radio hardware, if sufficient space is available.
  *
  * @return MICROBIT_OK on success, or MICROBIT_NO_RESOURCES if a replacement receiver buffer
  *         could not be allocated (either by policy or memory exhaustion).
  */
int MicroBitRadio::queueRxBuf()
{
    if (rxBuf == NULL)
        return MICROBIT_INVALID_PARAMETER;

    if (queueDepth >= MICROBIT_RADIO_MAXIMUM_RX_BUFFERS)
        return MICROBIT_NO_RESOURCES;

    // Store the received RSSI value in the frame
    rxBuf->rssi = getRSSI();

    // Ensure that a replacement buffer is available before queueing.
    FrameBuffer *newRxBuf = new FrameBuffer();

    if (newRxBuf == NULL)
        return MICROBIT_NO_RESOURCES;

    // We add to the tail of the queue to preserve causal ordering.
    rxBuf->next = NULL;

    if (rxQueue == NULL)
    {
        rxQueue = rxBuf;
    }
    else
    {
        FrameBuffer *p = rxQueue;
        while (p->next != NULL)
            p = p->next;

        p->next = rxBuf;
    }

    // Increase our received packet count
    queueDepth++;

    // Allocate a new buffer for the receiver hardware to use. the old on will be passed on to higher layer protocols/apps.
    rxBuf = newRxBuf;

    return MICROBIT_OK;
}

/**
  * Sets the RSSI for the most recent packet.
  * The value is measured in -dbm. The higher the value, the stronger the signal.
  * Typical values are in the range -42 to -128.
  *
  * @param rssi the new rssi value.
  *
  * @note should only be called from RADIO_IRQHandler...
  */
int MicroBitRadio::setRSSI(int rssi)
{
    if (!(status & MICROBIT_RADIO_STATUS_INITIALISED))
        return MICROBIT_NOT_SUPPORTED;

    this->rssi = rssi;

    return MICROBIT_OK;
}

/**
  * Retrieves the current RSSI for the most recent packet.
  * The return value is measured in -dbm. The higher the value, the stronger the signal.
  * Typical values are in the range -42 to -128.
  *
  * @return the most recent RSSI value or MICROBIT_NOT_SUPPORTED if the BLE stack is running.
  */
int MicroBitRadio::getRSSI()
{
    if (!(status & MICROBIT_RADIO_STATUS_INITIALISED))
        return MICROBIT_NOT_SUPPORTED;

    return this->rssi;
}

int MicroBitRadio::fastEnable(){
     // If the device is already initialised, then there's nothing to do.
    if (status & MICROBIT_RADIO_STATUS_INITIALISED)
        return MICROBIT_OK;

    // Only attempt to enable this radio mode if BLE is disabled.
    if (ble_running())
        return MICROBIT_NOT_SUPPORTED;

    //Assume already allocated rxBuf from the previous enable. Thus do not allocate rxBuf or check if rxBuf == NULL
    //Assume high frequency clock is already enabled, from the previous enable().
    //Assume transit and frequency bands set for radio from the previous enable().
    //Assume radio already has default mode configuration. 
    //Assume BASE0 set from previous enable().
    //Assume default group set from previous enable.
    //Assume all other settings taken care of by previous enable().

    NVIC_ClearPendingIRQ(RADIO_IRQn);
    NVIC_EnableIRQ(RADIO_IRQn);

    NRF_RADIO->SHORTS |= RADIO_SHORTS_ADDRESS_RSSISTART_Msk;

    // Start listening for the next packet
    NRF_RADIO->EVENTS_READY = 0;
    NRF_RADIO->TASKS_RXEN = 1;
    while(NRF_RADIO->EVENTS_READY == 0);

    NRF_RADIO->EVENTS_END = 0;
    NRF_RADIO->TASKS_START = 1;

    // register ourselves for a callback event, in order to empty the receive queue.
    fiber_add_idle_component(this);

    // Done. Record that our RADIO is configured.
    status |= MICROBIT_RADIO_STATUS_INITIALISED;

    return MICROBIT_OK;
}

/**
  * Initialises the radio for use as a multipoint sender/receiver
  *
  * @return MICROBIT_OK on success, MICROBIT_NOT_SUPPORTED if the BLE stack is running.
  */
int MicroBitRadio::enable()
{
    // If the device is already initialised, then there's nothing to do.
    if (status & MICROBIT_RADIO_STATUS_INITIALISED)
        return MICROBIT_OK;

    // Only attempt to enable this radio mode if BLE is disabled.
    if (ble_running())
        return MICROBIT_NOT_SUPPORTED;

    // If this is the first time we've been enable, allocate out receive buffers.
    if (rxBuf == NULL)
        rxBuf = new FrameBuffer();

    if (rxBuf == NULL)
        return MICROBIT_NO_RESOURCES;

    // Enable the High Frequency clock on the processor. This is a pre-requisite for
    // the RADIO module. Without this clock, no communication is possible.
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);

    // Bring up the nrf51822 RADIO module in Nordic's proprietary 1MBps packet radio mode.
    setTransmitPower(MICROBIT_RADIO_DEFAULT_TX_POWER);
    setFrequencyBand(MICROBIT_RADIO_DEFAULT_FREQUENCY);

    // Configure for 1Mbps throughput.
    // This may sound excessive, but running a high data rates reduces the chances of collisions...
    NRF_RADIO->MODE = RADIO_MODE_MODE_Nrf_1Mbit;

    // Configure the addresses we use for this protocol. We run ANONYMOUSLY at the core.
    // A 40 bit addresses is used. The first 32 bits match the ASCII character code for "uBit".
    // Statistically, this provides assurance to avoid other similar 2.4GHz protocols that may be in the vicinity.
    // We also map the assigned 8-bit GROUP id into the PREFIX field. This allows the RADIO hardware to perform
    // address matching for us, and only generate an interrupt when a packet matching our group is received.
    NRF_RADIO->BASE0 = MICROBIT_RADIO_BASE_ADDRESS;

    // Join the default group. This will configure the remaining byte in the RADIO hardware module.
    setGroup(this->group);

    // The RADIO hardware module supports the use of multiple addresses, but as we're running anonymously, we only need one.
    // Configure the RADIO module to use the default address (address 0) for both send and receive operations.
    NRF_RADIO->TXADDRESS = 0;
    NRF_RADIO->RXADDRESSES = 1;

    // Packet layout configuration. The nrf51822 has a highly capable and flexible RADIO module that, in addition to transmission
    // and reception of data, also contains a LENGTH field, two optional additional 1 byte fields (S0 and S1) and a CRC calculation.
    // Configure the packet format for a simple 8 bit length field and no additional fields.
    NRF_RADIO->PCNF0 = 0x00000008;
    NRF_RADIO->PCNF1 = 0x02040000 | MICROBIT_RADIO_MAX_PACKET_SIZE;

    // Most communication channels contain some form of checksum - a mathematical calculation taken based on all the data
    // in a packet, that is also sent as part of the packet. When received, this calculation can be repeated, and the results
    // from the sender and receiver compared. If they are different, then some corruption of the data ahas happened in transit,
    // and we know we can't trust it. The nrf51822 RADIO uses a CRC for this - a very effective checksum calculation.
    //
    // Enable automatic 16bit CRC generation and checking, and configure how the CRC is calculated.
    NRF_RADIO->CRCCNF = RADIO_CRCCNF_LEN_Two;
    NRF_RADIO->CRCINIT = 0xFFFF;
    NRF_RADIO->CRCPOLY = 0x11021;

    // Set the start random value of the data whitening algorithm. This can be any non zero number.
    NRF_RADIO->DATAWHITEIV = 0x18;

    // Set up the RADIO module to read and write from our internal buffer.
    NRF_RADIO->PACKETPTR = (uint32_t)rxBuf;

    // Configure the hardware to issue an interrupt whenever a task is complete (e.g. send/receive).
    NRF_RADIO->INTENSET = 0x00000008;
    NVIC_ClearPendingIRQ(RADIO_IRQn);
    NVIC_EnableIRQ(RADIO_IRQn);

    NRF_RADIO->SHORTS |= RADIO_SHORTS_ADDRESS_RSSISTART_Msk;

    // Start listening for the next packet
    NRF_RADIO->EVENTS_READY = 0;
    NRF_RADIO->TASKS_RXEN = 1;
    while(NRF_RADIO->EVENTS_READY == 0);

    NRF_RADIO->EVENTS_END = 0;
    NRF_RADIO->TASKS_START = 1;

    // register ourselves for a callback event, in order to empty the receive queue.
    fiber_add_idle_component(this);

    // Done. Record that our RADIO is configured.
    status |= MICROBIT_RADIO_STATUS_INITIALISED;

    this->configTimer();
    
    return MICROBIT_OK;
}

/**
  * Disables the radio for use as a multipoint sender/receiver.
  *
  * @return MICROBIT_OK on success, MICROBIT_NOT_SUPPORTED if the BLE stack is running.
  */
int MicroBitRadio::disable()
{
    // Only attempt to enable.disable the radio if the protocol is already running.
    if (ble_running())
        return MICROBIT_NOT_SUPPORTED;

    if (!(status & MICROBIT_RADIO_STATUS_INITIALISED))
        return MICROBIT_OK;

    // Disable interrupts and STOP any ongoing packet reception.
    NVIC_DisableIRQ(RADIO_IRQn);

    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_DISABLE = 1;
    while(NRF_RADIO->EVENTS_DISABLED == 0);

    // deregister ourselves from the callback event used to empty the receive queue.
    fiber_remove_idle_component(this);

    // record that the radio is now disabled
    status &= ~MICROBIT_RADIO_STATUS_INITIALISED;

    return MICROBIT_OK;
}

/**
  * Sets the radio to listen to packets sent with the given group id.
  *
  * @param group The group to join. A micro:bit can only listen to one group ID at any time.
  *
  * @return MICROBIT_OK on success, or MICROBIT_NOT_SUPPORTED if the BLE stack is running.
  */
int MicroBitRadio::setGroup(uint8_t group)
{
    if (ble_running())
        return MICROBIT_NOT_SUPPORTED;

    // Record our group id locally
    this->group = group;

    // Also append it to the address of this device, to allow the RADIO module to filter for us.
    NRF_RADIO->PREFIX0 = (uint32_t)group;

    return MICROBIT_OK;
}

/**
  * A background, low priority callback that is triggered whenever the processor is idle.
  * Here, we empty our queue of received packets, and pass them onto higher level protocol handlers.
  */
void MicroBitRadio::idleTick()
{
    // Walk the list of packets and process each one.
    while(rxQueue)
    {
        FrameBuffer *p = rxQueue;

        switch (p->protocol)
        {
            case MICROBIT_RADIO_PROTOCOL_DATAGRAM:
                datagram.packetReceived();
                break;

            case MICROBIT_RADIO_PROTOCOL_EVENTBUS:
                event.packetReceived();
                break;

            default:
                MicroBitEvent(MICROBIT_ID_RADIO_DATA_READY, p->protocol);
        }

        // If the packet was processed, it will have been recv'd, and taken from the queue.
        // If this was a packet for an unknown protocol, it will still be there, so simply free it.
        if (p == rxQueue)
        {
            recv();
            delete p;
        }
    }
}

/**
  * Determines the number of packets ready to be processed.
  *
  * @return The number of packets in the receive buffer.
  */
int MicroBitRadio::dataReady()
{
    return queueDepth;
}

/**
  * Retrieves the next packet from the receive buffer.
  * If a data packet is available, then it will be returned immediately to
  * the caller. This call will also dequeue the buffer.
  *
  * @return The buffer containing the the packet. If no data is available, NULL is returned.
  *
  * @note Once recv() has been called, it is the callers responsibility to
  *       delete the buffer when appropriate.
  */
FrameBuffer* MicroBitRadio::recv()
{
    FrameBuffer *p = rxQueue;

    if (p)
    {
         // Protect shared resource from ISR activity
        NVIC_DisableIRQ(RADIO_IRQn); 

        rxQueue = rxQueue->next;
        queueDepth--;

        // Allow ISR access to shared resource
        NVIC_EnableIRQ(RADIO_IRQn);
    }

    return p;
}


//Starts timer used for deferred operation of sending packet.
void MicroBitRadio::configTimer(void) {
    NRF_TIMER0->TASKS_STOP = 1; // Stop timer, can't configure it while running
    NRF_TIMER0->MODE = TIMER_MODE_MODE_Timer; //In contrast to counter mode
    NRF_TIMER0->BITMODE = (TIMER_BITMODE_BITMODE_24Bit << TIMER_BITMODE_BITMODE_Pos);
    NRF_TIMER0->PRESCALER = 4; // 1us resolution
    NRF_TIMER0->TASKS_CLEAR = 1; // Clear timer tasks
    NRF_TIMER0->INTENSET = TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos; 
    NRF_TIMER0->SHORTS = (TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos);
    NRF_TIMER0->CC[0] = 0;  //For until sleep
    NVIC_EnableIRQ(TIMER0_IRQn);
    NVIC_SetPriority(TIMER0_IRQn,0);
}

extern "C" void TIMER0_IRQHandler(void) {
    if ((NRF_TIMER0->EVENTS_COMPARE[0] != 0)) {
        NRF_TIMER0->TASKS_STOP = 1;
        MicroBitRadio::instance->toggleEnable();
        NRF_TIMER0->EVENTS_COMPARE[0] = 0;
    } 
}

void MicroBitRadio::toggleEnable(void) {
    if (status & MICROBIT_RADIO_STATUS_INITIALISED) {
        this->disable();
        NRF_TIMER0->CC[0] = this->untilSleep*this->sleepCycleFactor;
        NRF_TIMER0->TASKS_CLEAR = 1;
        NRF_TIMER0->TASKS_START = 1;

        this->n = 0;
        this->syncN = 0;
        this->timeSet = false;
        this->inGlossyRound = false;
        this->gotMeshPacket = false;

        if (initiator && gotJam) {
            this->collisions++;
        } else {
            this->collisions = 0;
        }
        //Process packet from this round, decide if it should be added onto received packets queue.
        if (this->rxBuf->protocol == MICROBIT_MESH_PROTOCOL_DATAGRAM && this->gotRepeat && !this->gotJam && this->intendedRecipient()) {
            int sample = (int) NRF_RADIO->RSSISAMPLE;
            MicroBitRadio::instance->setRSSI(-sample);
            FrameBuffer* buffer = MicroBitRadio::instance->getRxBuf();
            MicroBitRadio::instance->queueRxBuf();
            NRF_RADIO->PACKETPTR = (uint32_t) MicroBitRadio::instance->getRxBuf();
            MicroBitRadio::instance->datagram.packetReceived();
        } else {
            delete this->rxBuf;
            this->rxBuf = new FrameBuffer();
            NRF_RADIO->PACKETPTR = (uint32_t)this->rxBuf;
        }
        this->gotRepeat = false;
    } else {
        this->fastEnable(); 
        this->gotJam = false;
        if (this->initiator) {
            this->sendSyncPacket();
        }
    }
}



void MicroBitRadio::transmit(FrameBuffer* buffer){
    
    if (ble_running()){
        output("blue");
        return;
    }

    if (buffer == NULL){
        output("null buffer");
        return ;
    }
    if (buffer->length > MICROBIT_RADIO_MAX_PACKET_SIZE + MICROBIT_RADIO_HEADER_SIZE - 1){
        output("length too big ");
        printInt64(buffer->length);
        return;
    }
    
    // Firstly, disable the Radio interrupt. We want to wait until the transmission completes.
    NVIC_DisableIRQ(RADIO_IRQn);

    // Turn off the transceiver.
    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_DISABLE = 1;
    wait_us(WAIT_OFF_RX);
    // Configure the radio to send the buffer provided.
    NRF_RADIO->PACKETPTR = (uint32_t) buffer;
    
    // Turn on the transmitter, and wait for it to signal that it's ready to use.
    NRF_RADIO->EVENTS_READY = 0;
    NRF_RADIO->TASKS_TXEN = 1;
    wait_us(WAIT_ON_TX);
    // Start transmission and wait for end of packet.
    NRF_RADIO->TASKS_START = 1;
    NRF_RADIO->EVENTS_END = 0;
    while(NRF_RADIO->EVENTS_END == 0);
    
    // Return the radio to using the default receive buffer
    NRF_RADIO->PACKETPTR = (uint32_t) rxBuf;

    // Turn off the transmitter.
    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_DISABLE = 1;
    while(NRF_RADIO->EVENTS_DISABLED == 0);

    // Start listening for the next packet
    NRF_RADIO->EVENTS_READY = 0;
    NRF_RADIO->TASKS_RXEN = 1;
    while(NRF_RADIO->EVENTS_READY == 0);

    NRF_RADIO->EVENTS_END = 0;
    NRF_RADIO->TASKS_START = 1;
    
    // Re-enable the Radio interrupt.
    NVIC_ClearPendingIRQ(RADIO_IRQn);
    NVIC_EnableIRQ(RADIO_IRQn);
}
/**
 * Transmits the given buffer onto the broadcast radio.
 * The call will wait until the transmission of the packet has completed before returning.
 *
 * @param data The packet contents to transmit.
 *
 * @return MICROBIT_OK on success, or MICROBIT_NOT_SUPPORTED if the BLE stack is running.
 */
int MicroBitRadio::meshSend(FrameBuffer *buffer) {
    if (ble_running()){
        output("blue\r\n");
        return MICROBIT_NOT_SUPPORTED;
    }
    if (buffer == NULL){
        output("null buffer\r\n");
        return MICROBIT_INVALID_PARAMETER;
    }
    if (buffer->length > MICROBIT_RADIO_MAX_PACKET_SIZE + MICROBIT_RADIO_HEADER_SIZE - 1){
        output("invalid parameter\r\n");
        return MICROBIT_INVALID_PARAMETER;
    }
    //Experiment variables
    int attempts = 0;
    //
    int floodTime = (DIAMETER + 2*N - 1)*SEND_TIME;
    bool packetSent = false;
    while (!packetSent) {
        if (inGlossyRound){
            int rand = microbit_random(1 << (this->collisions+1));
            wait_us((rand) * floodTime);
            wait_us(floodTime);
            NVIC_DisableIRQ(TIMER0_IRQn);
            NVIC_DisableIRQ(RADIO_IRQn);    //Ensure state will not change when checking
            if (!(gotJam || gotMeshPacket)) { //See if we can send this round
                transmit(buffer);
                gotMeshPacket = true;
                gotPacket = false;      //See if we get a packet from our send, we expect repeats
                this->n++;
                this->logPacket(buffer);
                NVIC_EnableIRQ(RADIO_IRQn);
                NVIC_EnableIRQ(TIMER0_IRQn);
                attempts++;
            } else { //If we can't send this round, try again
                NVIC_EnableIRQ(RADIO_IRQn);
                NVIC_EnableIRQ(TIMER0_IRQn);
                while (inGlossyRound) {
                    wait_us(floodTime); //Wait until out of glossy round to attempt next send. Don't want to be spinning on and off interrupts.
                }
                continue;
            }
            while (inGlossyRound) {
                wait_us(floodTime); //Wait until out of glossy round to send next packet.
            }
            if (!gotJam && gotPacket) { //If we got a jam packet, we have a collision, so send failed and try again next round. Also check if we got a packet, to be confident it flooded the network.
                packetSent = true;
            } 
          
        }
    }
    printInt(attempts);
    return MICROBIT_OK;
}

/**
  * Transmits the given buffer onto the broadcast radio.
  * The call will wait until the transmission of the packet has completed before returning.
  *
  * @param data The packet contents to transmit.
  *
  * @return MICROBIT_OK on success, or MICROBIT_NOT_SUPPORTED if the BLE stack is running.
  */
int MicroBitRadio::send(FrameBuffer *buffer)
{
    if (ble_running())
        return MICROBIT_NOT_SUPPORTED;

    if (buffer == NULL)
        return MICROBIT_INVALID_PARAMETER;

    if (buffer->length > MICROBIT_RADIO_MAX_PACKET_SIZE + MICROBIT_RADIO_HEADER_SIZE - 1)
        return MICROBIT_INVALID_PARAMETER;
    

    // Firstly, disable the Radio interrupt. We want to wait until the transmission completes.
    NVIC_DisableIRQ(RADIO_IRQn);

    // Turn off the transceiver.
    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_DISABLE = 1;
    while(NRF_RADIO->EVENTS_DISABLED == 0);
    // Configure the radio to send the buffer provided.
    NRF_RADIO->PACKETPTR = (uint32_t) buffer;

    // Turn on the transmitter, and wait for it to signal that it's ready to use.
    NRF_RADIO->EVENTS_READY = 0;
    NRF_RADIO->TASKS_TXEN = 1;
    while (NRF_RADIO->EVENTS_READY == 0);
    // Start transmission and wait for end of packet.
    NRF_RADIO->TASKS_START = 1;
    NRF_RADIO->EVENTS_END = 0;
    while(NRF_RADIO->EVENTS_END == 0);
    // Return the radio to using the default receive buffer
    NRF_RADIO->PACKETPTR = (uint32_t) rxBuf;

    // Turn off the transmitter.
    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_DISABLE = 1;
    while(NRF_RADIO->EVENTS_DISABLED == 0);

    // Start listening for the next packet
    NRF_RADIO->EVENTS_READY = 0;
    NRF_RADIO->TASKS_RXEN = 1;
    while(NRF_RADIO->EVENTS_READY == 0);

    NRF_RADIO->EVENTS_END = 0;
    NRF_RADIO->TASKS_START = 1;

    // Re-enable the Radio interrupt.
    NVIC_ClearPendingIRQ(RADIO_IRQn);
    NVIC_EnableIRQ(RADIO_IRQn);

    return MICROBIT_OK;
}

void MicroBitRadio::transmit(SyncBuffer* buffer){
    
    if (ble_running()){
        output("blue");
        return;
    }

    if (buffer == NULL){
        output("null buffer");
        return;
    }
    if (buffer->length > MICROBIT_RADIO_MAX_PACKET_SIZE + MICROBIT_RADIO_HEADER_SIZE - 1){
        output("length too big ");
        printInt64(buffer->length);
        return;
    }
       
    // Firstly, disable the Radio interrupt. We want to wait until the transmission completes.
    NVIC_DisableIRQ(RADIO_IRQn);

    // Turn off the transceiver.
    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_DISABLE = 1;
    wait_us(WAIT_OFF_RX);
    // Configure the radio to send the buffer provided.
    NRF_RADIO->PACKETPTR = (uint32_t) buffer;

    // Turn on the transmitter, and wait for it to signal that it's ready to use.
    NRF_RADIO->EVENTS_READY = 0;
    NRF_RADIO->TASKS_TXEN = 1;
    wait_us(WAIT_ON_TX);
    // Start transmission and wait for end of packet.
    NRF_RADIO->TASKS_START = 1;
    NRF_RADIO->EVENTS_END = 0;
    while(NRF_RADIO->EVENTS_END == 0);

    // Return the radio to using the default receive buffer
    NRF_RADIO->PACKETPTR = (uint32_t) rxBuf;

    // Turn off the transmitter.
    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_DISABLE = 1;
    while(NRF_RADIO->EVENTS_DISABLED == 0);

    // Start listening for the next packet
    NRF_RADIO->EVENTS_READY = 0;
    NRF_RADIO->TASKS_RXEN = 1;
    while(NRF_RADIO->EVENTS_READY == 0);

    NRF_RADIO->EVENTS_END = 0;
    NRF_RADIO->TASKS_START = 1;

    // Re-enable the Radio interrupt.
    NVIC_ClearPendingIRQ(RADIO_IRQn);
    NVIC_EnableIRQ(RADIO_IRQn);
}