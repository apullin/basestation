/**
 * Copyright (c) 2011-2013, Regents of the University of California
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * - Neither the name of the University of California, Berkeley nor the names
 *   of its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * High Level Wireless Communications Driver
 *
 * by Humphrey Hu
 *
 * v.0.5
 *
 * Revisions:
 *  Humphrey Hu     2011-6-6    Initial implementation
 *  Humphrey Hu     2012-2-3    Structural changes to reduce irq handler runtime
 *  Humphrey Hu     2012-7-18   Consolidated state into data structures
 */

#include <xc.h>
//FreeRTOS includes
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

//Library includes
#include "utils.h"
#include "init_default.h"
#include "radio-freertos.h"
#include "mac_packet.h"
#include "sclock.h"
#include "timer.h"
#include "ppool.h"

#include "at86rf231.h"  // Current transceiver IC
#include "at86rf231_driver.h"

#include <stdlib.h>
#include <string.h>

#define RADIO_DEFAULT_SRC_ADDR          (0x1101)
#define RADIO_DEFAULT_SRC_PAN           (0x1001)
#define RADIO_DEFAULT_CHANNEL           (0x15)
#define RADIO_DEFAULT_HARD_RETRIES      (3)
#define RADIO_DEFAULT_SOFT_RETRIES      (2)
#define RADIO_DEFAULT_WATCHDOG_STATE    (0) // Default off
#define RADIO_DEFAULT_WATCHDOG_TIME     (400000) // 400 ms timeout

// TODO (fgb) : Calibration is recommended in the datasheet, yet needs testing
#define RADIO_AUTOCALIBRATE             (0)
#define RADIO_CALIB_PERIOD              (300000000) // 5 minutes

//FreeRTOS configuration macros
#define radiotaskSTACK_SIZE				configMINIMAL_STACK_SIZE

// =========== Static variables ===============================================
// State information
static unsigned char is_ready = 0;
static RadioStatus status;
static RadioConfiguration configuration;

// In/out packet FIFO queues
//static CircArray tx_queue, rx_queue;
static QueueHandle_t radioRXQueue;
static QueueHandle_t radioTXQueue;

// =========== Function stubs =================================================
static portTASK_FUNCTION_PROTO(vRadioTask, pvParameters);

// IRQ handlers
void trxCallback(unsigned int irq_cause);
static inline void watchdogProgress(void);

static void radioReset(void);

// Internal processing
static void radioProcessTx(TickType_t timeout);
static void radioProcessRx(TickType_t timeout);

// Internal state management methods
static unsigned int radioBeginTransition(void);
static unsigned int radioSetStateTx(void);
static unsigned int radioSetStateRx(void);
static unsigned int radioSetStateIdle(void);
static unsigned int radioSetStateOff(void);

// =========== Public functions ===============================================

// Initialize radio software and hardware
void radioInit(unsigned int tx_queue_length, unsigned int rx_queue_length) {

    RadioConfiguration conf;

    trxSetup(TRX_CS); // Configure transceiver IC and driver
    trxSetIrqCallback(&trxCallback);

    //tx_queue = carrayCreate(tx_queue_length);
    //rx_queue = carrayCreate(rx_queue_length);
    radioTXQueue = xQueueCreate(tx_queue_length, (unsigned portBASE_TYPE) sizeof ( MacPacket));
    radioRXQueue = xQueueCreate(rx_queue_length, (unsigned portBASE_TYPE) sizeof ( MacPacket));

    status.state = STATE_IDLE;
    status.packets_sent = 0;
    status.packets_received = 0;
    status.sequence_number = 0;
    status.retry_number = 0;
    status.last_rssi = 0;
    status.last_ed = 0;
    status.last_calibration = 0;
    status.last_progress = 0;

    conf.address.address = RADIO_DEFAULT_SRC_ADDR;
    conf.address.pan_id = RADIO_DEFAULT_SRC_PAN;
    conf.address.channel = RADIO_DEFAULT_CHANNEL;
    conf.soft_retries = RADIO_DEFAULT_SOFT_RETRIES;
    conf.hard_retries = RADIO_DEFAULT_HARD_RETRIES;
    conf.watchdog_running = RADIO_DEFAULT_WATCHDOG_STATE;
    conf.watchdog_timeout = RADIO_DEFAULT_WATCHDOG_TIME;
    radioConfigure(&conf);

    is_ready = 1;

    radioSetStateRx();

}

void radioConfigure(RadioConfiguration *conf) {

    if(conf == NULL) { return; }
    memcpy(&configuration, conf, sizeof(RadioConfiguration));

    radioSetAddress(&conf->address);
    radioSetSoftRetries(conf->soft_retries);
    radioSetHardRetries(conf->hard_retries);
    if(conf->watchdog_running) { radioEnableWatchdog(); }
    else { radioDisableWatchdog(); }
    radioSetWatchdogTime(conf->watchdog_timeout);

}

void radioGetConfiguration(RadioConfiguration *conf) {
    if(conf == NULL) { return; }
    memcpy(conf, &configuration, sizeof(RadioConfiguration));
}

void radioGetStatus(RadioStatus *stat) {
    if(stat == NULL) { return; }
    memcpy(stat, &status, sizeof(RadioStatus));
}

unsigned char radioGetLastRSSI(){
    return status.last_rssi;
}

void radioSetAddress(RadioAddress *address) {

    if(address == NULL) { return; }

    radioSetSrcAddr(address->address);
    radioSetSrcPanID(address->pan_id);
    radioSetChannel(address->channel);

}

void radioSetSrcAddr(unsigned int src_addr) {
    configuration.address.address = src_addr;
    trxSetAddress(src_addr);
}

unsigned int radioGetSrcAddr(){
    return configuration.address.address;
}

void radioSetSrcPanID(unsigned int src_pan_id) {
    configuration.address.pan_id = src_pan_id;
    trxSetPan(src_pan_id);
}

unsigned int radioGetSrcPanID() {
    return configuration.address.pan_id;
}

void radioSetChannel(unsigned char channel) {
    configuration.address.channel = channel;
    trxSetChannel(channel);
}

unsigned char radioGetChannel() {
    return configuration.address.channel;
}

void radioSetSoftRetries(unsigned char retries) {
    configuration.soft_retries = retries;
}

void radioSetHardRetries(unsigned char retries) {
    trxSetRetries(retries);
}

void radioSetWatchdogState(unsigned char state) {
    if(state == 0) { radioDisableWatchdog(); }
    else if(state == 1) { radioEnableWatchdog(); }
}

void radioEnableWatchdog(void) {
    configuration.watchdog_running = 1;
    watchdogProgress();
}

void radioDisableWatchdog(void) {
    configuration.watchdog_running = 0;
}

void radioSetWatchdogTime(unsigned int time) {
    configuration.watchdog_timeout = time;
    watchdogProgress();
}

MacPacket radioDequeueRxPacket(TickType_t delay)
{
    if ( !is_ready ) return NULL;

    //return (MacPacket)carrayPopTail(rx_queue);
    MacPacket packet;
    portBASE_TYPE xStatus;
    xStatus = xQueueReceive(radioRXQueue, &packet,  delay);
    return packet;
}

unsigned int radioEnqueueTxPacket(MacPacket packet, TickType_t delay) {
    if(!is_ready) { return 0; }
    //return carrayAddTail(tx_queue, packet);
    portBASE_TYPE xStatus;
    xStatus = xQueueSendToBack(radioRXQueue, &packet,  delay);
    return (xStatus == pdPASS);
}

unsigned int radioTxQueueEmpty(void) {
    //return carrayIsEmpty(tx_queue);
    return ( radioGetTxQueueSize() == 0);
}

unsigned int radioTxQueueFull(void) {
    //return carrayIsFull(tx_queue);
    portBASE_TYPE spaces;
    spaces = uxQueueSpacesAvailable(radioTXQueue);
    return (spaces ==0 );
}

unsigned int radioGetTxQueueSize(void) {
    //return carrayGetSize(tx_queue);
    portBASE_TYPE size;
    size = uxQueueMessagesWaiting(radioTXQueue);
    return size;
}

unsigned int radioRxQueueEmpty(void){
    return ( radioGetRxQueueSize() == 0);
}

unsigned int radioRxQueueFull(void) {
    //return carrayIsFull(rx_queue);
    portBASE_TYPE spaces;
    spaces = uxQueueSpacesAvailable(radioRXQueue);
    return ( spaces ==0 );
}

unsigned int radioGetRxQueueSize(void) {
    //return carrayGetSize(rx_queue);
    portBASE_TYPE size;
    size = uxQueueMessagesWaiting(radioRXQueue);
    return size;
}

void radioFlushQueues(void) {
    MacPacket pkt;

    while (!radioRxQueueEmpty()) {
        pkt = (MacPacket)xQueueReceive(radioRXQueue, &pkt, 0);
        radioReturnPacket(pkt);
    }
    while (!radioTxQueueEmpty()) {
        pkt = (MacPacket)xQueueReceive(radioTXQueue, &pkt, 0);
        radioReturnPacket(pkt);
    }
}

MacPacket radioRequestPacket(unsigned int data_size) {

    MacPacket packet;

    packet = ppoolRequestFullPacket(data_size);
    if(packet == NULL) { return NULL; }

    macSetSrc(packet, configuration.address.pan_id, configuration.address.address);
    macSetDestPan(packet, configuration.address.pan_id);

    return packet;

}

MacPacket radioCreatePacket(unsigned int data_size) {
    return NULL;
}

unsigned int radioReturnPacket(MacPacket packet) {
    return ppoolReturnFullPacket(packet);
}

void radioDeletePacket(MacPacket packet) {
    return;
}

// The Big Function
void radioProcess(void) {

    unsigned long currentTime;

    currentTime = sclockGetTime();

    if(configuration.watchdog_running) {
        if(currentTime - status.last_progress > configuration.watchdog_timeout) {
            radioReset();
            return;
        }
    }

    // Process pending outgoing packets
    if(!radioTxQueueEmpty()) {

        // Return if can't get to Tx state at the moment
        if(!radioSetStateTx()) { return; }
        watchdogProgress();
        radioProcessTx(0); // Process outgoing buffer
        return;

    }

    if (RADIO_AUTOCALIBRATE)
    {
        // Check if calibration is necessary
        currentTime = sclockGetTime();
        if(currentTime - status.last_calibration > RADIO_CALIB_PERIOD)
        {
            if ( !radioSetStateOff() ) return;
            trxCalibrate();
            status.last_calibration = currentTime;
        }
    }

    // Default to Rx state
    if(!radioSetStateRx()) { return; }

    // If the code runs to this point, all buffers are clear and radio is idle
    watchdogProgress();

}

unsigned char radioSendData (unsigned int dest_addr, unsigned char status,
                             unsigned char type, unsigned int datalen,
                             unsigned char* dataptr, unsigned char fast_fail)
{
    MacPacket packet;
    Payload pld;

    packet = radioRequestPacket(datalen);
    if( packet == NULL ){
        if (fast_fail) {
            return EXIT_FAILURE;
        } else {
            while ( packet == NULL ) {
               radioProcess();
               packet = radioRequestPacket(datalen);
            }
        }
    }
    macSetDestAddr(packet, dest_addr);          // SRC and PAN already set

    pld = macGetPayload(packet);
    paySetData(pld, datalen, (unsigned char*) dataptr);
    paySetType(pld, type);
    paySetStatus(pld, status);

    if (fast_fail)
    {
        if ( !radioEnqueueTxPacket(packet, 0) ){
            radioReturnPacket(packet);
            return EXIT_FAILURE;
        }
    } else {
        while ( !radioEnqueueTxPacket(packet, 0) ) radioProcess();
    }

    return EXIT_SUCCESS;
}


// =========== Private functions ==============================================
/*
void __attribute__((interrupt, no_auto_psv)) _T3Interrupt(void) {

    // Disable and reset timer
    DisableIntT3;
    WriteTimer3(0);
    radioReset();
    _T3IF = 0;

}
*/
static void radioReset(void) {

    trxReset();
    status.state = STATE_OFF;
    radioSetStateIdle();
    watchdogProgress();
    LED_1 = ~LED_1;

}

/**
 * Transceiver interrupt handler
 *
 * Note that this doesn't need critical sections since this will
 * only be called in interrupt context
 * @param irq_cause Interrupt source code
 */
void trxCallback(unsigned int irq_cause) {

    if(status.state == STATE_SLEEP) {
        // Shouldn't be here since sleep isn't implemented yet!
    }
    else if(status.state == STATE_IDLE) {
        // Shouldn't be getting interrupts when idle
    }
    else if(status.state == STATE_RX_IDLE) {

        // Beginning reception process
        if(irq_cause == RADIO_RX_START) {
            status.state = STATE_RX_BUSY;
        }

    } else if(status.state == STATE_RX_BUSY) {

        // Reception complete
        if(irq_cause == RADIO_RX_SUCCESS) {
            radioProcessRx(0);   // Process newly received data
            status.last_rssi = trxReadRSSI();
            status.last_ed = trxReadED();
            status.state = STATE_RX_IDLE;    // Transition after data processed
        }

    } else if(status.state == STATE_TX_IDLE) {
        // Shouldn't be getting interrupts when waiting to transmit
    } else if(status.state == STATE_TX_BUSY) {

        status.state = STATE_TX_IDLE;
        // Transmit successful
        if(irq_cause == RADIO_TX_SUCCESS) {
            MacPacket pkt;
            portBASE_TYPE xStatus;
            xStatus = xQueueReceive(radioTXQueue, &pkt, 0);
            radioReturnPacket(pkt);
            radioSetStateRx();
        } else if(irq_cause == RADIO_TX_FAILURE) {
            // If no more retries, reset retry counter
            status.retry_number++;
            if(status.retry_number > configuration.soft_retries) {
                status.retry_number = 0;
                MacPacket pkt;
                portBASE_TYPE xStatus;
                xStatus = xQueueReceive(radioTXQueue, &pkt, 0);
                radioReturnPacket(pkt);
                radioSetStateRx();
            }
        }
    }

    // Hardware error
    if(irq_cause == RADIO_HW_FAILURE) {
        // Reset everything
        trxReset();
        //radioFlushQueues();
    }
}

/**
 * Set the radio to a transmit state
 */
static unsigned int radioSetStateTx(void) {

    unsigned int lockAcquired;

    // If already in Tx mode
    if(status.state == STATE_TX_IDLE) { return 1; }

    // Attempt to begin transition
    lockAcquired = radioBeginTransition();
    if(!lockAcquired) { return 0; }

    trxSetStateTx();
    status.state = STATE_TX_IDLE;
    return 1;

}

/**
 * Set the radio to a receive state
 */
static unsigned int radioSetStateRx(void) {

    unsigned int lockAcquired;

    // If already in Rx mode
    if(status.state == STATE_RX_IDLE) { return 1; }

    // Attempt to begin transitionin
    lockAcquired = radioBeginTransition();
    if(!lockAcquired) { return 0; }

    trxSetStateRx();
    status.state = STATE_RX_IDLE;
    return 1;

}

/**
 * Sets the radio to an idle state
 */
static unsigned int radioSetStateIdle(void) {

    unsigned int lockAcquired;

    // If already in idle mode
    if(status.state == STATE_IDLE) { return 1; }

    // Attempt to begin transitionin
    lockAcquired = radioBeginTransition();
    if(!lockAcquired) { return 0; }

    trxSetStateIdle();
    status.state = STATE_IDLE;
    return 1;

}

/**
* Sets the radio to an off state
*/
static unsigned int radioSetStateOff(void) {

   unsigned int lockAcquired;

   // If already in idle mode
   if(status.state == STATE_OFF) { return 1; }

   // Attempt to begin transitionin
   lockAcquired = radioBeginTransition();
   if(!lockAcquired) { return 0; }

   trxSetStateOff();
   status.state = STATE_OFF;
   return 1;

}

/**
 * Atomically checks and set the radio to transitioning state.
 *
 * Note that the radio in transitioning state will capture but disregard
 * interrupts from the transceiver.
 * @return 1 if lock acquired, 0 otherwise
 */
static unsigned int radioBeginTransition(void) {

    unsigned int busy;

    CRITICAL_SECTION_START

    busy =  (status.state == STATE_RX_BUSY)
    || (status.state == STATE_TX_BUSY)
    || (status.state == STATE_TRANSITIONING);

    if(!busy) {
        status.state = STATE_TRANSITIONING;
    }

    CRITICAL_SECTION_END

    return !busy;

}

/**
 * Process a pending packet send request
 */
static void radioProcessTx(TickType_t timeout) {

    MacPacket packet;
    portBASE_TYPE xStatus;

    xStatus = xQueuePeek(radioTXQueue, &packet, timeout);

    //packet = (MacPacket) carrayPeekHead(tx_queue); // Find an outgoing packet
    //if(packet == NULL) { return; }
    if(xStatus != pdPASS) { return; }

    // State should be STATE_TX_IDLE upon entering function
    status.state = STATE_TX_BUSY;    // Update state

    macSetSeqNum(packet, status.sequence_number++); // Set packet sequence number

    trxWriteFrameBuffer(packet); // Write packet to transmitter and send
    trxBeginTransmission();

}

/**
 * Process a pending packet receive request
 */
static void radioProcessRx(TickType_t timeout) {

    MacPacket packet;
    unsigned char len;

    if(radioRxQueueFull()) { return; } // Don't bother if rx queue full

    len = trxReadBufferDataLength(); // Read received frame data length
    packet = radioRequestPacket(len - PAYLOAD_HEADER_LENGTH); // Pull appropriate packet from pool

    if(packet == NULL) { return; }

    trxReadFrameBuffer(packet); // Retrieve frame from transceiver
    packet->timestamp = sclockGetTime(); // Mark local time of reception

    //if(!carrayAddTail(rx_queue, packet)) {
    //    radioReturnPacket(packet); // Check for failure
    //}
    portBASE_TYPE xStatus;
    xStatus = xQueueSendToBack(radioRXQueue, &packet, timeout);
    if(xStatus != pdPASS) {
        radioReturnPacket(packet); // Check for failure
    }
}

static inline void watchdogProgress(void) {

    status.last_progress = sclockGetTime();

}

QueueHandle_t radioGetTXQueueHandle(void) {
    return radioTXQueue;
}

QueueHandle_t radioGetRXQueueHandle(void) {
    return radioRXQueue;
}

void vXbeeHandlerStartTasks(unsigned portBASE_TYPE uxPriority) {
    xTaskCreate(vRadioTask, (const char *) "RadioRXTask", radiotaskSTACK_SIZE, NULL, uxPriority, (xTaskHandle *) NULL);
}

static portTASK_FUNCTION(vRadioTask, pvParameters) {
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    portBASE_TYPE xStatus;

    for (;;) {
        //TODO: Is yielding neccesary here?
        //taskYIELD();
    }
}