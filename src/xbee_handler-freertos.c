/*
 * Copyright (c) 2010, Regents of the University of California
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
 * Firmware for the Biomimetic Millisystems Lab 802.15.4 USB basestation.
 *
 * by Kevin Peterson and Andrew Pullin
 *
 * v.00beta
 *
 * Revisions:
 *  Kevin Peterson   <??>    Initial verison of Xbee UART handler
 *  Andrew Pullin   4-23-14  Cleanup of file, move to proper module, FreeRTOS task
 *
 * Notes:
 *
 * At the moment the basestation uses a very stripped down MAC layer
 * implementation Inter-PAN communication is not possible and all
 * addressing is 16-bit only.
 *
 */

#include <xc.h>
//FreeRTOS includes
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

//Library includes
#include "generic_typedefs.h"
#include "init_default.h"
#include "utils.h"
#include "serial-freertos.h"
#include "mac_packet.h"
#include "radio.h"   //for radio stack calls
#include "payload.h"
#include "xbee_constants.h"
#include "xbee_handler-freertos.h"
#include "at86rf231_driver.h" //for TRX calls
#include "blob.h"
#include <string.h>

//FreeRTOS configuration macros
#define xbeehandlerSTACK_SIZE				configMINIMAL_STACK_SIZE


//////////////////////////////////////////////////
//////////       Private Macros        ///////////
//////////////////////////////////////////////////
//#define FCY                     40000000
#define BAUDRATE                230400  //921600
#define BRGVAL4                 ((FCY/BAUDRATE)/4)-1
#define BRGVAL16                ((FCY/BAUDRATE)/16)-1

#define XB_OVERHEAD_LENGTH      5
#define XB_RX_START             0x7E
#define XB_API_ID               0x81    //API identifier - Currently only support the RX type

#define RX_FRAME_OFFSET         8
//#define RX_DATA_OFFSET          3 //Offset for accounting for RX_START and LEN_HB/LB bytes



//////////////////////////////////////////////////
//////////      Private variables      ///////////
//////////////////////////////////////////////////
static QueueHandle_t serialRXCharQueue;
static QueueHandle_t serialTXBlobQueue;
static QueueHandle_t radioRXQueue;
static QueueHandle_t radioTXQueue;

//////////////////////////////////////////////////
////////// Private function prototypes ///////////
//////////////////////////////////////////////////
static portTASK_FUNCTION_PROTO(vXBeeRXTask, pvParameters);
static portTASK_FUNCTION_PROTO(vXBeeTXTask, pvParameters);

/******************************************************************************
* Function Name : xbeeHandleAt
* Description   : Read the AT command byte and perform the appropriate action
* Parameters    : A payload received from the UART/Python
* Return Value  : None
*******************************************************************************/
void xbeeHandleAT(Payload rx_pld);

/******************************************************************************
* Function Name : xbeeHandleTx
* Description   : Take a payload received over the UART, package it and send
*                 over the radio
* Parameters    : A payload received from the UART/Python
* Return Value  : None
*******************************************************************************/
void xbeeHandleTX(Payload rx_pld);

/******************************************************************************
* Function Name : xbeeHandleRx
* Description   : Retrieve a packet from the radio, put it in the correct format
*                 and pass it along the UART
* Parameters    :
* Return Value  : None
*******************************************************************************/
Blob_t xbeeHandleRX(MacPacket packet);

/******************************************************************************
* Function Name : xbeeHandleATR
* Description   : Create a AT response packet and send it over the UART
* Parameters    : frame_id: pass the frame id from Python back to allow syncing
*                 this packet with the request
*                 command: The command executed by the AT packet
*                 *data: a pointer to an array with the requested return values
*                 length: the number of bytes in *data to send
* Return Value  : None
*******************************************************************************/
void xbeeHandleATR(unsigned char frame_id, WordVal command, unsigned char *data, unsigned char length);

/******************************************************************************
* Function Name : sendUART
* Description   : Send a frame header and payload over the UART
* Parameters    : frame_header: a 5 byte header for sending UART packets in xbee
*                 format
*                 data: any additional payload to be sent
*                 length: length of the data array
* Return Value  : None
*******************************************************************************/
void sendUART(unsigned char *frame_header, unsigned char *data, unsigned char length);



//////////////////////////////////////////////////
////////// Public function definitions ///////////
//////////////////////////////////////////////////

void vXbeeHandlerStartTasks(unsigned portBASE_TYPE uxPriority) {

    //Retrieve queue handles
    radioRXQueue = radioGetRXQueueHandle();
    radioTXQueue = radioGetTXQueueHandle();
    serialTXBlobQueue = serialGetTXQueueHandle();
    serialRXCharQueue = serialGetRXQueueHandle();
    //TODO: check here is handles are OK? Tasks must be started in the right order

    //Create task
    //vXBeeRXTask takes packets coming into the radio, turns them into Xbee
    // formatted data blobs for UART.
    xTaskCreate(vXBeeRXTask, (const char *) "SerialTask", xbeehandlerSTACK_SIZE, NULL, uxPriority, (xTaskHandle *) NULL);
    //xXBeeTXTask takes characters from the serial RX queue and advances
    // the XbeeRX state machine, which handles AT, ATR, and sending operations
    xTaskCreate(vXBeeTXTask, (const char *) "SerialTask", xbeehandlerSTACK_SIZE, NULL, uxPriority, (xTaskHandle *) NULL);

}

//////////////////////////////////////////////////
///////// Private function definitions ///////////
//////////////////////////////////////////////////

//Recieved radio packet, send over UART
Blob_t xbeeHandleRX(MacPacket packet) {

    int i;
    unsigned char checksum;
    unsigned char pld_len = payGetPayloadLength(packet->payload);
    unsigned char* pld_str = payToString(packet->payload);
    unsigned char xb_frame_len = pld_len + XB_OVERHEAD_LENGTH;
    WordVal src_addr = packet->src_addr;

    Blob_t xbeeFmt = blobCreate(pld_len + RX_FRAME_OFFSET);

    checksum = XB_API_ID;
    checksum += src_addr.byte.HB;
    checksum += src_addr.byte.LB;

    xbeeFmt.data[0] = XB_RX_START; //Start Byte
    xbeeFmt.data[1] = 0x00; //Length High Byte
    xbeeFmt.data[2] = xb_frame_len; //Length Low Byte
    xbeeFmt.data[3] = XB_API_ID; //API Identifier - Currently only support the RX type
    xbeeFmt.data[4] = src_addr.byte.HB; //Source Address High byte
    xbeeFmt.data[5] = src_addr.byte.LB; //Source Address Low Byte
    xbeeFmt.data[6] = radioGetLastRSSI(); //'RSSI' Not currently implemented
    xbeeFmt.data[7] = 0x00; //'Options' Not currently implemented

    for (i = 0; i < pld_len; i++) {
        checksum += pld_str[i];
        xbeeFmt.data[RX_FRAME_OFFSET + i] = pld_str[i];
    }

    xbeeFmt.data[RX_FRAME_OFFSET + pld_len] = 0xFF - checksum; //set Checksum byte

    return xbeeFmt;
}


//Recieved UART Xbee packet, send packet out over the radio
//TODO: FREERTOS, change how radio packet is enqueued
void xbeeHandleTX(Payload uart_pld){

    MacPacket tx_packet;
    WordVal dst_addr;
    Payload rx_pld;
    //char test;

    //Get destination address from uart_pld package
    //Frame ID and options packet are currently ignored for this type of packet...
    dst_addr.byte.HB = uart_pld->pld_data[DEST_ADDR_HB_POS - RX_DATA_OFFSET];
    dst_addr.byte.LB = uart_pld->pld_data[DEST_ADDR_LB_POS - RX_DATA_OFFSET];

    //test = dst_addr.byte.LB;

    //Create new packet with just the data that needs to be sent by the radio
    int payloadLength = payGetPayloadLength(uart_pld)-(RX_FRAME_OFFSET-RX_DATA_OFFSET)-PAYLOAD_HEADER_LENGTH-1;
    rx_pld = payCreateEmpty(payloadLength);

    //test = payGetPayloadLength(rx_pld);

    payAppendData(rx_pld, -PAYLOAD_HEADER_LENGTH,
            payGetPayloadLength(rx_pld),
            &(uart_pld->pld_data[RX_DATA_OFFSET]));

    //Place packet in radio queue for sending

    tx_packet = radioRequestPacket(payloadLength);
    tx_packet->payload = rx_pld;
    tx_packet->payload_length = payGetPayloadLength(rx_pld);//rx_pld_len.byte.LB - (RX_FRAME_OFFSET - RX_DATA_OFFSET);
    //tx_packet->dest_pan_id = src_pan_id; //Already set when macCreatePacket is called.
    tx_packet->dest_addr = dst_addr;
    radioEnqueueTxPacket(tx_packet);

}

void xbeeHandleAT(Payload rx_pld)
{
    unsigned char frame;
    unsigned char length;
//    unsigned char test;

    //This should really be some sort of dynamic queue to be generalized
    unsigned char bytes[10];


    //unsigned short int command;
    WordVal command;
    WordVal data;

    length = payGetPayloadLength(rx_pld);
    frame = rx_pld->pld_data[0];

    //test = rx_pld->pld_data[1];
    //test = rx_pld->pld_data[2];

/*
    command = rx_pld->pld_data[1];
    command = command << 8;

    command = command + rx_pld->pld_data[2];
*/
    command.byte.HB = rx_pld->pld_data[1];
    command.byte.LB = rx_pld->pld_data[2];

    switch (command.val)
    {
        case AT_CHANNEL:
            if (length != 3)
            {
                radioSetChannel(rx_pld->pld_data[3]);
            }
            if (frame != 0)
            {
                bytes[0] = radioGetChannel();
                xbeeHandleATR(frame, command, bytes, 1);
            }
            break;
        case AT_PAN_ID:
            if (length != 3)
            {
                data.byte.HB = rx_pld->pld_data[3];
                data.byte.LB = rx_pld->pld_data[4];
                radioSetSrcPanID(data.val);
            }
            if (frame != 0)
            {
                data.val = (unsigned int)radioGetSrcPanID();
                bytes[0] = data.byte.HB;
                bytes[1] = data.byte.LB;

                xbeeHandleATR(frame, command, bytes, 2);
            }
            break;
        case AT_SRC_ADR:
            if (length != 3)
            {
                data.byte.HB = rx_pld->pld_data[3];
                data.byte.LB = rx_pld->pld_data[4];
                radioSetSrcAddr(data.val);
            }
            if (frame != 0)
            {
                data.val = radioGetSrcAddr();
                bytes[0] = data.byte.HB;
                bytes[1] = data.byte.LB;

                xbeeHandleATR(frame, command, bytes, 2);
            }
            break;
        case AT_ACK_LAST:
            if (frame != 0)
            {
                bytes[0] = 0;
                bytes[1] = trxGetLastACKd();

                xbeeHandleATR(frame, command, bytes, 2);
            }
            break;
        case AT_SNIFFER:
            if (length != 3)
                trxSetPromMode(rx_pld->pld_data[3]);  //Put radio in sniffer mode
            break;
    }
}


//Handle an AT response packet and pass it to the sendUART function
void xbeeHandleATR(unsigned char frame_id, WordVal command, unsigned char *data, unsigned char length)
{
#define PREDATA_OFFSET 3
    //Total length:
    //  1 start byte
    //  1 length high byte = 0
    //  1 length low byte  = length+FRAME_HEADER_LEN
    //  5 header bytes
    //  #length data bytes
    //  1 CRC byte
    //  Total: 3 + FRAME_HEADER_LEN + length + 1
    Blob_t response = blobCreate(PREDATA_OFFSET + FRAME_HEADER_LEN + length + 1);

    //Pre-data bytes
    response.data[0] = RX_START;
    response.data[1] = 0x00;                      //Length high byte
    response.data[2] = length + FRAME_HEADER_LEN; //Length low byte
    //Message portion
    response.data[RX_API_POS + PREDATA_OFFSET] = AT_RESPONSE;
    response.data[ATR_FRAME_ID_POS + PREDATA_OFFSET] = frame_id;
    response.data[ATR_COMMAND_HB_POS + PREDATA_OFFSET] = command.byte.HB;
    response.data[ATR_COMMAND_LB_POS + PREDATA_OFFSET] = command.byte.LB;
    response.data[ATR_STATUS_POS + PREDATA_OFFSET] = 0x00;

    //Copy data from passed pointer into a blob to enqueue for serial TX
    memcpy(response.data + ATR_STATUS_POS + PREDATA_OFFSET + 1, data, length);

    //Calculate CRC
    //CRC is over frame header and all data
    unsigned char checksum = 0;
    int i;
    for(i = PREDATA_OFFSET; i < PREDATA_OFFSET + length; i++){
        checksum += response.data[i];
    }
    //Set last byte of blob to checksum value
    response.data[response.length - 1] = 0xff - checksum;
    //response.data[ATR_STATUS_POS + 3 + 1 + length] = 0xff - checksum;

    //Enqueue blob for TX
    portBASE_TYPE xStatus;
    xStatus = xQueueSendToBack( serialTXBlobQueue, &response, portMAX_DELAY );
    blobDestroy(response);
}

//General UART send function
/*
void sendUART(unsigned char *frame_header, unsigned char *data, unsigned char length)
{
    int i;
    unsigned char checksum=0;

    //send frame header
    for(i = 0; i < FRAME_HEADER_LEN; i++)
    {
        checksum += frame_header[i];
        while(BusyUART1());
        WriteUART1(frame_header[i]);
    }

    //send payload data
    for (i = 0; i < length; i++)
    {
        checksum += data[i];
        while(BusyUART1());
        WriteUART1(data[i]);

    }

    //Send Checksum Data
    while(BusyUART1());
    WriteUART1(0xFF - checksum);
}
*/

void xbeeRXStateMachine(unsigned char c){

    static unsigned char uart_rx_state = UART_RX_WAITING;
    static unsigned char uart_rx_cnt = 0;
    static Payload uart_pld;
    static WordVal uart_pld_len;
    static byte    uart_checksum;
    static unsigned char packet_type = 0;
    static unsigned char test;

    if (uart_rx_state == UART_RX_WAITING && c == RX_START)
    {
        uart_rx_state = UART_RX_ON;
        packet_type = 0;
        uart_rx_cnt = 1;
        uart_checksum = 0x00;
    }else if (uart_rx_state == UART_RX_ON)
    {
        switch (uart_rx_cnt)
        {
            //XBee interface uses two bytes for payload length, despite the
            //fact that packets can't be longer than 128 bytes. The high byte
            //is extracted, but never used here.
            case LEN_HB_POS:
                uart_pld_len.byte.HB = c;
                uart_rx_cnt++;
                break;
            case LEN_LB_POS:
                uart_pld_len.byte.LB = c;
                //We create a payload structure to store the data incoming from the uart
                //uart_pld = payCreateEmpty(uart_pld_len.byte.LB-PAYLOAD_HEADER_LENGTH -1 );  //AP 4/18/2014
                uart_pld = payCreateEmpty(uart_pld_len.byte.LB-PAYLOAD_HEADER_LENGTH);
                test = uart_pld_len.byte.LB;
                uart_rx_cnt++;
                break;
            case API_ID_POS:
                //Currently, we're only supporting the 16-bit TX/RX API,
                //and the AT command mode
                packet_type = c;
                uart_checksum += c;
                uart_rx_cnt++;
                break;
            default:
            if (uart_rx_cnt == (uart_pld_len.byte.LB + RX_DATA_OFFSET-1))
            {
                if (uart_checksum + c == 0xFF) //We have a legit packet
                {
                    //Check for type of packet and call relevant function
                    switch (packet_type)
                    {
                        case AT_COMMAND_MODE:
                            xbeeHandleAT(uart_pld);
                            break;

                        case TX_16BIT:
                            xbeeHandleTX(uart_pld);
                            break;

                        default:
                            //do nothing, but probably should send an error
                            break;
                    }
                    payDelete(uart_pld);

                }else //Start over
                {
                    payDelete(uart_pld);
                }

                uart_rx_state = UART_RX_WAITING;
            }else
            {
                uart_pld->pld_data[uart_rx_cnt-RX_DATA_OFFSET] = c;
                uart_checksum += c;
                uart_rx_cnt++;
            }
            break;
        }

    }
}


//////////////////////////////////////////////////
/////////        FreeRTOS Tasks        ///////////
//////////////////////////////////////////////////

// Xbee RX Task:
// This task will simply take packets from the radio RX queue, format them
// into Xbee transactions, and send them serial queue.
static portTASK_FUNCTION(vXBeeRXTask, pvParameters) {
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    portBASE_TYPE xStatus;

    MacPacket recvdRadioPacket;
    Blob_t xbeeFmt;

    for (;;) {
        //Blocking read from radio packet queue
        xStatus = xQueueReceive(radioRXQueue, &recvdRadioPacket, portMAX_DELAY);
        //Send packet to be formatted to appears as an Xbee communication
        xbeeFmt = xbeeHandleRX(recvdRadioPacket);
        //As soon as we have a packet, push it onto the Serial TX queue, also blocking
        xStatus = xQueueSendToBack( serialTXBlobQueue, &xbeeFmt, portMAX_DELAY );
        //Once the blob is safely in the serial queue, the data can be discarded
        blobDestroy(xbeeFmt);
        //TODO: Is yielding neccesary here?
        //taskYIELD();
    }
}

//Xbee TX Task:
// This task will will proccess the serial RX queue of characters, and advance
// the state machine for reciving and parsing packets, or responding to AT/ATR
static portTASK_FUNCTION(vXBeeTXTask, pvParameters) {
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    portBASE_TYPE xStatus;

    char cChar;

    for (;;) {
        //Blocking read from serial char RX queue
        xStatus = xQueueReceive(serialRXCharQueue, &cChar, portMAX_DELAY);
        xbeeRXStateMachine(cChar);
        //Completed incoming packets will be pushed onto the radio queue by the
        // xbeeRXStateMachine function.
        
        //TODO: Is yielding neccesary here?
        //taskYIELD();
    }
}

