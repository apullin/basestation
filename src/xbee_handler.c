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
 * by Kevin Peterson
 *
 * v.00beta
 *
 * Revisions:
 *  <AUTHOR     DATE    COMMENT>
 *
 * Notes:
 *
 * At the moment the basestation uses a very stripped down MAC layer
 * implementation Inter-PAN communication is not possible and all
 * addressing is 16-bit only.
 *
 * Usage:
 *  <SAMPLE USAGE>
 */

#include <xc.h>
#include "generic_typedefs.h"
#include "init_default.h"
#include "utils.h"
#include "init.h"
#include "uart.h"
#include "mac_packet.h"
#include "radio.h"   //for radio stack calls
#include "payload.h"
#include <stdio.h>
#include "xbee_constants.h"
#include "xbee_handler.h"
#include "at86rf231_driver.h" //for TRX calls

/////////////////////////////////////////////////////////////////
//#define FCY                     40000000
#define BAUDRATE                230400  //921600
#define BRGVAL4                 ((FCY/BAUDRATE)/4)-1
#define BRGVAL16                ((FCY/BAUDRATE)/16)-1

#define SER_DATAWAIT            100


#define XB_OVERHEAD_LENGTH      5
#define XB_RX_START             0x7E
#define XB_API_ID               0x81    //API identifier - Currently only support the RX type

#define RX_FRAME_OFFSET         8
//#define RX_DATA_OFFSET          3 //Offset for accounting for RX_START and LEN_HB/LB bytes


#define DMA_UART_TX_BUFFER_SIZE 128
#define DMA_UART_RX_BUFFER_SIZE 128


/*-----------------------------------------------------------------------------
 *          Static Variables
-----------------------------------------------------------------------------*/

unsigned char txBufferA[DMA_UART_TX_BUFFER_SIZE] __attribute__((space(dma)));
unsigned char rxBuffer[DMA_UART_RX_BUFFER_SIZE];

static volatile unsigned char dmaTxReady = 1;

void __attribute__((__interrupt__, no_auto_psv)) _DMA6Interrupt(void) {
    dmaTxReady = 1;
    IFS4bits.DMA6IF = 0;    //clear the DMA6 interrupt flag
}

void xbSetupDma(void) {

    DMA6CON = 0x6001;       // byte, one-shot, post-increment, RAM 2 Peripheral
    DMA6CNT = 0;
    #if defined(__IMAGEPROC1) || defined(__BASESTATION)
        DMA6REQ = 0x000C;       //  select UART1 transmitter
        DMA6PAD = (volatile unsigned int) &U1TXREG;
    #elif defined(__EXP16DEV) || defined(__BASESTATION2)
        DMA6REQ = 0x001F;       //  select UART2 transmitter
        DMA6PAD = (volatile unsigned int) &U2TXREG;
    #else
        #error "UART/DMA is not defined on this project"
    #endif


    DMA6STA = __builtin_dmaoffset(txBufferA);

    IPC17bits.DMA6IP = 4;
    IFS4bits.DMA6IF = 0;    // clear DMA interrupt flag
    IEC4bits.DMA6IE = 1;    // enable DMA interrupt
}

//Recieved radio packet, send over UART
void xbeeHandleRx(MacPacket packet) {

    int i;
    unsigned char checksum;
    unsigned char pld_len = payGetPayloadLength(packet->payload);
    unsigned char* pld_str = payToString(packet->payload);
    unsigned char xb_frame_len = pld_len + XB_OVERHEAD_LENGTH;
    WordVal src_addr = packet->src_addr;

    dmaTxReady = 0;
    checksum = XB_API_ID;
    checksum += src_addr.byte.HB;
    checksum += src_addr.byte.LB;

    CRITICAL_SECTION_START
        txBufferA[0] = XB_RX_START;     //Start Byte
        txBufferA[1] = 0x00; 	        //Length High Byte
        txBufferA[2] = xb_frame_len;	//Length Low Byte
        txBufferA[3] = XB_API_ID;       //API Identifier - Currently only support the RX type
        txBufferA[4] = src_addr.byte.HB;    //Source Address High byte
        txBufferA[5] = src_addr.byte.LB;    //Source Address Low Byte
        txBufferA[6] = 0x00;	        //'RSSI' Not currently implemented
        txBufferA[7] = 0x00;            //'Options' Not currently implemented

        for(i = 0; i < pld_len; i++) {
            checksum += pld_str[i];
            txBufferA[RX_FRAME_OFFSET + i] = pld_str[i];
        }

        txBufferA[RX_FRAME_OFFSET + pld_len] = 0xFF - checksum;	//set Checksum byte
    CRITICAL_SECTION_END

    payDelete(packet->payload);
    macDeletePacket(packet);

    // pld_len + RX_FRAME_OFFSET + length(CHKSUM) - 1
    DMA6CNT = pld_len + RX_FRAME_OFFSET;
    DMA6CONbits.CHEN = 1;
    DMA6REQbits.FORCE = 1;

}


//Recieved UART Xbee packet, send packet out over the radio
void xbeeHandleTx(Payload uart_pld){

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

void xbeeHandleAt(Payload rx_pld)
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

//Handle a Rx packet and pass it to the sendUART function
/*void xbeeHandleRx()
{
    MacPacket rx_packet;
    unsigned char frame_header[5];

    rx_packet = radioDequeueRxPacket();

    frame_header[RX_API_POS] = RX_16BIT;

    frame_header[RX_SRC_ADR_HB_POS] = rx_packet->src_addr.byte.HB;
    frame_header[RX_SRC_ADR_LB_POS] = rx_packet->src_addr.byte.LB;

    frame_header[RX_RSSI_POS] = phyReadRSSI();
    frame_header[RX_OPTIONS_POS] = 0x00;

    CRITICAL_SECTION_START
        //Start Byte
        while(BusyUART1());
        WriteUART1(RX_START);

        //Length High Byte
        while(BusyUART1());
        WriteUART1(0x00);

        //Length Low Byte
        while(BusyUART1());
        WriteUART1(payGetPayloadLength(rx_packet->payload)+FRAME_HEADER_LEN);

        sendUART(frame_header, rx_packet->payload->pld_data,payGetPayloadLength(rx_packet->payload));

        payDelete(rx_packet->payload);
        macDeletePacket(rx_packet);

    CRITICAL_SECTION_END
}
*/

//Handle an AT response packet and pass it to the sendUART function
void xbeeHandleATR(unsigned char frame_id, WordVal command, unsigned char *data, unsigned char length)
{
    unsigned char frame_header[5];

    frame_header[RX_API_POS] = AT_RESPONSE;

    frame_header[ATR_FRAME_ID_POS] = frame_id;
    frame_header[ATR_COMMAND_HB_POS] = command.byte.HB;

    frame_header[ATR_COMMAND_LB_POS] = command.byte.LB;
    frame_header[ATR_STATUS_POS] = 0x00;

    CRITICAL_SECTION_START
        //Start Byte
        while(BusyUART1());
        WriteUART1(RX_START);

        //Length High Byte
        while(BusyUART1());
        WriteUART1(0x00);

        //Length Low Byte
        while(BusyUART1());
        WriteUART1(length+FRAME_HEADER_LEN);

        sendUART(frame_header, data, length);

    CRITICAL_SECTION_END
}

//General UART send function
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


