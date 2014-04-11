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
#include <Generic.h>
#include "generic_typedefs.h"
#include "init_default.h"
#include "utils.h"
#include "init.h"
#include "uart.h"
#include "mac_packet.h"
#include "radio.h"
#include "payload.h"
#include <stdio.h>
#include "xbee_handler.h"
//#include "lcd.h"

#ifdef __BASESTATION
#define UART_RX_IF _U1RXIF
#define UART_TX_IF _U1TXIF
#define UART_RX_INTERRUPT _U1RXInterrupt
#define UART_TX_INTERRUPT _U1TXInterrupt
#define UARTSTAbits U1STAbits
#define BusyUART BusyUART1
#define WriteUART WriteUART1
#elif defined __BASESTATION2
#define UART_RX_IF _U2RXIF
#define UART_TX_IF _U2TXIF
#define UART_RX_INTERRUPT _U2RXInterrupt
#define UART_TX_INTERRUPT _U2TXInterrupt
#define UARTSTAbits U2STAbits
#define BusyUART BusyUART2
#define WriteUART WriteUART2
#else
#error "This code is only supported on the BML Basestation and Basestation2"
#endif


////////// Constants ///////////
#define RX_START            0x7E
#define UART_RX_WAITING     0x00
#define UART_RX_ON          0x01

//Offsets for packets received from computer
#define RX_FRAME_OFFSET     8
#define RX_DATA_OFFSET      4 //Offset for accounting for RX_START, LEN_HB/LB bytes, and API ID
#define FRAME_HEADER_LEN    5

//These are transmit modes for the xbee
#define AT_COMMAND_MODE     0x08
#define TX_16BIT            0x01

//Position of bytes in a TX_16BIT packet
#define RX_START_POS        0
#define LEN_HB_POS          1
#define LEN_LB_POS          2
#define API_ID_POS          3
#define FRAME_ID_POS        4
#define DEST_ADDR_HB_POS    5
#define DEST_ADDR_LB_POS    6
#define OPTION_POS          7

//These are various parameters of the xbee
#define AT_CHANNEL          0x4348  //CH
#define AT_PAN_ID           0x4944  //ID
#define AT_SRC_ADR          0x4D59  //MY
#define AT_ACK_LAST         0x4541  //EA
#define AT_SNIFFER          0x534E  //SN

//These are the response modes of the xbee
#define AT_RESPONSE         0x88
#define ATR_API_POS         0
#define ATR_FRAME_ID_POS    1
#define ATR_COMMAND_HB_POS  2
#define ATR_COMMAND_LB_POS  3
#define ATR_STATUS_POS      4

#define RX_16BIT            0x81
#define RX_API_POS          0
#define RX_SRC_ADR_HB_POS   1
#define RX_SRC_ADR_LB_POS   2
#define RX_RSSI_POS         3
#define RX_OPTIONS_POS      4

///// Public functions ////

void xbeeHandleTx(Payload uart_pld) {

    
    WordVal dst_addr;
    Payload tx_pld;
    char test;

    //Get destination address from uart_pld package
    //Frame ID and options packet are currently ignored for this type of packet...
    dst_addr.byte.HB = uart_pld->pld_data[DEST_ADDR_HB_POS - RX_DATA_OFFSET];
    dst_addr.byte.LB = uart_pld->pld_data[DEST_ADDR_LB_POS - RX_DATA_OFFSET];

    //test = dst_addr.byte.LB;

    unsigned int txlen = payGetPayloadLength(uart_pld)-
        (RX_FRAME_OFFSET - RX_DATA_OFFSET) - PAYLOAD_HEADER_LENGTH - 1;
    
    //Create new packet with just the data that needs to be sent by the radio
    tx_packet = radioRequestPacket(txlen);
    if(tx_packet == NULL) { return;}
    tx_pld = macGetPayload(tx_packet);
    
    payAppendData(rx_pld, -PAYLOAD_HEADER_LENGTH,
            payGetPayloadLength(rx_pld),
            &(uart_pld->pld_data[RX_DATA_OFFSET]));

    //Place packet in radio queue for sending
    MacPacket tx_packet;
    
    //tx_packet->payload = rx_pld;
    tx_packet->payload_length = payGetPayloadLength(rx_pld); //rx_pld_len.byte.LB - (RX_FRAME_OFFSET - RX_DATA_OFFSET);
    //tx_packet->dest_pan_id = src_pan_id; //Already set when macCreatePacket is called.
    macSetDestAddr(tx_packet, dst_addr);
    
    //Place packet in radio queue for sending
    if(!radioEnqueueTxPacket(packet)) {
            radioReturnPacket(packet);	// Delete packet if append fails
    }

    //radioReturnPacket(pkt);

}

void xbeeHandleAt(Payload rx_pld) {
    unsigned char frame;
    unsigned char length;
    //unsigned char test;

    //This should really be some sort of dynamic queue to be generalized
    unsigned char bytes[10];


    //unsigned short int command;
    WordVal command;
    WordVal data;

    length = payGetPayloadLength(rx_pld);
    frame = rx_pld->pld_data[0];

    RadioConfiguration radioconf;
    radioGetConfiguration(&radioconf);
    unsigned int current_radio_chan = radioconf.address.channel;
    unsigned int current_radio_pan  = radioconf.address.pan_id;
    unsigned int current_src_addr = radioconf.address.address;

    //test = rx_pld->pld_data[1];
    //test = rx_pld->pld_data[2];

    /*
        command = rx_pld->pld_data[1];
        command = command << 8;

        command = command + rx_pld->pld_data[2];
     */
    command.byte.HB = rx_pld->pld_data[1];
    command.byte.LB = rx_pld->pld_data[2];

    switch (command.val) {
        case AT_CHANNEL:
            if (length != 3) {
                radioSetChannel(rx_pld->pld_data[3]);
            }
            if (frame != 0) {
                bytes[0] = current_radio_chan;
                xbeeHandleATR(frame, command, bytes, 1);
            }
            break;
        case AT_PAN_ID:
            if (length != 3) {
                data.byte.HB = rx_pld->pld_data[3];
                data.byte.LB = rx_pld->pld_data[4];
                radioSetSrcPanID(data);
            }
            if (frame != 0) {
                data = current_radio_pan;
                bytes[0] = data.byte.HB;
                bytes[1] = data.byte.LB;

                xbeeHandleATR(frame, command, bytes, 2);
            }
            break;
        case AT_SRC_ADR:
            if (length != 3) {
                data.byte.HB = rx_pld->pld_data[3];
                data.byte.LB = rx_pld->pld_data[4];
                radioSetSrcAddr(data);
            }
            if (frame != 0) {
                data = current_src_addr;
                bytes[0] = data.byte.HB;
                bytes[1] = data.byte.LB;

                xbeeHandleATR(frame, command, bytes, 2);
            }
            break;
        case AT_ACK_LAST:
            if (frame != 0) {
                bytes[0] = 0;
                bytes[1] = phyGetLastAckd();
                ;

                xbeeHandleATR(frame, command, bytes, 2);
            }
            break;
        case AT_SNIFFER:
            if (length != 3)
                atSetPromMode(rx_pld->pld_data[3]); //Put radio in sniffer mode
            break;
    }
}

//Handle a Rx packet and pass it to the sendUART function

void xbeeHandleRx() {
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
    while (BusyUART());
    WriteUART(RX_START);

    //Length High Byte
    while (BusyUART());
    WriteUART(0x00);

    //Length Low Byte
    while (BusyUART());
    WriteUART(payGetPayloadLength(rx_packet->payload) + FRAME_HEADER_LEN);

    sendUART(frame_header, rx_packet->payload->pld_data, payGetPayloadLength(rx_packet->payload));

    payDelete(rx_packet->payload);
    macDeletePacket(rx_packet);

    CRITICAL_SECTION_END
}

//Handle an AT response packet and pass it to the sendUART function

void xbeeHandleATR(unsigned char frame_id, WordVal command, unsigned char *data, unsigned char length) {
    unsigned char frame_header[5];

    frame_header[RX_API_POS] = AT_RESPONSE;

    frame_header[ATR_FRAME_ID_POS] = frame_id;
    frame_header[ATR_COMMAND_HB_POS] = command.byte.HB;

    frame_header[ATR_COMMAND_LB_POS] = command.byte.LB;
    frame_header[ATR_STATUS_POS] = 0x00;

    CRITICAL_SECTION_START
            //Start Byte
    while (BusyUART());
    WriteUART(RX_START);

    //Length High Byte
    while (BusyUART());
    WriteUART(0x00);

    //Length Low Byte
    while (BusyUART());
    WriteUART(length + FRAME_HEADER_LEN);

    sendUART(frame_header, data, length);

    CRITICAL_SECTION_END
}

//General UART send function

void sendUART(unsigned char *frame_header, unsigned char *data, unsigned char length) {
    int i;
    unsigned char checksum = 0;

    //send frame header
    for (i = 0; i < FRAME_HEADER_LEN; i++) {
        checksum += frame_header[i];
        while (BusyUART());
        WriteUART(frame_header[i]);
    }

    //send payload data
    for (i = 0; i < length; i++) {
        checksum += data[i];
        while (BusyUART());
        WriteUART(data[i]);

    }

    //Send Checksum Data
    while (BusyUART());
    WriteUART(0xFF - checksum);
}


/////////// Interrupt Handlers /////////////


//read data from the UART, and call the proper function based on the Xbee code
void __attribute__((__interrupt__, no_auto_psv)) UART_RX_INTERRUPT(void) {
    static unsigned char uart_rx_state = UART_RX_WAITING;
    static unsigned char uart_rx_cnt = 0;
    static Payload uart_pld;
    static WordVal uart_pld_len;
    static byte uart_checksum;
    static unsigned char packet_type = 0;
    static unsigned char test;

    unsigned char c;

    if (UARTSTAbits.OERR) {
        UARTSTAbits.OERR = 0;
    }

    c = ReadUART1();
    if (uart_rx_state == UART_RX_WAITING && c == RX_START) {
        uart_rx_state = UART_RX_ON;
        packet_type = 0;
        uart_rx_cnt = 1;
        uart_checksum = 0x00;
    } else if (uart_rx_state == UART_RX_ON) {
        switch (uart_rx_cnt) {
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
                uart_pld = payCreateEmpty(uart_pld_len.byte.LB - PAYLOAD_HEADER_LENGTH);
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
                if (uart_rx_cnt == (uart_pld_len.byte.LB + RX_DATA_OFFSET - 1)) {
                    if (uart_checksum + c == 0xFF) //We have a legit packet
                    {
                        //Check for type of packet and call relevant function
                        switch (packet_type) {
                            case AT_COMMAND_MODE:
                                xbeeHandleAt(uart_pld);
                                break;

                            case TX_16BIT:
                                xbeeHandleTx(uart_pld);
                                break;

                            default:
                                //do nothing, but probably should send an error
                                break;
                        }
                        payDelete(uart_pld);

                    } else //Start over
                    {
                        payDelete(uart_pld);
                    }

                    uart_rx_state = UART_RX_WAITING;
                } else {
                    uart_pld->pld_data[uart_rx_cnt - RX_DATA_OFFSET] = c;
                    uart_checksum += c;
                    uart_rx_cnt++;
                }
                break;
        }

    }
    UART_RX_IF = 0;
}

void __attribute__((interrupt, no_auto_psv)) UART_TX_INTERRUPT(void) {
    UART_TX_IF = 0;
}