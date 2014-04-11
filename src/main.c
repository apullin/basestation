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
 * by Aaron M. Hoover & Kevin Peterson
 *
 * v2.00
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

#include <xc.h>                 //Microchip chip specific
#include <Generic.h>            //Microchip generic typedefs
#include "generic_typedefs.h"   //iplib generic tyepdefs
#include "init_default.h"
#include "utils.h"
#include "init.h"
#include "uart.h"               //Microchip
#include "mac_packet.h"
#include "radio.h"
#include "payload.h"
#include <stdio.h>
#include "xbee_constants.h"
#include "xbee_handler.h"
#include "led.h"

void init(void);

// Default source address and pan id, eventually will get them from program memory
// For now, you need to replace these values with the appropriate ones for your
// project.  These can be changed from python now.
// RSF working address 9/2011
/////// Radio settings ///////
#define RADIO_CHANNEL		0x19
#define RADIO_SRC_ADDR 		0x2052
#define RADIO_PAN_ID            0x2050

//Motile Release
//#define SRC_ADDR	    0x3001
//#define SRC_PAN_ID	    0x3000
//#define MY_CHAN             0x0e

#define RXPQ_MAX_SIZE 	32
#define TXPQ_MAX_SIZE	32

#define RADIO_TXPQ_MAX_SIZE 40
#define RADIO_RXPQ_MAX_SIZE 40

#define ANTENNA_DIVERSITY 0

#define LED_RED             LED_0
#define LED_YLW1            LED_1
#define LED_YLW2            LED_2
#define LED_BLU             LED_3

int main(void) {
    init();

    MacPacket packet;
    Payload pld;
    unsigned char command, status;

    while (1) {

        //Packet into from radio, send to UART
        if (!radioRxQueueEmpty()) {
            xbeeHandleRx();
            LED_BLU ^= 1;
        }
        //Packet from UART, to be sent over raido
        if (!radioTxQueueEmpty()) {
            xbeeHandleTx();
            LED_RED ^= 1;
        }

        radioProcess();
    }
}

void init(void) {
    int i;

    SetupClock();
    SetupPorts();

    int old_ipl;
    mSET_AND_SAVE_CPU_IP(old_ipl, 1);

    SwitchClocks();
    sclockSetup();

    ppoolInit();
    radioInit(RADIO_TXPQ_MAX_SIZE, RADIO_RXPQ_MAX_SIZE);
    radioSetChannel(RADIO_CHANNEL);
    radioSetSrcPanID(RADIO_PAN_ID);
    radioSetSrcAddr(RADIO_SRC_ADDR);

    ppoolInit();
    radioInit(RADIO_TXPQ_MAX_SIZE, RADIO_RXPQ_MAX_SIZE);
    radioSetChannel(RADIO_CHANNEL);
    radioSetSrcPanID(RADIO_PAN_ID);
    radioSetSrcAddr(RADIO_SRC_ADDR);


    for (i = 0; i < 6; i++) {
        LED_RED = ~LED_RED;
        delay_ms(50);
        LED_YLW1 = ~LED_YLW1;
        delay_ms(50);
        LED_YLW2 = ~LED_YLW2;
        delay_ms(50);
        LED_BLU = ~LED_BLU;
        delay_ms(50);
    }

    SetupUART1();
	xbSetupDma();
    SetupInterrupts();
    EnableIntU1TX;
    EnableIntU1RX;

    //Set this if the electronics for Ant diversity are installed
    atSetAntDiversity(ANTENNA_DIVERSITY);
}

