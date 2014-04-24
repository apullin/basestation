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
 * FreeRTOS based firmware for the Biomimetic Millisystems Lab 802.15.4 USB basestation. 
 *
 * by Andrew Pullin, based on work by Aaron M. Hoover & Kevin Peterson
 *
 *
 * Revisions:
 *  A. Pullin       4-23-2014       Initial verison, porting main.c to FreeRTOS primitives
 *
 * Notes:
 * 
 * At the moment the basestation uses a very stripped down MAC layer
 * implementation Inter-PAN communication is not possible and all 
 * addressing is 16-bit only. 
 *
 */

#include <xc.h>                 //Microchip chip specific
#include <Generic.h>            //Microchip generic typedefs
#include "generic_typedefs.h"   //iplib generic tyepdefs

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

//Library includes
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
#include "settings.h"
#include "sclock.h"

/* Standard includes. */
#include <stdio.h>

void init(void);

int main(void) {
    init();

    MacPacket packet;
    Payload pld;
    unsigned char command, status;

    while (1) {

        //Packet into from radio, send to UART
        if (!radioRxQueueEmpty()) {
            if ((packet = radioDequeueRxPacket()) != NULL) {
                pld = macGetPayload(packet);
                status = payGetStatus(pld);
                command = payGetType(pld);  // For debugging only
                xbeeHandleRx(packet);
                LED_BLU ^= 1;
                radioReturnPacket(packet);
            }    
        }

        //Packet from UART, to be sent over raido
        if (!radioTxQueueEmpty()) {
            radioProcess();
            //xbeeHandleTx(uart_pld);
            //xbeeHandleTx is called when we hit the end of a packet over UART, in the UART1 interrupt
            LED_RED ^= 1;
        }

        //radioProcess();
        
    }
}

void prvStartupLights(void) {
    int i;

    for (i = 0; i < 4; i++) {
        LED_RED = ~LED_RED;
        delay_ms(30);
        LED_YLW1 = ~LED_YLW1;
        delay_ms(30);
        LED_YLW2 = ~LED_YLW2;
        delay_ms(30);
        LED_BLU = ~LED_BLU;
        delay_ms(30);
    }

    SetupUART1();
    xbSetupDma();
    SetupInterrupts();
    //EnableIntU1TX;  //Now done by DMA
    EnableIntU1RX;

    //Set this if the electronics for Ant diversity are installed
    //atSetAntDiversity(ANTENNA_DIVERSITY);
}


//It is unclear if this interrupt is needed;
//The CPU seemed to get stuck without it declared.
void __attribute__((__interrupt__, no_auto_psv)) _U1TXInterrupt(void)
{
    _U1TXIF = 0;
}




// Task prototypes
//static void vToggleLED1(void *pvParameters);

//Private function prototypes
static void prvSetupHardware(void);

unsigned long ulIdleCycleCount = 0UL;

int main(void) {
    /* Perform any hardware setup necessary. */
    prvSetupHardware();

    // Create tasks
    xTaskCreate(vXBeeHandlerTask, /* Pointer to the function that implements the task. */
            "Task 1", /* Text name for the task. This is to facilitate debugging. */
            240, /* Stack depth in words. */
            NULL, /* We are not using the task parameter. */
            1, /* This task will run at priority 1. */
            NULL); /* We are not going to use the task handle. */

    /* Start the created tasks running. */
    vTaskStartScheduler();
    /* Execution will only reach here if there was insufficient heap to
    start the scheduler. */
    for (;;);
    return 0;
}




void prvSetupHardware(void){
    SetupClock();   //from imageproc-lib , config PLL
    SetupPorts();   //from imageproc-lib, set up LATn, TRISn, etc , requires __IMAGEPROC2
    SwitchClocks(); //from imageproc-lib , switch to PLL clock
    sclockSetup();

    radioInit(RADIO_TXPQ_MAX_SIZE, RADIO_RXPQ_MAX_SIZE);
    radioSetChannel(RADIO_CHANNEL);
    radioSetSrcPanID(RADIO_PAN_ID);
    radioSetSrcAddr(RADIO_SRC_ADDR);
    
}

//This task code is retained here as a template for other tasks
/*
vToggleLED1(void *pvParameters) {
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    
    for (;;) {
        LED_1 = ~LED_1; //Toggle LED #1 (color?)
        vTaskDelayUntil(&xLastWakeTime, (1000 / portTICK_RATE_MS));
    }
}
*/

/* Idle hook functions MUST be called vApplicationIdleHook(), take no parameters,
and return void. */
void vApplicationIdleHook(void) {
    /* This hook function does nothing but increment a counter. */
    ulIdleCycleCount++;
    Idle();  //dsPIC idle function; CPU core off, wakes on any interrupt
    portSWITCH_CONTEXT();
}