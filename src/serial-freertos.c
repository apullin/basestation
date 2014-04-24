/*
 * File:   serial-freertos.c
 * Author: Andrew Pullin
 *
 * Created on April 23, 2014, 10:51 PM
 */

#include<xc.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "serial-freertos.h"

#include "utils.h"
#include "uart.h"

#define UART_TX_QUEUE_SIZE 128
#define UART_RX_QUEUE_SIZE 128

#define UART_TX_DMA_BUFFER_SIZE 128

#define serialSTACK_SIZE				configMINIMAL_STACK_SIZE

static portTASK_FUNCTION_PROTO(vSerialTask, pvParameters);

/* The queues used to communicate between tasks and ISR's. */
static QueueHandle_t serialRXQueue;
static QueueHandle_t serialTXQueue;

//TODO: Semaphores on UART DMA TX buffers
//TODO: Are two DMA TX buffers needed? Now that we have a FreeRTOS
//      queue AND a TX buffer, maybe not
unsigned char txBufferA[UART_TX_DMA_BUFFER_SIZE] __attribute__((space(dma)));
unsigned char txBufferB[UART_TX_DMA_BUFFER_SIZE] __attribute__((space(dma)));

void SetupUART1(void) {
    /// UART1 for RS-232 w/PC @ 230400, 8bit, No parity, 1 stop bit
    unsigned int U1MODEvalue, U1STAvalue, U1BRGvalue;
    U1MODEvalue = UART_EN & UART_IDLE_CON & UART_IrDA_DISABLE &
            UART_MODE_FLOW & UART_UEN_10 & UART_DIS_WAKE &
            UART_DIS_LOOPBACK & UART_DIS_ABAUD & UART_UXRX_IDLE_ONE &
            UART_BRGH_FOUR & UART_NO_PAR_8BIT & UART_1STOPBIT;
    U1STAvalue = UART_INT_TX & UART_INT_RX_CHAR & UART_SYNC_BREAK_DISABLED &
            UART_TX_ENABLE & UART_ADR_DETECT_DIS &
            UART_IrDA_POL_INV_ZERO; // If not, whole output inverted.
    U1BRGvalue = 43; // (Fcy / ({16|4} * baudrate)) - 1
    OpenUART1(U1MODEvalue, U1STAvalue, U1BRGvalue);

    ConfigIntUART1(UART_TX_INT_EN & UART_TX_INT_PR4 & UART_RX_INT_EN &
            UART_RX_INT_PR4);

    /* Create the queues used by the com test task. */
    serialTXQueue = xQueueCreate(UART_TX_QUEUE_SIZE, (unsigned portBASE_TYPE) sizeof ( signed char));
    serialRXQueue = xQueueCreate(UART_RX_QUEUE_SIZE, (unsigned portBASE_TYPE) sizeof ( signed char));

    /* It is assumed that this function is called prior to the scheduler being
    started.  Therefore interrupts must not be allowed to occur yet as they
    may attempt to perform a context switch. */
    portDISABLE_INTERRUPTS();

    /* Clear the Rx buffer. */
    char cChar;
    while (U2STAbits.URXDA == 1) {
        cChar = U2RXREG;
    }

}


//UART recieve interrupt; contains entire state machine for xbee parsing
//read data from the UART peripheral, and enqueue it

void __attribute__((__interrupt__, auto_psv)) _UART_RX_INTERRUPT(void) {
    char cChar;
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    /* Get the character and post it on the queue of Rxed characters.
    If the post causes a task to wake force a context switch as the woken task
    may have a higher priority than the task we have interrupted. */
    _U2RXIF = 0;
    while (U2STAbits.URXDA) {
        cChar = U2RXREG;
        xQueueSendFromISR(serialRXQueue, &cChar, &xHigherPriorityTaskWoken);
    }

    if (xHigherPriorityTaskWoken != pdFALSE) {
        taskYIELD();
    }
}

void vAltStartComTestTasks(unsigned portBASE_TYPE uxPriority) {
    //Peripheral setup
    SetupUART1();

    //Create task
    //TODO: Should we two separate tasks, one for RX, one for TX?
    xTaskCreate(vSerialTask, (const char *) "SerialTask", serialSTACK_SIZE, NULL, uxPriority, (xTaskHandle *) NULL);
}

static portTASK_FUNCTION(vSerialTask, pvParameters) {
    //signed char cExpectedByte, cByteRxed;
    //portBASE_TYPE xResyncRequired = pdFALSE, xErrorOccurred = pdFALSE;

    //Pseudocode:

    for (;;) {

    }
}