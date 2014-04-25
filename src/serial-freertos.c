/*
 * File:   serial-freertos.c
 * Author: Andrew Pullin
 *
 * Created on April 23, 2014, 10:51 PM
 */

#include<xc.h>
//FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
//Library includes
#include <string.h> //for memcpy
#include "serial-freertos.h"
#include "utils.h"
#include "uart.h"
#include "blob.h"

#define UART_TX_QUEUE_SIZE 8
#define UART_RX_QUEUE_SIZE 32

#define UART_TX_DMA_BUFFER_SIZE 128

#define serialSTACK_SIZE				configMINIMAL_STACK_SIZE

//////////////////////////////////////////////////
//////////      Private variables      ///////////
//////////////////////////////////////////////////
/* The queues used to communicate between tasks and ISR's. */
static QueueHandle_t serialRXCharQueue;
static QueueHandle_t serialTXBlobQueue;
//Semaphore on UART DMA TX
static SemaphoreHandle_t xUART_DMA_TX_Semaphore;

//TODO: Semaphores on UART DMA TX buffers
//TODO: Are two DMA TX buffers needed? Now that we have a FreeRTOS
//      queue AND a TX buffer, maybe not
unsigned char uart1DMABufferA[UART_TX_DMA_BUFFER_SIZE] __attribute__((space(dma)));
//unsigned char txBufferB[UART_TX_DMA_BUFFER_SIZE] __attribute__((space(dma)));


//////////////////////////////////////////////////
////////// Private function prototypes ///////////
//////////////////////////////////////////////////
void SetupUART1();
void SetupDMAChannel();
static portTASK_FUNCTION_PROTO(vSerialTask, pvParameters); //FreeRTOS task


//////////////////////////////////////////////////
////////// Public function definitions ///////////
//////////////////////////////////////////////////

void vSerialStartTask(unsigned portBASE_TYPE uxPriority) {
    //Peripheral setup, including DMA
    SetupUART1();

    /* Create the queues used by the serial task. */
    //serialTXQueue = xQueueCreate(UART_TX_QUEUE_SIZE, (unsigned portBASE_TYPE) sizeof ( signed char));
    //The TX queue is going to be explicitely built for MacPackets
    serialTXBlobQueue = xQueueCreate(UART_TX_QUEUE_SIZE, (unsigned portBASE_TYPE) sizeof ( Blob_t));
    serialRXCharQueue = xQueueCreate(UART_RX_QUEUE_SIZE, (unsigned portBASE_TYPE) sizeof ( signed char));

    xUART_DMA_TX_Semaphore = xSemaphoreCreateBinary();

    //Create task
    //TODO: Should we two separate tasks, one for RX, one for TX?
    xTaskCreate(vSerialTask, (const char *) "SerialTask", serialSTACK_SIZE, NULL, uxPriority, (xTaskHandle *) NULL);
}

QueueHandle_t serialGetTXQueueHandle() {
    return serialTXBlobQueue;
}

QueueHandle_t serialGetRXQueueHandle() {
    return serialRXCharQueue;
}

//////////////////////////////////////////////////
////////// Private function definitions //////////
//////////////////////////////////////////////////

void SetupUART1() {
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

    /* It is assumed that this function is called prior to the scheduler being
    started.  Therefore interrupts must not be allowed to occur yet as they
    may attempt to perform a context switch. */
    portDISABLE_INTERRUPTS();

    /* Clear the Rx buffer. */
    char cChar;
    while (U2STAbits.URXDA == 1) {
        cChar = U2RXREG;
    }

    SetupDMAChannel();

}

void SetupDMAChannel(void) {

    DMA6CON = 0x6001; // byte, one-shot, post-increment, RAM 2 Peripheral
    DMA6CNT = 0;
#if defined(__IMAGEPROC1) || defined(__BASESTATION)
    DMA6REQ = 0x000C; //  select UART1 transmitter
    DMA6PAD = (volatile unsigned int) &U1TXREG;
#elif defined(__EXP16DEV) || defined(__BASESTATION2)
    DMA6REQ = 0x001F; //  select UART2 transmitter
    DMA6PAD = (volatile unsigned int) &U2TXREG;
#else
#error "UART/DMA is not defined on this project"
#endif

    DMA6STA = __builtin_dmaoffset(uart1DMABufferA);

    IPC17bits.DMA6IP = 4;
    IFS4bits.DMA6IF = 0; // clear DMA interrupt flag
    IEC4bits.DMA6IE = 1; // enable DMA interrupt
}

//////////////////////////////////////////////////
//////////          Interrupts          //////////
//////////////////////////////////////////////////

//UART recieve interrupt; contains entire state machine for xbee parsing
//read data from the UART peripheral, and enqueue it
//TODO: How to have the interrupt determined by bsp header? Compiler complains if U1RXInterrupt replcaed by a macro
//void __attribute__((__interrupt__, auto_psv)) UART_RX_INTERRUPT(void) {

void __attribute__((__interrupt__, auto_psv)) _U1RXInterrupt(void) {

    char cChar;
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    /* Get the character and post it on the queue of Rxed characters.
    If the post causes a task to wake force a context switch as the woken task
    may have a higher priority than the task we have interrupted. */
    _U2RXIF = 0;
    while (U2STAbits.URXDA) {
        cChar = U2RXREG;
        xQueueSendFromISR(serialRXCharQueue, &cChar, &xHigherPriorityTaskWoken);
    }

    if (xHigherPriorityTaskWoken != pdFALSE) {
        taskYIELD();
    }
}

void __attribute__((__interrupt__, no_auto_psv)) _DMA6Interrupt(void) {
    static BaseType_t xHigherPriorityTaskWoken;

    IFS4bits.DMA6IF = 0; //clear the DMA6 interrupt flag

    xSemaphoreGiveFromISR(xUART_DMA_TX_Semaphore, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken != pdFALSE) {
        // We can force a context switch here.  Context switching from an
        // ISR uses port specific syntax.
        taskYIELD();
    }

}


//////////////////////////////////////////////////
//////////        FreeRTOS Task         //////////
//////////////////////////////////////////////////

static portTASK_FUNCTION(vSerialTask, pvParameters) {

    Blob_t xbeeFmt;
    portBASE_TYPE xStatus;

    for (;;) {
        //Get blob from serialTXQueue, blocking read
        xStatus = xQueueReceive(serialTXBlobQueue, &xbeeFmt, portMAX_DELAY);
        //Take UART DMA semaphore?
        xSemaphoreTake(xUART_DMA_TX_Semaphore, portMAX_DELAY);
        //Write blob to DMA
        memcpy(uart1DMABufferA, xbeeFmt.data, xbeeFmt.length);
        //Start DMA transfer
        DMA6CNT = xbeeFmt.length;
        DMA6CONbits.CHEN = 1;
        DMA6REQbits.FORCE = 1;
        //Take UART semaphore: blocks until end of DMA (DMA interrupt gives it back)
        //xSemaphoreTake(xUART_DMA_TX_Semaphore, portMAX_DELAY);

    }
}
