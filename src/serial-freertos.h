/* 
 * File:   serial-freertos.h
 * Author: pullin
 *
 * Created on April 23, 2014, 11:15 PM
 */

#ifndef SERIAL_FREERTOS_H
#define	SERIAL_FREERTOS_H

//Startup function for FreeRTOS serial task
void vSerialStartTask( unsigned portBASE_TYPE uxPriority);

//Getter functions
QueueHandle_t serialGetTXQueueHandle();
QueueHandle_t serialGetRXQueueHandle();

#endif	/* SERIAL_FREERTOS_H */

