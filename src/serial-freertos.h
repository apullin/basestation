/* 
 * File:   serial-freertos.h
 * Author: pullin
 *
 * Created on April 23, 2014, 11:15 PM
 */

#ifndef SERIAL_FREERTOS_H
#define	SERIAL_FREERTOS_H

void SetupUART1(void);

void vSerialStartTasks( unsigned portBASE_TYPE uxPriority);

#endif	/* SERIAL_FREERTOS_H */

