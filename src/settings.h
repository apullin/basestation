// Contents of this file are copyright Andrew Pullin, 2013

/******************************************************************************
* Name: settings.h
* Desc: Constants used by Andrew P. are included here.
* Author: pullin
******************************************************************************/
#ifndef __SETTINGS_H
#define __SETTINGS_H


//#warning "REQUIRED: Review and set radio channel & network parameters in firmware/source/settings.h  , then comment out this line."
/////// Radio settings ///////
#define RADIO_CHANNEL		0x19
//#warning "You have changed the radio channel from 0x0E to something else"
#define RADIO_SRC_ADDR 		0x2051
#define RADIO_PAN_ID  	0x2050

#define RADIO_RXPQ_MAX_SIZE 	16
#define RADIO_TXPQ_MAX_SIZE	16

#define ANTENNA_DIVERSITY 0

/////// Configuration options ///////

//#define TELEM_TYPE orTelemStruct_t
//#define TELEM_INCLUDE "or_telem.h"
//#define TELEMPACKFUNC(x) orTelemGetData(x)

//Motor controller output routing
// The leg controllers can be directed to different motor outputs from here

#endif //__SETTINGS_H
