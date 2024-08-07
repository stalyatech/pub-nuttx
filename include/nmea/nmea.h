/*
*********************************************************************************************************
*								                    IMARINE NMAE MESSAGE PARSER MODULE
*
*                           (c) Copyright 2015; i-marine, Inc.; Istanbul, TR
*
*                   All rights reserved.  Protected by international copyright laws.
*                   Knowledge of the source code may not be used to write a similar
*                   product.  This file may only be used in accordance with a license
*                   and should not be redistributed in any way.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*									                  NMAE MESSAGE PARSER MODULE
*
*                                      	 	   GENERIC
*
*
* Filename      : nmea.h
* Version       : V1.00
* Programmer(s) : tmk
*********************************************************************************************************
*/

#ifndef __NMEA_H__
#define __NMEA_H__

#ifdef __cplusplus
extern "C" {
#endif


/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#include <stdint.h>
#include <stdbool.h>


/*
*********************************************************************************************************
*											  GLOBAL TYPES
*********************************************************************************************************
*/

// NMEA framer data structure
typedef struct
{
    uint16_t cnt;
    uint8_t  sta;
    uint8_t *buf;
} nmea_data_t;


/*
*********************************************************************************************************
*											                  GLOBAL DEFINITIONS
*********************************************************************************************************
*/

/* NMEA frame characters */

#define  NMEA_STX1			('$')
#define  NMEA_STX2			('!')
#define  NMEA_ETX1			(0x0D)
#define  NMEA_ETX2			(0x0A)

/* NMEA length definitions */

#define  MAX_MSG_CONTENT	(116)
#define  MAX_STR_CONTENT	(MAX_MSG_CONTENT)

/*
*********************************************************************************************************
*										                  VARIABLE DECLERATIONS
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*										                  FUNCTION DECLERATIONS
*********************************************************************************************************
*/

bool NMEA_Checksum  (uint8_t *Buffer, int BufLen);
int  NMEA_Framer    (uint8_t inData, nmea_data_t *frmData, void *inArg);

#ifdef __cplusplus
}
#endif

#endif //__NMEA_H__
