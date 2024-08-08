/*
*********************************************************************************************************
*								   IMARINE NMAE MESSAGE PARSER MODULE
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
*									  NMAE MESSAGE PARSER MODULE
*
*                                      	 	   Generic
*
*
* Filename      : nmea.cpp
* Version       : V1.00
* Programmer(s) : tmk
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#include <string.h>
#include <nmea/nmea.h>


/*
*********************************************************************************************************
*                                            LOCAL DEFINITIONS
*********************************************************************************************************
*/

// NMEA protocol states
enum
{
	WAIT_STX,
	WAIT_STAR,
	WAIT_CHKS1,
	WAIT_CHKS2,
	WAIT_ETX1,
	WAIT_ETX2,
};


/*
*********************************************************************************************************
*											GLOBAL VARIABLES
*********************************************************************************************************
*/

/*******************************************************************************
* Function Name  : char_to_hex
* Description    : Function to convert unsigned char to string of length 2
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void char_to_hex(uint8_t ch, char *szHex)
{
	uint8_t byte[2];
	int i;

	byte[0] = ch/16;
	byte[1] = ch%16;
	for(i=0; i<2; i++)
	{
		if (byte[i] <= 9)
			szHex[i] = '0' + byte[i];
		else
			szHex[i] = 'A' + byte[i] - 10;
	}
	szHex[2] = 0;
}//char_to_hex


/*******************************************************************************
* Function Name  : hex_to_char
* Description    : Function to convert string of length 2 to unsigned char
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static uint8_t hex_to_char(char *szHex)
{
	uint8_t i, rch=0;

	for (i=0; i<2; i++)
	{
		if (szHex[i] >='0' && szHex[i] <= '9')
			rch = (rch << 4) + (szHex[i] - '0');
		else if (szHex[i] >='A' && szHex[i] <= 'F')
			rch = (rch << 4) + (szHex[i] - 'A' + 10);
		else if (szHex[i] >='a' && szHex[i] <= 'f')
			rch = (rch << 4) + (szHex[i] - 'a' + 10);
		else
			break;
	}//for

	return rch;
}//hex_to_char


/*******************************************************************************
* Function Name  : NMEA_Framer
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int NMEA_Framer(uint8_t inData, nmea_data_t *frmData, void *inArg)
{
	(void)inArg;

	if (!frmData || !frmData->buf)
		return 0;

	switch (frmData->sta)
	{
		case WAIT_STX:
		{
			if ((inData == NMEA_STX1) || (inData == NMEA_STX2))
			{
				frmData->buf[frmData->cnt++] = inData;
				frmData->sta = WAIT_STAR;
			}//if
			break;
    }//WAIT_STX

		case WAIT_STAR:
		{
			frmData->buf[frmData->cnt++] = inData;

			if ((frmData->cnt >= MAX_STR_CONTENT) || (inData == NMEA_STX1) || (inData == NMEA_STX2))
			{
				frmData->cnt  = 0;
				frmData->sta = WAIT_STX;
			}//if
			else if (inData == '*')
			{
				frmData->sta = WAIT_CHKS1;
			}//else
			break;
    }//WAIT_STAR

		case WAIT_CHKS1:
		{
			frmData->buf[frmData->cnt++] = inData;
			frmData->sta = WAIT_CHKS2;
			break;
	  }//WAIT_CHKS1

		case WAIT_CHKS2:
		{
			frmData->buf[frmData->cnt++] = inData;
			frmData->sta = WAIT_ETX1;
			break;
    }//WAIT_CHKS2

		case WAIT_ETX1:
		{
			if (inData == NMEA_ETX1)
			{
				frmData->buf[frmData->cnt++] = inData;
				frmData->sta = WAIT_ETX2;
			}//if
			else
			{
				frmData->cnt  = 0;
				frmData->sta = WAIT_STX;
			}//else
			break;
		}//WAIT_ETX1

		case WAIT_ETX2:
		{
			if (inData == NMEA_ETX2)
			{
			  int len;

				frmData->buf[frmData->cnt++] = inData;
        frmData->buf[frmData->cnt] =  0;
				len = frmData->cnt;
        frmData->sta = WAIT_STX;
        frmData->cnt = 0;
				if (NMEA_Checksum(frmData->buf, len))
				{
				    return len;
				}//if
			}//if
			break;
	  }//WAIT_ETX2
	}//switch

	return 0;
}//NMEA_Framer


/*******************************************************************************
* Function Name  : NMEA_Checksum
* Description    : Calculate NMEA sentence chechsum
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
bool NMEA_Checksum(uint8_t *Buffer, int BufLen)
{
	char szChk[3];
	uint8_t Chk=0;
	uint16_t i;

	if (BufLen < 4)
	    return false;

	for (i=1; (Buffer[i]!='*') && (i<BufLen); i++)
	{
		Chk ^= Buffer[i];
	}//for

	szChk[0] = Buffer[i+1];
	szChk[1] = Buffer[i+2];
	szChk[2] = 0;

	if (Chk != hex_to_char(szChk))
		return false;

	return true;
}//NMEA_Checksum
