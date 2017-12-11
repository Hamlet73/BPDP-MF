#include "Modbus.h"
#include "main.h"
#include <stdio.h>

USHORT   usRegInputBuf[REG_INPUT_NREGS];
USHORT   usRegInputStart = REG_INPUT_START;

USHORT   usRegDiscreteStart = REG_DISCRETE_START;
UCHAR    usRegDiscreteInput=0;

USHORT   usRegHoldingBuf[REG_HOLDING_NREGS];
USHORT   usRegHoldingStart = REG_HOLDING_START;


extern void FLASHHOLDWrite(uint16_t* data);

extern uint16_t Counts[NumCount];
extern uint8_t  flag_ovf;
extern uint8_t SysReset;
extern uint32_t MotoTime1,MotoTime2;

eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    if( ( usAddress >= REG_INPUT_START )
        && ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) )
    {
        set_time(TIME_LED,3);LedOFF();// LED show RS-485 LINK 
        iRegIndex = ( int )( usAddress - usRegInputStart );
        while( usNRegs > 0 )
        {
            *pucRegBuffer++ =
                ( unsigned char )( usRegInputBuf[iRegIndex] >> 8 );
            *pucRegBuffer++ =
                ( unsigned char )( usRegInputBuf[iRegIndex] & 0xFF );
            iRegIndex++;
            usNRegs--;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}

eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    if( ( usAddress >= REG_HOLDING_START ) &&
        ( usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS ) )
    {
        set_time(TIME_LED,3);LedOFF();// LED show RS-485 LINK 
        iRegIndex = ( int )( usAddress - usRegHoldingStart );
        switch ( eMode )
        {
        case MB_REG_READ:
            while( usNRegs > 0 )
            {
                *pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] >> 8 );
                *pucRegBuffer++ = ( unsigned char )( usRegHoldingBuf[iRegIndex] & 0xFF );
                iRegIndex++;
                usNRegs--;
            }
            break;

        case MB_REG_WRITE:
            while( usNRegs > 0 )
            {
                uint16_t	NewData	=	*pucRegBuffer++ << 8;
                                NewData |=	*pucRegBuffer++;                 
                switch (iRegIndex){
                case REG_HOLDING_AOUT:
                  if((NewData<=AOUTMAX)&&(NewData>=AOUTMIN)){
                    usRegHoldingBuf[iRegIndex]=NewData;
                    //FLASHHOLDWrite((uint16_t*)&usRegHoldingBuf);
                  }
                  else eStatus = MB_EINVAL;
		  break;  
                case REG_HOLDING_RKICONFIG:
                  if((NewData<=RKICONFIGMAX)&&(NewData>=RKICONFIGMIN)){
                    usRegHoldingBuf[iRegIndex]=NewData;
                    set_time(TIME_FLASHWRITE,TIME_TO_FLASH);//FLASHHOLDWrite((uint16_t*)&usRegHoldingBuf);
                    //SysReset=SET;
                  }
                  else eStatus = MB_EINVAL;
		  break;  
                case REG_HOLDING_UOHMSHUNT1:
                case REG_HOLDING_UOHMSHUNT2:
                  if((NewData<=UOHMSHUNTMAX)&&(NewData>=UOHMSHUNTMIN)){
                    usRegHoldingBuf[iRegIndex]=NewData;
                    set_time(TIME_FLASHWRITE,TIME_TO_FLASH);//FLASHHOLDWrite((uint16_t*)&usRegHoldingBuf);

                  }
                  else eStatus = MB_EINVAL;
		  break;  
                case REG_HOLDING_DIVRPM1:
                  if((NewData<=DIVRPMMAX)&&(NewData>=DIVRPMMIN)){
                    usRegHoldingBuf[iRegIndex]=NewData;
                    set_time(TIME_FLASHWRITE,TIME_TO_FLASH);//FLASHHOLDWrite((uint16_t*)&usRegHoldingBuf);
                  }
                  else eStatus = MB_EINVAL;
		  break;
                case REG_HOLDING_DIVRPM2:
                  if((NewData<=DIVRPMMAX)&&(NewData>=DIVRPMMIN)){
                    usRegHoldingBuf[iRegIndex]=NewData;
                    set_time(TIME_FLASHWRITE,TIME_TO_FLASH);//FLASHHOLDWrite((uint16_t*)&usRegHoldingBuf);
                  }
                  else eStatus = MB_EINVAL;
		  break;
                case REG_HOLDING_THDRPM1:
                  if((NewData<=THDRPMMAX)&&(NewData>=THDRPMMIN)){
                    usRegHoldingBuf[iRegIndex]=NewData;
                    set_time(TIME_FLASHWRITE,TIME_TO_FLASH);//FLASHHOLDWrite((uint16_t*)&usRegHoldingBuf);
                  }
                  else eStatus = MB_EINVAL;
		  break;
                case REG_HOLDING_THDRPM2:
                  if((NewData<=THDRPMMAX)&&(NewData>=THDRPMMIN)){
                    usRegHoldingBuf[iRegIndex]=NewData;
                    set_time(TIME_FLASHWRITE,TIME_TO_FLASH);//FLASHHOLDWrite((uint16_t*)&usRegHoldingBuf);
                  }
                  else eStatus = MB_EINVAL;
		  break;
                case REG_HOLDING_RSTMOTOTIME1:
                  if(NewData>0){
                    MotoTime1=0;
                    set_time(TIME_DEFFERWRITE,5);
                  }
                  else eStatus = MB_EINVAL;
		  break;
                case REG_HOLDING_RSTMOTOTIME2:
                  if(NewData>0){
                    MotoTime2=0;
                    set_time(TIME_DEFFERWRITE,5);
                  }
                  else eStatus = MB_EINVAL;
		  break;
                }
                iRegIndex++;
                usNRegs--;
            }
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils,
eMBRegisterMode eMode )
{
	return MB_ENOREG;
}

eMBErrorCode 
eMBRegDiscreteCB( 	UCHAR * pucRegBuffer,USHORT usAddress,USHORT usNDiscrete)
{
  eMBErrorCode    eStatus = MB_ENOERR;

  
  if((usNDiscrete + usAddress)<=REG_DISCRETE_START + REG_DISCRETE_NREGS)
  {
        set_time(TIME_LED,3);LedOFF();// LED show RS-485 LINK 
         
        *pucRegBuffer++ = ( unsigned char ) usRegDiscreteInput; 
  }
  else
  {
      eStatus = MB_ENOREG;
  }
  
  return eStatus;
}