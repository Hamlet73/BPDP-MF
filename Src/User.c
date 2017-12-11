//#include "stm32F4xx.h"
#include "stm32f4xx_hal.h"
#include "User.h"
#include "Modbus.h"

extern uint32_t MotoTime1,MotoTime2,OperTime;
extern USHORT   usRegHoldingBuf[];



void RKInit(uint8_t Config){
  uint8_t Ind;    
  for(Ind=0;Ind<RKINUM;Ind++){
    switch(Ind){
    case RKI1:if(RKTypHI(RKI1)) PUD1HI(); break;
    case RKI2:if(RKTypHI(RKI2)) PUD2HI(); break;
    case RKI3:if(RKTypHI(RKI3)) PUD3HI(); break;
    case RKI4:if(RKTypHI(RKI4)) PUD4HI(); break;
    case RKI5:if(RKTypHI(RKI5)) PUD5HI(); break;
    case RKI6:if(RKTypHI(RKI6)) PUD6HI(); break;
    case RKI7:if(RKTypHI(RKI7)) PUD7HI(); break;
    case RKI8:if(RKTypHI(RKI8)) PUD8HI(); break;
    }
  }
}

uint8_t RKRead(uint8_t Config){
    uint8_t RKSTATE=0;
    RKSTATE|=((RKI1STATE() ^ RKTypHI(RKI1))<<RKI1);
    RKSTATE|=((RKI2STATE() ^ RKTypHI(RKI2))<<RKI2);
    RKSTATE|=((RKI3STATE() ^ RKTypHI(RKI3))<<RKI3);
    RKSTATE|=((RKI4STATE() ^ RKTypHI(RKI4))<<RKI4);
    RKSTATE|=((RKI5STATE() ^ RKTypHI(RKI5))<<RKI5);
    RKSTATE|=((RKI6STATE() ^ RKTypHI(RKI6))<<RKI6);
    RKSTATE|=((RKI7STATE() ^ RKTypHI(RKI7))<<RKI7);
    RKSTATE|=((RKI8STATE() ^ RKTypHI(RKI8))<<RKI8);
    return RKSTATE;
}

void ReadParam(void)
{
  uint8_t Ind;
  for(Ind=0;Ind<REG_HOLDING_NREGS;Ind++)
    switch(Ind){
    case REG_HOLDING_AOUT:      if((usRegHoldingBuf[Ind]=(uint16_t)FLASHRead(FLASH_HOLD_DATA_ADDR + 4*Ind))>AOUTMAX)            usRegHoldingBuf[Ind]=0;break;
    case REG_HOLDING_RKICONFIG: if((usRegHoldingBuf[Ind]=(uint16_t)FLASHRead(FLASH_HOLD_DATA_ADDR + 4*Ind))>RKICONFIGMAX)       usRegHoldingBuf[Ind]=0;break;
    case REG_HOLDING_UOHMSHUNT1:
    case REG_HOLDING_UOHMSHUNT2:if((usRegHoldingBuf[Ind]=(uint16_t)FLASHRead(FLASH_HOLD_DATA_ADDR + 4*Ind))<UOHMSHUNTMIN)       usRegHoldingBuf[Ind]=UOHMSHUNTMIN;break;
    case REG_HOLDING_DIVRPM1:   if((usRegHoldingBuf[Ind]=(uint16_t)FLASHRead(FLASH_HOLD_DATA_ADDR + 4*Ind))>DIVRPMMAX)          usRegHoldingBuf[Ind]=0;break;
    case REG_HOLDING_DIVRPM2:   if((usRegHoldingBuf[Ind]=(uint16_t)FLASHRead(FLASH_HOLD_DATA_ADDR + 4*Ind))>DIVRPMMAX)          usRegHoldingBuf[Ind]=0;break;
    case REG_HOLDING_THDRPM1:   if((usRegHoldingBuf[Ind]=(uint16_t)FLASHRead(FLASH_HOLD_DATA_ADDR + 4*Ind))>THDRPMMAX)          usRegHoldingBuf[Ind]=0;break;
    case REG_HOLDING_THDRPM2:   if((usRegHoldingBuf[Ind]=(uint16_t)FLASHRead(FLASH_HOLD_DATA_ADDR + 4*Ind))>THDRPMMAX)          usRegHoldingBuf[Ind]=0;break;
    case REG_HOLDING_RSTMOTOTIME1: usRegHoldingBuf[REG_HOLDING_RSTMOTOTIME1]=0;break;
    case REG_HOLDING_RSTMOTOTIME2: usRegHoldingBuf[REG_HOLDING_RSTMOTOTIME2]=0;break;
    }
  
}
  
uint32_t StepPRSC(uint32_t PRSC)
{
  uint32_t Step=PRSC/2;
  if(Step<1) Step=1;
  if(Step>50) Step=50;
  return Step;
}
  
uint16_t PRMRead(TIM_HandleTypeDef htim,uint16_t *dHZValue,uint32_t *TimeOutRPM,uint16_t Div)
{
  uint16_t RPMTick,RPMPRSC;
  uint32_t ClockRPM=HAL_RCC_GetHCLKFreq();
  uint16_t RPMValue;
  RPMPRSC=__HAL_TIM_GET_PRESCALER(&htim);
  if(__HAL_TIM_GET_FLAG(&htim,TIM_FLAG_CC1)==SET){
    if(__HAL_TIM_GET_FLAG(&htim,TIM_FLAG_UPDATE)!=SET){
      RPMTick=HAL_TIM_ReadCapturedValue(&htim,TIM_CHANNEL_1);
      uint32_t temp=RPMTick;
      temp*=(uint32_t)RPMPRSC+1;
      *dHZValue=(uint16_t)(ClockRPM*10/temp);
      *TimeOutRPM=0;
      if((RPMTick<DOWNTICK)&&(RPMPRSC>=(PRSCMIN+StepPRSC(RPMPRSC)))){
        RPMPRSC-=StepPRSC(RPMPRSC);        
        __HAL_TIM_SET_PRESCALER(&htim, RPMPRSC);
      }
      if((RPMTick>UPTICK)&&(RPMPRSC<PRSCMAX)){
        RPMPRSC+=StepPRSC(RPMPRSC);
        __HAL_TIM_SET_PRESCALER(&htim, RPMPRSC);
      }
    }
    else {__HAL_TIM_CLEAR_FLAG(&htim,TIM_FLAG_UPDATE);
          if(RPMPRSC<PRSCMAX){
            RPMPRSC+=StepPRSC(RPMPRSC);
            __HAL_TIM_SET_PRESCALER(&htim, RPMPRSC);
          }
         }
    __HAL_TIM_CLEAR_FLAG(&htim,TIM_FLAG_CC1);
  }
  
  if(*TimeOutRPM<TIMEOUTRPM){
    (*TimeOutRPM)+=1;
  }
  else{
    *dHZValue= 0;
    if((RPMPRSC+STEPPRSCUPDATE) <PRSCMAX){
      RPMPRSC+=STEPPRSCUPDATE;         
      __HAL_TIM_SET_PRESCALER(&htim, RPMPRSC);
    }
  }

  switch (Div){
  case 1:       RPMValue=(*dHZValue)*6;         break;
  case 2:       RPMValue=(*dHZValue)*3;         break;
  case 3:       RPMValue=(*dHZValue)*2;         break;
  case 4:       RPMValue=((*dHZValue)*3)/2;     break;
  case 5:       RPMValue=((*dHZValue)*6)/5;     break;
  case 6:       RPMValue=(*dHZValue);           break;
  case 7:       RPMValue=((*dHZValue)*6)/7;     break;
  case 8:       RPMValue=((*dHZValue)*3)/4;     break;
  case 9:       RPMValue=((*dHZValue)*2)/3;     break;
  case 10:      RPMValue=((*dHZValue)*3)/5;     break;
  default:      RPMValue=(*dHZValue);           break;
  }
    
  return RPMValue;
}



