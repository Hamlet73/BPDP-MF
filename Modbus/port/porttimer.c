/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: porttimer.c,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"


/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
/* ----------------------- Defines ------------------------------------------*/
  #define MB_TIMER_PRESCALER      ( 32 )
  #define MB_50US_TICKS           ( 50 )

/* ----------------------- Static variables ---------------------------------*/
static USHORT   usTimerAutoReload;
extern TIM_HandleTypeDef htim4;

/* ----------------------- Start implementation -----------------------------*/
BOOL
xMBPortTimersInit( USHORT usTim1Timerout50us )
{

  usTimerAutoReload = (usTim1Timerout50us * MB_50US_TICKS)-1;
  htim4.Init.Period = usTimerAutoReload;
  htim4.Init.Prescaler = MB_TIMER_PRESCALER;
  htim4.Init.ClockDivision = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  HAL_TIM_Base_Init(&htim4);
  __HAL_TIM_DISABLE(&htim4);
  __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);
  return TRUE;
}


//inline 
void vMBPortTimersEnable(  )
{
    /* Enable the timer with the timeout passed to xMBPortTimersInit( ) */
  
  if( usTimerAutoReload > 0 )
  {
    __HAL_TIM_SET_COUNTER(&htim4,usTimerAutoReload);
  }
  __HAL_TIM_ENABLE_IT(&htim4,TIM_IT_UPDATE);
  __HAL_TIM_ENABLE(&htim4);
}

//inline 
void vMBPortTimersDisable(  )
{
    /* Disable any pending timers. */
  __HAL_TIM_DISABLE_IT(&htim4,TIM_IT_UPDATE);
  __HAL_TIM_DISABLE(&htim4);
}



  

BOOL timerMB_IRQHandler(void) 
{

  if(__HAL_TIM_GET_IT_SOURCE(&htim4,TIM_IT_UPDATE)!=RESET)
  {
    ( void )pxMBPortCBTimerExpired(  );
    __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);
    return TRUE;
  }
  return FALSE;
}
    
