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
 * File: $Id: portserial.c,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

#include "port.h"


/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "stm32f4xx_hal.h"

/* ----------------------- static functions ---------------------------------*/
static void prvvUARTTxReadyISR( void );
static void prvvUARTRxISR( void );
extern UART_HandleTypeDef huart2;

/* ----------------------- Start implementation -----------------------------*/
void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
    /* If xRXEnable enable serial receive interrupts. If xTxENable enable
     * transmitter empty interrupts.
     */
 if (xRxEnable) 
 {
   __HAL_UART_ENABLE_IT(&huart2,UART_IT_RXNE);
 } 
 else 
 {
   __HAL_UART_DISABLE_IT(&huart2,UART_IT_RXNE);
 }
 if (xTxEnable) 
 {
   RTS_HIGH;
   __HAL_UART_ENABLE_IT(&huart2,UART_IT_TXE);
 } 
 else 
 {
   __HAL_UART_DISABLE_IT(&huart2,UART_IT_TXE);
   __HAL_UART_ENABLE_IT(&huart2,UART_IT_TC);
 }
  
}

BOOL
xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = ulBaudRate;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  switch(eParity)
  {
   case MB_PAR_NONE:{huart2.Init.Parity = UART_PARITY_NONE ;break;}
   case MB_PAR_ODD:{
    huart2.Init.WordLength = UART_WORDLENGTH_9B;
    huart2.Init.Parity = UART_PARITY_ODD;
    break;
   }
   case MB_PAR_EVEN:{
    huart2.Init.WordLength = UART_WORDLENGTH_9B;
    huart2.Init.Parity = UART_PARITY_EVEN;
    break;
   }
  }
    
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

  return TRUE;
}

BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
    /* Put a byte in the UARTs transmit buffer. This function is called
     * by the protocol stack if pxMBFrameCBTransmitterEmpty( ) has been
     * called. */
    (&huart2)->Instance->DR = (ucByte & (uint8_t)0xFF);
    return TRUE;
}

BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
    /* Return the byte in the UARTs receive buffer. This function is called
     * by the protocol stack after pxMBFrameCBByteReceived( ) has been called.
     */
     *pucByte = (CHAR)((&huart2)->Instance->DR & (uint16_t)0x00FF);
    return TRUE;
}



BOOL usartMB_IRQHandler(void)
{ 
  if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) != RESET)
  {
    RTS_LOW;
    __HAL_UART_DISABLE_IT(&huart2, UART_IT_TC);
    __HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_TC);
    return TRUE;
  }
  if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) != RESET)
  {
      pxMBFrameCBByteReceived();
      __HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_RXNE);
    return TRUE;
  }
  if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE) != RESET)
  {
    pxMBFrameCBTransmitterEmpty();
    __HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_TXE);
    return TRUE;
  }


  return FALSE;
}


