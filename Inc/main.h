/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
#define TrueToggle(PORT,PIN)  (HAL_GPIO_ReadPin(PORT,PIN)==GPIO_PIN_SET?HAL_GPIO_WritePin(PORT,PIN,GPIO_PIN_RESET):HAL_GPIO_WritePin(PORT,PIN,GPIO_PIN_SET))
#define LedON()         HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET)
#define LedOFF()        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET)
#define LedToggle()     TrueToggle(GPIOA,GPIO_PIN_1)

//#define TestLedON()         HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);
//#define TestLedOFF()        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);

#define FLASH_EE1OF2_SECTOR    FLASH_SECTOR_2
#define FLASH_EE2OF2_SECTOR    FLASH_SECTOR_3
#define FLASH_SECTOR2_START_ADDR  ((uint32_t)0x08008000)
#define FLASH_SECTOR3_START_ADDR  ((uint32_t)0x0800C000)

#define FLASH_CONTRDATA_ADDR   ((uint32_t)         0)
#define FLASH_OLD_MOTOTIME1    ((uint32_t)       0x04)
#define FLASH_OLD_MOTOTIME2    ((uint32_t)       0x08)
#define FLASH_OLD_OPERTIME     ((uint32_t)       0x0C)
#define FLASH_NEW_CONTRDATA    ((uint32_t)       0x10)
#define FLASH_NEW_MOTOTIME1    ((uint32_t)       0x14)
#define FLASH_NEW_MOTOTIME2    ((uint32_t)       0x18)
#define FLASH_NEW_OPERTIME     ((uint32_t)       0x1C)

#define ContrData              ((uint32_t)0xAAAAAAAA)

#define FLASH_HOLD_START_ADDR           ((uint32_t)0x08010000)
#define FLASH_HOLD_END_ADDR             ((uint32_t)0x0801001C)
#define FLASH_HOLD_SECTOR               FLASH_SECTOR_4
#define FLASH_HOLD_DATA_ADDR            ((uint32_t)0x08010000)


#define MAXVALUE        31999.0


#define CHEKPWRPIN              GPIO_PIN_0
    
#define VERFIRMWARE             101

#define	NumCount		        5//Numeric of counts
#define TIME_DEFFERWRITE                0
#define TIME_LED                        1
#define TIME_FILTER                     2
#define TIME_FILTERRPM                  3
#define TIME_FLASHWRITE                 4
#define set_time(name,ms10_set)         {clear_ovf(name);Counts[name]=ms10_set;}
#define time_ovf(name) 			(flag_ovf&(1<<name))
#define clear_ovf(name)			(flag_ovf&=~(1<<name))
#define time_stopd(name)		(Counts[name]==0)

#define TIME_TO_FLASH                   500

/******************************************************************************
******************              ADC Parameters          ***********************
******************************************************************************/
#define	ADCNUMCH      4
#define	ADCCHA1       0
#define	ADCCHA2       1
#define	ADCCHV1       2
#define	ADCCHV2       3

/******************************************************************************
******************            Filters Parameters        ***********************
******************************************************************************/
  
#define NUMPARAM        6
#define FilV1           0
#define FilV2           1
#define FilA1           2
#define FilA2           3
#define FILRPM1         4
#define FILRPM2         5

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
