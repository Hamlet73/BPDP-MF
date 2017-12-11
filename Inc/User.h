#define RKINUM      8

#define RKI1        0
#define RKI2        1
#define RKI3        2
#define RKI4        3
#define RKI5        4
#define RKI6        5
#define RKI7        6
#define RKI8        7    
    
#define PUD1HI()        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET)
#define PUD2HI()        HAL_GPIO_WritePin(GPIOD,GPIO_PIN_1,GPIO_PIN_SET)
#define PUD3HI()        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET)
#define PUD4HI()        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_SET)
#define PUD5HI()        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET)
#define PUD6HI()        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET)
#define PUD7HI()        HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3,GPIO_PIN_SET)
#define PUD8HI()        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET)

#define RKTypHI(RKNum)  ((Config & (1<<RKNum))>=1)

#define RKI1STATE()     (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8)==GPIO_PIN_SET)   
#define RKI2STATE()     (HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_2)==GPIO_PIN_SET)  
#define RKI3STATE()     (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6)==GPIO_PIN_SET)  
#define RKI4STATE()     (HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_1)==GPIO_PIN_SET)  
#define RKI5STATE()     (HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_8)==GPIO_PIN_SET)  
#define RKI6STATE()     (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_11)==GPIO_PIN_SET)  
#define RKI7STATE()     (HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_7)==GPIO_PIN_SET)  
#define RKI8STATE()     (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8)==GPIO_PIN_SET)  

void    RKInit(uint8_t Config);
uint8_t RKRead(uint8_t Config);

/******************************************************************************
******************              Timer for RPM           ***********************
******************************************************************************/
#define TIMRPM1 htim1
#define TIMRPM2 htim8

#define UPTICK                  45000       
#define DOWNTICK                15000
#define STEPPRSCUPDATE          40
#define PRSCMAX                 1000    
#define PRSCMIN                 0
#define PRSCDEF                 140

#define __HAL_TIM_GET_PRESCALER(__HANDLE__) ((__HANDLE__)->Instance->PSC)

#define TIMEOUTRPM      10000
#define MAXRPMVALUE     20000



uint32_t StepPRSC(uint32_t PRSC);// return value step
uint16_t PRMRead(TIM_HandleTypeDef htim,uint16_t *dHZValue,uint32_t *TimeOutRPM,uint16_t Div);
void ReadParam(void);