#include "stm32F4xx.h"
#include "mb.h"
#include "mbport.h"
#include "User.h"



#define MBAdressMIN		2
#define MBAdressMAX		247
#define MBParityNone		0
#define MBParityOdd		1
#define MBParityEven		2

#define MBAdressDefault         2
#define MBSpeedDefault	        9216
#define MBParityDefault		MBParityEven


#define REG_INPUT_START         0
#define REG_INPUT_NREGS         16
#define REG_INPUT_V1	        0
#define REG_INPUT_V2	        1
#define REG_INPUT_A1	        2
#define REG_INPUT_A2	        3
#define REG_INPUT_RPM1	        4
#define REG_INPUT_RPM2          5
#define REG_INPUT_MOTOTIME1     6
#define REG_INPUT_MOTOTIME2     8
#define REG_INPUT_OPERTIME      10
#define REG_INPUT_STATUS        15

#define REG_INPUT_STARTSERVICE          65527 //0xFFF7
#define REG_INPUT_NREGSSERVICE          4
#define REG_INPUT_SERIALNUM             0
#define REG_INPUT_VERFIRMWARE           2

#define REG_HOLDING_START	        0
#define REG_HOLDING_NREGS	        10
#define REG_HOLDING_AOUT		0
#define REG_HOLDING_RKICONFIG   	1
#define REG_HOLDING_UOHMSHUNT1	        2
#define REG_HOLDING_UOHMSHUNT2	        3
#define REG_HOLDING_DIVRPM1	        4
#define REG_HOLDING_DIVRPM2	        5
#define REG_HOLDING_THDRPM1	        6
#define REG_HOLDING_THDRPM2	        7
#define REG_HOLDING_RSTMOTOTIME1        8
#define REG_HOLDING_RSTMOTOTIME2        9

#define AOUTMAX                 10000
#define AOUTMIN                 0
#define RKICONFIGMAX            0x00FF
#define RKICONFIGMIN            0
#define UOHMSHUNTMAX            0xFFFF
#define UOHMSHUNTMIN            1
#define DIVRPMMAX               10
#define DIVRPMMIN               0
#define THDRPMMAX               6000
#define THDRPMMIN               0

#define REG_DISCRETE_START      0
#define REG_DISCRETE_NREGS      RKINUM //not over 8 input
#define REG_DISCRETE_RK1        RKI1
#define REG_DISCRETE_RK2        RKI2
#define REG_DISCRETE_RK3        RKI3
#define REG_DISCRETE_RK4        RKI4
#define REG_DISCRETE_RK5        RKI5
#define REG_DISCRETE_RK6        RKI6
#define REG_DISCRETE_RK7        RKI7
#define REG_DISCRETE_RK8        RKI8