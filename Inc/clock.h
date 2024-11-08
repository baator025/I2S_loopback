#ifndef __CLOCK_H
#define __CLOCK_H

#define STM32F411xE
#include "stm32f4xx.h"

#define PLL_M_NUCL   4
#define PLL_N_NUCL   200
#define PLL_P_NUCL   4
#define PLL_Q_NUCL   9

#define PLL_M_DISC   8
#define PLL_N_DISC   200
#define PLL_P_DISC   4
#define PLL_Q_DISC   9

void nucleo_clock_100mhz_config(void);
void discovery_clock_100mhz_config(void);

#endif //__CLOCK_H