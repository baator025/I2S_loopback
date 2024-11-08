#ifndef UART_H
#define UART_H

#include <stdint.h>

#define STM32F411xE
#include "stm32f4xx.h"
#include "platform_definitions.h"

#define GPIOAEN (1U << 0)
#define UART2EN (1U << 17)
#define CR1_TE (1U << 3)
#define CR1_UI (1U << 13)
#define SR_TXE (1U << 7)

#define APB1_CLK (SYS_FREQ/2)

#define UART_BAUDRATE 115200

void uart2_tx_init(void);

#endif //UART_h