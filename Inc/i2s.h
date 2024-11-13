#ifndef __I2S_H__
#define __I2S_H__

#include "platform_definitions.h"
#include "stm32f4xx.h"

#include <stdint.h>

#define I2S_MASTER_TRANSMIT (0x02)
#define I2S_MASTER_RECEIVER (0x03)

#define I2S_MSB_STANDARD (0x01)
#define I2S_16_BIT_DATA_LEN (0x00)
#define GPIO_ALTERNATE_FUN (0x02)
#define GPIO_ALTERNATE_FUN_I2S (0x05)

#define I2S_PLLM_VAL (4U)
#define I2S_PLLN_VAL (200U)
#define I2S_PLLR_VAL (4U)

typedef enum
{
    INTERRUPTS_USED,
    INTERRUPTS_NOT_USED
} InterruptsUsed_t;

void i2s_init(InterruptsUsed_t interrupt_switch);
void i2s_transmit(uint16_t data);

// typedef struct
// {
//     /* data */
// } I2sInterface;

#endif //__I2S_H__
