#ifndef __I2S_H__
#define __I2S_H__

#include "platform_definitions.h"
#include "stm32f4xx.h"

#include "i2s_definitions.h"
#include <stdint.h>

#define I2S_MASTER_TRANSMIT (0x02)
#define I2S_MASTER_RECEIVER (0x03)

#define GPIO_ALTERNATE_FUN_I2S (0x05)

#define I2S_PLLM_VAL (4U)
#define I2S_PLLN_VAL (200U)
#define I2S_PLLR_VAL (4U)

typedef enum
{
    INTERRUPTS_USED,
    INTERRUPTS_NOT_USED
} InterruptsUsed_t;

typedef void (*RxInterruptCallback_t)();

typedef enum
{
    RISING_EDGE,
    FALLING_EDGE
} ClockPolarity_t;

typedef struct
{
    uint32_t pll_m_value;
    uint32_t pll_n_value;
    uint32_t pll_r_value;
} I2sPllConfig_t;

typedef uint8_t ClockDivider_t;

typedef enum
{
    NOT_ODD,
    ODD
} PrescalerOddBit_t;
typedef struct
{
    ClockDivider_t clock_divider_value;
    PrescalerOddBit_t prescaler_odd_bit;
} PrescalerConfig_t;

typedef struct
{
    I2sConfigsID_t i2s_config_id;
    InterruptsUsed_t receive_interrupt_flag;
    RxInterruptCallback_t receive_interrupt_callback;
    ClockPolarity_t clock_polarity;
    I2sPllConfig_t pll_config;
    PrescalerConfig_t prescaler_config;
}   I2sInterface_t;

void i2s_init(const I2sInterface_t* const i2s_interface);
void i2s_transmit(uint16_t data);

// typedef struct
// {
//     /* data */
// } I2sInterface;

#endif //__I2S_H__
