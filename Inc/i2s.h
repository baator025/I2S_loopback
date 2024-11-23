#ifndef __I2S_H__
#define __I2S_H__

#include "platform_definitions.h"
#include "stm32f4xx.h"

#include "i2s_definitions.h"
#include <stdint.h>

#define GPIO_ALTERNATE_FUN_I2S (0x05)

#define I2S_PLLM_VAL (4U)
#define I2S_PLLN_VAL (200U)
#define I2S_PLLR_VAL (4U)

/**
 * Typedef section
 */

typedef void (*RxInterruptCallback_t)();
typedef uint8_t ClockDivider_t;

/**
 * Enum Section
 */

typedef enum
{
    INTERRUPTS_USED,
    INTERRUPTS_NOT_USED
} InterruptsUsed_t;

typedef enum
{
    RISING_EDGE,
    FALLING_EDGE
} ClockPolarity_t;

typedef enum
{
    NOT_ODD,
    ODD
} PrescalerOddBit_t;

typedef enum
{
    MASTER_RECEIVER,
    MASTER_TRANSMITTER,
    SLAVE_TRANSMITTER
} I2sMode_t;

/**
 * Struct section
 */

typedef struct
{
    uint32_t pll_m_value;
    uint32_t pll_n_value;
    uint32_t pll_r_value;
} I2sPllConfig_t;
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
    I2sMode_t i2s_mode;
}   I2sInterface_t;

/**
 * Functions section
 */

void i2s_init(const I2sInterface_t* const i2s_interface);
void i2s_transmit(uint16_t data, const I2sInterface_t* const i2s_interface);


#endif //__I2S_H__
