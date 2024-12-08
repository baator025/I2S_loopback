#ifndef __I2S_DEFINITIONS_H__
#define __I2S_DEFINITIONS_H__

#include "platform_definitions.h"
#include "stm32f4xx.h"

typedef enum
{
    I2S2_CONF,
    I2S3_CONF,
    I2S4_CONF,
    NUMBER_OF_CONFIGS
}  I2sConfigsID_t;

typedef struct
{
    GPIO_TypeDef* pin_bank; // pin bank register - GPIOB/GPIOA
    uint32_t rcc_pin_bank_position;
    uint32_t pin_mode_position;
    uint8_t afr_register_index; // AFRx register
    uint32_t afr_pin_position;
    uint32_t afr_value; //AF5/AF6
} GpioI2sConfig_t;

typedef struct
{
    // i2s/spi
    SPI_TypeDef* spi_register;  // spi register - f.eg. spi2
    volatile uint32_t* i2s_rcc_register; // i2s rcc register - apb1enr/apb2enr
    uint32_t i2s_rcc_register_mask;
    IRQn_Type irq_id;
} I2sPeripheralConfig_t;

typedef struct
{
    GpioI2sConfig_t clk_pin;
    GpioI2sConfig_t data_pin;
    GpioI2sConfig_t ws_pin;
    I2sPeripheralConfig_t i2s_configuration;
} I2sConfig;

extern const I2sConfig i2s_configurations[NUMBER_OF_CONFIGS];

#endif // __I2S_DEFINITIONS_H__
