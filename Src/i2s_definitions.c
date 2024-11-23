#include "i2s_definitions.h"

/*
 * local defines section
*/
#define I2S2_PIN_ALTERNATE_FUNCTION 0x05
#define I2S3_PIN_ALTERNATE_FUNCTION 0x06
#define I2S4_PIN_ALTERNATE_FUNCTION 0x05

/*
 * config definition section
*/

const I2sConfig i2s_configurations[NUMBER_OF_CONFIGS] =
{
    //I2S2
    {.clk_pin = {.pin_bank = GPIOB,
                .pin_mode_position = GPIO_MODER_MODE10_Pos,
                .rcc_pin_bank_position = RCC_AHB1ENR_GPIOBEN,
                .afr_value = I2S2_PIN_ALTERNATE_FUNCTION,
                .afr_pin_position = GPIO_AFRH_AFSEL10_Pos,
                .afr_register_index = 1},
    .data_pin = {.pin_bank = GPIOC,
                .pin_mode_position = GPIO_MODER_MODE3_Pos,
                .rcc_pin_bank_position = RCC_AHB1ENR_GPIOCEN,
                .afr_value = I2S2_PIN_ALTERNATE_FUNCTION,
                .afr_pin_position = GPIO_AFRL_AFSEL3_Pos,
                .afr_register_index = 0},
    .i2s_configuration= {.spi_register = SPI2,
                        .i2s_rcc_register = &(RCC->APB1ENR),
                        .i2s_rcc_register_mask = RCC_APB1ENR_SPI2EN,
                        .irq_id = SPI2_IRQn}},
    //I2S3
    {.clk_pin = {.pin_bank = GPIOC,
                .pin_mode_position = GPIO_MODER_MODE10_Pos,
                .rcc_pin_bank_position = RCC_AHB1ENR_GPIOCEN,
                .afr_value = I2S3_PIN_ALTERNATE_FUNCTION,
                .afr_pin_position = GPIO_AFRH_AFSEL10_Pos,
                .afr_register_index = 1},
    .data_pin = {.pin_bank = GPIOC,
                .pin_mode_position = GPIO_MODER_MODE12_Pos,
                .rcc_pin_bank_position = RCC_AHB1ENR_GPIOCEN,
                .afr_value = I2S3_PIN_ALTERNATE_FUNCTION,
                .afr_pin_position = GPIO_AFRH_AFSEL12_Pos,
                .afr_register_index = 1},
    .i2s_configuration= {.spi_register = SPI3,
                        .i2s_rcc_register = &(RCC->APB1ENR),
                        .i2s_rcc_register_mask = RCC_APB1ENR_SPI3EN,
                        .irq_id = SPI3_IRQn}},
    //I2S4
    {.clk_pin = {.pin_bank = GPIOE,
                .pin_mode_position = GPIO_MODER_MODE12_Pos,
                .rcc_pin_bank_position = RCC_AHB1ENR_GPIOEEN,
                .afr_value = I2S4_PIN_ALTERNATE_FUNCTION,
                .afr_pin_position = GPIO_AFRH_AFSEL12_Pos,
                .afr_register_index = 1},
    .data_pin = {.pin_bank = GPIOE,
                .pin_mode_position = GPIO_MODER_MODE14_Pos,
                .rcc_pin_bank_position = RCC_AHB1ENR_GPIOCEN,
                .afr_value = I2S4_PIN_ALTERNATE_FUNCTION,
                .afr_pin_position = GPIO_AFRH_AFSEL14_Pos,
                .afr_register_index = 1},
    .i2s_configuration= {.spi_register = SPI4,
                        .i2s_rcc_register = &(RCC->APB2ENR),
                        .i2s_rcc_register_mask = RCC_APB2ENR_SPI4EN,
                        .irq_id = SPI4_IRQn}}
};