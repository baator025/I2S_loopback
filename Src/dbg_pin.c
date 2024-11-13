#include "dbg_pin.h"

void configure_debug_pin()
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    const uint32_t PIN_OUTPUT_MODE = 0x01U;
    GPIOA->MODER &= ~GPIO_MODER_MODE3;
    GPIOA->MODER |= (PIN_OUTPUT_MODE << GPIO_MODER_MODE3_Pos);
}

void toggle_debug_pin()
{
    GPIOA->ODR ^= GPIO_ODR_OD3;
}