#include "clock.h"
uint32_t rcc_view;

/*
 * System clk source    = HSE
 * SYSCLK               = 100 MHz
 * HCLK                 = 100 MHz
 * AHB Prescaler        = 1
 * APB1 Prescaler       = 2
 * APB2 Prescaler       = 2
 * HSE                  = 8 MHz
 */

void nucleo_clock_100mhz_config(void)
{
    // HSE Enable
    RCC->CR |= RCC_CR_HSEON;

    // Wait until HSERDY is low
    while((RCC->CR & RCC_CR_HSERDY) != (RCC_CR_HSERDY)){};

    // Set HCLK Prescaler
    RCC->CFGR &= ~RCC_CFGR_HPRE;
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;

    // Set APB1 Prescaler
    RCC->CFGR &= ~RCC_CFGR_PPRE1;
    RCC->CFGR |= ~RCC_CFGR_PPRE1_DIV2;

    // Set APB2 Prescaler
    RCC->CFGR &= ~RCC_CFGR_PPRE2;
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;

    // Configure PLL
    RCC->PLLCFGR |= (PLL_M_NUCL) |
                    (PLL_N_NUCL << 6) |
                    (((PLL_P_NUCL>>1) - 1) << 16) |
                    (RCC_PLLCFGR_PLLSRC_HSE) |
                    (PLL_Q_NUCL << 24);

    // Turn PLL on
    RCC->CR |= RCC_CR_PLLON;

    // Wait for PLL to start
    while((RCC->CR & RCC_CR_PLLRDY) == 0){};

    // Set flash latency
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY_3WS;

    // Select main PLL as the clock source
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;

    // Wait for mail PLL to be used as the clock source
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL){};

}

/*
 * System clk source    = HSI
 * SYSCLK               = 100 MHz
 * HCLK                 = 100 MHz
 * AHB Prescaler        = 1
 * APB1 Prescaler       = 2
 * APB2 Prescaler       = 1
 */

void discovery_clock_100mhz_config(void)
{
    RCC->CR |= RCC_CR_HSION;

    // Wait until HSIRDY is low
    while((RCC->CR & RCC_CR_HSIRDY) != (RCC_CR_HSIRDY)){};

    // Set HCLK Prescaler
    RCC->CFGR &= ~RCC_CFGR_HPRE;
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;

    // Set APB1 Prescaler
    RCC->CFGR &= ~RCC_CFGR_PPRE1;
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;

    // Set APB2 Prescaler
    RCC->CFGR &= ~RCC_CFGR_PPRE2;
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;

    // Clear PLL config
    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM_Msk |
                        RCC_PLLCFGR_PLLN_Msk |
                        RCC_PLLCFGR_PLLP_Msk);

    // Configure PLL
    RCC->PLLCFGR |= (PLL_M_DISC) |
                    (PLL_N_DISC << 6) |
                    (((PLL_P_DISC>>1) - 1) << 16) |
                    (RCC_PLLCFGR_PLLSRC_HSI);

    // Turn PLL on
    RCC->CR |= RCC_CR_PLLON;

    // Wait for PLL to start
    while((RCC->CR & RCC_CR_PLLRDY) == 0){};

    // Set flash latency
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= FLASH_ACR_LATENCY_3WS;

    // Select main PLL as the clock source
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;

    // Wait for mail PLL to be used as the clock source
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL){};
}