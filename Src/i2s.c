#include "i2s.h"

static void i2s_configure_gpio();
static void i2s_configure_params();
static void i2s_configure_pll();
static void i2s_configure_clock_divider();
static void start_i2s();
static void i2s_configure_interrupt(InterruptsUsed_t interrupt_switch);

static volatile uint16_t data;
static volatile uint16_t reg_status[3];

static volatile uint32_t log_ctr = 0;

void i2s_init(InterruptsUsed_t interrupt_switch)
{
    i2s_configure_gpio();
    i2s_configure_pll();
    i2s_configure_clock_divider();
    i2s_configure_params();
    i2s_configure_interrupt(interrupt_switch);

    start_i2s();
}

static void i2s_configure_gpio()
{
    // gpio setup - PDM_OUT - PC3, CLK_IN - PB10
    // gpio clock enable
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    // gpio alternate function
    GPIOB->MODER &= ~GPIO_MODER_MODE10;
    GPIOB->MODER |= (GPIO_ALTERNATE_FUN << GPIO_MODER_MODE10_Pos);
    GPIOC->MODER &= ~GPIO_MODER_MODE3;
    GPIOC->MODER |= (GPIO_ALTERNATE_FUN << GPIO_MODER_MODE3_Pos);

    // gpio alternate function type
    GPIOB->AFR[1] &= ~GPIO_AFRH_AFSEL10;
    GPIOB->AFR[1] |= (GPIO_ALTERNATE_FUN_I2S << GPIO_AFRH_AFSEL10_Pos);   //PB10 AF5
    GPIOB->AFR[0] &= ~GPIO_AFRL_AFSEL3;
    GPIOC->AFR[0] |= (GPIO_ALTERNATE_FUN_I2S << GPIO_AFRL_AFSEL3_Pos);   //PC3 AF5
}

static void i2s_configure_params()
{
    // spi in i2s mode
    SPI2->I2SCFGR |= SPI_I2SCFGR_I2SMOD;

    // clock polarity
    SPI2->I2SCFGR &= ~SPI_I2SCFGR_CKPOL;

    // i2s standard - msb
    SPI2->I2SCFGR &= ~SPI_I2SCFGR_I2SSTD;
    SPI2->I2SCFGR |= (I2S_MSB_STANDARD << SPI_I2SCFGR_I2SSTD_Pos);

    // i2s master receiver config
    SPI2->I2SCFGR &= ~SPI_I2SCFGR_I2SCFG;
    SPI2->I2SCFGR |= (I2S_MASTER_RECEIVER << SPI_I2SCFGR_I2SCFG_Pos);

    SPI2->I2SCFGR &= ~SPI_I2SCFGR_DATLEN;
    SPI2->I2SCFGR |= (I2S_16_BIT_DATA_LEN << SPI_I2SCFGR_DATLEN_Pos);

    // DMA on
    SPI2->CR2 |= SPI_CR2_RXDMAEN;
}

static void i2s_configure_interrupt(InterruptsUsed_t interrupt_switch)
{
    if(interrupt_switch == INTERRUPTS_USED)
    {
        SPI2->CR2 |= SPI_CR2_RXNEIE;
        NVIC_EnableIRQ(SPI2_IRQn);
    }
}

static void i2s_configure_pll()
{
    // I2S PLL configuration (RCC_PLLI2SCFGR)
    // I2SPLL Clk -> 100MHz
    RCC->PLLI2SCFGR &= ~RCC_PLLI2SCFGR_PLLI2SM;
    RCC->PLLI2SCFGR |= (I2S_PLLM_VAL << RCC_PLLI2SCFGR_PLLI2SM_Pos);

    RCC->PLLI2SCFGR &= ~RCC_PLLI2SCFGR_PLLI2SN;
    RCC->PLLI2SCFGR |= (I2S_PLLN_VAL << RCC_PLLI2SCFGR_PLLI2SN_Pos);

    RCC->PLLI2SCFGR &= ~RCC_PLLI2SCFGR_PLLI2SR;
    RCC->PLLI2SCFGR |= (I2S_PLLR_VAL << RCC_PLLI2SCFGR_PLLI2SR_Pos);

    // I2S PLL as clock source
    RCC->CFGR &= ~RCC_CFGR_I2SSRC;

    // I2S PLL RCC start (RCC_CR)
    RCC->CR |= RCC_CR_PLLI2SON;
    while(!(RCC->CR & RCC_CR_PLLI2SRDY)){};
}

static void i2s_configure_clock_divider()
{
    // I2S(SPI) APB1 clock enable (RCC_APB1ENR)
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

    // I2C registers prescaler
    //placeholder values below:
    SPI2->I2SPR &= ~SPI_I2SPR_I2SDIV;
    SPI2->I2SPR |= 50U << SPI_I2SPR_I2SDIV_Pos;

    SPI2->I2SPR &= ~SPI_I2SPR_ODD;
    SPI2->I2SPR &= ~SPI_I2SPR_MCKOE;
}

void start_i2s()
{
    SPI2->I2SCFGR |= SPI_I2SCFGR_I2SE;
}

void i2s_transmit(uint16_t data)
{
    SPI2->DR |= data;
    while(!(SPI2->SR & SPI_SR_TXE)){};
}

void SPI2_IRQHandler()
{
}