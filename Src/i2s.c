#include "i2s.h"

/*
 * local defines section
*/
#define GPIO_ALTERNATE_FUN (0x02)
#define GPIO_MODE_BASE_MASK (0x3U)
#define GPIO_AFR_BASE_MASK (0x0FU)

#define I2S_MSB_STANDARD (0x01)
#define I2S_16_BIT_DATA_LEN (0x00)

#define I2S_MASTER_TRANSMIT (0x02)
#define I2S_MASTER_RECEIVER (0x03)
#define I2S_SLAVE_TRANSMITTER (0x00)

/*
 * static functions section
*/
static void i2s_configure_gpios(const I2sInterface_t * const i2s_interface);
static void i2s_configure_plls(const I2sPllConfig_t * const pll_config);
static void i2s_configure_interrupts(const I2sInterface_t * const i2s_interface);
static void i2s_configure_parameters(const I2sInterface_t * const i2s_interface);
static void i2s_configure_clock(const I2sInterface_t * const i2s_interface);
void start_i2s(const I2sInterface_t * const i2s_interface);

static volatile uint16_t data;
static volatile uint32_t reg_status[3];
static volatile uint32_t log_ctr = 0;

/*
 * functions definition section
*/

void i2s_init(const I2sInterface_t* const i2s_interface)
{
    i2s_configure_gpios(i2s_interface);
    i2s_configure_plls(&(i2s_interface->pll_config));
    i2s_configure_interrupts(i2s_interface);
    i2s_configure_clock(i2s_interface);
    i2s_configure_parameters(i2s_interface);
    start_i2s(i2s_interface);
}

static void i2s_configure_gpios(const I2sInterface_t * const i2s_interface)
{
    const GpioI2sConfig_t * clk_pin = &(i2s_configurations[i2s_interface->i2s_config_id].clk_pin);
    const GpioI2sConfig_t * data_pin = &(i2s_configurations[i2s_interface->i2s_config_id].data_pin);

    // enable clock for gpio_banks:
    RCC->AHB1ENR |= clk_pin->rcc_pin_bank_position;
    RCC->AHB1ENR |= data_pin->rcc_pin_bank_position;

    // set gpios to alternate functions:
    (clk_pin->pin_bank)->MODER &= ~(GPIO_MODE_BASE_MASK << clk_pin->pin_mode_position);
    (clk_pin->pin_bank)->MODER |= (GPIO_ALTERNATE_FUN << clk_pin->pin_mode_position);

    (data_pin->pin_bank)->MODER &= ~(GPIO_MODE_BASE_MASK << data_pin->pin_mode_position);
    (data_pin->pin_bank)->MODER |= (GPIO_ALTERNATE_FUN << data_pin->pin_mode_position);

    // specify alternate functions:
    (clk_pin->pin_bank)->AFR[clk_pin->afr_register_index] &= ~(GPIO_AFR_BASE_MASK << clk_pin->afr_pin_position);
    (clk_pin->pin_bank)->AFR[clk_pin->afr_register_index] |= (clk_pin->afr_value << clk_pin->afr_pin_position);

    (data_pin->pin_bank)->AFR[data_pin->afr_register_index] &= ~(GPIO_AFR_BASE_MASK << data_pin->afr_pin_position);
    (data_pin->pin_bank)->AFR[data_pin->afr_register_index] |= (data_pin->afr_value << data_pin->afr_pin_position);
}

static void i2s_configure_plls(const I2sPllConfig_t * const pll_config)
{
    // I2S PLL configuration (RCC_PLLI2SCFGR)
    RCC->PLLI2SCFGR &= ~RCC_PLLI2SCFGR_PLLI2SM;
    RCC->PLLI2SCFGR |= (pll_config->pll_m_value << RCC_PLLI2SCFGR_PLLI2SM_Pos);

    RCC->PLLI2SCFGR &= ~RCC_PLLI2SCFGR_PLLI2SN;
    RCC->PLLI2SCFGR |= (pll_config->pll_n_value << RCC_PLLI2SCFGR_PLLI2SN_Pos);

    RCC->PLLI2SCFGR &= ~RCC_PLLI2SCFGR_PLLI2SR;
    RCC->PLLI2SCFGR |= (pll_config->pll_r_value << RCC_PLLI2SCFGR_PLLI2SR_Pos);

    // I2S PLL as clock source
    RCC->CFGR &= ~RCC_CFGR_I2SSRC;

    // I2S PLL RCC start (RCC_CR)
    RCC->CR |= RCC_CR_PLLI2SON;
    while(!(RCC->CR & RCC_CR_PLLI2SRDY)){};
}

static void i2s_configure_parameters(const I2sInterface_t * const i2s_interface)
{
    SPI_TypeDef* const spi_reg = i2s_configurations[i2s_interface->i2s_config_id].i2s_configuration.spi_register;

    // set spi peripheral to i2s mode
    spi_reg->I2SCFGR |= SPI_I2SCFGR_I2SMOD;

    // set clock polarity
    if(i2s_interface->clock_polarity == RISING_EDGE)
    {
        spi_reg->I2SCFGR |= SPI_I2SCFGR_CKPOL;
    }
    else if(i2s_interface->clock_polarity == FALLING_EDGE)
    {
        spi_reg->I2SCFGR &= ~SPI_I2SCFGR_CKPOL;
    }

    // set i2s standard - msb for now, needs to be parametrized later
    spi_reg->I2SCFGR &= ~SPI_I2SCFGR_I2SSTD;
    spi_reg->I2SCFGR |= (I2S_MSB_STANDARD << SPI_I2SCFGR_I2SSTD_Pos);

    // set i2s data length - 16 bit for now, needs to be parametrized later
    spi_reg->I2SCFGR &= ~SPI_I2SCFGR_DATLEN;
    spi_reg->I2SCFGR |= (I2S_16_BIT_DATA_LEN << SPI_I2SCFGR_DATLEN_Pos);

    // 16 bit channel width
    spi_reg->I2SCFGR &= ~SPI_I2SCFGR_CHLEN;

    // set i2s bus configuration
    switch (i2s_interface->i2s_mode)
    {
    case MASTER_RECEIVER:
        spi_reg->I2SCFGR &= ~SPI_I2SCFGR_I2SCFG;
        spi_reg->I2SCFGR |= (I2S_MASTER_RECEIVER << SPI_I2SCFGR_I2SCFG_Pos);
        break;
    case MASTER_TRANSMITTER:
        spi_reg->I2SCFGR &= ~SPI_I2SCFGR_I2SCFG;
        spi_reg->I2SCFGR |= (I2S_MASTER_TRANSMIT << SPI_I2SCFGR_I2SCFG_Pos);
        break;
    case SLAVE_TRANSMITTER:
        spi_reg->I2SCFGR &= ~SPI_I2SCFGR_I2SCFG;
        spi_reg->I2SCFGR |= (I2S_SLAVE_TRANSMITTER << SPI_I2SCFGR_I2SCFG_Pos);
        break;
    default:
        break;
    }

    // set DMA on, needs to be parametrized later
    spi_reg->CR2 |= SPI_CR2_RXDMAEN;

}

static void i2s_configure_interrupts(const I2sInterface_t * const i2s_interface)
{
    SPI_TypeDef* const spi_reg = i2s_configurations[i2s_interface->i2s_config_id].i2s_configuration.spi_register;
    const IRQn_Type irq_id = i2s_configurations[i2s_interface->i2s_config_id].i2s_configuration.irq_id;

    if(i2s_interface->receive_interrupt_flag == INTERRUPTS_USED)
    {
        spi_reg->CR2 |= SPI_CR2_RXNEIE;
        NVIC_EnableIRQ(irq_id);
    }
    else if(i2s_interface->receive_interrupt_flag == INTERRUPTS_NOT_USED)
    {
        spi_reg->CR2 &= ~SPI_CR2_RXNEIE;
        NVIC_DisableIRQ(irq_id);
    }

    //handling callback needs to be implemented
}

static void i2s_configure_clock(const I2sInterface_t * const i2s_interface)
{
    SPI_TypeDef* const spi_reg = i2s_configurations[i2s_interface->i2s_config_id].i2s_configuration.spi_register;
    volatile uint32_t* rcc_peripheral_register = i2s_configurations[i2s_interface->i2s_config_id].i2s_configuration.i2s_rcc_register;
    uint32_t i2s_rcc_mask = i2s_configurations[i2s_interface->i2s_config_id].i2s_configuration.i2s_rcc_register_mask;

    // enable clock access to spi/i2s
    *rcc_peripheral_register |= i2s_rcc_mask;

    // set divider register
    spi_reg->I2SPR &= ~SPI_I2SPR_I2SDIV;
    spi_reg->I2SPR |= (i2s_interface->prescaler_config.clock_divider_value << SPI_I2SPR_I2SDIV_Pos);

    if(i2s_interface->prescaler_config.prescaler_odd_bit == NOT_ODD)
    {
        spi_reg->I2SPR &= ~SPI_I2SPR_ODD;
    }
    else if(i2s_interface->prescaler_config.prescaler_odd_bit != ODD)
    {
        spi_reg->I2SPR |= SPI_I2SPR_ODD;
    }

    //disable master clock output
    spi_reg->I2SPR &= ~SPI_I2SPR_MCKOE;
}

void start_i2s(const I2sInterface_t * const i2s_interface)
{
    SPI_TypeDef* const spi_reg = i2s_configurations[i2s_interface->i2s_config_id].i2s_configuration.spi_register;
    spi_reg->I2SCFGR |= SPI_I2SCFGR_I2SE;
}


void i2s_transmit(uint16_t data, const I2sInterface_t* const i2s_interface)
{
    SPI_TypeDef* const spi_reg = i2s_configurations[i2s_interface->i2s_config_id].i2s_configuration.spi_register;
    spi_reg->DR |= data;
    // while(!(spi_reg->SR & SPI_SR_TXE)){
    //     reg_status[0] = ((spi_reg->CR2) & SPI_SR_FRE);
    //     reg_status[1] = ((spi_reg->CR2) & SPI_SR_OVR);
    //     reg_status[2] = ((spi_reg->CR2) & SPI_SR_UDR);
    // };
}

void SPI2_IRQHandler()
{
}