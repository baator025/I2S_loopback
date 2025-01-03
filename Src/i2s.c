#include "i2s.h"

/*
 * local defines section
*/
#define GPIO_ALTERNATE_FUN (0x02)
#define GPIO_MODE_BASE_MASK (0x3U)
#define GPIO_AFR_BASE_MASK (0x0FU)
#define GPIO_OSPEEDR_MASK (0x03U)

#define I2S_MSB_STANDARD (0x01)
#define I2S_LSB_STANDARD (0x02)
#define I2S_16_BIT_DATA_LEN (0x00)

#define I2S_MASTER_TRANSMIT (0x02)
#define I2S_MASTER_RECEIVER (0x03)
#define I2S_SLAVE_TRANSMITTER (0x00)

/*
 * static functions section
*/
static void i2s_configure_gpios(const I2sInterface_t * const i2s_interface);
static void i2s_configure_interrupts(const I2sInterface_t * const i2s_interface);
static void i2s_configure_parameters(const I2sInterface_t * const i2s_interface);
static void i2s_configure_clock(const I2sInterface_t * const i2s_interface);


static volatile uint32_t rx_reg_status[4];
static volatile uint32_t tx_reg_status[4];
static volatile uint32_t log_ctr = 0;

// static volatile uint16_t data[4];
// static volatile uint16_t data[4] = {0x0000, 0x0000, 0xFFFE, 0x5551};
#define DATA_SIZE (16)
// static volatile uint16_t data[16] = {0x1000, 0x2000, 0x3000, 0x4000,
//                                         0x5000, 0x6000, 0x7000, 0x8000,
//                                         0x9000, 0xA000, 0xB000, 0xC000,
//                                         0xD000, 0xE000, 0xF000, 0x0000};
// static volatile uint16_t data[16] = {0x0000, 0x0001, 0x0002, 0x0003,
//                                         0x0004, 0x0005, 0x0006, 0x0007,
//                                         0x0008, 0x0009, 0x000A, 0x000B,
//                                         0x000C, 0x000D, 0x000E, 0x000F};
// static volatile uint16_t data[16] = {0x0000, 0x0000, 0xFFFE, 0x5551,
//                                         0xFFFE, 0x5551, 0x7000, 0x8000,
//                                         0xFFFE, 0x5551, 0xFFFE, 0x5551,
//                                         0xFFFE, 0x5551, 0xFFFE, 0x5551};
static volatile uint16_t data[16] = {0x5551, 0xFFFE, 0x5551, 0xFFFE,
                                        0x5551, 0xFFFE, 0x5551, 0xFFFE,
                                        0x5551, 0xFFFE, 0x5551, 0xFFFE,
                                        0x5551, 0xFFFE, 0x5551, 0xFFFE};
static volatile uint8_t data_counter = 0;
static volatile uint8_t data_ready = 0;

/*
 * functions definition section
*/

void i2s_init(const I2sInterface_t* const i2s_interface)
{
    i2s_configure_gpios(i2s_interface);
    i2s_configure_clock(i2s_interface);
    i2s_configure_interrupts(i2s_interface);
    i2s_configure_parameters(i2s_interface);
    // start_i2s(i2s_interface);
}

static void i2s_configure_gpios(const I2sInterface_t * const i2s_interface)
{
    /* those pointers may be later placed in an array and all configuration
        may be done in one loop*/
    const GpioI2sConfig_t * clk_pin = &(i2s_configurations[i2s_interface->i2s_config_id].clk_pin);
    const GpioI2sConfig_t * data_pin = &(i2s_configurations[i2s_interface->i2s_config_id].data_pin);
    const GpioI2sConfig_t * ws_pin = &(i2s_configurations[i2s_interface->i2s_config_id].ws_pin);

    // enable clock for gpio_banks:
    RCC->AHB1ENR |= clk_pin->rcc_pin_bank_position;
    RCC->AHB1ENR |= data_pin->rcc_pin_bank_position;
    RCC->AHB1ENR |= ws_pin->rcc_pin_bank_position;

    // set gpios to alternate functions:
    (clk_pin->pin_bank)->MODER &= ~(GPIO_MODE_BASE_MASK << clk_pin->pin_mode_position);
    (clk_pin->pin_bank)->MODER |= (GPIO_ALTERNATE_FUN << clk_pin->pin_mode_position);

    (data_pin->pin_bank)->MODER &= ~(GPIO_MODE_BASE_MASK << data_pin->pin_mode_position);
    (data_pin->pin_bank)->MODER |= (GPIO_ALTERNATE_FUN << data_pin->pin_mode_position);

    (ws_pin->pin_bank)->MODER &= ~(GPIO_MODE_BASE_MASK << ws_pin->pin_mode_position);
    (ws_pin->pin_bank)->MODER |= (GPIO_ALTERNATE_FUN << ws_pin->pin_mode_position);

    // specify alternate functions:
    (clk_pin->pin_bank)->AFR[clk_pin->afr_register_index] &= ~(GPIO_AFR_BASE_MASK << clk_pin->afr_pin_position);
    (clk_pin->pin_bank)->AFR[clk_pin->afr_register_index] |= (clk_pin->afr_value << clk_pin->afr_pin_position);

    (data_pin->pin_bank)->AFR[data_pin->afr_register_index] &= ~(GPIO_AFR_BASE_MASK << data_pin->afr_pin_position);
    (data_pin->pin_bank)->AFR[data_pin->afr_register_index] |= (data_pin->afr_value << data_pin->afr_pin_position);

    (ws_pin->pin_bank)->AFR[ws_pin->afr_register_index] &= ~(GPIO_AFR_BASE_MASK << ws_pin->afr_pin_position);
    (ws_pin->pin_bank)->AFR[ws_pin->afr_register_index] |= (ws_pin->afr_value << ws_pin->afr_pin_position);

    //ospeedr config
    (clk_pin->pin_bank)->OSPEEDR &= ~(GPIO_OSPEEDR_MASK << clk_pin->ospeedr_pin_position);
    (clk_pin->pin_bank)->OSPEEDR |= (i2s_interface->gpio_ospeedr << clk_pin->ospeedr_pin_position);

    (data_pin->pin_bank)->OSPEEDR &= ~(GPIO_OSPEEDR_MASK << data_pin->ospeedr_pin_position);
    (data_pin->pin_bank)->OSPEEDR |= (i2s_interface->gpio_ospeedr << data_pin->ospeedr_pin_position);

    (ws_pin->pin_bank)->OSPEEDR &= ~(GPIO_OSPEEDR_MASK << ws_pin->ospeedr_pin_position);
    (ws_pin->pin_bank)->OSPEEDR |= (i2s_interface->gpio_ospeedr << ws_pin->ospeedr_pin_position);
}

void i2s_configure_pll(const I2sPllConfig_t * const pll_config)
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

    // set DMA on or off
    if(i2s_interface->dma_status == I2S_DMA_ON)
    {
        spi_reg->CR2 |= SPI_CR2_RXDMAEN;
    }
    else
    {
        spi_reg->CR2 &= ~SPI_CR2_RXDMAEN;
    }

}

static void i2s_configure_interrupts(const I2sInterface_t * const i2s_interface)
{
    SPI_TypeDef* const spi_reg = i2s_configurations[i2s_interface->i2s_config_id].i2s_configuration.spi_register;
    const IRQn_Type irq_id = i2s_configurations[i2s_interface->i2s_config_id].i2s_configuration.irq_id;
    const I2sMode_t i2s_mode = i2s_interface->i2s_mode;

    if(i2s_interface->receive_interrupt_flag == INTERRUPTS_USED)
    {
        if(i2s_mode == MASTER_TRANSMITTER || i2s_mode == SLAVE_TRANSMITTER)
        {
            spi_reg->CR2 |= SPI_CR2_TXEIE;
        }
        else
        {
            spi_reg->CR2 |= SPI_CR2_RXNEIE;
            spi_reg->CR2 |= SPI_CR2_ERRIE;
        }
        NVIC_SetPriority(irq_id, 0);
        NVIC_EnableIRQ(irq_id);
    }
    else if(i2s_interface->receive_interrupt_flag == INTERRUPTS_NOT_USED)
    {
        spi_reg->CR2 &= ~SPI_CR2_RXNEIE;
        spi_reg->CR2 &= ~SPI_CR2_ERRIE;
        spi_reg->CR2 &= ~SPI_CR2_TXEIE;
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

I2sTxReadinessState_t i2s_is_tx_ready(const I2sInterface_t* const i2s_interface)
{
    I2sTxReadinessState_t return_value = I2S_TX_NOT_READY;
    SPI_TypeDef* const spi_reg = i2s_configurations[i2s_interface->i2s_config_id].i2s_configuration.spi_register;
    if((spi_reg->SR & SPI_SR_TXE) == SPI_SR_TXE)
    {
        return_value = I2S_TX_READY;
    }
    return return_value;
}


void i2s_transmit(uint16_t data, const I2sInterface_t* const i2s_interface)
{
    SPI_TypeDef* const spi_reg = i2s_configurations[i2s_interface->i2s_config_id].i2s_configuration.spi_register;
    spi_reg->DR = data;
    // while(!(spi_reg->SR & SPI_SR_TXE)){
    //     reg_status[0] = ((spi_reg->CR2) & SPI_SR_FRE);
    //     reg_status[1] = ((spi_reg->CR2) & SPI_SR_OVR);
    //     reg_status[2] = ((spi_reg->CR2) & SPI_SR_UDR);
    // };
}

void SPI2_IRQHandler()
{
    data_counter++;
}


void SPI4_IRQHandler()
{
    if(data_counter < 4)
    {
        rx_reg_status[data_counter] = SPI4->SR;
        tx_reg_status[data_counter] = SPI3->SR;
        data[data_counter] = SPI4->DR;
        data_counter++;
    }
    else
    {
        data_ready = 1;
        // SPI4->I2SCFGR &= ~SPI_I2SCFGR_I2SE;
        // SPI3->I2SCFGR &= ~SPI_I2SCFGR_I2SE;
    }
}

void SPI3_IRQHandler()
{
    // const uint64_t LIMIT = 205;
    // for(uint64_t i  = 0; i < LIMIT; i++){}
    if(data_counter < DATA_SIZE)
    {
        SPI3->DR = data[data_counter];
        // rx_reg_status[data_counter] = SPI4->SR;
        // tx_reg_status[data_counter] = SPI3->SR;
        // data[data_counter] = SPI4->DR;
        data_counter++;
    }
    else
    {
        data_ready = 1;
        SPI4->I2SCFGR &= ~SPI_I2SCFGR_I2SE;
        SPI3->I2SCFGR &= ~SPI_I2SCFGR_I2SE;
    }
}

void get_readouts(uint16_t* data_ptr)
{
    for (uint8_t i = 0; i < 4; i++)
    {
        data_ptr[i] = data[i];
    }
}

uint8_t is_data_ready()
{
    return(data_ready);
}