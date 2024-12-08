#include "dma.h"

/*
* static funtions prototypes
*/

static void dma_clock_init(const DmaNumber_t dma_main_register);
static void set_dma_status(DMA_Stream_TypeDef * const dma_stream, DmaStatus_t status);
static void configure_transfer_path(uint64_t *data_ptr, Dma_t * const dma);
// static void configure_interrupt();
static void configure_interrupt(Dma_t * const dma);
static void configure_data_flow(Dma_t * const dma);

static void configure_fifo(Dma_t * const dma);

static volatile uint32_t DMA_LISR_FEIF3_ctr = 0;
static volatile uint32_t DMA_LISR_DMEIF3_ctr = 0;
static volatile uint32_t DMA_LISR_TEIF3_ctr = 0;
static volatile uint32_t DMA_LISR_HTIF3_ctr = 0;
static volatile uint32_t DMA_LISR_TCIF3_ctr = 0;
static volatile uint32_t interrupt_cnt;

/**
 * local defines section
 */

#define DMA_DIR_PERIPH_TO_MEM (00)
#define DMA_DATA_SIZE_BYTE          (00)        //8 bit
#define DMA_DATA_SIZE_HALF_WORD     (0x01)      //16 bit
#define DMA_DATA_SIZE_WORD          (0x02)      //32 bit
#define DMA_PRIORITY (3U)
#define DMA_SINGLE_TRANSFER (0x00U)

#define LISR_CLR_MASK (DMA_LIFCR_CFEIF3 | DMA_LIFCR_CDMEIF3 | \
                            DMA_LIFCR_CTEIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTCIF3)

#define LISR_CLR_MASK0 (DMA_LIFCR_CFEIF0 | DMA_LIFCR_CDMEIF0 | \
                            DMA_LIFCR_CTEIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTCIF0)

#define LISR_ANY_IRQ_FLAG (DMA_LISR_FEIF3 | DMA_LISR_DMEIF3 | \
                            DMA_LISR_TEIF3 | DMA_LISR_HTIF3 | DMA_LISR_TCIF3)

#define LISR_ANY_IRQ_FLAG0 (DMA_LISR_FEIF0 | DMA_LISR_DMEIF0 | \
                            DMA_LISR_TEIF0 | DMA_LISR_HTIF0 | DMA_LISR_TCIF0)

/*
* function implementation
*/

static volatile BufferStatus_t *buffer_ready_flag;

void dma_init(void* data_ptr, volatile BufferStatus_t* buffer_status_flag, Dma_t * const dma)
{
    dma_clock_init(dma->dma_main_register);

    // 1. disable dma and check - DMA_SxCR EN bit must be 0
    set_dma_status(dma->dma_stream, DMA_DISABLED);
    configure_transfer_path(data_ptr, dma);

    // 7. stream priority - DMA_SxCR - PL[1:0]
    (dma->dma_stream)->CR &= ~DMA_SxCR_PL;
    (dma->dma_stream)->CR |= (DMA_PRIORITY<<DMA_SxCR_PL_Pos);

    configure_fifo(dma);
    configure_data_flow(dma);
    configure_interrupt(dma);

    buffer_ready_flag = buffer_status_flag;

    // 10. enable dma - EN bit in DMA_SxCR
    set_dma_status(dma->dma_stream, DMA_ENABLED);
    // set_dma_status(DMA_ENABLED);
}

static uint32_t get_data_size_reg_value(const DmaDataSize_t data_size)
{
    uint32_t data_size_register_value = 0;
    switch(data_size)
    {
        case BYTE:
            data_size_register_value = DMA_DATA_SIZE_BYTE;
            break;
        case HALF_WORD:
            data_size_register_value = DMA_DATA_SIZE_HALF_WORD;
            break;
        case WORD:
            data_size_register_value = DMA_DATA_SIZE_WORD;
            break;
    }

    return data_size_register_value;
}

static void configure_fifo(Dma_t * const dma)
{
    // 8. FIFO usage    -   fixed for now
    // fifos off - direct stream
    (dma->dma_stream)->FCR &= ~DMA_SxFCR_DMDIS;
}


static void configure_data_flow(Dma_t * const dma)
{
    // 4. total number of data - DMA_SxNDTR
    (dma->dma_stream)->NDTR &= ~DMA_SxNDT;
    (dma->dma_stream)->NDTR |= (dma->dma_data_length << DMA_SxNDT_Pos);

    configure_fifo(dma);

    // 9. Configure peripheral and memory incremented/fixed mode,

    //peripheral data width, do not increment
    uint32_t peripheral_data_size = get_data_size_reg_value(dma->peripheral_data_size);
    (dma->dma_stream)->CR &= ~DMA_SxCR_PSIZE;
    (dma->dma_stream)->CR |= (peripheral_data_size << DMA_SxCR_PSIZE_Pos);
    (dma->dma_stream)->CR &= ~DMA_SxCR_PINC;        //incrementing fixed, needs to be parametrized

    //memory data width, increment memory pointer
    uint32_t memory_data_size = get_data_size_reg_value(dma->memory_data_size);
    (dma->dma_stream)->CR &= ~DMA_SxCR_MSIZE;
    (dma->dma_stream)->CR |= (memory_data_size << DMA_SxCR_MSIZE_Pos);
    (dma->dma_stream)->CR |= DMA_SxCR_MINC;


    // Circular mode on             - fixed for now
    (dma->dma_stream)->CR |= DMA_SxCR_CIRC;

    // Double buffer moder off      - fixed for now
    (dma->dma_stream)->CR &= ~DMA_SxCR_DBM;

    //  single or burst transactions,   - fixed for now
    (dma->dma_stream)->CR &= ~DMA_SxCR_MBURST;
    (dma->dma_stream)->CR |= (DMA_SINGLE_TRANSFER << DMA_SxCR_MBURST_Pos);
    (dma->dma_stream)->CR &= ~DMA_SxCR_PBURST;
    (dma->dma_stream)->CR |= (DMA_SINGLE_TRANSFER << DMA_SxCR_PBURST_Pos);

    // 6. DMA is the flow controller    - fixed for now
    (dma->dma_stream)->CR &= ~DMA_SxCR_PFCTRL;
}


static void configure_transfer_path(uint64_t *data_ptr, Dma_t * const dma)
{
    // data transfer direction
    dma->dma_stream->CR &= ~DMA_SxCR_DIR;
    switch (dma->dma_direction)
    {
    case PERIPHERAL_TO_MEMORY:
        dma->dma_stream->CR |= (DMA_DIR_PERIPH_TO_MEM << DMA_SxCR_DIR_Pos);
        break;
    default:
        break;
    }

    // 2. set peripheral address - DMA_SxPAR
    dma->dma_stream->PAR = (uint32_t) dma->peripheral_address;

    // 3. memory address DMA_SxMA0R
    dma->dma_stream->M0AR = (uint32_t) dma->memory_address;

    // 5. set DMA channel - CHSEL in DMA_SxCR
    dma->dma_stream->CR &= ~DMA_SxCR_CHSEL;
    dma->dma_stream->CR |= (dma->dma_channel << DMA_SxCR_CHSEL_Pos);
}

// static void configure_interrupt()
// {
//     DMA1->LIFCR |= LISR_CLR_MASK;
//     while(DMA1->LISR & LISR_ANY_IRQ_FLAG){};
//     // interrupts after half and/or full transfer, and/or errors in the DMA_SxCR register
//     // direct mode interrupt
//     DMA1_Stream3->CR |= DMA_SxCR_DMEIE;
//     DMA1_Stream3->CR |= DMA_SxCR_TEIE;
    // DMA1_Stream3->CR |= DMA_SxCR_HTIE;
//     DMA1_Stream3->CR |= DMA_SxCR_TCIE;

//     NVIC_EnableIRQ(DMA1_Stream3_IRQn);
// }

static void configure_interrupt(Dma_t * const dma)
{
    if((dma->dma_main_register) == DMA_1)
    {
        DMA1->LIFCR |= LISR_CLR_MASK;
        while(DMA1->LISR & LISR_ANY_IRQ_FLAG){};
    }
    else if((dma->dma_main_register) == DMA_2)
    {
        DMA2->LIFCR |= LISR_CLR_MASK;
        while(DMA2->LISR & LISR_ANY_IRQ_FLAG){};
    }

    // interrupts after half and/or full transfer, and/or errors in the DMA_SxCR register
    // direct mode interrupt
    (dma->dma_stream)->CR |= DMA_SxCR_DMEIE;
    (dma->dma_stream)->CR |= DMA_SxCR_TEIE;
    // (dma->dma_stream)->CR |= DMA_SxCR_HTIE;
    (dma->dma_stream)->CR |= DMA_SxCR_TCIE;

    NVIC_EnableIRQ(dma->dma_irq);
}


static void set_dma_status(DMA_Stream_TypeDef * const dma_stream, DmaStatus_t status)
{
    if(status == DMA_ENABLED)
    {
        dma_stream->CR |= DMA_SxCR_EN;
        while (!(dma_stream->CR & DMA_SxCR_EN)) {}
    } else if (status == DMA_DISABLED)
    {
        dma_stream->CR &= ~DMA_SxCR_EN;
        while (dma_stream->CR & DMA_SxCR_EN) {}
    }
}

static void dma_clock_init(DmaNumber_t const dma_main_register)
{
    if(dma_main_register == DMA_1)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    }
    else if(dma_main_register == DMA_2)
    {
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    }
}

static inline void toggle_debug_pin()
{
    GPIOA->ODR ^= GPIO_ODR_OD3;
}

void DMA1_Stream3_IRQHandler()
{
    toggle_debug_pin();
    *buffer_ready_flag = BUFFER_READY;
    // if(DMA1->LISR & DMA_LISR_FEIF3){DMA_LISR_FEIF3_ctr++;}
    // if(DMA1->LISR & DMA_LISR_DMEIF3){DMA_LISR_DMEIF3_ctr++;}
    // if(DMA1->LISR & DMA_LISR_TEIF3){DMA_LISR_TEIF3_ctr++;}
    // if(DMA1->LISR & DMA_LISR_HTIF3){DMA_LISR_HTIF3_ctr++;}
    // if(DMA1->LISR & DMA_LISR_TCIF3){DMA_LISR_TCIF3_ctr++;}
    // interrupt_cnt++;
    DMA1->LIFCR |= LISR_CLR_MASK;
    while(DMA1->LISR & LISR_ANY_IRQ_FLAG){};
    toggle_debug_pin();
}

void DMA2_Stream0_IRQHandler()
{
    SPI4->I2SCFGR &= ~SPI_I2SCFGR_I2SE;
    SPI3->I2SCFGR &= ~SPI_I2SCFGR_I2SE;
    toggle_debug_pin();
    *buffer_ready_flag = BUFFER_READY;
    if(DMA2->LISR & DMA_LISR_FEIF0){DMA_LISR_FEIF3_ctr++;}
    if(DMA2->LISR & DMA_LISR_DMEIF0){DMA_LISR_DMEIF3_ctr++;}
    if(DMA2->LISR & DMA_LISR_TEIF0){DMA_LISR_TEIF3_ctr++;}
    if(DMA2->LISR & DMA_LISR_HTIF0){DMA_LISR_HTIF3_ctr++;}
    if(DMA2->LISR & DMA_LISR_TCIF0){DMA_LISR_TCIF3_ctr++;}
    interrupt_cnt++;
    DMA2->LIFCR |= LISR_CLR_MASK0;
    while(DMA2->LISR & LISR_ANY_IRQ_FLAG0){};
    DMA2_Stream0->CR &= ~DMA_SxCR_EN;


    toggle_debug_pin();
}