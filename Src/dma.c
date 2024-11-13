#include "dma.h"

/*
* static funtions prototypes
*/

static void dma_clock_init();
static void set_dma_status(DmaStatus_t status);
static void configure_transfer_path(uint16_t *data_ptr);
static void configure_interrupt();


static volatile uint32_t DMA_LISR_FEIF3_ctr = 0;
static volatile uint32_t DMA_LISR_DMEIF3_ctr = 0;
static volatile uint32_t DMA_LISR_TEIF3_ctr = 0;
static volatile uint32_t DMA_LISR_HTIF3_ctr = 0;
static volatile uint32_t DMA_LISR_TCIF3_ctr = 0;

static volatile uint32_t interrupt_cnt;
/*
* function implementation
*/

void dma_init(uint16_t* data_ptr)
{
    dma_clock_init();

    // 1. disable dma and check - DMA_SxCR EN bit must be 0
    set_dma_status(DMA_DISABLED);
    configure_transfer_path(data_ptr);

    // 4. total number of data - DMA_SxNDTR
    DMA1_Stream3->NDTR &= ~DMA_SxNDT;
    DMA1_Stream3->NDTR |= (DMA_DATA_LEN << DMA_SxNDT_Pos);

    // 6. DMA is the flow controller
    DMA1_Stream3->CR &= ~DMA_SxCR_PFCTRL;

    // 7. stream priority - DMA_SxCR - PL[1:0]
    DMA1_Stream3->CR &= ~DMA_SxCR_PL;
    DMA1_Stream3->CR |= (DMA_PRIORITY<<DMA_SxCR_PL_Pos);

    // 8. FIFO usage (enable or disable, threshold in transmission and reception)
    // fifos off - direct stream
    DMA1_Stream3->FCR &= ~DMA_SxFCR_DMDIS;

    // 9. Configure peripheral and memory incremented/fixed mode,

    //peripheral data width, do not increment
    DMA1_Stream3->CR &= ~DMA_SxCR_PSIZE;
    DMA1_Stream3->CR |= (DMA_PERIPH_DATA_WIDTH_16_BIT << DMA_SxCR_PSIZE_Pos);
    DMA1_Stream3->CR &= ~DMA_SxCR_PINC;

    //memory data width, do not increment
    DMA1_Stream3->CR &= ~DMA_SxCR_MSIZE;
    DMA1_Stream3->CR |= (DMA_MEM_DATA_WIDTH_16_BIT << DMA_SxCR_MSIZE_Pos);
    DMA1_Stream3->CR &= ~DMA_SxCR_MINC;

    // Circular mode on
    DMA1_Stream3->CR |= DMA_SxCR_CIRC;

    // Double buffer moder off
    DMA1_Stream3->CR &= ~DMA_SxCR_DBM;

    //  single or burst transactions,
    DMA1_Stream3->CR &= ~DMA_SxCR_MBURST;
    DMA1_Stream3->CR |= (DMA_SINGLE_TRANSFER << DMA_SxCR_MBURST_Pos);
    DMA1_Stream3->CR &= ~DMA_SxCR_PBURST;
    DMA1_Stream3->CR |= (DMA_SINGLE_TRANSFER << DMA_SxCR_PBURST_Pos);

    configure_interrupt();

    // 10. enable dma - EN bit in DMA_SxCR
    set_dma_status(DMA_ENABLED);
}

static void configure_transfer_path(uint16_t *data_ptr)
{
    // 2. set peripheral address - DMA_SxPAR
    DMA1_Stream3->PAR = (uint32_t) &(SPI2->DR);

    // 3. memory address DMA_SxMA0R
    DMA1_Stream3->M0AR = (uint32_t) data_ptr;

    // 5. set DMA channel - CHSEL in DMA_SxCR
    DMA1_Stream3->CR &= ~DMA_SxCR_CHSEL;
    DMA1_Stream3->CR |= (DMA_CHANNEL_SPI2_RX << DMA_SxCR_CHSEL_Pos);

    // data transfer direction
    DMA1_Stream3->CR &= ~DMA_SxCR_DIR;
    DMA1_Stream3->CR |= (DMA_DIR_PERIPH_TO_MEM << DMA_SxCR_DIR_Pos);

}

static void configure_interrupt()
{
    DMA1->LIFCR |= LISR_CLR_MASK;
    while(DMA1->LISR & LISR_ANY_IRQ_FLAG){};
    // interrupts after half and/or full transfer, and/or errors in the DMA_SxCR register
    // direct mode interrupt
    DMA1_Stream3->CR |= DMA_SxCR_DMEIE;
    DMA1_Stream3->CR |= DMA_SxCR_TEIE;
    // DMA1_Stream3->CR |= DMA_SxCR_HTIE;
    DMA1_Stream3->CR |= DMA_SxCR_TCIE;

    NVIC_EnableIRQ(DMA1_Stream3_IRQn);
}

static void set_dma_status(DmaStatus_t status)
{
    if(status == DMA_ENABLED)
    {
        DMA1_Stream3->CR |= DMA_SxCR_EN;
        while (!(DMA1_Stream3->CR & DMA_SxCR_EN)) {}
    } else if (status == DMA_DISABLED)
    {
        DMA1_Stream3->CR &= ~DMA_SxCR_EN;
        while (DMA1_Stream3->CR & DMA_SxCR_EN) {}
    }
}

static void dma_clock_init()
{
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
}

static inline void toggle_debug_pin()
{
    GPIOA->ODR ^= GPIO_ODR_OD3;
}

void DMA1_Stream3_IRQHandler()
{
    toggle_debug_pin();
    if(DMA1->LISR & DMA_LISR_FEIF3){DMA_LISR_FEIF3_ctr++;}
    if(DMA1->LISR & DMA_LISR_DMEIF3){DMA_LISR_DMEIF3_ctr++;}
    if(DMA1->LISR & DMA_LISR_TEIF3){DMA_LISR_TEIF3_ctr++;}
    if(DMA1->LISR & DMA_LISR_HTIF3){DMA_LISR_HTIF3_ctr++;}
    if(DMA1->LISR & DMA_LISR_TCIF3){DMA_LISR_TCIF3_ctr++;}
    interrupt_cnt++;
    DMA1->LIFCR |= LISR_CLR_MASK;
    while(DMA1->LISR & LISR_ANY_IRQ_FLAG){};
    toggle_debug_pin();
}