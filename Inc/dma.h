#ifndef __DMA_H__
#define __DMA_H__

#include "platform_definitions.h"
#include "stm32f4xx.h"

#define DMA_DATA_LEN (4U)
#define DMA_PERIPH_DATA_WIDTH_16_BIT (0x01U)
#define DMA_MEM_DATA_WIDTH_16_BIT (0x01U)

#define DMA_CHANNEL_SPI2_RX (0U)
#define DMA_PRIORITY (3U)
#define DMA_FIFO_THRESH_32_BIT (00)
#define DMA_DIR_PERIPH_TO_MEM (00)
#define DMA_SINGLE_TRANSFER (0x00U)

#define LISR_CLR_MASK (DMA_LIFCR_CFEIF3 | DMA_LIFCR_CDMEIF3 | \
                            DMA_LIFCR_CTEIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTCIF3)

#define LISR_ANY_IRQ_FLAG (DMA_LISR_FEIF3 | DMA_LISR_DMEIF3 | \
                            DMA_LISR_TEIF3 | DMA_LISR_HTIF3 | DMA_LISR_TCIF3)

typedef enum
{
    BUFFER_READY,
    BUFFER_NOT_READY
} BufferStatus_t;

void dma_init(uint64_t* data_ptr, volatile BufferStatus_t* buffer_status_flag);

typedef enum
{
    DMA_DISABLED = 0U,
    DMA_ENABLED = 1U
} DmaStatus_t;

#endif //__DMA_H__