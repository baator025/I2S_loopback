#ifndef __DMA_H__
#define __DMA_H__

#include "platform_definitions.h"
#include "stm32f4xx.h"

typedef enum
{
    BUFFER_READY,
    BUFFER_NOT_READY
} BufferStatus_t;

typedef enum
{
    PERIPHERAL_TO_MEMORY,
} DmaDirection_t;

typedef enum
{
    LOW,
    MEDIUM,
    HIGH,
    VERY_HIGH,
    NUMBER_OF_PRIORITIES
} DmaPriority_t;

typedef enum
{
    BYTE,
    HALF_WORD,
    WORD
} DmaDataSize_t;

typedef enum
{
    DMA_1,
    DMA_2
} DmaNumber_t;

typedef struct
{
    DmaNumber_t dma_main_register;
    DMA_Stream_TypeDef * const dma_stream;  //table 27 in docs
    DmaDirection_t dma_direction;
    IRQn_Type dma_irq;
    uint32_t dma_channel;
    uint32_t dma_data_length;
    void * const memory_address;
    DmaDataSize_t memory_data_size;
    volatile uint32_t * const peripheral_address;
    DmaDataSize_t peripheral_data_size;
} Dma_t;

void dma_init(void* data_ptr, volatile BufferStatus_t* buffer_status_flag, Dma_t * const dma);
// void dma_init(uint64_t* data_ptr, volatile BufferStatus_t* buffer_status_flag);

typedef enum
{
    DMA_DISABLED = 0U,
    DMA_ENABLED = 1U
} DmaStatus_t;

#endif //__DMA_H__