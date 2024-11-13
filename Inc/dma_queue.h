#ifndef __FIFO_H__
#define __FIFO_H__

#include <string.h>

#include "platform_definitions.h"
#include "stm32f4xx.h"

#define QUEUE_LENGTH 32
#define BUFFER_LENGTH (QUEUE_LENGTH + 1)

typedef enum
{
    OP_STATUS_OK,
    OP_STATUS_FAILED
} OperationStatus_t;

struct DmaQueue_s;
typedef uint64_t sample_t;
typedef OperationStatus_t (*shift_fun_ptr) (struct DmaQueue_s *this);
typedef OperationStatus_t (*pop_fun_ptr) (struct DmaQueue_s *this, sample_t* sample_value);

//oldest sample - id = samples_counter
typedef struct DmaQueue_s
{
    sample_t samples_buffer[BUFFER_LENGTH];
    uint8_t samples_counter;
    shift_fun_ptr shift_samples;
    pop_fun_ptr pop_sample;
} DmaQueue_t;

void initialize_queue(struct DmaQueue_s *fifo_object);

#endif //__FIFO_H__