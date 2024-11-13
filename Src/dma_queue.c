#include "dma_queue.h"

OperationStatus_t pop_sample(struct DmaQueue_s *this, sample_t* sample_value);
OperationStatus_t shift_samples(struct DmaQueue_s *this);

void initialize_queue(struct DmaQueue_s *fifo_object)
{
    fifo_object->pop_sample = pop_sample;
    fifo_object->shift_samples = shift_samples;
    memset(&(fifo_object->samples_buffer[0]), 0UL, BUFFER_LENGTH*sizeof(uint64_t));
    fifo_object->samples_counter = 0;
}

OperationStatus_t pop_sample(struct DmaQueue_s *this, sample_t* sample_value)
{
    OperationStatus_t return_value = OP_STATUS_FAILED;
    if(this->samples_counter > 0){
        uint8_t sample_id = this->samples_counter;
        this->samples_counter--;
        *sample_value = this->samples_buffer[sample_id];
        return_value = OP_STATUS_OK;
    }

    return return_value;
}

OperationStatus_t shift_samples(struct DmaQueue_s *this)
{
    OperationStatus_t return_value = OP_STATUS_FAILED;
    this->samples_counter++;
    const uint8_t sample_counter = this->samples_counter;
    sample_t *buffer_ptr = &(this->samples_buffer[0]);
    if(sample_counter > 0 && sample_counter < (BUFFER_LENGTH - 1))
    {
        for(uint8_t i = sample_counter; i > 0; i--)
        {
            buffer_ptr[i] = buffer_ptr[i-1];
        }

        return_value = OP_STATUS_OK;
    }

    return return_value;
}