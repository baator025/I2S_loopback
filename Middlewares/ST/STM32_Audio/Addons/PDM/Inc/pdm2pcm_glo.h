/**
  ******************************************************************************
  * @file    pdm2pcm_glo.h
  * @author  MCD Application Team
  * @brief   Global header for PDM2PCM conversion code
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2011 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PDM2PCM_FILTER_H
#define __PDM2PCM_FILTER_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported constants --------------------------------------------------------*/
#define PDM_FILTER_ENDIANNESS_LE     ((uint16_t)0x0000)
#define PDM_FILTER_ENDIANNESS_BE     ((uint16_t)0x0001)

#define PDM_FILTER_BIT_ORDER_LSB     ((uint16_t)0x0000)
#define PDM_FILTER_BIT_ORDER_MSB     ((uint16_t)0x0001)

#define PDM_FILTER_DEC_FACTOR_48     ((uint16_t)0x0001)
#define PDM_FILTER_DEC_FACTOR_64     ((uint16_t)0x0002)
#define PDM_FILTER_DEC_FACTOR_80     ((uint16_t)0x0003)
#define PDM_FILTER_DEC_FACTOR_128    ((uint16_t)0x0004)
#define PDM_FILTER_DEC_FACTOR_16     ((uint16_t)0x0005)
#define PDM_FILTER_DEC_FACTOR_24     ((uint16_t)0x0006)
#define PDM_FILTER_DEC_FACTOR_32     ((uint16_t)0x0007)

#define PDM_FILTER_INIT_ERROR           ((uint16_t)0x0010)
#define PDM_FILTER_CONFIG_ERROR         ((uint16_t)0x0020)
#define PDM_FILTER_ENDIANNESS_ERROR     ((uint16_t)0x0001)
#define PDM_FILTER_BIT_ORDER_ERROR      ((uint16_t)0x0002)
#define PDM_FILTER_CRC_LOCK_ERROR       ((uint16_t)0x0004)
#define PDM_FILTER_DECIMATION_ERROR     ((uint16_t)0x0008)
#define PDM_FILTER_GAIN_ERROR           ((uint16_t)0x0040)
#define PDM_FILTER_SAMPLES_NUMBER_ERROR ((uint16_t)0x0080)
#define PDM2PCM_INTERNAL_MEMORY_SIZE 16

/* Exported types ------------------------------------------------------------*/
typedef struct{
  uint16_t bit_order;
  uint16_t endianness;
  uint32_t high_pass_tap;
  uint16_t in_ptr_channels;
  uint16_t out_ptr_channels;
  uint32_t pInternalMemory[PDM2PCM_INTERNAL_MEMORY_SIZE];
}PDM_Filter_Handler_t;

typedef struct{
  uint16_t decimation_factor;
  uint16_t output_samples_number;
  int16_t  mic_gain;
}PDM_Filter_Config_t;

/* Exported macros -----------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
uint32_t PDM_Filter_Init(PDM_Filter_Handler_t *pHandler);
uint32_t PDM_Filter_setConfig(PDM_Filter_Handler_t *pHandler, PDM_Filter_Config_t *pConfig); 
uint32_t PDM_Filter_getConfig(PDM_Filter_Handler_t *pHandler, PDM_Filter_Config_t *pConfig);
uint32_t PDM_Filter_deInterleave(void *pDataIn, void *pDataOut, PDM_Filter_Handler_t * pHandler);
uint32_t PDM_Filter(void *pDataIn, void *pDataOut, PDM_Filter_Handler_t *pHandler);

#ifdef __cplusplus
}
#endif

#endif /* __PDM2PCM_FILTER_H */
