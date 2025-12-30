/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : pdm2pcm.h
  * Description        : This file provides code for the configuration
  *                      of the pdm2pcm instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __pdm2pcm_H
#define __pdm2pcm_H
#ifdef __cplusplus
  extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "pdm2pcm_glo.h"
#include "stdbool.h"


#define MAX_DECIMATION_FACTOR 128
#define DECIMATION_FACTOR  PDM_FILTER_DEC_FACTOR_64
#define PDM_BUFFER_SIZE         (64 * 2 * 2)  // Ping-pong buffer for DMA
#define PCM_BUFFER_SIZE         FFT_SIZE      // Output PCM samples
#define PCM_OUT_SIZE            16            // PDM filter output size

  // Transfer states
  typedef enum {
  	TRANSFER_WAIT = 0, TRANSFER_COMPLETE, TRANSFER_HALF, TRANSFER_ERROR
  } transfer_state_t;

  // PDM buffer union for half/full access
  typedef union {
  	struct {
  		uint8_t first_half[PDM_BUFFER_SIZE / 2];
  		uint8_t last_half[PDM_BUFFER_SIZE / 2];
  	};
  	uint8_t PDM_In[PDM_BUFFER_SIZE];
  } pdm_buffer_t;






/* Global variables ---------------------------------------------------------*/
extern PDM_Filter_Handler_t PDM1_filter_handler;
extern PDM_Filter_Config_t PDM1_filter_config;

// Buffer management
extern int16_t *output_cursor;
extern int16_t *end_output_block;
extern volatile int16_t *pcm_full;
extern bool block_ready;
extern volatile transfer_state_t transfer_state;
extern int16_t *pcm_current_block;


/* PDM2PCM init function */
void MX_PDM2PCM_Init(void);
void Audio_Process_PDM(void);
void Audio_Switch_Block(void);

/* USER CODE BEGIN 2 */

/* PDM2PCM process function */
uint8_t MX_PDM2PCM_Process(uint16_t *PDMBuf, uint16_t *PCMBuf);

void Audio_Process_PDM(void);



#ifdef __cplusplus
}
#endif
#endif /*__pdm2pcm_H */

/**
  * @}
  */
