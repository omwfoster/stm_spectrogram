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
#include "audio_stream.h"


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

  	typedef struct {
  		uint8_t first_half[PDM_BUFFER_SIZE / 2];
  		uint32_t guard1;
  		uint8_t last_half[PDM_BUFFER_SIZE / 2];
  		uint32_t guard2;
  	} pdm_buffer_t;



  typedef struct {
  		int16_t ping[FFT_SIZE / 2];
  		uint32_t guard1;
  		int16_t pong[FFT_SIZE / 2];
  		uint32_t guard2;
  		int16_t * pcm_current_block;
  		int16_t * pcm_full;
  		int16_t * cursor;
  		int16_t * end_output_block;
  		bool initialised;
  	}pcm_buffer_t;





/* Global variables ---------------------------------------------------------*/
extern PDM_Filter_Handler_t PDM1_filter_handler;
extern PDM_Filter_Config_t PDM1_filter_config;

// Buffer management


extern bool block_ready;
extern volatile transfer_state_t transfer_state;
extern int16_t *pcm_current_block;


/* PDM2PCM init function */
void MX_PDM2PCM_Init(void);
uint32_t Audio_Process_PDM(pcm_buffer_t *pcm_buffer, pdm_buffer_t * pdm_buffer );
void Audio_Switch_Block(pcm_buffer_t * pcm_buffer);
void Pcm_Initialise(pcm_buffer_t *pcm_buffer);

void Check_PDM_Guards(pdm_buffer_t *buf);
void Check_PCM_Guards(pcm_buffer_t *buf);

/* USER CODE BEGIN 2 */

/* PDM2PCM process function */
uint8_t MX_PDM2PCM_Process(uint16_t *PDMBuf, uint16_t *PCMBuf);





#ifdef __cplusplus
}
#endif
#endif /*__pdm2pcm_H */

/**
  * @}
  */
