/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * File Name          : pdm2pcm.c
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

/* Includes ------------------------------------------------------------------*/
#include <audio_stream_dsp/audio_stream_PDM.h>
#include <audio_stream_dsp/audio_stream.h>

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/* Global variables ---------------------------------------------------------*/
PDM_Filter_Handler_t PDM1_filter_handler;
PDM_Filter_Config_t PDM1_filter_config;

#define SAMPLES_NUMBER = (uint16_t)((AUDIO_IN_SAMPLING_FREQUENCY / 1000U) * 1U);
#define HTONS(A)  ((((uint16_t)(A) & 0xff00) >> 8) | (((uint16_t)(A) & 0x00ff) << 8))


//int16_t PDM_Buffer[PDM_BUFFER_SIZE];
pdm_buffer_t pdm_buffer;
int16_t RecBuf[PCM_OUT_SIZE];

int16_t pcm_output_block_ping[FFT_SIZE];
int16_t pcm_output_block_pong[FFT_SIZE];


/* USER CODE BEGIN 1 */
/* USER CODE END 1 */

/* PDM2PCM init function */
void MX_PDM2PCM_Init(void)
{
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

   /**
  */
  PDM1_filter_handler.bit_order = PDM_FILTER_BIT_ORDER_MSB ;
  PDM1_filter_handler.endianness = PDM_FILTER_ENDIANNESS_LE;
  PDM1_filter_handler.high_pass_tap = 2122358088;
  PDM1_filter_handler.in_ptr_channels = 1;
  PDM1_filter_handler.out_ptr_channels = 1;
  PDM_Filter_Init(&PDM1_filter_handler);

  PDM1_filter_config.decimation_factor = PDM_FILTER_DEC_FACTOR_32;
  PDM1_filter_config.output_samples_number = 16;
  PDM1_filter_config.mic_gain = 12;
  PDM_Filter_setConfig(&PDM1_filter_handler, &PDM1_filter_config);

  /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */

}

/**
 * @brief Process PDM data from DMA buffer
 * Filters PDM to PCM and manages ping-pong buffers
 */
void Audio_Process_PDM(void) {
	int16_t *next_cursor = output_cursor + PCM_OUT_SIZE ;

	 uint32_t index = 0;

	if (transfer_state == TRANSFER_COMPLETE) {


		if (next_cursor <= end_output_block) {
			// Still room in current block

			PDM_Filter((uint8_t *)pdm_buffer.last_half, (int16_t*)RecBuf, &PDM1_filter_handler);
			    int16_t min_val = 32767, max_val = -32768;
			    for(int i = 0; i < PCM_OUT_SIZE; i++) {
			        if(RecBuf[i] < min_val) min_val = RecBuf[i];
			        if(RecBuf[i] > max_val) max_val = RecBuf[i];
			    }
			    printf("PCM range: %d to %d\n", min_val, max_val);
			memcpy(output_cursor, RecBuf, PCM_OUT_SIZE * sizeof(int16_t));
			output_cursor = next_cursor;
		} else {
			// Block full, switch to next block
			pcm_full = pcm_current_block;
			Audio_Switch_Block();
			PDM_Filter((uint8_t *)pdm_buffer.last_half, (int16_t*)RecBuf, &PDM1_filter_handler);
			memcpy(output_cursor, RecBuf, PCM_OUT_SIZE * sizeof(int16_t));
			output_cursor += PCM_OUT_SIZE;
		}
	} else if (transfer_state == TRANSFER_HALF) {


		if (next_cursor <= end_output_block) {
			// Still room in current block
			PDM_Filter((uint8_t *)pdm_buffer.first_half, (int16_t*)RecBuf, &PDM1_filter_handler);
			memcpy(output_cursor, RecBuf, PCM_OUT_SIZE * sizeof(int16_t));
			output_cursor = next_cursor;
		} else {
			// Block full, switch to next block
			pcm_full = pcm_current_block;
			Audio_Switch_Block();
			PDM_Filter((uint8_t *)pdm_buffer.first_half, (int16_t*)RecBuf, &PDM1_filter_handler);
			memcpy(output_cursor, RecBuf, PCM_OUT_SIZE * sizeof(int16_t));
			output_cursor += PCM_OUT_SIZE;
		}
	}

	transfer_state = TRANSFER_WAIT;
}


void Audio_Switch_Block(void) {
    block_ready = true;
    pcm_current_block =
            (pcm_current_block == pcm_output_block_ping) ?
                    pcm_output_block_pong : pcm_output_block_ping;
    output_cursor = &pcm_current_block[0];
    end_output_block = &pcm_current_block[FFT_SIZE];  // Change to 512, not 496
}




