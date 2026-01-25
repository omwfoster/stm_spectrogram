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
pdm_buffer_t pdm_buffer = { .guard1 = 0xDEADBEEF, .guard2 = 0xDEADBEEF };
pcm_buffer_t pcm_buffer = { .guard1 = 0xDEADBEEF, .guard2 = 0xDEADBEEF,
		.initialised = false };
int16_t RecBuf[PCM_OUT_SIZE];

pcm_buffer_t pcm_output_block;

/* USER CODE BEGIN 1 */
/* USER CODE END 1 */

/* PDM2PCM init function */
void MX_PDM2PCM_Init(void) {
	/* USER CODE BEGIN 2 */
	/* USER CODE END 2 */

	/**
	 */
	PDM1_filter_handler.bit_order = PDM_FILTER_BIT_ORDER_MSB;
	PDM1_filter_handler.endianness = PDM_FILTER_ENDIANNESS_BE;
	PDM1_filter_handler.high_pass_tap = 2122358088;
	PDM1_filter_handler.in_ptr_channels = 1;
	PDM1_filter_handler.out_ptr_channels = 1;
	PDM_Filter_Init(&PDM1_filter_handler);

	PDM1_filter_config.decimation_factor = DECIMATION_FACTOR;
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
uint32_t Audio_Process_PDM(pcm_buffer_t *pcm, pdm_buffer_t * pdm_buffer ) {
	uint32_t index = 0;
	uint32_t ret = 0;
	int16_t *next_cursor = NULL;

	if (transfer_state == TRANSFER_COMPLETE) {

//		for (uint16_t i = 0; i < PDM_BUFFER_SIZE/2; i++) {
//					((uint16_t *)pdm_buffer.last_half)[i] = HTONS(((uint16_t *)pdm_buffer.last_half)[i]);
//		}
		next_cursor = pcm->cursor + PCM_OUT_SIZE;

		if (next_cursor <= pcm->end_output_block) {
			// Still room in current block

			ret = PDM_Filter((uint8_t*) pdm_buffer->last_half, (int16_t*) RecBuf,
					&PDM1_filter_handler);
			memcpy(pcm->cursor, RecBuf, PCM_OUT_SIZE * sizeof(int16_t));
			pcm->cursor = next_cursor;
		} else {
			// Block full, switch to next block
			pcm->pcm_full = pcm->pcm_current_block;
			Audio_Switch_Block(pcm);
			ret = PDM_Filter((uint8_t*) pdm_buffer->last_half, (int16_t*) RecBuf,
					&PDM1_filter_handler);
			memcpy(pcm->cursor, RecBuf, PCM_OUT_SIZE * sizeof(int16_t));
			pcm->cursor += PCM_OUT_SIZE;
		}
	} else if (transfer_state == TRANSFER_HALF) {

		if (next_cursor <= pcm->end_output_block) {
			// Still room in current block
			ret = PDM_Filter((uint8_t*) pdm_buffer->first_half,
					(int16_t*) RecBuf, &PDM1_filter_handler);
			memcpy(pcm->cursor, RecBuf, PCM_OUT_SIZE * sizeof(int16_t));
			pcm->cursor = next_cursor;
		} else {
			// Block full, switch to next block
			pcm->pcm_full = pcm->pcm_current_block;
			Audio_Switch_Block(pcm);
			ret = PDM_Filter((uint8_t*) pdm_buffer->first_half,
					(int16_t*) RecBuf, &PDM1_filter_handler);
			memcpy(pcm->cursor, RecBuf, PCM_OUT_SIZE * sizeof(int16_t));
			pcm->cursor += PCM_OUT_SIZE;
		}
	}

	transfer_state = TRANSFER_WAIT;
	return ret;
}

void Pcm_Initialise(pcm_buffer_t *pcm) {
	pcm->pcm_full = NULL;
	pcm->pcm_current_block = &pcm->ping[0];
	pcm->initialised = true;
}

void Audio_Switch_Block(pcm_buffer_t *pcm) {

	block_ready = true;
	pcm->pcm_current_block =
			(pcm->pcm_current_block == &pcm->ping[0]) ? &pcm->pong[0] : &pcm->ping[0];
	pcm->cursor = &pcm->pcm_current_block[0];

	pcm->end_output_block =
			(pcm->pcm_current_block == &pcm->ping[0]) ? (int16_t *)&pcm->guard1 : (int16_t *)pcm->guard2;
}

