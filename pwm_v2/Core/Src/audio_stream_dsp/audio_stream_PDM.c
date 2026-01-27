
#include <audio_stream_dsp/audio_stream_PDM.h>
#include <audio_stream_dsp/audio_stream.h>



/* Global variables ---------------------------------------------------------*/
PDM_Filter_Handler_t PDM1_filter_handler;
PDM_Filter_Config_t PDM1_filter_config;

#define SAMPLES_NUMBER  (uint16_t)((AUDIO_IN_SAMPLING_FREQUENCY / 1000U) * 1U);
#define HTONS(A)  ((((uint16_t)(A) & 0xff00) >> 8) | (((uint16_t)(A) & 0x00ff) << 8))

//int16_t PDM_Buffer[PDM_BUFFER_SIZE];
pdm_buffer_t pdm_buffer = { .guard1 = 0xDEADBEEF, .guard2 = 0xDEADBEEF };
pcm_buffer_t pcm_buffer = { .guard1 = 0xDEADBEEF, .guard2 = 0xDEADBEEF,
		.initialised = false };
int16_t RecBuf[PCM_OUT_SIZE];

pcm_buffer_t pcm_output_block;



/* PDM2PCM init function */
void MX_PDM2PCM_Init(void) {

	PDM1_filter_handler.bit_order = PDM_FILTER_BIT_ORDER_MSB;
	PDM1_filter_handler.endianness = PDM_FILTER_ENDIANNESS_BE;
	PDM1_filter_handler.high_pass_tap = 2122358088;
	PDM1_filter_handler.in_ptr_channels = 1;
	PDM1_filter_handler.out_ptr_channels = 1;
	PDM_Filter_Init(&PDM1_filter_handler);

	PDM1_filter_config.decimation_factor = DECIMATION_FACTOR;
	PDM1_filter_config.output_samples_number = 16;
	PDM1_filter_config.mic_gain = 32;
	PDM_Filter_setConfig(&PDM1_filter_handler, &PDM1_filter_config);

}

uint32_t Audio_Process_PDM(pcm_buffer_t *pcm_buffer, pdm_buffer_t *pdm_buffer) {


	uint32_t ret = 0;
	int16_t *next_cursor = pcm_buffer->cursor + PCM_OUT_SIZE;

	if (transfer_state == TRANSFER_COMPLETE) {
		if (next_cursor < pcm_buffer->end_output_block) {
			// Still room in current block
			ret = PDM_Filter((uint8_t*) pdm_buffer->last_half,
					(int16_t*) RecBuf, &PDM1_filter_handler);
			memcpy(pcm_buffer->cursor, RecBuf, PCM_OUT_SIZE * sizeof(int16_t));
			pcm_buffer->cursor = next_cursor;
		} else {
			// Block full, switch to next block
			pcm_buffer->pcm_full = pcm_buffer->pcm_current_block;
			Audio_Switch_Block(pcm_buffer);
			ret = PDM_Filter((uint8_t*) pdm_buffer->last_half,
					(int16_t*) RecBuf, &PDM1_filter_handler);
			memcpy(pcm_buffer->cursor, RecBuf, PCM_OUT_SIZE * sizeof(int16_t));
			pcm_buffer->cursor += PCM_OUT_SIZE;
		}
	} else if (transfer_state == TRANSFER_HALF) {
		if (next_cursor < pcm_buffer->end_output_block) {
			// Still room in current block
			ret = PDM_Filter((uint8_t*) pdm_buffer->first_half,
					(int16_t*) RecBuf, &PDM1_filter_handler);
			memcpy(pcm_buffer->cursor, RecBuf, PCM_OUT_SIZE * sizeof(int16_t));
			pcm_buffer->cursor = next_cursor;
		} else {
			// Block full, switch to next block
			pcm_buffer->pcm_full = pcm_buffer->pcm_current_block;
			Audio_Switch_Block(pcm_buffer);
			ret = PDM_Filter((uint8_t*) pdm_buffer->first_half,
					(int16_t*) RecBuf, &PDM1_filter_handler);
			memcpy(pcm_buffer->cursor, RecBuf, PCM_OUT_SIZE * sizeof(int16_t));
			pcm_buffer->cursor += PCM_OUT_SIZE;
		}
	}

    Check_PDM_Guards(pdm_buffer);
    Check_PCM_Guards(pcm_buffer);

	transfer_state = TRANSFER_WAIT;
	return ret;


}

void Pcm_Initialise(pcm_buffer_t *pcm_buffer) {

	pcm_buffer->pcm_full = NULL;
	pcm_buffer->pcm_current_block = pcm_buffer->ping;
	pcm_buffer->cursor = pcm_buffer->ping;
	pcm_buffer->end_output_block = &pcm_buffer->pcm_current_block[FFT_SIZE-1] ;
	pcm_buffer->initialised = true;

}

void Audio_Switch_Block(pcm_buffer_t *pcm_buffer) {

	block_ready = true;

	if (pcm_buffer->pcm_current_block == &pcm_buffer->ping[0]) {
		pcm_buffer->pcm_current_block = &pcm_buffer->pong[0];
		pcm_buffer->end_output_block = &pcm_buffer->pcm_current_block[FFT_SIZE-1];
	} else {
		pcm_buffer->pcm_current_block = &pcm_buffer->ping[0];
		pcm_buffer->end_output_block = &pcm_buffer->pcm_current_block[FFT_SIZE-1];
	}

	pcm_buffer->cursor = pcm_buffer->pcm_current_block;

}



// Add to pdm2pcm.c
void Check_PDM_Guards(pdm_buffer_t *buf) {

	if (buf->guard1 != 0xDEADBEEF) {
        Error_Handler(); // first_half overflow!
    }
    if (buf->guard2 != 0xDEADBEEF) {
        Error_Handler(); // last_half overflow!
    }

}

void Check_PCM_Guards(pcm_buffer_t *buf) {

	if (buf->guard1 != 0xDEADBEEF) {
        Error_Handler(); // ping overflow!
    }
    if (buf->guard2 != 0xDEADBEEF) {
        Error_Handler(); // pong overflow!
    }

}
