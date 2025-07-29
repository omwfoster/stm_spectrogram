/*
 * fft_module.c
 *
 *  Created on: Jun 10, 2025
 *      Author: oliverfoster
 */

#include "fft_module.h"
#include <arm_math.h>

uint16_t pcm_output_block_ping[FFT_SIZE * 2];
uint16_t pcm_output_block_pong[FFT_SIZE * 2];
uint16_t *pcm_current_block = pcm_output_block_ping;

extern q15_t fft_output[];
extern q15_t mag_bins[FFT_SIZE];

q15_t windowed_samples_q15[FFT_SIZE];

q15_t pcm_samples[FFT_SIZE];

void fft_test(int16_t *sample_block) {
	static arm_rfft_instance_q15 fft_instance;

	arm_status status;
	status = arm_rfft_init_q15(&fft_instance, FFT_SIZE/*bin count*/,
			0/*forward FFT*/, 1/*output bit order is normal*/);
	arm_rfft_q15(&fft_instance, (q15_t*) sample_block, fft_output);
	arm_cmplx_mag_q15(fft_output, mag_bins, FFT_SIZE);

}



void convert_char(const audio_sample_t * s_16,q15_t * pcm, uint16_t num) {
// Convert your hex array to 16-bit samples
	for (int i = 0; i < num; i += 2) {
		int16_t sample = (int16_t) ((uint8_t) s_16[i]
				| ((uint8_t) s_16[i + 1] << 8));
		pcm[i / 2] = (q15_t)sample;
	}


}

void fft_test_440_sample() {
	static arm_rfft_instance_q15 fft_instance;


	convert_char(test2k, pcm_samples , (FFT_SIZE * 2));
	arm_status status;

	status = arm_rfft_init_q15(&fft_instance, FFT_SIZE/*bin count*/,
			0/*forward FFT*/, 1/*output bit order is normal*/);
	arm_rfft_q15(&fft_instance, (q15_t*) pcm_samples, fft_output);
	arm_cmplx_mag_q15(fft_output, mag_bins, FFT_SIZE);

}

