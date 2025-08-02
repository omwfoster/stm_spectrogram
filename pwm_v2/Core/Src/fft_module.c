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

extern q15_t fft_output[]; // FFT_SIZE * 2
q15_t abs_output[FFT_SIZE*2];
extern q15_t mag_bins[FFT_SIZE];
extern q15_t mag_bins_output[FFT_SIZE];
q15_t mag_bins_previous[FFT_SIZE] = { [0 ... FFT_SIZE-1] = (q15_t)6533 };
q15_t mag_bins_new[FFT_SIZE];

q15_t windowed_samples_q15[FFT_SIZE];
q15_t hann_window[FFT_SIZE];

q15_t pcm_samples[FFT_SIZE];

/* Generate Hann window coefficients in Q15 format
 * This function should be called once during initialization
 * @param window: output array to store window coefficients
 * @param size: window size (FFT_SIZE)
 */
void generate_hann_window_q15(q15_t *window, uint16_t size) {
	for (uint16_t i = 0; i < size; i++) {
		// Hann window: w[n] = 0.5 * (1 - cos(2*pi*n/(N-1)))
		// Convert to Q15: multiply by 32767 and round
		float w = 0.5f * (1.0f - cosf(2.0f * 3.14159265f * i / (size - 1)));
		window[i] = (q15_t) (w * 32767.0f + 0.5f);
	}
}

void window_init() {

	generate_hann_window_q15(hann_window, FFT_SIZE);

}

/**
 * Apply windowing function to PCM samples
 * @param pcm_samples: input PCM samples in Q15 format
 * @param windowed_samples: output windowed samples in Q15 format
 * @param window: window coefficients in Q15 format
 * @param size: number of samples (FFT_SIZE)
 */
void apply_window_q15(const q15_t *pcm_samples, q15_t *windowed_samples,
		const q15_t *window, uint16_t size) {
	arm_mult_q15(pcm_samples, window, windowed_samples, size);
}

void dynamic_range(const audio_sample_t *s_16, q15_t *pcm, uint16_t num)
{

}



void fft_test(int16_t *sample_block) {
	static arm_rfft_instance_q15 fft_instance;

	arm_status status;

	convert_char(sample_block, pcm_samples, (FFT_SIZE * 2));
	apply_window_q15(pcm_samples, windowed_samples_q15, hann_window, FFT_SIZE);
	status = arm_rfft_init_q15(&fft_instance, FFT_SIZE/*bin count*/,
			0/*forward FFT*/, 1/*output bit order is normal*/);
	arm_rfft_q15(&fft_instance, (q15_t*) windowed_samples_q15, fft_output);
	arm_cmplx_mag_q15(fft_output, mag_bins, FFT_SIZE);
	arm_scale_q15(mag_bins,(q15_t)26132 , 0 , mag_bins_new , FFT_SIZE);
	arm_add_q15(mag_bins_new , mag_bins_previous , mag_bins_output , FFT_SIZE);
	arm_scale_q15(mag_bins,(q15_t)6533 , 0 , mag_bins_previous , FFT_SIZE);



}

void convert_char(const audio_sample_t *s_16, q15_t *pcm, uint16_t num) {
// Convert your hex array to 16-bit samples
	for (int i = 0; i < num; i += 2) {
		int16_t sample = (int16_t) ((uint8_t) s_16[i]
				| ((uint8_t) s_16[i + 1] << 8));
		pcm[i / 2] = (q15_t) sample;
	}

}

void fft_test_440_sample() {
	static arm_rfft_instance_q15 fft_instance;

	convert_char(test_440, pcm_samples, (FFT_SIZE * 2));
	arm_status status;

	apply_window_q15(pcm_samples, windowed_samples_q15, hann_window, FFT_SIZE);

	status = arm_rfft_init_q15(&fft_instance, FFT_SIZE/*bin count*/,
			0/*forward FFT*/, 1/*output bit order is normal*/);
	arm_rfft_q15(&fft_instance, (q15_t*) windowed_samples_q15, fft_output);
	arm_cmplx_mag_q15(fft_output, mag_bins_output, FFT_SIZE);

}

