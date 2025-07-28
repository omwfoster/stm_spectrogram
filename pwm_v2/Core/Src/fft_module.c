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

void apply_hanning_window_q15(q15_t *input, q15_t *output, uint32_t length) {
	uint32_t i;
	q15_t window_coeff;
	q31_t temp;

	for (i = 0; i < length; i++) {
		// Calculate Hanning window coefficient
		// Formula: 0.5 * (1 - cos(2*pi*i/(N-1)))

		// Calculate 2*pi*i/(N-1) in Q15 format
		// 2*pi in Q15 = 205887 (approximately)
		q31_t angle = (205887L * i) / (length - 1);

		// Get cosine value (CMSIS returns Q15)
		q15_t cos_val = arm_cos_q15((q15_t) angle);

		// Calculate window coefficient: 0.5 * (1 - cos_val)
		// 0.5 in Q15 = 16384, 1.0 in Q15 = 32767
		window_coeff = 16384 - (cos_val >> 1);

		// Apply window: multiply input by window coefficient
		temp = ((q31_t) input[i] * window_coeff) >> 15;

		// Clamp to Q15 range and store result
		if (temp > 32767)
			temp = 32767;
		if (temp < -32768)
			temp = -32768;
		output[i] = (q15_t) temp;
	}
}

convert_char() {
// Convert your hex array to 16-bit samples
	for (int i = 0; i < 256; i += 2) {
		int16_t sample = (int16_t) ((uint8_t) test2k[i]
				| ((uint8_t) test_440[i + 1] << 8));
		pcm_samples[i / 2] = (q15_t)sample;
	}

}

void fft_test_440_sample() {
	static arm_rfft_instance_q15 fft_instance;


	convert_char();
	arm_status status;

	status = arm_rfft_init_q15(&fft_instance, FFT_SIZE/*bin count*/,
			0/*forward FFT*/, 1/*output bit order is normal*/);
	arm_rfft_q15(&fft_instance, (q15_t*) pcm_samples, fft_output);
	arm_cmplx_mag_q15(fft_output, mag_bins, FFT_SIZE);

}

