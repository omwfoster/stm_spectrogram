/*
 * fft_module.h
 *
 *  Created on: Jun 10, 2025
 *      Author: oliverfoster
 */

#ifndef SRC_FFT_MODULE_H_
#define SRC_FFT_MODULE_H_

#define FFT_SIZE	128

#include "stdint.h"
#include "tone.h"
#include "arm_math.h"
void fft_test_raw(int16_t * sample_block);
void fft_postprocess(int16_t *sample_block);
void fft_test_440_sample();
void convert_char(const audio_sample_t * s_16,q15_t * pcm, uint16_t num);
void convert_magnitude_to_db_q15(q15_t *mag_input, q15_t *db_output, uint16_t length);
q15_t calculate_threshold_fast(q15_t *mag_bins, uint32_t size);
void adaptive_averaging(q15_t *current, q15_t *previous, q15_t *output, uint32_t size);
void adaptive_averaging_db(q15_t *current, q15_t *previous, q15_t *output, uint32_t size);
void window_init();
void dc_norm(int16_t * mag_block,uint32_t length);





#endif /* SRC_FFT_MODULE_H_ */
