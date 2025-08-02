/*
 * fft_module.h
 *
 *  Created on: Jun 10, 2025
 *      Author: oliverfoster
 */

#ifndef SRC_FFT_MODULE_H_
#define SRC_FFT_MODULE_H_

#define FFT_SIZE		128

#include "stdint.h"
#include "tone.h"
#include "arm_math.h"
void fft_test(int16_t * sample_block);
void fft_test_440_sample();
void convert_char(const audio_sample_t * s_16,q15_t * pcm, uint16_t num);
void window_init();





#endif /* SRC_FFT_MODULE_H_ */
