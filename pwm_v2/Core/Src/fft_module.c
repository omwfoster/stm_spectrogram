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




void fft_test(uint16_t * sample_block){
    static arm_rfft_instance_q15 fft_instance;
    static q15_t output[FFT_SIZE*2]; //has to be twice FFT size

    arm_status status;

    status = arm_rfft_init_q15(&fft_instance, FFT_SIZE/*bin count*/, 0/*forward FFT*/, 1/*output bit order is normal*/);
    arm_rfft_q15(&fft_instance, (q15_t*)sample_block, output);

}
