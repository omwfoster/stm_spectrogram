/*
 * fft_module.c
 *
 *  Created on: Jun 10, 2025
 *      Author: oliverfoster
 */

#include "fft_module.h"
#include <arm_math.h>



uint16_t pcm_output_block_ping[FFT_SIZE];
uint16_t pcm_output_block_pong[FFT_SIZE];




void fft_test(void){
    static arm_rfft_instance_q15 fft_instance;
    static q15_t output[FFT_SIZE*2]; //has to be twice FFT size





    arm_status status;

    status = arm_rfft_init_q15(&fft_instance, 256/*bin count*/, 0/*forward FFT*/, 1/*output bit order is normal*/);


    for (uint32_t i = 0; i < sizeof(pcm_output_block_ping)/sizeof(pcm_output_block_ping[0]); i++){


            arm_rfft_q15(&fft_instance, (q15_t*)pcm_output_block_ping, output);
            arm_abs_q15(output, output, FFT_SIZE);
            for (uint32_t j = 0; j < FFT_SIZE; j++){

            }

        }
}
