/*
 * audio_stream.h
 *
 *  Created on: Oct 12, 2025
 *      Author: oliverfoster
 */



// audio_streaming.h
#ifndef AUDIO_STREAMING_H
#define AUDIO_STREAMING_H

#include "ai_logging.h"
#include "stdint.h"
#include "stdbool.h"
#include "stm32f4xx_hal.h"
#include "arm_math.h"

// Command bytes from PC
#define CMD_START_RAW_STREAM    0x10
#define CMD_STOP_RAW_STREAM     0x11
#define CMD_START_FFT_STREAM    0x20
#define CMD_STOP_FFT_STREAM     0x21
#define CMD_SEND_SINGLE_RAW     0x30
#define CMD_SEND_SINGLE_FFT     0x31
#define CMD_SET_SAMPLE_RATE     0x40
#define CMD_GET_STATUS          0x50

// Response codes to PC
#define RESP_ACK                0x01
#define RESP_NACK               0x02
#define RESP_STATUS             0x03

// Streaming modes
typedef enum {
    STREAM_MODE_IDLE = 0,
    STREAM_MODE_RAW,
    STREAM_MODE_FFT
} StreamMode_t;

// Status structure
typedef struct {
    StreamMode_t mode;
    bool is_streaming;
    uint32_t sample_rate;
    uint16_t fft_size;
    uint16_t packets_sent;
    uint8_t decimation_factor;
} AudioStreamStatus_t;

// Function prototypes
void AudioStream_Init(UART_HandleTypeDef *huart);
void AudioStream_ProcessCommand(void);
void AudioStream_SendRawSamples(q15_t *samples, uint16_t num_samples);
void AudioStream_SendFFTData(q15_t *fft_magnitude, uint16_t fft_size);
void AudioStream_SendStatus(void);
void AudioStream_Task(void);

#endif // AUDIO_STREAMING_H




