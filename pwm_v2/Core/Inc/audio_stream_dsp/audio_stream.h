// audio_stream.h
#ifndef AUDIO_STREAM_H
#define AUDIO_STREAM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "stdbool.h"
#include "arm_math.h"
#include "stm32f4xx_hal.h"

#define FFT_SIZE	512

// Command bytes from PC (matching device_commands.h)
#define CMD_START_RAW_STREAM    0x10
#define CMD_STOP_RAW_STREAM     0x11
#define CMD_SEND_SINGLE_RAW     0x12
#define CMD_START_FFT_STREAM    0x20
#define CMD_STOP_FFT_STREAM     0x21
#define CMD_SEND_SINGLE_FFT     0x22
#define CMD_GET_STATUS          0x50

// Response codes to PC
#define RESP_ACK                0x01
#define RESP_NACK               0x02
#define RESP_STATUS             0x03

// Streaming modes
typedef enum {
    STREAM_MODE_IDLE = 0,
    STREAM_MODE_RAW,
    STREAM_MODE_FFT,
    STREAM_MODE_FFT_DB
} stream_mode_t;

// Status structure
typedef struct {
    stream_mode_t mode;
    bool is_streaming;
    uint32_t sample_rate;
    uint16_t fft_size;
    uint16_t packets_sent;
    uint8_t decimation_factor;
} AudioStreamStatus_t;

// Global status (extern declaration)
extern AudioStreamStatus_t stream_status;

// Function prototypes
void AudioStream_Init(UART_HandleTypeDef *huart);
void AudioStream_ProcessCommand(void);
void AudioStream_SendRawSamples(q15_t *samples, uint16_t num_samples);
void AudioStream_SendFFTData(q15_t *fft_magnitude, uint16_t fft_size);
void AudioStream_SendFFTDataDB(q15_t *fft_magnitude_db, uint16_t fft_size);
void AudioStream_SendSpectrogram(q15_t *spectrogram_data, uint16_t num_frames, uint16_t fft_size);
void AudioStream_SendStatus(void);
void AudioStream_Task(void);
stream_mode_t AudioStream_GetMode(void);
bool AudioStream_IsStreaming(void);

#ifdef __cplusplus
}
#endif

#endif // AUDIO_STREAM_H
