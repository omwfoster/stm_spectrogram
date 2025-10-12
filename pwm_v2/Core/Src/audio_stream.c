/*
 * audio_stream.c
 *
 *  Created on: Oct 12, 2025
 *      Author: oliverfoster
 */


// audio_streaming.c
#include "audio_stream.h"
#include "ai_logging.h"
#include "string.h"
#include "stdio.h"

// AI Logging device
static ai_logging_device_t ai_device;
static AudioStreamStatus_t stream_status;

// Buffers for AI Logging
#define AI_SEND_BUFFER_SIZE 256
#define AI_RECEIVE_BUFFER_SIZE 128
static uint8_t ai_send_buffer[AI_SEND_BUFFER_SIZE];
static uint8_t ai_receive_buffer[AI_RECEIVE_BUFFER_SIZE];

// UART handle
static UART_HandleTypeDef *uart_handle;

// Forward declarations
static uint32_t uart_send(uint8_t *data, uint32_t size);
static uint32_t uart_receive(uint8_t *data, uint32_t size);
static void process_command_packet(ai_logging_packet_t *packet);
static void send_ack(uint8_t cmd);
static void send_nack(uint8_t cmd);

/**
 * Initialize audio streaming module
 */
void AudioStream_Init(UART_HandleTypeDef *huart) {
    uart_handle = huart;

    // Initialize AI Logging
    ai_logging_init(&ai_device);
    ai_logging_init_send(&ai_device, uart_send, ai_send_buffer, AI_SEND_BUFFER_SIZE);
    ai_logging_init_receive(&ai_device, uart_receive, 1, ai_receive_buffer, AI_RECEIVE_BUFFER_SIZE);

    // Initialize status
    stream_status.mode = STREAM_MODE_IDLE;
    stream_status.is_streaming = false;
    stream_status.sample_rate = 11718;
    stream_status.fft_size = 512;
    stream_status.packets_sent = 0;
    stream_status.decimation_factor = 16;

    // Send startup message using packet structure
    ai_logging_packet_t startup_packet;
    ai_logging_clear_packet(&startup_packet);

    char msg[] = "Audio Streaming Ready";
    startup_packet.payload_type = AI_STR;
    startup_packet.payload = (uint8_t*)msg;
    startup_packet.payload_size = strlen(msg);

    ai_logging_send_packet(&ai_device, &startup_packet);
}

/**
 * UART send function for AI Logging
 */
static uint32_t uart_send(uint8_t *data, uint32_t size) {
    if (HAL_UART_Transmit(uart_handle, data, size, 100) == HAL_OK) {
        return size;
    }
    return 0;
}

/**
 * UART receive function for AI Logging
 */
static uint32_t uart_receive(uint8_t *data, uint32_t size) {
    if (HAL_UART_Receive(uart_handle, data, size, 10) == HAL_OK) {
        return size;
    }
    return 0;
}

/**
 * Process received commands from PC
 */
void AudioStream_ProcessCommand(void) {
    ai_logging_packet_t packet;
    ai_logging_clear_packet(&packet);

    int result = ai_logging_check_for_received_packet(&ai_device, &packet);

    if (result == AI_PACKET_OK) {
        if (packet.payload_type == AI_COMMAND && packet.payload_size > 0) {
            process_command_packet(&packet);
        }
        ai_logging_prepare_next_packet(&ai_device);
    }
}

/**
 * Process command packet
 */
static void process_command_packet(ai_logging_packet_t *packet) {
    if (packet->payload_size < 1) {
        return;
    }

    uint8_t cmd = packet->payload[0];

    switch (cmd) {
        case CMD_START_RAW_STREAM:
            stream_status.mode = STREAM_MODE_RAW;
            stream_status.is_streaming = true;
            stream_status.packets_sent = 0;
            send_ack(cmd);
            break;

        case CMD_STOP_RAW_STREAM:
            if (stream_status.mode == STREAM_MODE_RAW) {
                stream_status.is_streaming = false;
                stream_status.mode = STREAM_MODE_IDLE;
                send_ack(cmd);
            } else {
                send_nack(cmd);
            }
            break;

        case CMD_START_FFT_STREAM:
            stream_status.mode = STREAM_MODE_FFT;
            stream_status.is_streaming = true;
            stream_status.packets_sent = 0;
            send_ack(cmd);
            break;

        case CMD_STOP_FFT_STREAM:
            if (stream_status.mode == STREAM_MODE_FFT) {
                stream_status.is_streaming = false;
                stream_status.mode = STREAM_MODE_IDLE;
                send_ack(cmd);
            } else {
                send_nack(cmd);
            }
            break;

        case CMD_SEND_SINGLE_RAW:
            stream_status.mode = STREAM_MODE_RAW;
            stream_status.is_streaming = false;  // Single shot
            send_ack(cmd);
            break;

        case CMD_SEND_SINGLE_FFT:
            stream_status.mode = STREAM_MODE_FFT;
            stream_status.is_streaming = false;  // Single shot
            send_ack(cmd);
            break;

        case CMD_GET_STATUS:
            AudioStream_SendStatus();
            break;

        default:
            send_nack(cmd);
            break;
    }
}

/**
 * Send ACK response using packet structure
 */
static void send_ack(uint8_t cmd) {
    ai_logging_packet_t response_packet;
    ai_logging_clear_packet(&response_packet);

    static uint8_t response[2];
    response[0] = RESP_ACK;
    response[1] = cmd;

    response_packet.payload_type = AI_UINT8;
    response_packet.payload = response;
    response_packet.payload_size = 2;

    // Set shape
    ai_logging_create_shape_1d(&response_packet.shape, 2);

    ai_logging_send_packet(&ai_device, &response_packet);
}

/**
 * Send NACK response using packet structure
 */
static void send_nack(uint8_t cmd) {
    ai_logging_packet_t response_packet;
    ai_logging_clear_packet(&response_packet);

    static uint8_t response[2];
    response[0] = RESP_NACK;
    response[1] = cmd;

    response_packet.payload_type = AI_UINT8;
    response_packet.payload = response;
    response_packet.payload_size = 2;

    // Set shape
    ai_logging_create_shape_1d(&response_packet.shape, 2);

    ai_logging_send_packet(&ai_device, &response_packet);
}

/**
 * Send raw audio samples to PC using packet structure
 */
void AudioStream_SendRawSamples(q15_t *samples, uint16_t num_samples) {
    if (!stream_status.is_streaming && stream_status.mode != STREAM_MODE_RAW) {
        return;
    }

    // Create packet
    ai_logging_packet_t data_packet;
    ai_logging_clear_packet(&data_packet);

    // Set message to identify data type
    static char msg[] = "RAW";
    data_packet.message = (uint8_t*)msg;
    data_packet.message_size = strlen(msg);

    // Set payload
    data_packet.payload_type = AI_INT16;
    data_packet.payload = (uint8_t*)samples;
    data_packet.payload_size = num_samples * sizeof(q15_t);

    // Set shape (1D array)
    ai_logging_create_shape_1d(&data_packet.shape, num_samples);

    // Set timestamp (optional)
    data_packet.timestamp = HAL_GetTick();

    // Send packet
    ai_logging_send_packet(&ai_device, &data_packet);

    stream_status.packets_sent++;

    // If single shot, reset mode
    if (!stream_status.is_streaming) {
        stream_status.mode = STREAM_MODE_IDLE;
    }
}

/**
 * Send FFT magnitude data to PC using packet structure
 */
void AudioStream_SendFFTData(q15_t *fft_magnitude, uint16_t fft_size) {
    if (!stream_status.is_streaming && stream_status.mode != STREAM_MODE_FFT) {
        return;
    }

    // Create packet
    ai_logging_packet_t data_packet;
    ai_logging_clear_packet(&data_packet);

    // Set message to identify data type
    static char msg[] = "FFT";
    data_packet.message = (uint8_t*)msg;
    data_packet.message_size = strlen(msg);

    // Set payload
    data_packet.payload_type = AI_INT16;
    data_packet.payload = (uint8_t*)fft_magnitude;
    data_packet.payload_size = fft_size * sizeof(q15_t);

    // Set shape (1D array)
    ai_logging_create_shape_1d(&data_packet.shape, fft_size);

    // Set timestamp (optional)
    data_packet.timestamp = HAL_GetTick();

    // Send packet
    ai_logging_send_packet(&ai_device, &data_packet);

    stream_status.packets_sent++;

    // If single shot, reset mode
    if (!stream_status.is_streaming) {
        stream_status.mode = STREAM_MODE_IDLE;
    }
}

/**
 * Send status information to PC using packet structure
 */
void AudioStream_SendStatus(void) {
    ai_logging_packet_t status_packet;
    ai_logging_clear_packet(&status_packet);

    // Pack status into bytes
    static uint8_t status_data[12];
    status_data[0] = RESP_STATUS;
    status_data[1] = (uint8_t)stream_status.mode;
    status_data[2] = stream_status.is_streaming ? 1 : 0;

    // Sample rate (4 bytes)
    memcpy(&status_data[3], &stream_status.sample_rate, 4);

    // FFT size (2 bytes)
    memcpy(&status_data[7], &stream_status.fft_size, 2);

    // Packets sent (2 bytes)
    memcpy(&status_data[9], &stream_status.packets_sent, 2);

    // Decimation factor
    status_data[11] = stream_status.decimation_factor;

    // Set up packet
    status_packet.payload_type = AI_UINT8;
    status_packet.payload = status_data;
    status_packet.payload_size = 12;

    // Set shape
    ai_logging_create_shape_1d(&status_packet.shape, 12);

    // Set message
    static char msg[] = "STATUS";
    status_packet.message = (uint8_t*)msg;
    status_packet.message_size = strlen(msg);

    // Send packet
    ai_logging_send_packet(&ai_device, &status_packet);
}

/**
 * Send FFT data with dB conversion (Q15 format)
 */
void AudioStream_SendFFTDataDB(q15_t *fft_magnitude_db, uint16_t fft_size) {
    if (!stream_status.is_streaming && stream_status.mode != STREAM_MODE_FFT) {
        return;
    }

    // Create packet
    ai_logging_packet_t data_packet;
    ai_logging_clear_packet(&data_packet);

    // Set message to identify data type
    static char msg[] = "FFT_DB";
    data_packet.message = (uint8_t*)msg;
    data_packet.message_size = strlen(msg);

    // Set payload
    data_packet.payload_type = AI_INT16;
    data_packet.payload = (uint8_t*)fft_magnitude_db;
    data_packet.payload_size = fft_size * sizeof(q15_t);

    // Set shape (1D array)
    ai_logging_create_shape_1d(&data_packet.shape, fft_size);

    // Set timestamp
    data_packet.timestamp = HAL_GetTick();

    // Send packet
    ai_logging_send_packet(&ai_device, &data_packet);

    stream_status.packets_sent++;

    if (!stream_status.is_streaming) {
        stream_status.mode = STREAM_MODE_IDLE;
    }
}

/**
 * Send 2D FFT spectrogram data (time vs frequency)
 */
void AudioStream_SendSpectrogram(q15_t *spectrogram_data, uint16_t num_frames, uint16_t fft_size) {
    // Create packet
    ai_logging_packet_t data_packet;
    ai_logging_clear_packet(&data_packet);

    // Set message
    static char msg[] = "SPECTROGRAM";
    data_packet.message = (uint8_t*)msg;
    data_packet.message_size = strlen(msg);

    // Set payload
    data_packet.payload_type = AI_INT16;
    data_packet.payload = (uint8_t*)spectrogram_data;
    data_packet.payload_size = num_frames * fft_size * sizeof(q15_t);

    // Set 2D shape (time x frequency)
    ai_logging_create_shape_2d(&data_packet.shape, num_frames, fft_size);

    // Set timestamp
    data_packet.timestamp = HAL_GetTick();

    // Send packet
    ai_logging_send_packet(&ai_device, &data_packet);
}

/**
 * Periodic task to check for commands
 */
void AudioStream_Task(void) {
    AudioStream_ProcessCommand();
}

/**
 * Get current streaming mode
 */
StreamMode_t AudioStream_GetMode(void) {
    return stream_status.mode;
}

/**
 * Check if currently streaming
 */
bool AudioStream_IsStreaming(void) {
    return stream_status.is_streaming;
}
