/**
 ******************************************************************************
 * @file           : pdm_buffers.h
 * @brief          : PDM to PCM buffer management header
 ******************************************************************************
 * @attention
 *
 * Optimized PDM buffer configuration for STM32 with 1.5 MHz SPI clock
 * PDM data transferred in 16-bit packets via SPI
 *
 ******************************************************************************
 */

#ifndef PDM_BUFFERS_H
#define PDM_BUFFERS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "pdm2pcm_glo.h"
#include "stdbool.h"
#include "arm_math.h"
#include <string.h>

/* Exported defines ----------------------------------------------------------*/

#define DECIMATION_FACTOR_128

// Define values based on selected decimation factor
#ifdef DECIMATION_FACTOR_16
    #define DEC_FACTOR_REG  ((uint16_t)0x0005)
    #define DEC_FACTOR_VAL  16
#elif defined(DECIMATION_FACTOR_24)
    #define DEC_FACTOR_REG  ((uint16_t)0x0006)
    #define DEC_FACTOR_VAL  24
#elif defined(DECIMATION_FACTOR_32)
    #define DEC_FACTOR_REG  ((uint16_t)0x0007)
    #define DEC_FACTOR_VAL  32
#elif defined(DECIMATION_FACTOR_48)
    #define DEC_FACTOR_REG  ((uint16_t)0x0001)
    #define DEC_FACTOR_VAL  48
#elif defined(DECIMATION_FACTOR_64)
    #define DEC_FACTOR_REG  ((uint16_t)0x0002)
    #define DEC_FACTOR_VAL  64
#elif defined(DECIMATION_FACTOR_80)
    #define DEC_FACTOR_REG  ((uint16_t)0x0003)
    #define DEC_FACTOR_VAL  80
#elif defined(DECIMATION_FACTOR_128)
    #define DEC_FACTOR_REG  ((uint16_t)0x0004)
    #define DEC_FACTOR_VAL  128
#else
    #error "Must define exactly one DECIMATION_FACTOR_XX (16, 24, 32, 48, 64, 80, 128)"
#endif

/* Basic parameters ----------------------------------------------------------*/
#define NUMBER_SAMPLES_PCM      16      // PCM samples per processing block
#define BYTES_PER_SAMPLE        2       // 16-bit samples = 2 bytes
#define PCM_BITS                16      // PCM resolution

/* PDM to PCM conversion relationship macros ---------------------------------*/

// Calculate PDM bits required for one PCM sample
#define PDM_BITS_PER_PCM_SAMPLE(decimation_factor) (decimation_factor)

// Calculate total PDM bits needed for multiple PCM samples
#define PDM_BITS_TOTAL(pcm_samples, decimation_factor) \
    ((pcm_samples) * (decimation_factor))

/* PDM buffer calculations for 16-bit SPI transfers -------------------------*/

// PDM bits needed for PCM processing block
#define PDM_BITS_PER_PCM_BLOCK      (NUMBER_SAMPLES_PCM * DEC_FACTOR_VAL)     // 1280 PDM bits

// PDM transferred as 16-bit packets via SPI
#define PDM_BITS_PER_UINT16         16
#define PDM_BUFFER_LENGTH           (((PDM_BITS_PER_PCM_BLOCK + 15) / 16) *2)     // 160 uint16_t words
#define PDM_HALF_BUFFER_SIZE        (PDM_BUFFER_LENGTH / 2)                   // 80 uint16_t words

// Memory calculations
#define PDM_BUFFER_BYTES            (PDM_BUFFER_LENGTH * sizeof(uint16_t))     // 160 bytes per buffer
#define PDM_MEMORY_USAGE            (PDM_BUFFER_BYTES)                    // 320 bytes (double buffered)

/* Audio system parameters ---------------------------------------------------*/
#define REC_FREQ                    8000    // Target PCM sample rate (Hz)
#define SPI_CLOCK_FREQ              1500000 // SPI clock frequency (1.5 MHz)
#define PDM_EFFECTIVE_FREQ          SPI_CLOCK_FREQ  // PDM samples at SPI clock rate
#define ACTUAL_DECIMATION           (PDM_EFFECTIVE_FREQ / REC_FREQ)  // 187.5 (theoretical)
#define PCM_BLOCK_TIME_MS           (NUMBER_SAMPLES_PCM * 1000 / REC_FREQ)      // 2ms per block

/* Timing calculations based on 16-bit SPI transfers ------------------------*/
#define SPI_TRANSFERS_PER_BLOCK     PDM_BUFFER_LENGTH                         // 80 SPI transfers
#define SPI_TRANSFERS_PER_SECOND    (SPI_CLOCK_FREQ / 16)                     // 93,750 transfers/sec
#define SPI_TRANSFER_TIME_US        (16.0 * 1000000.0 / SPI_CLOCK_FREQ)       // 10.67 µs per 16-bit transfer

#define PDM_HALF_BUFFER_TIME_MS     (PDM_HALF_BUFFER_SIZE * SPI_TRANSFER_TIME_US / 1000) // 0.427 ms
#define PDM_FULL_BUFFER_TIME_MS     (PDM_BUFFER_LENGTH * SPI_TRANSFER_TIME_US / 1000)    // 0.853 ms

/* DMA transfer timing -------------------------------------------------------*/
#define DMA_HALF_TRANSFER_TIME_MS   PDM_HALF_BUFFER_TIME_MS                   // 0.427 ms
#define DMA_FULL_TRANSFER_TIME_MS   PDM_FULL_BUFFER_TIME_MS                   // 0.853 ms
#define PROCESSING_DEADLINE_MS      DMA_HALF_TRANSFER_TIME_MS                 // Must process within 0.427 ms

/* Performance analysis for 1.5 MHz operation -------------------------------*/
#define SAMPLES_PER_SECOND          REC_FREQ                                  // 8000 PCM samples/sec
#define PDM_SAMPLES_PER_SECOND      SPI_CLOCK_FREQ                            // 1,500,000 PDM samples/sec
#define DMA_INTERRUPTS_PER_SECOND   (1000.0 / PDM_HALF_BUFFER_TIME_MS)       // ~2340 interrupts/sec
#define PROCESSING_BUDGET_MS        DMA_HALF_TRANSFER_TIME_MS                 // 0.427 ms processing time
#define PROCESSING_LATENCY_MS       DMA_FULL_TRANSFER_TIME_MS                 // 0.853 ms total latency

/* Critical timing requirements ----------------------------------------------*/
#define MAX_FILTER_TIME_US          200     // PDM filtering should complete in <200µs
#define MAX_MEMCPY_TIME_US          50      // Memory copy should complete in <50µs
#define SAFETY_MARGIN_US            100     // Safety margin for jitter
#define AVAILABLE_PROCESSING_US     (PROCESSING_BUDGET_US - MAX_FILTER_TIME_US - MAX_MEMCPY_TIME_US - SAFETY_MARGIN_US)

/* FFT and processing buffers -----------------------------------------------*/
#define FFT_SIZE                    64     // Must be >= NUMBER_SAMPLES_PCM
#define PCM_DOUBLE_BUFFER_SIZE      (FFT_SIZE * 2)  // For continuous processing

/* Buffer alignment for optimal DMA performance -----------------------------*/
#define BUFFER_ALIGNMENT            32      // Cache line alignment for STM32H7/F7
#define ALIGN_BUFFER                __attribute__((aligned(BUFFER_ALIGNMENT)))

/* Memory usage summary ------------------------------------------------------*/
#define PDM_BUFFER_MEMORY           (sizeof(pdm_buffer_t))                    // 320 bytes
#define PCM_BUFFER_MEMORY           (sizeof(pcm_output_block_ping) + \
                                     sizeof(pcm_output_block_pong) + \
                                     sizeof(PCM_Block))                       // ~1540 bytes
#define FFT_BUFFER_MEMORY           (sizeof(fft_output) + sizeof(mag_bins_output)) // ~768 bytes
#define TOTAL_BUFFER_MEMORY         (PDM_BUFFER_MEMORY + PCM_BUFFER_MEMORY + FFT_BUFFER_MEMORY) // ~2628 bytes

/* Validation checks ---------------------------------------------------------*/
#if (PDM_BUFFER_LENGTH * 16 < PDM_BITS_PER_PCM_BLOCK)
    #error "PDM buffer too small - need more 16-bit words"
#endif

#if (PDM_HALF_BUFFER_SIZE != (PDM_BUFFER_LENGTH / 2))
    #error "PDM half buffer size mismatch"
#endif

#if (FFT_SIZE < NUMBER_SAMPLES_PCM)
    #error "FFT size must be at least NUMBER_SAMPLES_PCM"
#endif

#if (AVAILABLE_PROCESSING_MS <= 0)
    #warning "Processing budget may be too tight - consider optimizing timing requirements"
#endif

/* Exported types ------------------------------------------------------------*/

/* Transfer state enumeration  */
typedef enum {
    TRANSFER_WAIT = 0,
    TRANSFER_COMPLETE,
    TRANSFER_HALF,
    TRANSFER_ERROR
} transfer_state_t;

/* Audio processing status enumeration  */
typedef enum {
    AUDIO_OK = 0,
    AUDIO_ERROR_DMA,
    AUDIO_ERROR_SPI,
    AUDIO_ERROR_BUFFER_OVERFLOW,
    AUDIO_ERROR_PDM_FILTER,
    AUDIO_ERROR_FFT
} audio_status_t;

/* PDM buffer union - each uint16_t contains 16 PDM bits */
typedef volatile union {
    struct {
        uint16_t first_half[PDM_HALF_BUFFER_SIZE];   // 40 x 16-bit words = 640 PDM bits
        uint16_t last_half[PDM_HALF_BUFFER_SIZE];    // 40 x 16-bit words = 640 PDM bits
    };
    uint16_t PDM_In[PDM_BUFFER_LENGTH];             // 80 x 16-bit words = 1280 PDM bits total
} pdm_buffer_t;

/* Performance monitoring structure */
typedef struct {
    uint32_t pdm_transfers_completed;
    uint32_t pdm_half_transfers;
    uint32_t pcm_blocks_processed;
    uint32_t buffer_switches;
    uint32_t buffer_overruns;
    uint32_t filter_errors;
    uint32_t fft_processes;
    uint32_t memory_usage_bytes;
    uint32_t max_processing_time_us;
    uint32_t avg_processing_time_us;
    uint32_t missed_deadlines;
    uint32_t spi_overrun_errors;
    float    actual_sample_rate_hz;
    uint32_t dma_interrupts_per_second;
    float    cpu_load_percentage;
} high_freq_buffer_stats_t;

/* Exported variables --------------------------------------------------------*/

/* Main PDM buffer */
extern ALIGN_BUFFER pdm_buffer_t t_U_Pdm;

/* PCM processing buffers */
extern ALIGN_BUFFER volatile uint16_t PCM_Block[NUMBER_SAMPLES_PCM];
extern ALIGN_BUFFER volatile uint16_t pcm_output_block_ping[PCM_DOUBLE_BUFFER_SIZE];
extern ALIGN_BUFFER volatile uint16_t pcm_output_block_pong[PCM_DOUBLE_BUFFER_SIZE];

/* FFT working buffers */
extern ALIGN_BUFFER q15_t fft_output[FFT_SIZE * 2];
extern ALIGN_BUFFER q15_t mag_bins_output[FFT_SIZE / 2];

/* Buffer management variables */
extern volatile uint16_t *pcm_current_block;
extern volatile bool block_ready;
extern volatile uint16_t *output_cursor;
extern volatile uint16_t *end_output_block;
extern volatile uint16_t *pcm_full;
extern volatile transfer_state_t wTransferState;

/* Performance monitoring */
extern high_freq_buffer_stats_t g_buffer_stats;

/* External SPI handle */
extern SPI_HandleTypeDef hspi1;

/* External PDM filter */
extern PDM_Filter_Handler_t PDM1_filter_handler;

/* Exported functions --------------------------------------------------------*/

/* Buffer management functions */
void initialize_audio_buffers(void);
audio_status_t switch_block(void);
HAL_StatusTypeDef start_pdm_acquisition(void);

/* PDM processing functions */
audio_status_t process_pdm_high_freq(void);
audio_status_t handle_audio_processing(void);
void handle_audio_error(audio_status_t error);

/* Utility functions */
bool is_fft_ready(void);
void process_fft_and_send(void);

/* Performance monitoring */
void handle_deadline_miss(void);
void report_memory_usage(void);
void reset_buffer_stats(void);
void update_buffer_stats(void);



/* Performance validation macros */
#define VALIDATE_PROCESSING_TIME() do { \
    static uint32_t last_process_time = 0; \
    uint32_t current_time = HAL_GetTick(); \
    uint32_t processing_time = current_time - last_process_time; \
    if (processing_time > PROCESSING_DEADLINE_MS) { \
        handle_deadline_miss(); \
    } \
    last_process_time = current_time; \
} while(0)

#define VALIDATE_TIMING() do { \
    static uint32_t last_time = 0; \
    uint32_t current_time = HAL_GetTick(); \
    uint32_t delta = current_time - last_process_time; \
    if (delta > (PCM_BLOCK_TIME_MS + 1)) { \
        /* Timing violation detected */ \
    } \
    last_time = current_time; \
} while(0)

/* Debug information macros */
#define PDM_BUFFER_INFO() do { \
    printf("PDM Buffer Configuration:\n"); \
    printf("  - PDM bits per block: %d\n", PDM_BITS_PER_PCM_BLOCK); \
    printf("  - Buffer length: %d uint16_t words\n", PDM_BUFFER_LENGTH); \
    printf("  - Half buffer: %d uint16_t words\n", PDM_HALF_BUFFER_SIZE); \
    printf("  - Memory usage: %d bytes\n", PDM_MEMORY_USAGE); \
    printf("  - Processing deadline: %d µs\n", PROCESSING_DEADLINE_US); \
    printf("  - DMA interrupts/sec: %d\n", DMA_INTERRUPTS_PER_SECOND); \
} while(0)

#ifdef __cplusplus
}
#endif

#endif /* PDM_BUFFERS_H */
