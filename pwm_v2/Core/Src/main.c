/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file : main.c
* @brief : Main program body - PDM Audio Processing with FFT
******************************************************************************
* @attention
*
* Copyright (C) 2025 Oliver Foster
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <https://www.gnu.org/licenses/>.
*
******************************************************************************
*/
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdbool.h"
#include "ai_logging.h"
#include "arm_math.h"
#include <audio_stream_dsp/audio_stream.h>
#include <audio_stream_dsp/audio_stream_fft.h>
#include <audio_stream_dsp/audio_stream_PDM.h>
#include <audio_stream_dsp/audio_stream_spi.h>
//#include "device_commands.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// Buffer size definitions


#define SEND_BUFFER_MAX_SIZE    2048          // AI Logging send buffer
#define REC_FREQ                8000          // Recording frequency



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
TIM_HandleTypeDef htim1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// Audio buffers (external dependencies from fft_module.c)
extern uint16_t pcm_output_block_ping[FFT_SIZE];
extern uint16_t pcm_output_block_pong[FFT_SIZE];
extern int16_t pcm_q15[FFT_SIZE];
extern uint16_t *pcm_current_block;
extern PDM_Filter_Handler_t PDM1_filter_handler;
extern AudioStreamStatus_t stream_status;
extern uint16_t PDM_Buffer[PDM_BUFFER_SIZE];

// Local audio processing buffers
q15_t fft_output[FFT_SIZE * 2];        // Complex FFT output
q15_t mag_bins_output[FFT_SIZE];       // Magnitude spectrum
q15_t db_bins_output[FFT_SIZE];        // dB spectrum
q15_t mag_bins[FFT_SIZE];              // Magnitude bins

// PDM/PCM processing buffers
extern uint16_t RecBuf[PCM_OUT_SIZE];

static uint16_t pcm_deinterleaved[FFT_SIZE];
extern pdm_buffer_t pdm_buffer;

// Buffer management
uint16_t *output_cursor = NULL;
uint16_t *end_output_block = NULL;
volatile uint16_t *pcm_full = NULL;
bool block_ready = false;

// Transfer state
volatile transfer_state_t transfer_state = TRANSFER_WAIT;

// AI Logging
static ai_logging_device_t ai_device;
static uint8_t send_buffer[SEND_BUFFER_MAX_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_CRC_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */




/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	// MX_TIM1_Init();
	MX_SPI1_Init();
	MX_CRC_Init();
	MX_PDM2PCM_Init();
	MX_USART2_UART_Init();

	/* USER CODE BEGIN 2 */

	// Initialize FFT windowing
	FFT_Window_Init();

	// Initialize audio streaming
	AudioStream_Init(&huart2);

	// Initialize buffer pointers
	output_cursor = pcm_output_block_ping;
	end_output_block = &pcm_output_block_ping[(FFT_SIZE * 2) - PCM_OUT_SIZE];

	// Start PDM reception via SPI DMA
	HAL_SPI_Receive_DMA(&hspi1, (uint8_t*) &pdm_buffer, PDM_BUFFER_SIZE);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		// Check for incoming commands from PC
		AudioStream_Task();

		// Process PDM data when available
		if (transfer_state != TRANSFER_WAIT
				&& transfer_state != TRANSFER_ERROR) {
			Audio_Process_PDM();
		}

		// Process full PCM block with FFT
		if (pcm_full != NULL) {
			// Perform FFT with adaptive averaging
			FFT_Postprocess_Adaptive((int16_t*) pcm_full);

			if ((block_ready == true) && (stream_status.is_streaming == true)) {



				switch (stream_status.mode) {
				case STREAM_MODE_RAW:
					AudioStream_SendRawSamples((q15_t*) pcm_full, FFT_SIZE);
					break;

				case STREAM_MODE_FFT:
					AudioStream_SendFFTData(mag_bins_output, FFT_SIZE / 2);
					break;

				case STREAM_MODE_FFT_DB:
					AudioStream_SendFFTDataDB(db_bins_output, FFT_SIZE / 2);
					break;

				case STREAM_MODE_IDLE:
				default:
					// Do nothing
					break;
				}
				block_ready = false;
			}
			// Clear flags

			pcm_full = NULL;
		}

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 96;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CRC_Init(void) {
	/* USER CODE BEGIN CRC_Init 0 */
	/* USER CODE END CRC_Init 0 */

	/* USER CODE BEGIN CRC_Init 1 */
	/* USER CODE END CRC_Init 1 */

	hcrc.Instance = CRC;
	if (HAL_CRC_Init(&hcrc) != HAL_OK) {
		Error_Handler();
	}
	__HAL_CRC_DR_RESET(&hcrc);

	/* USER CODE BEGIN CRC_Init 2 */
	/* USER CODE END CRC_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {
	/* USER CODE BEGIN TIM1_Init 0 */
	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */
	/* USER CODE END TIM1_Init 1 */

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 1;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 47;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}

	/* USER CODE BEGIN TIM1_Init 2 */
	/* USER CODE END TIM1_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {
	/* USER CODE BEGIN USART2_Init 0 */
	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */
	/* USER CODE END USART2_Init 1 */

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}

	/* USER CODE BEGIN USART2_Init 2 */
	/* USER CODE END USART2_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {
	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA2_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief Switch between ping-pong PCM buffers
 */
void Audio_Switch_Block(void) {
	block_ready = true;
	pcm_current_block =
			(pcm_current_block == pcm_output_block_ping) ?
					pcm_output_block_pong : pcm_output_block_ping;
	output_cursor = &pcm_current_block[0];
	end_output_block = &pcm_current_block[(FFT_SIZE * 2) - PCM_OUT_SIZE];
}





/**
 * @brief SPI RX Complete Callback
 * @param hspi SPI handle
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
	transfer_state = TRANSFER_COMPLETE;
}

/**
 * @brief SPI RX Half Complete Callback
 * @param hspi SPI handle
 */
void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi) {
	transfer_state = TRANSFER_HALF;
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
