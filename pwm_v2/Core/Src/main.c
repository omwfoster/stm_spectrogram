/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "pdm2pcm.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "fft_module.h"
#include "ai_logging.h"
#include "arm_math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define SEND_BUFFER_MAX_SIZE 1024

enum {
	TRANSFER_WAIT, TRANSFER_COMPLETE, TRANSFER_HALF, TRANSFER_ERROR
};

#define REC_FREQ                          8000

/* PDM buffer input size */
#define INTERNAL_BUFF_SIZE      128

/* PCM buffer output size */
#define PCM_OUT_SIZE            16

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

uint16_t RecBuf[PCM_OUT_SIZE];

/* Temporary data sample */
//static uint16_t InternalBuffer[INTERNAL_BUFF_SIZE];
extern uint16_t pcm_output_block_ping[FFT_SIZE * 2]; // pcm data is transmitted as stereo ( empty data interleaved , so 2 * required number of samples)
extern uint16_t pcm_output_block_pong[FFT_SIZE * 2];

q15_t fft_output[FFT_SIZE * 2]; //has to be twice FFT size
q15_t mag_bins[FFT_SIZE];

extern uint16_t *pcm_current_block;
bool block_ready = false;
uint16_t *output_cursor = pcm_output_block_ping;
static uint16_t *end_output_block = &pcm_output_block_ping[(FFT_SIZE * 2)
		- PCM_OUT_SIZE];
uint16_t pcm_deinterleaved[FFT_SIZE];
uint16_t *pcm_full = 0;

union U_Pdm {
	struct {
		uint8_t first_half[INTERNAL_BUFF_SIZE / 2];
		uint8_t last_half[INTERNAL_BUFF_SIZE / 2];
	};
	uint8_t PDM_In[INTERNAL_BUFF_SIZE / 2];
} t_U_Pdm;

uint16_t PDM_Out[16];

__IO uint32_t wTransferState = TRANSFER_WAIT;

ai_logging_device_t device;

//const audio_data_t test_440;

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI1_Init(void);
static void MX_CRC_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

void deinterleave(int16_t *mixed, int16_t Length);
uint32_t usart_send_function(uint8_t*, uint32_t);
void pack_data(ai_logging_packet_t *p);
void pack_data_test(ai_logging_packet_t *p);
void pack_data_raw(ai_logging_packet_t *p);
void pack_data_fft(ai_logging_packet_t *p);
void process_pdm();
uint8_t send_buffer[SEND_BUFFER_MAX_SIZE];

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
	uint8_t SPI_In[16];

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* Filter LP & HP Init */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_TIM1_Init();
	MX_SPI1_Init();
	MX_CRC_Init();
	MX_PDM2PCM_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */

	ai_logging_init(&device);
	ai_logging_init_send(&device, usart_send_function, send_buffer,
	SEND_BUFFER_MAX_SIZE);
	HAL_SPI_Receive_DMA(&hspi1, (uint8_t*) &t_U_Pdm, INTERNAL_BUFF_SIZE);

	ai_logging_packet_t packet;
	packet.timestamp = -1;

	/* USER CODE END 2 */
	ai_logging_clear_packet(&packet);
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		ai_logging_clear_packet(&packet);

		if (wTransferState != TRANSFER_WAIT )
		{
			process_pdm();
		}


		if (pcm_full != 0x0) {
			deinterleave(pcm_full, FFT_SIZE * 2);

		}

		if (block_ready != 0x0) {


//			fft_test_440_sample();
			fft_test(pcm_deinterleaved);
			pack_data_fft(&packet);
			block_ready = 0x0;
		}


		HAL_Delay(15);
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

	/** Configure the main internal regulator output voltage
	 */
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

	/** Initializes the CPU, AHB and APB buses clocks
	 */
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
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
	hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 15;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

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

void switch_block() {
	block_ready = pcm_current_block;
	pcm_current_block =
			(pcm_current_block == pcm_output_block_ping) ?
					pcm_output_block_pong : pcm_output_block_ping;
	output_cursor = &pcm_current_block[0];
	end_output_block = &pcm_current_block[(FFT_SIZE * 2) - PCM_OUT_SIZE];
}

void deinterleave(int16_t *mixed, int16_t Length) {

	for (uint16_t i = 0; i < Length; i += 2) {
		pcm_deinterleaved[i / 2] = mixed[i];

	}
	pcm_full = 0x0;
	block_ready = true;
}

void process_pdm() {

	uint16_t *next_cursor = output_cursor + PCM_OUT_SIZE; //* sizeof(uint16_t);

	if (wTransferState == TRANSFER_COMPLETE) {

		if (next_cursor <= end_output_block) {

			output_cursor = next_cursor;
			PDM_Filter(t_U_Pdm.last_half, &RecBuf, &PDM1_filter_handler);
			memcpy(output_cursor, RecBuf, PCM_OUT_SIZE * sizeof(uint16_t));

		} else {

			pcm_full = pcm_current_block;
			switch_block();
			PDM_Filter(t_U_Pdm.last_half, &RecBuf, &PDM1_filter_handler);
			memcpy(output_cursor, RecBuf, PCM_OUT_SIZE * sizeof(uint16_t));

		}
	}

	else if (wTransferState == TRANSFER_HALF) {

		if (next_cursor <= end_output_block) {

			output_cursor = next_cursor;
			PDM_Filter(t_U_Pdm.first_half, &RecBuf, &PDM1_filter_handler);
			memcpy(output_cursor, RecBuf, PCM_OUT_SIZE * sizeof(uint16_t));

		} else {

			pcm_full = pcm_current_block;
			switch_block();
			PDM_Filter(t_U_Pdm.first_half, &RecBuf, &PDM1_filter_handler);
			memcpy(output_cursor, RecBuf, PCM_OUT_SIZE * sizeof(uint16_t));
		}

	}
	wTransferState == TRANSFER_WAIT;

}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {

	wTransferState = TRANSFER_COMPLETE;
}

void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi) {

//	uint16_t *next_cursor = output_cursor + PCM_OUT_SIZE; // * sizeof(uint16_t);

	wTransferState = TRANSFER_HALF;
}

uint32_t usart_send_function(uint8_t *data_ptr, uint32_t data_size) {

	HAL_UART_Transmit(&huart2, data_ptr, data_size, 100);

}

void pack_data_raw(ai_logging_packet_t *p) {

	p->message = "fft_bins";
	p->message_size = strlen(p->message);
	p->payload = (uint8_t*) &fft_output;
	p->payload_size = sizeof(fft_output);
	ai_logging_create_shape_1d(&p->shape, 512);
	p->payload_type = AI_INT16;
	p->timestamp = HAL_GetTick();
	ai_logging_send_packet(&device, p);

}

void pack_data_fft(ai_logging_packet_t *p) {

	p->message = "fft_bins";
	p->message_size = strlen(p->message);
	p->payload = (uint8_t*) &mag_bins;
	p->payload_size = FFT_SIZE;
	ai_logging_create_shape_1d(&p->shape, FFT_SIZE);
	p->payload_type = AI_INT16;
	p->timestamp = 0xFF;   //HAL_GetTick();
	ai_logging_send_packet(&device, p);

}

void pack_data_test(ai_logging_packet_t *p) {

	uint8_t *test_pack = "124816";
	p->message = "HELLO WORLD";
	p->message_size = strlen(p->message);
	p->payload = test_pack;
	p->payload_size = strlen(test_pack);
	ai_logging_create_shape_1d(&p->shape, p->payload_size);
	p->payload_type = AI_UINT8;
	p->timestamp = 0xFF;  // HAL_GetTick();
	ai_logging_send_packet(&device, p);
	ai_logging_clear_packet(p);

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

#ifdef  USE_FULL_ASSERT
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
