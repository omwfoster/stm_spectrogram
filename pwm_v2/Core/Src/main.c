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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define SEND_BUFFER_MAX_SIZE 512

enum {
	TRANSFER_WAIT, TRANSFER_COMPLETE, TRANSFER_HALF, TRANSFER_ERROR
};

#define REC_FREQ                          8000

/* PDM buffer input size */
#define INTERNAL_BUFF_SIZE      128

/* PCM buffer output size */
#define PCM_OUT_SIZE            16

#define FFT_SIZE		512

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

uint16_t RecBuf[PCM_OUT_SIZE];

/* Temporary data sample */
static uint16_t InternalBuffer[INTERNAL_BUFF_SIZE];
extern uint16_t pcm_output_block_ping[FFT_SIZE * 2];
extern uint16_t pcm_output_block_pong[FFT_SIZE * 2];

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





/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;

TIM_HandleTypeDef htim1;

USART_HandleTypeDef husart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI1_Init(void);
static void MX_CRC_Init(void);
static void MX_USART1_Init(void);
/* USER CODE BEGIN PFP */

void deinterleave(uint16_t *mixed, uint16_t Length);
uint32_t usart_send_function(uint8_t *, uint32_t);
void pack_data(ai_logging_packet_t * p);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
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
  MX_USART1_Init();
  /* USER CODE BEGIN 2 */


  uint8_t send_buffer[SEND_BUFFER_MAX_SIZE];
  ai_logging_init(&device);
  ai_logging_init_send(&device, usart_send_function, send_buffer,SEND_BUFFER_MAX_SIZE);
  HAL_SPI_Receive_DMA(&hspi1, &t_U_Pdm, INTERNAL_BUFF_SIZE);

  ai_logging_packet_t packet;
  packet.timestamp = -1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

		if (pcm_full != 0x0) {
			deinterleave(pcm_full, FFT_SIZE * 2);
		}

		if (block_ready != 0x0) {
			fft_test(pcm_deinterleaved);
			pack_data(&packet);
			block_ready = false;

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
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
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
static void MX_SPI1_Init(void)
{

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
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
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
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 47;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  husart1.Instance = USART1;
  husart1.Init.BaudRate = 115200;
  husart1.Init.WordLength = USART_WORDLENGTH_8B;
  husart1.Init.StopBits = USART_STOPBITS_1;
  husart1.Init.Parity = USART_PARITY_NONE;
  husart1.Init.Mode = USART_MODE_TX_RX;
  husart1.Init.CLKPolarity = USART_POLARITY_LOW;
  husart1.Init.CLKPhase = USART_PHASE_1EDGE;
  husart1.Init.CLKLastBit = USART_LASTBIT_DISABLE;
  if (HAL_USART_Init(&husart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

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
static void MX_GPIO_Init(void)
{
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
end_output_block = &pcm_current_block[FFT_SIZE - PCM_OUT_SIZE];
}

void deinterleave(uint16_t *mixed, uint16_t Length) {

for (uint16_t i = 0; i < Length; i += 2) {
	pcm_deinterleaved[i / 2] = mixed[i];

}
pcm_full = 0x0;
block_ready = true;
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {

PDM_Filter(t_U_Pdm.last_half, &RecBuf, &PDM1_filter_handler);
memcpy(output_cursor, RecBuf, PCM_OUT_SIZE * sizeof(uint16_t));
uint16_t *next_cursor = output_cursor + PCM_OUT_SIZE;  //* sizeof(uint16_t);

if (next_cursor <= end_output_block) {
	output_cursor = next_cursor;
} else {
	switch_block();
	pcm_full = pcm_current_block;
}

wTransferState = TRANSFER_COMPLETE;
}

void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi) {

PDM_Filter(t_U_Pdm.first_half, &RecBuf, &PDM1_filter_handler);
memcpy(output_cursor, RecBuf, PCM_OUT_SIZE * sizeof(uint16_t));
uint16_t *next_cursor = output_cursor + PCM_OUT_SIZE; // * sizeof(uint16_t);

if (next_cursor <= end_output_block) {
	output_cursor = next_cursor;
} else {
	switch_block();
	pcm_full = pcm_current_block;
}

wTransferState = TRANSFER_HALF;
}

uint32_t usart_send_function(uint8_t *data_ptr, uint32_t data_size)
{
 /* int32_t bytesLeft = data_size;
  while(bytesLeft > 0)
  {
    while(((USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData)->TxState != 0);
    if(bytesLeft - MAX_SEND_PACKET_SIZE <= 0)
    {
      USBD_CDC_SetTxBuffer(&hUsbDeviceFS, (uint8_t*)data_ptr+(data_size - bytesLeft), bytesLeft);
      bytesLeft = 0;
    }
    else
    {
      USBD_CDC_SetTxBuffer(&hUsbDeviceFS, (uint8_t*)data_ptr+(data_size - bytesLeft), MAX_SEND_PACKET_SIZE);
      bytesLeft = bytesLeft - MAX_SEND_PACKET_SIZE;
    }

    if(USBD_CDC_TransmitPacket(&hUsbDeviceFS) != USBD_OK)
    {
      while(1);
    }

    while(((USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData)->TxState != 0);
    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, (uint8_t*)data_ptr, 0);
    if(USBD_CDC_TransmitPacket(&hUsbDeviceFS) != USBD_OK)
    {
      while(1);
    }
    HAL_Delay(1);
  }
  return data_size; // Ok all data have been sent*/
}

void pack_data(ai_logging_packet_t * p)
{

      p->message = "Accelero";
      p->message_size = strlen(p->message);
   //   p->payload = (uint8_t*)&acc_axes;
    //  p->payload_size = sizeof(acc_axes);
   //   ai_logging_create_shape_1d(* shape, 3);
      p->payload_type = AI_INT32;
      p->timestamp = HAL_GetTick();
      ai_logging_send_packet(&device, p);

}






/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
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
