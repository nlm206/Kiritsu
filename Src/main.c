/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h> // strlen
#include <stdlib.h>  // atoi
#include <stdio.h>  // sprintf
#include <math.h>   // sin\cos
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  DATA_FRAME_12 = 12,
  DATA_FRAME_9 = 9,
  DATA_FRAME_BOTH = 0,
  DATA_FRAME_INVALID
} eDATA_FRAME_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DSPACE_USART        USART6
#define TERMINAL_USART      USART3
#define dSpace_huart        huart6
#define terminal_huart      huart3

#define enc1_hspi           hspi4
#define enc2_hspi           hspi3
#define enc3_hspi           hspi5

#define htimer_1s           htim3
#define htimer_2s           htim4

#define MY_PRINT_0(buffer, pString) \
    do {\
        sprintf(buffer, pString);\
        HAL_UART_Transmit(&terminal_huart, (uint8_t *) buffer, strlen(buffer), 5000);\
    } while (0);

#define MY_PRINT_F(buffer, format, ...) \
    do {\
        sprintf(buffer, format, __VA_ARGS__);\
        HAL_UART_Transmit(&terminal_huart, (uint8_t *) buffer, strlen(buffer), 5000);\
    } while (0);



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef hspi4;
SPI_HandleTypeDef hspi5;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

static char buffer[1024];
uint8_t buffer_tx[10] = {69, 77, 67, 85, 46, 69, 85, 13, 10};
volatile uint8_t buffer_rx[2048];
uint8_t received_data, received_data_from_dspace;
volatile uint8_t received_buffer_size = 0;
volatile uint8_t command_handler_request = 0; // 0=NO, 1=Process

volatile int8_t led3_blink = -1;      // 0=OFF, 1=blink, 2=ON
volatile uint8_t dSpace_Alive = 1;    // =0, dSpace is alive, !=0, dSpace is disconnected
volatile uint8_t dSpace_req_data = 0; // =0, dSpace not request data, =1, dSpace requested for data

// Configuration - Variables
uint8_t b_encoder_1_enable = 1;
uint8_t b_encoder_2_enable = 1;
uint8_t b_encoder_3_enable = 1;
uint8_t b_mcu_dspace_enable = 1;
uint8_t b_mcu_pc_terminal_enable = 0;
uint8_t b_setting_mode = 0;
eDATA_FRAME_t data_frame_mcu2termial = DATA_FRAME_9;
int32_t b_frequency_ms = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI4_Init(void);
static void MX_SPI5_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

uint32_t Read_SSI_SPIx(SPI_HandleTypeDef *hspi);
uint32_t raw_data[3];              // Raw-Data of Encoders

// Convert raw_data to an array of byte before sending via USART
int Cal_EncoderData(eDATA_FRAME_t data_frame, uint32_t raw_data[3],
    uint8_t *array_data, int32_t angle_data[3]);

uint8_t array_data[13];            // maximum 13-byte array of data
int32_t angle_data[3];             // angle in degree - scaled 1000x

void Generate_CRCTable();          // make a look-up table of 1-byte CRC
uint8_t crc_table[256];            // CRC look-up table

uint8_t CRCChecksum(uint8_t *data, uint8_t data_len);
uint8_t CRC_Cal = 0;               // Computed CRC

void Cmd_Handler();                // to handle receiption of data to come
void CommandDecoder();             // when a compelete command has been received, this func is to decode it!

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// BUTTON Press Interrupt Handler
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_13)
  {// the RESET button is pressed, it's gonna reset the encode position!
    // To tell the terminal what is gonna to happen
    MY_PRINT_0(buffer, "the RESET buttun is pressed, it's gonna reset the encode position!\n");

    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);   // TURN OFF LED-2

    HAL_GPIO_WritePin(Enc_Reset_1_GPIO_Port, Enc_Reset_1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(Enc_Reset_2_GPIO_Port, Enc_Reset_2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(Enc_Reset_3_GPIO_Port, Enc_Reset_3_Pin, GPIO_PIN_SET);
    HAL_Delay(100); // ms
    HAL_GPIO_WritePin(Enc_Reset_1_GPIO_Port, Enc_Reset_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Enc_Reset_2_GPIO_Port, Enc_Reset_2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Enc_Reset_3_GPIO_Port, Enc_Reset_3_Pin, GPIO_PIN_RESET);
    HAL_Delay(100); // ms

    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);   // TURN ON LED-2

    MY_PRINT_0(buffer, "the process of resetting is DONE!\n");
  }

}

// UART RECEIVE Complete Callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
///////////////////////////////////////////////////////////////////////////////
// DSpace ~~~~~ UART
///////////////////////////////////////////////////////////////////////////////
  if(huart->Instance == DSPACE_USART)
  {
    if (received_data_from_dspace == 'd') {
        dSpace_Alive = 0; // dSpace is alive!
        dSpace_req_data = 1;
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    }
    if (received_data_from_dspace == 'n'){
      dSpace_Alive = 3; // set to allow LED-2 blink
      dSpace_req_data = 1;
    }
    // Waiting for another data to come
    HAL_UART_Receive_IT(&dSpace_huart, &received_data_from_dspace, 1);
  }

///////////////////////////////////////////////////////////////////////////////
// PC-Terminal Connected UART --- NO NEED To care about these lines of code!!//
///////////////////////////////////////////////////////////////////////////////
  if(huart->Instance == TERMINAL_USART)                                      //
  {                                                                          //
    if (received_buffer_size == 0){                                          //
      if (received_data == (uint8_t)'@'){                                    //
        buffer_rx[received_buffer_size] = received_data;                     //
        received_buffer_size += 1;                                           //
        // to start 1-sec time-out waiting timer                             //
        HAL_TIM_Base_Start_IT(&htimer_1s); // start 1s-period timer          //
        // indicating the command has be receiving                           //
        command_handler_request = 0;                                         //
      }                                                                      //
      else {                                                                 //
        // do nothing                                                        //
      }                                                                      //
    }                                                                        //
    else {                                                                   //
      if (received_data == (uint8_t)'#'){                                    //
        buffer_rx[received_buffer_size] = received_data;                     //
        received_buffer_size += 1;                                           //
                                                                             //
        // receive a valid command, then stop time-out timer                 //
        HAL_TIM_Base_Stop_IT(&htimer_1s); // stop timer                      //
        // indicating a command has ben received completely                  //
        command_handler_request = 1;                                         //
      }                                                                      //
      else {                                                                 //
        // Otherwise the special key (@ #) put all into the buffer.          //
        buffer_rx[received_buffer_size] = received_data;                     //
        received_buffer_size += 1;                                           //
      }                                                                      //
    }                                                                        //
    // Waiting for another data to come                                      //
    HAL_UART_Receive_IT(&terminal_huart, &received_data, 1);                 //
  }                                                                          //
///////////////////////////////////////////////////////////////////////////////
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  int num_byte = -1;
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
  MX_SPI3_Init();
  MX_SPI4_Init();
  MX_SPI5_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  // Generate CRC Checksum Table
  Generate_CRCTable();

  // Initialize TMINAL-UART REceive Complete Interrupt with 1-byte buffer
  HAL_UART_Receive_IT(&terminal_huart, &received_data, 1);

  // Initialize DSPACE-UART REceive Complete Interrupt with 1-byte buffer
  HAL_UART_Receive_IT(&dSpace_huart, &received_data_from_dspace, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_TIM_Base_Start_IT(&htimer_2s); // 2s-period timer start

  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // PC-Terminal Command Handler
    // When PC-Terminal is not gonna to be used, then this function does nothing!
    Cmd_Handler();

    // Read raw-data from encoders
    if ((dSpace_req_data == 1 && b_mcu_dspace_enable == 1)
            || b_mcu_pc_terminal_enable == 1){
#ifdef ENCODERS_HARDWARE_SUPPORT
      if (b_encoder_1_enable == 1)
        raw_data[0] = Read_SSI_SPIx(&enc1_hspi);
      if (b_encoder_2_enable == 1)
        raw_data[1] = Read_SSI_SPIx(&enc2_hspi);
      if (b_encoder_3_enable == 1)
        raw_data[2] = Read_SSI_SPIx(&enc3_hspi);
#else
      raw_data[0] = 0x00102132;
      raw_data[1] = 0x0021AB3E;
      raw_data[2] = 0x00B151D2;
#endif
    }

    // Prepare data and transmit to DSpace
    if (dSpace_req_data == 1 && b_mcu_dspace_enable == 1){
      // convert encoder-data into an array bytes of data
      num_byte = Cal_EncoderData(DATA_FRAME_9, raw_data, array_data, angle_data);
      if (num_byte > 0)
      {
        // Compute CRC code
        CRC_Cal = CRCChecksum(array_data, num_byte);
        // Save the computed CRC into the last element of the array
        array_data[num_byte] = CRC_Cal;
        // Increase the size of the array
        num_byte += 1;
        // Now send the array to dSpace
        HAL_UART_Transmit(&dSpace_huart, array_data, num_byte, 5000);
      }
      // send data to dSpace's been done!
      dSpace_req_data = 0;
    }

///////////////////////////////////////////////////////////////////////////////
// NO NEED To care about these lines of code!!                               //
///////////////////////////////////////////////////////////////////////////////
    // Prepare data and send to Terminal                                     //
    if (b_setting_mode == 0 && b_mcu_pc_terminal_enable == 1){               //
      // convert encoder-data into an array bytes of data                    //
      num_byte = Cal_EncoderData(data_frame_mcu2termial, raw_data,           //
        array_data, angle_data);                                             //
      if (num_byte > 0)                                                      //
      {                                                                      //
      // Compute CRC code                                                    //
        CRC_Cal = CRCChecksum(array_data, num_byte);                         //
        // Save the computed CRC into the last element of the array          //
        array_data[num_byte] = CRC_Cal;                                      //
        // Increase the size of the array                                    //
        num_byte += 1;                                                       //
        // Now send the array to Terminal                                    //
        if (data_frame_mcu2termial == DATA_FRAME_9)                          //
          MY_PRINT_F(buffer, "RAW-[%x,%x,%x]-[%x,%x,%x]-[%x,%x,%x]-CRC(%x)\n",
                  array_data[0], array_data[1], array_data[2],               //
                  array_data[3], array_data[4], array_data[5],               //
                  array_data[6], array_data[7], array_data[8],               //
                  array_data[9]);                                            //
        if (data_frame_mcu2termial == DATA_FRAME_12)                         //
          MY_PRINT_F(buffer, "ANGLESx1000-%ld, %ld, %ld \n",                 //
                          angle_data[0], angle_data[1], angle_data[2]);      //
       }                                                                     //
      if (b_frequency_ms > 0)                                                //
        HAL_Delay(b_frequency_ms);                                           //
    }                                                                        //
///////////////////////////////////////////////////////////////////////////////
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_USART6;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Usart6ClockSelection = RCC_USART6CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi4.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 7;
  hspi4.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi5.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 7;
  hspi5.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi5.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 15999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 2000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ENC_RESET_3_GPIO_Port, ENC_RESET_3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin|Enc_Reset_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Enc_Reset_1_GPIO_Port, Enc_Reset_1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, ENC_RESET_1_Pin|ENC_RESET_2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Enc_Reset_2_GPIO_Port, Enc_Reset_2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ENC_RESET_3_Pin USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = ENC_RESET_3_Pin|USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : DATA_CH3_Pin USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = DATA_CH3_Pin|USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD2_Pin Enc_Reset_2_Pin Enc_Reset_3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD2_Pin|Enc_Reset_2_Pin|Enc_Reset_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Enc_Reset_1_Pin */
  GPIO_InitStruct.Pin = Enc_Reset_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Enc_Reset_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ENC_RESET_1_Pin ENC_RESET_2_Pin */
  GPIO_InitStruct.Pin = ENC_RESET_1_Pin|ENC_RESET_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* Read_SSI_SPIx() Using SPI Interface
 *
 * SPI3_CLK = PC_10, SPI3_MISO = PC_11, SPI3_MOSI = PC_12
 * SPI4_CLK = PE_2, SPI4_MISO = PE_5, SPI4_MOSI = PE_6
 * SPI5_CLK = PF_7, SPI5_MISO = PE_8, SPI5_MOSI = PE_9
 *
 * */
uint32_t Read_SSI_SPIx(SPI_HandleTypeDef *hspi) {
  uint8_t pData[3] = { 0 }; // dummy data
  uint32_t data1 = 0;
  if (HAL_OK == HAL_SPI_TransmitReceive(hspi, pData, pData, 3, 5000)) {
    data1 = ((uint32_t) pData[0] << 16) | ((uint32_t) pData[1] << 8)
        | (uint32_t) pData[2];
  }
  return data1;
}


/**
  * @brief GPIO Initialization Function
  * @param [IN] eDATA_FRAME_t: data_frame (DATA_FRAME)
  * @param [IN] uint32_t raw_data[3]: raw-data of encoders
  * @param [OUT] uint8_t *array_data (not including CRC)
  * @param [OUT] int32_t angle_data[3]
  * @retval int: # of bytes returned
  */
int Cal_EncoderData(eDATA_FRAME_t data_frame, uint32_t raw_data[3], uint8_t *array_data, int32_t angle_data[3])
{
  uint32_t enc_data[3];              // Encoder Data either RAW-DATA or ANGLE-DATA
  int idx = 0, i;
  uint32_t scale = 1000;
  uint32_t value;
  float convert_factor = 0;

  switch (data_frame){
    case DATA_FRAME_12:
      convert_factor = 360.0f * scale / MAX_VALUE_19_BIT;
      for (idx = 0; idx < 3; idx++)
      {
        value = raw_data[idx] & (uint32_t)0x007FFFF0;
        value = value / 16;         // last bit is ERROR bit, value >>= 4
        //enc_data[idx] = value;
        enc_data[idx] = (uint32_t)(value * convert_factor);
        angle_data[idx] = enc_data[idx];
      }
      break;
    case DATA_FRAME_9:
      for (idx = 0; idx < 3; idx++)
      {
        enc_data[idx] = raw_data[idx];
        angle_data[idx] = 0;
      }
      break;
  default:
    return -1;
  }
  idx = 0;

  for (i = 0; i < 3; i++){
    array_data[idx++] = (uint8_t)(enc_data[i] & 0xFF);                   // 1st byte
    array_data[idx++] = (uint8_t)((uint32_t)(enc_data[i]>>8) & 0xFF);    // 2nd byte
    array_data[idx++] = (uint8_t)((uint32_t)(enc_data[i]>>16) & 0xFF);   // 3rd byte
    if (data_frame == DATA_FRAME_12)
      array_data[idx++] = (uint8_t)((uint32_t)(enc_data[i]>>24) & 0xFF); // 4th byte
  }

  return idx;
}

// Generate CRC Checksum Table
void Generate_CRCTable() {
  const uint8_t generator = 0x1D;
  uint8_t currentByte;
  for (int divident = 0; divident < 256; divident++) {
    currentByte = (uint8_t) divident;
    // calculate the CRC-8 value for current byte
    for (uint8_t bit = 0; bit < 8; bit++) {
      if ((currentByte & 0x80) != 0) {
        currentByte <<= 1;
        currentByte ^= generator;
      }
      else {
        currentByte <<= 1;
      }
    }
    // store CRC value in lookup table
    crc_table[divident] = currentByte;
  }
  crc_table[0] = 1;
}

uint8_t CRCChecksum(uint8_t *data, uint8_t data_len){
  uint8_t crc = 0xAB;     // CRC_0
  uint8_t tmp;
  for (int i = 0; i < data_len; i++) {
    tmp = data[i] ^ crc;
    crc = crc_table[tmp];
  }

  return crc;
}

// Command Handler
void Cmd_Handler(){
  uint8_t i = 0;
  if (command_handler_request > 0) {
    if (command_handler_request == 1 && received_buffer_size > 0){
      MY_PRINT_0(buffer, "Command RECEIVED: ");
    }

    // After 1-sec since the starting command '@' has been received, there is no '#' comming
    // in the 1-sec timeout handler, command_handler_request is set to 2.
    if (command_handler_request == 2 && received_buffer_size > 0){
      MY_PRINT_0(buffer, "BAD Command RECEIVED: ");
    }
    // a valid command received
    if (command_handler_request >= 1 && received_buffer_size > 0) {
      for (i = 0; i < received_buffer_size; i++){
        buffer[i] = buffer_rx[i];
      }
      buffer[received_buffer_size] = '\n';
      HAL_UART_Transmit(&terminal_huart, (uint8_t *) buffer, received_buffer_size + 1, 5000);

      // Command Decoder - SOlving here
      CommandDecoder();
    }

    // clear the buffer
    command_handler_request = 0;
    for (i = 0; i < received_buffer_size; i++){ buffer_rx[i] = 0; }
    received_buffer_size = 0;
  }
}

void CommandDecoder()
{
  // Decode the command
  uint8_t cmd_grp = buffer_rx[1];
  uint8_t data_len = buffer_rx[2];
  char data_buff[256];
  uint8_t cmd_sub = 0xFF;
  if (data_len >= 1){
    cmd_sub = buffer_rx[3];
  }
  switch (cmd_grp){
    ////////////////////////////////////////////////////////////////////////////////////
    case '0':
      if (cmd_sub == '0'){ // setting-menu
        MY_PRINT_0(buffer, "@010# : Setting - Mode\n");
        MY_PRINT_0(buffer, "@011# : Running - Mode\n");
        MY_PRINT_0(buffer, "@111# : Turn ON Encoder 1\n");
        MY_PRINT_0(buffer, "@110# : Turn OFF Encoder 1\n");
        MY_PRINT_0(buffer, "@211# : Turn ON Encoder 2\n");
        MY_PRINT_0(buffer, "@210# : Turn OFF Encoder 2\n");
        MY_PRINT_0(buffer, "@311# : Turn ON Encoder 3\n");
        MY_PRINT_0(buffer, "@310# : Turn OFF Encoder 3\n");
        MY_PRINT_0(buffer, "@410# : MCU Sends RAW_DATA to TERMINAL\n");
        MY_PRINT_0(buffer, "@411# : MCU Sends ANGLE_DATA to TERMINAL\n");
        MY_PRINT_0(buffer, "@73100# : Set frequency of 100ms\n");
        MY_PRINT_0(buffer, "@810# : Update Encoders to TERMINAL\n");
        MY_PRINT_0(buffer, "@811# : Stop Updating to TERMINAL\n");
        MY_PRINT_0(buffer, "@910# : Update Encoders to DSpace\n");
        MY_PRINT_0(buffer, "@911# : Stop Updating Encoders to DSpace\n");
        b_setting_mode = 1;
        led3_blink = 1;
      }
      else if (cmd_sub == '1') { // Enter running mode
        MY_PRINT_0(buffer, "Enter Running Mode\n");
        b_setting_mode = 0;
        if (b_mcu_pc_terminal_enable == 1){ // LED3 is ON!
          HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
          led3_blink = 2;
        }
        else { // LED3 is OFF!
          HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
          led3_blink = 0;
        }
      }
      else {
        MY_PRINT_F(buffer, "Invalid Command CMD_GRP(%c), CMD_SUB(%c)\n", cmd_grp, cmd_sub);
      }
      break;
    ////////////////////////////////////////////////////////////////////////////////////
    case '1':              // Encoder -1
      if (cmd_sub == '1'){ // turn on
        MY_PRINT_0(buffer, "TURN-ON Encoder-1\n");
        b_encoder_1_enable = 1;
      }
      else if (cmd_sub == '0'){ // turn off
        MY_PRINT_0(buffer, "TURN-OFF Encoder-1\n");
        b_encoder_1_enable = 0;
      }
      else {
        MY_PRINT_F(buffer, "Invalid Command CMD_GRP(%c), CMD_SUB(%c)\n", cmd_grp, cmd_sub);
      }
      break;
    ////////////////////////////////////////////////////////////////////////////////////
    case '2':              // Encoder -2
      if (cmd_sub == '1'){ // turn on
        MY_PRINT_0(buffer, "TURN-ON Encoder-2\n");
        b_encoder_2_enable = 1;
      }
      else if (cmd_sub == '0'){ // turn off
        MY_PRINT_0(buffer, "TURN-OFF Encoder-2\n");
        b_encoder_2_enable = 0;
      }
      else {
        MY_PRINT_F(buffer, "Invalid Command CMD_GRP(%c), CMD_SUB(%c)\n", cmd_grp, cmd_sub);
      }
      break;
    ////////////////////////////////////////////////////////////////////////////////////
    case '3':              // Encoder -3
      if (cmd_sub == '1'){ // turn on
        MY_PRINT_0(buffer, "TURN-ON Encoder-3\n");
        b_encoder_3_enable = 1;
      }
      else if (cmd_sub == '0'){ // turn off
        MY_PRINT_0(buffer, "TURN-OFF Encoder-3\n");
        b_encoder_3_enable = 0;
      }
      else {
        MY_PRINT_F(buffer, "Invalid Command CMD_GRP(%c), CMD_SUB(%c)\n", cmd_grp, cmd_sub);
      }
      break;
    ////////////////////////////////////////////////////////////////////////////////////
    case '4':              // DATA Format
      if (cmd_sub == '0')  { // MCU sends RAW-Data to Terminal
        MY_PRINT_0(buffer, "MCU sends RAW-Data to Terminal\n");
        data_frame_mcu2termial = DATA_FRAME_9;
      }
      else if (cmd_sub == '1')  { // MCU sends ANGLE-Data to Terminal
        MY_PRINT_0(buffer, "MCU sends ANGLE-Data to Terminal\n");
        data_frame_mcu2termial = DATA_FRAME_12;
      }
      else {
        MY_PRINT_F(buffer, "Invalid Command CMD_GRP(%c), CMD_SUB(%c)\n", cmd_grp, cmd_sub);
        data_frame_mcu2termial = DATA_FRAME_INVALID;
      }
      break;
    ////////////////////////////////////////////////////////////////////////////////////
    case '7': // Frequency
      if (data_len <= '9' && data_len >= '0'){ //  maximum of 4-byte frequency
        for (int i = 0; i < data_len - '0'; i++){
            data_buff[i] = buffer_rx[3+i];
        }
        data_buff[data_len - '0'] = '\0';
        b_frequency_ms = atoi(data_buff);
        MY_PRINT_F(buffer, "Frequency is set to %ld (ms)\n", b_frequency_ms);
      }
      else {
        MY_PRINT_F(buffer, "Invalid Command CMD_GRP(%c), data_len(%c)\n", cmd_grp, data_len);
      }
      break;
    ////////////////////////////////////////////////////////////////////////////////////
    case '8': // Update Encoders to Terminal
      if (cmd_sub == '1'){ // Encoder Data is sent to Terminal
        b_mcu_pc_terminal_enable = 1;
        MY_PRINT_0(buffer, "Update Encoders to Terminal\n");
      }
      else if (cmd_sub == '0') {
        b_mcu_pc_terminal_enable = 0;
        MY_PRINT_0(buffer, "Stop Updating Encoders to Terminal\n");
      }
      else {
        MY_PRINT_F(buffer, "Invalid Command CMD_GRP(%c), CMD_SUB(%c)\n", cmd_grp, cmd_sub);
      }
      break;
    ////////////////////////////////////////////////////////////////////////////////////
    case '9': // Update Encoders to DSpace
      if (cmd_sub == '1'){ // Encoder Data is sent to Dspace
        b_mcu_dspace_enable = 1;
        MY_PRINT_0(buffer, "Update Encoders to DSpace\n");
      }
      else if (cmd_sub == '0') {
        b_mcu_dspace_enable = 0;
        MY_PRINT_0(buffer, "Stop Updating Encoders to DSpace\n");
      }
      else {
        MY_PRINT_F(buffer, "Invalid Command CMD_GRP(%c), CMD_SUB(%c)\n", cmd_grp, cmd_sub);
      }
      break;
    ////////////////////////////////////////////////////////////////////////////////////
    default:
        MY_PRINT_F(buffer, "Invalid Command CMD_GRP(%c)\n", cmd_grp);
      break;
  }
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
  while(1)
  {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
