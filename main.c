/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h" // library for C strings
#include <stdbool.h> // library for bool data type

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define CE_PORT GPIOB // PB6 chip enable (aka slave select)
#define CE_PIN GPIO_PIN_6
#define DC_PORT GPIOA // PA0 data/control
#define DC_PIN GPIO_PIN_0
#define RESET_PORT GPIOA // PA1 reset
#define RESET_PIN GPIO_PIN_1
#define GLCD_WIDTH 84
#define GLCD_HEIGHT 48
#define NUM_BANKS 6
#define UART_DELAY 100 // wait max of 100 ms between frames in message
#define MAX_MESSAGE_SIZE 100 // 100 characters maximum message size
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
const char font_table[][6] = {
 {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // space
{0x7E, 0x11, 0x11, 0x11, 0x7E, 0x00}, // 'A'
{0x7F, 0x49, 0x49, 0x49, 0x36, 0x00}, // 'B'
{0x3E, 0x41, 0x41, 0x41, 0x22, 0x00}, // 'C'
{0x7F, 0x41, 0x41, 0x41, 0x3E, 0x00}, // 'D'
{0x7F, 0x49, 0x49, 0x41, 0x41, 0x00}, // 'E'
{0x7F, 0x09, 0x09, 0x09, 0x01, 0x00}, // 'F'
{0x3E, 0x41, 0x49, 0x49, 0x39, 0x00}, // 'G'
{0x7F, 0x08, 0x08, 0x08, 0x7F, 0x00}, // 'H'
{0x41, 0x41, 0x7F, 0x41, 0x41, 0x00}, // 'I'
{0x21, 0x41, 0x41, 0x3F, 0x01, 0x00}, // 'J'
{0x7F, 0x08, 0x14, 0x22, 0x41, 0x00}, // 'K'
{0x7F, 0x40, 0x40, 0x40, 0x40, 0x00}, // 'L'
{0x7F, 0x06, 0x78, 0x06, 0x7F, 0x00}, // 'M'
{0x7F, 0x06, 0x08, 0x30, 0x7F, 0x00}, // 'N'
{0x3E, 0x41, 0x41, 0x41, 0x3E, 0x00}, // 'O'
{0x7F, 0x11, 0x11, 0x11, 0x0E, 0x00}, // 'P'
{0x3E, 0x41, 0x51, 0x61, 0x3E, 0x00}, // 'Q'
{0x7F, 0x11, 0x11, 0x31, 0x4E, 0x00}, // 'R'
{0x46, 0x49, 0x49, 0x49, 0x31, 0x00}, // 'S'
{0x01, 0x01, 0x7F, 0x01, 0x01, 0x00}, // 'T'
{0x3F, 0x40, 0x40, 0x40, 0x3F, 0x00}, // 'U'
{0x0F, 0x30, 0x40, 0x30, 0x0F, 0x00}, // 'V'
{0x7F, 0x30, 0x08, 0x30, 0x7F, 0x00}, // 'W'
{0x63, 0x14, 0x08, 0x14, 0x63, 0x00}, // 'X'
{0x03, 0x04, 0x78, 0x04, 0x03, 0x00}, // 'Y'
{0x61, 0x51, 0x49, 0x45, 0x43, 0x00}, // 'Z'
{0x7E, 0x81, 0xB5, 0xA1, 0xA1, 0xB5}, // smile left
{0x81, 0x7E, 0x00, 0x00, 0x00, 0x00}, // smile right
{0x00, 0x00, 0x5F, 0x00, 0x00, 0x00}, // '!'
};
uint8_t message[MAX_MESSAGE_SIZE] = {0}; // char array to store message received
uint8_t response[MAX_MESSAGE_SIZE] = {0}; // char array to store response message
uint8_t uart2_byte; // byte received from UART2
uint8_t buffer_position = 0; // how many bytes received so far
bool blink = false; // LED blink status
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void SPI_write(unsigned char data);
void GLCD_data_write(unsigned char data);
void GLCD_command_write(unsigned char data);
void GLCD_init(void);
void GLCD_setCursor(unsigned char x, unsigned char y);
void GLCD_clear(void);
void GLCD_putchar(int font_table_row);

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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  // Start UART2 to receive interrupts
  HAL_UART_Receive_IT(&huart2, &uart2_byte, 1); // put byte from UART2 in "uart2_byte"
  GLCD_init(); // initialize the screen
  GLCD_clear(); // clear the screen
  //row 1
  GLCD_putchar(0); // space
  GLCD_putchar(0); // space
  GLCD_putchar(0); // space
  GLCD_putchar(0); // space
  GLCD_putchar(0); // space
  GLCD_putchar(0); // space
  GLCD_putchar(0); // space
  GLCD_putchar(0); // space
  GLCD_putchar(0); // space
  GLCD_putchar(0); // space
  GLCD_putchar(0); // space
  GLCD_putchar(0); // space
  GLCD_putchar(0); // space
  GLCD_putchar(0); // space
  //row 2
  GLCD_putchar(13); // 'M'
  GLCD_putchar(5); // 'E'
  GLCD_putchar(18); // 'R'
  GLCD_putchar(18); // 'R'
  GLCD_putchar(25); // 'Y'
  GLCD_putchar(0); // space
  GLCD_putchar(24); // 'X'
  GLCD_putchar(13); // 'M'
  GLCD_putchar(1); // 'A'
  GLCD_putchar(19); // 'S'
  GLCD_putchar(29); // '!'
  GLCD_putchar(27); // left smiley
  GLCD_putchar(28); // right smiley
  GLCD_putchar(0); // space
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  if (blink){

		//blink all LEDs
		HAL_GPIO_TogglePin(RED1_GPIO_Port, RED1_Pin);
        HAL_GPIO_TogglePin(RED2_GPIO_Port, RED2_Pin);
        HAL_GPIO_TogglePin(RED3_GPIO_Port, RED3_Pin);
        HAL_GPIO_TogglePin(RED4_GPIO_Port, RED4_Pin);
        HAL_GPIO_TogglePin(RED5_GPIO_Port, RED5_Pin);
      	HAL_GPIO_TogglePin(GREEN1_GPIO_Port, GREEN1_Pin);
      	HAL_GPIO_TogglePin(GREEN2_GPIO_Port, GREEN2_Pin);
      	HAL_GPIO_TogglePin(GREEN3_GPIO_Port, GREEN3_Pin);
      	HAL_GPIO_TogglePin(GREEN4_GPIO_Port, GREEN4_Pin);
      	HAL_GPIO_TogglePin(GREEN5_GPIO_Port, GREEN5_Pin);
      	HAL_GPIO_TogglePin(WHITE1_GPIO_Port, WHITE1_Pin);
      	HAL_GPIO_TogglePin(WHITE2_GPIO_Port, WHITE2_Pin);
      	HAL_GPIO_TogglePin(WHITE3_GPIO_Port, WHITE3_Pin);
      	HAL_GPIO_TogglePin(WHITE4_GPIO_Port, WHITE4_Pin);
      	HAL_GPIO_TogglePin(WHITE5_GPIO_Port, WHITE5_Pin);
      	HAL_Delay(200);

      	//Blink red LEDs
        HAL_GPIO_TogglePin(RED1_GPIO_Port, RED1_Pin);
        HAL_GPIO_TogglePin(RED2_GPIO_Port, RED2_Pin);
        HAL_GPIO_TogglePin(RED3_GPIO_Port, RED3_Pin);
        HAL_GPIO_TogglePin(RED4_GPIO_Port, RED4_Pin);
        HAL_GPIO_TogglePin(RED5_GPIO_Port, RED5_Pin);
        HAL_Delay(200);
        HAL_GPIO_TogglePin(RED1_GPIO_Port, RED1_Pin);
        HAL_GPIO_TogglePin(RED2_GPIO_Port, RED2_Pin);
        HAL_GPIO_TogglePin(RED3_GPIO_Port, RED3_Pin);
        HAL_GPIO_TogglePin(RED4_GPIO_Port, RED4_Pin);
        HAL_GPIO_TogglePin(RED5_GPIO_Port, RED5_Pin);
        HAL_Delay(200);

        //blink white LEDs
      	HAL_GPIO_TogglePin(WHITE1_GPIO_Port, WHITE1_Pin);
      	HAL_GPIO_TogglePin(WHITE2_GPIO_Port, WHITE2_Pin);
      	HAL_GPIO_TogglePin(WHITE3_GPIO_Port, WHITE3_Pin);
      	HAL_GPIO_TogglePin(WHITE4_GPIO_Port, WHITE4_Pin);
      	HAL_GPIO_TogglePin(WHITE5_GPIO_Port, WHITE5_Pin);
        HAL_Delay(200);
      	HAL_GPIO_TogglePin(WHITE1_GPIO_Port, WHITE1_Pin);
      	HAL_GPIO_TogglePin(WHITE2_GPIO_Port, WHITE2_Pin);
      	HAL_GPIO_TogglePin(WHITE3_GPIO_Port, WHITE3_Pin);
      	HAL_GPIO_TogglePin(WHITE4_GPIO_Port, WHITE4_Pin);
      	HAL_GPIO_TogglePin(WHITE5_GPIO_Port, WHITE5_Pin);
        HAL_Delay(200);

        //blink green LEDs
      	HAL_GPIO_TogglePin(GREEN1_GPIO_Port, GREEN1_Pin);
      	HAL_GPIO_TogglePin(GREEN2_GPIO_Port, GREEN2_Pin);
      	HAL_GPIO_TogglePin(GREEN3_GPIO_Port, GREEN3_Pin);
      	HAL_GPIO_TogglePin(GREEN4_GPIO_Port, GREEN4_Pin);
      	HAL_GPIO_TogglePin(GREEN5_GPIO_Port, GREEN5_Pin);
        HAL_Delay(200);
      	HAL_GPIO_TogglePin(GREEN1_GPIO_Port, GREEN1_Pin);
      	HAL_GPIO_TogglePin(GREEN2_GPIO_Port, GREEN2_Pin);
      	HAL_GPIO_TogglePin(GREEN3_GPIO_Port, GREEN3_Pin);
      	HAL_GPIO_TogglePin(GREEN4_GPIO_Port, GREEN4_Pin);
      	HAL_GPIO_TogglePin(GREEN5_GPIO_Port, GREEN5_Pin);
        HAL_Delay(200);

	  }

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|RED5_Pin|RED4_Pin
                          |RED3_Pin|RED2_Pin|RED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GREEN5_Pin|GREEN4_Pin|GREEN3_Pin|GREEN2_Pin
                          |GREEN1_Pin|WHITE5_Pin|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, WHITE4_Pin|WHITE3_Pin|WHITE2_Pin|WHITE1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 RED5_Pin RED4_Pin
                           RED3_Pin RED2_Pin RED1_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|RED5_Pin|RED4_Pin
                          |RED3_Pin|RED2_Pin|RED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : GREEN5_Pin GREEN4_Pin GREEN3_Pin GREEN2_Pin
                           GREEN1_Pin WHITE5_Pin PB6 */
  GPIO_InitStruct.Pin = GREEN5_Pin|GREEN4_Pin|GREEN3_Pin|GREEN2_Pin
                          |GREEN1_Pin|WHITE5_Pin|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : WHITE4_Pin WHITE3_Pin WHITE2_Pin WHITE1_Pin */
  GPIO_InitStruct.Pin = WHITE4_Pin|WHITE3_Pin|WHITE2_Pin|WHITE1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void GLCD_putchar(int font_table_row){
 int i;
 for (i=0; i<6; i++){
 GLCD_data_write(font_table[font_table_row][i]);
 }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

    // Check if byte received was on UART2 (from laptop)
    if (huart == &huart2){

        // If we get here, it means we received a byte from UART2 and it was placed in "uart2_byte"

        // If uart2_byte isn't \r, \n, or \0, it means the message isn't over yet
        if ((uart2_byte != '\r') && (uart2_byte != '\n') && (uart2_byte != '\0')){

            // Add uart2_byte to the message
            message[buffer_position] = uart2_byte;
            buffer_position++;

        } else {

            // If we get here, it means we received \r, \n, or \0 and the message is over

            // Check message
            if (strcmp((char*)message, "RED_ON") == 0) {

                // If "RED_ON\n" was the message received -> Turn on Red LEDs
                HAL_GPIO_WritePin(RED1_GPIO_Port, RED1_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(RED2_GPIO_Port, RED2_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(RED3_GPIO_Port, RED3_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(RED4_GPIO_Port, RED4_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(RED5_GPIO_Port, RED5_Pin, GPIO_PIN_SET);
                blink = false;

                GLCD_clear(); // clear the screen
                //row 1
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                //row 2
                GLCD_putchar(13); // 'M'
                GLCD_putchar(5); // 'E'
                GLCD_putchar(18); // 'R'
                GLCD_putchar(18); // 'R'
                GLCD_putchar(25); // 'Y'
                GLCD_putchar(0); // space
                GLCD_putchar(24); // 'X'
                GLCD_putchar(13); // 'M'
                GLCD_putchar(1); // 'A'
                GLCD_putchar(19); // 'S'
                GLCD_putchar(29); // '!'
                GLCD_putchar(27); // left smiley
                GLCD_putchar(28); // right smiley
                GLCD_putchar(0); // space
                //row 3
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                //row 4
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(18); // 'R'
                GLCD_putchar(5); // 'E'
                GLCD_putchar(4); // 'D'
                GLCD_putchar(0); // space
                GLCD_putchar(15); // 'O'
                GLCD_putchar(14); // 'N'
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space

                // Prepare response message
                strncpy((char*)response, "Red LEDs turned on.\n\r", MAX_MESSAGE_SIZE);

            } else if (strcmp((char*)message, "RED_OFF") == 0) {

                // If "RED_OFF\n" was the message received -> Turn off Red LEDs
            	HAL_GPIO_WritePin(RED1_GPIO_Port, RED1_Pin, GPIO_PIN_RESET);
            	HAL_GPIO_WritePin(RED2_GPIO_Port, RED2_Pin, GPIO_PIN_RESET);
            	HAL_GPIO_WritePin(RED3_GPIO_Port, RED3_Pin, GPIO_PIN_RESET);
            	HAL_GPIO_WritePin(RED4_GPIO_Port, RED4_Pin, GPIO_PIN_RESET);
            	HAL_GPIO_WritePin(RED5_GPIO_Port, RED5_Pin, GPIO_PIN_RESET);
                blink = false;

                GLCD_clear(); // clear the screen
                //row 1
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                //row 2
                GLCD_putchar(13); // 'M'
                GLCD_putchar(5); // 'E'
                GLCD_putchar(18); // 'R'
                GLCD_putchar(18); // 'R'
                GLCD_putchar(25); // 'Y'
                GLCD_putchar(0); // space
                GLCD_putchar(24); // 'X'
                GLCD_putchar(13); // 'M'
                GLCD_putchar(1); // 'A'
                GLCD_putchar(19); // 'S'
                GLCD_putchar(29); // '!'
                GLCD_putchar(27); // left smiley
                GLCD_putchar(28); // right smiley
                GLCD_putchar(0); // space
                //row 3
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                //row 4
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(18); // 'R'
                GLCD_putchar(5); // 'E'
                GLCD_putchar(4); // 'D'
                GLCD_putchar(0); // space
                GLCD_putchar(15); // 'O'
                GLCD_putchar(6); // 'F'
                GLCD_putchar(6); // 'F'
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space

                // Prepare response message
                strncpy((char*)response, "Red LEDs turned off.\n\r", MAX_MESSAGE_SIZE);

            } else if (strcmp((char*)message, "GREEN_ON") == 0) {

                // If "GREEN_ON\n" was the message received -> Turn on GREEN LEDs
            	HAL_GPIO_WritePin(GREEN1_GPIO_Port, GREEN1_Pin, GPIO_PIN_SET);
            	HAL_GPIO_WritePin(GREEN2_GPIO_Port, GREEN2_Pin, GPIO_PIN_SET);
            	HAL_GPIO_WritePin(GREEN3_GPIO_Port, GREEN3_Pin, GPIO_PIN_SET);
            	HAL_GPIO_WritePin(GREEN4_GPIO_Port, GREEN4_Pin, GPIO_PIN_SET);
            	HAL_GPIO_WritePin(GREEN5_GPIO_Port, GREEN5_Pin, GPIO_PIN_SET);
                blink = false;

                GLCD_clear(); // clear the screen
                //row 1
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                //row 2
                GLCD_putchar(13); // 'M'
                GLCD_putchar(5); // 'E'
                GLCD_putchar(18); // 'R'
                GLCD_putchar(18); // 'R'
                GLCD_putchar(25); // 'Y'
                GLCD_putchar(0); // space
                GLCD_putchar(24); // 'X'
                GLCD_putchar(13); // 'M'
                GLCD_putchar(1); // 'A'
                GLCD_putchar(19); // 'S'
                GLCD_putchar(29); // '!'
                GLCD_putchar(27); // left smiley
                GLCD_putchar(28); // right smiley
                GLCD_putchar(0); // space
                //row 3
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                //row 4
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(7); // 'G'
                GLCD_putchar(18); // 'R'
                GLCD_putchar(5); // 'E'
                GLCD_putchar(5); // 'E'
                GLCD_putchar(14); // 'N'
                GLCD_putchar(0); // space
                GLCD_putchar(15); // 'O'
                GLCD_putchar(14); // 'N'
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space

                // Prepare response message
                strncpy((char*)response, "Green LEDs turned on.\n\r", MAX_MESSAGE_SIZE);

            } else if (strcmp((char*)message, "GREEN_OFF") == 0) {

                // If "GREEN_OFF\n" was the message received -> Turn off GREEN LEDs
            	HAL_GPIO_WritePin(GREEN1_GPIO_Port, GREEN1_Pin, GPIO_PIN_RESET);
            	HAL_GPIO_WritePin(GREEN2_GPIO_Port, GREEN2_Pin, GPIO_PIN_RESET);
            	HAL_GPIO_WritePin(GREEN3_GPIO_Port, GREEN3_Pin, GPIO_PIN_RESET);
            	HAL_GPIO_WritePin(GREEN4_GPIO_Port, GREEN4_Pin, GPIO_PIN_RESET);
            	HAL_GPIO_WritePin(GREEN5_GPIO_Port, GREEN5_Pin, GPIO_PIN_RESET);
                blink = false;

                GLCD_clear(); // clear the screen
                //row 1
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                //row 2
                GLCD_putchar(13); // 'M'
                GLCD_putchar(5); // 'E'
                GLCD_putchar(18); // 'R'
                GLCD_putchar(18); // 'R'
                GLCD_putchar(25); // 'Y'
                GLCD_putchar(0); // space
                GLCD_putchar(24); // 'X'
                GLCD_putchar(13); // 'M'
                GLCD_putchar(1); // 'A'
                GLCD_putchar(19); // 'S'
                GLCD_putchar(29); // '!'
                GLCD_putchar(27); // left smiley
                GLCD_putchar(28); // right smiley
                GLCD_putchar(0); // space
                //row 3
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                //row 4
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(7); // 'G'
                GLCD_putchar(18); // 'R'
                GLCD_putchar(5); // 'E'
                GLCD_putchar(5); // 'E'
                GLCD_putchar(14); // 'N'
                GLCD_putchar(0); // space
                GLCD_putchar(15); // 'O'
                GLCD_putchar(6); // 'F'
                GLCD_putchar(6); // 'F'
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space

                // Prepare response message
                strncpy((char*)response, "Green LEDs turned off.\n\r", MAX_MESSAGE_SIZE);

            } else if (strcmp((char*)message, "WHITE_ON") == 0) {

                // If "WHITE_ON\n" was the message received -> Turn on WHITE LEDs
            	HAL_GPIO_WritePin(WHITE1_GPIO_Port, WHITE1_Pin, GPIO_PIN_SET);
            	HAL_GPIO_WritePin(WHITE2_GPIO_Port, WHITE2_Pin, GPIO_PIN_SET);
            	HAL_GPIO_WritePin(WHITE3_GPIO_Port, WHITE3_Pin, GPIO_PIN_SET);
            	HAL_GPIO_WritePin(WHITE4_GPIO_Port, WHITE4_Pin, GPIO_PIN_SET);
            	HAL_GPIO_WritePin(WHITE5_GPIO_Port, WHITE5_Pin, GPIO_PIN_SET);
                blink = false;

                GLCD_clear(); // clear the screen
                //row 1
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                //row 2
                GLCD_putchar(13); // 'M'
                GLCD_putchar(5); // 'E'
                GLCD_putchar(18); // 'R'
                GLCD_putchar(18); // 'R'
                GLCD_putchar(25); // 'Y'
                GLCD_putchar(0); // space
                GLCD_putchar(24); // 'X'
                GLCD_putchar(13); // 'M'
                GLCD_putchar(1); // 'A'
                GLCD_putchar(19); // 'S'
                GLCD_putchar(29); // '!'
                GLCD_putchar(27); // left smiley
                GLCD_putchar(28); // right smiley
                GLCD_putchar(0); // space
                //row 3
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                //row 4
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(23); // 'W'
                GLCD_putchar(8); // 'H'
                GLCD_putchar(9); // 'I'
                GLCD_putchar(20); // 'T'
                GLCD_putchar(5); // 'E'
                GLCD_putchar(0); // space
                GLCD_putchar(15); // 'O'
                GLCD_putchar(14); // 'N'
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space

                // Prepare response message
                strncpy((char*)response, "White LEDs turned on.\n\r", MAX_MESSAGE_SIZE);

            } else if (strcmp((char*)message, "WHITE_OFF") == 0) {

                // If "WHITE_OFF\n" was the message received -> Turn off WHITE LEDs
            	HAL_GPIO_WritePin(WHITE1_GPIO_Port, WHITE1_Pin, GPIO_PIN_RESET);
            	HAL_GPIO_WritePin(WHITE2_GPIO_Port, WHITE2_Pin, GPIO_PIN_RESET);
            	HAL_GPIO_WritePin(WHITE3_GPIO_Port, WHITE3_Pin, GPIO_PIN_RESET);
            	HAL_GPIO_WritePin(WHITE4_GPIO_Port, WHITE4_Pin, GPIO_PIN_RESET);
            	HAL_GPIO_WritePin(WHITE5_GPIO_Port, WHITE5_Pin, GPIO_PIN_RESET);
                blink = false;

                GLCD_clear(); // clear the screen
                //row 1
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                //row 2
                GLCD_putchar(13); // 'M'
                GLCD_putchar(5); // 'E'
                GLCD_putchar(18); // 'R'
                GLCD_putchar(18); // 'R'
                GLCD_putchar(25); // 'Y'
                GLCD_putchar(0); // space
                GLCD_putchar(24); // 'X'
                GLCD_putchar(13); // 'M'
                GLCD_putchar(1); // 'A'
                GLCD_putchar(19); // 'S'
                GLCD_putchar(29); // '!'
                GLCD_putchar(27); // left smiley
                GLCD_putchar(28); // right smiley
                GLCD_putchar(0); // space
                //row 3
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                //row 4
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(23); // 'W'
                GLCD_putchar(8); // 'H'
                GLCD_putchar(9); // 'I'
                GLCD_putchar(20); // 'T'
                GLCD_putchar(5); // 'E'
                GLCD_putchar(0); // space
                GLCD_putchar(15); // 'O'
                GLCD_putchar(6); // 'F'
                GLCD_putchar(6); // 'F'
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space

                // Prepare response message
                strncpy((char*)response, "White LEDs turned off.\n\r", MAX_MESSAGE_SIZE);

            } else if (strcmp((char*)message, "ALL_ON") == 0) {

                // If "ALL_ON\n" was the message received -> Turn on all LEDs
                HAL_GPIO_WritePin(RED1_GPIO_Port, RED1_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(RED2_GPIO_Port, RED2_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(RED3_GPIO_Port, RED3_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(RED4_GPIO_Port, RED4_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(RED5_GPIO_Port, RED5_Pin, GPIO_PIN_SET);
            	HAL_GPIO_WritePin(GREEN1_GPIO_Port, GREEN1_Pin, GPIO_PIN_SET);
            	HAL_GPIO_WritePin(GREEN2_GPIO_Port, GREEN2_Pin, GPIO_PIN_SET);
            	HAL_GPIO_WritePin(GREEN3_GPIO_Port, GREEN3_Pin, GPIO_PIN_SET);
            	HAL_GPIO_WritePin(GREEN4_GPIO_Port, GREEN4_Pin, GPIO_PIN_SET);
            	HAL_GPIO_WritePin(GREEN5_GPIO_Port, GREEN5_Pin, GPIO_PIN_SET);
            	HAL_GPIO_WritePin(WHITE1_GPIO_Port, WHITE1_Pin, GPIO_PIN_SET);
            	HAL_GPIO_WritePin(WHITE2_GPIO_Port, WHITE2_Pin, GPIO_PIN_SET);
            	HAL_GPIO_WritePin(WHITE3_GPIO_Port, WHITE3_Pin, GPIO_PIN_SET);
            	HAL_GPIO_WritePin(WHITE4_GPIO_Port, WHITE4_Pin, GPIO_PIN_SET);
            	HAL_GPIO_WritePin(WHITE5_GPIO_Port, WHITE5_Pin, GPIO_PIN_SET);
                blink = false;

                GLCD_clear(); // clear the screen
                //row 1
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                //row 2
                GLCD_putchar(13); // 'M'
                GLCD_putchar(5); // 'E'
                GLCD_putchar(18); // 'R'
                GLCD_putchar(18); // 'R'
                GLCD_putchar(25); // 'Y'
                GLCD_putchar(0); // space
                GLCD_putchar(24); // 'X'
                GLCD_putchar(13); // 'M'
                GLCD_putchar(1); // 'A'
                GLCD_putchar(19); // 'S'
                GLCD_putchar(29); // '!'
                GLCD_putchar(27); // left smiley
                GLCD_putchar(28); // right smiley
                GLCD_putchar(0); // space
                //row 3
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                //row 4
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(1); // 'A'
                GLCD_putchar(12); // 'L'
                GLCD_putchar(12); // 'L'
                GLCD_putchar(0); // space
                GLCD_putchar(15); // 'O'
                GLCD_putchar(14); // 'N'
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space

                // Prepare response message
                strncpy((char*)response, "All LEDs turned on.\n\r", MAX_MESSAGE_SIZE);

            } else if (strcmp((char*)message, "ALL_OFF") == 0) {

                // If "ALL_OFF\n" was the message received -> Turn off all LEDs
                HAL_GPIO_WritePin(RED1_GPIO_Port, RED1_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(RED2_GPIO_Port, RED2_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(RED3_GPIO_Port, RED3_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(RED4_GPIO_Port, RED4_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(RED5_GPIO_Port, RED5_Pin, GPIO_PIN_RESET);
            	HAL_GPIO_WritePin(GREEN1_GPIO_Port, GREEN1_Pin, GPIO_PIN_RESET);
            	HAL_GPIO_WritePin(GREEN2_GPIO_Port, GREEN2_Pin, GPIO_PIN_RESET);
            	HAL_GPIO_WritePin(GREEN3_GPIO_Port, GREEN3_Pin, GPIO_PIN_RESET);
            	HAL_GPIO_WritePin(GREEN4_GPIO_Port, GREEN4_Pin, GPIO_PIN_RESET);
            	HAL_GPIO_WritePin(GREEN5_GPIO_Port, GREEN5_Pin, GPIO_PIN_RESET);
            	HAL_GPIO_WritePin(WHITE1_GPIO_Port, WHITE1_Pin, GPIO_PIN_RESET);
            	HAL_GPIO_WritePin(WHITE2_GPIO_Port, WHITE2_Pin, GPIO_PIN_RESET);
            	HAL_GPIO_WritePin(WHITE3_GPIO_Port, WHITE3_Pin, GPIO_PIN_RESET);
            	HAL_GPIO_WritePin(WHITE4_GPIO_Port, WHITE4_Pin, GPIO_PIN_RESET);
            	HAL_GPIO_WritePin(WHITE5_GPIO_Port, WHITE5_Pin, GPIO_PIN_RESET);
                blink = false;

                GLCD_clear(); // clear the screen
                //row 1
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                //row 2
                GLCD_putchar(13); // 'M'
                GLCD_putchar(5); // 'E'
                GLCD_putchar(18); // 'R'
                GLCD_putchar(18); // 'R'
                GLCD_putchar(25); // 'Y'
                GLCD_putchar(0); // space
                GLCD_putchar(24); // 'X'
                GLCD_putchar(13); // 'M'
                GLCD_putchar(1); // 'A'
                GLCD_putchar(19); // 'S'
                GLCD_putchar(29); // '!'
                GLCD_putchar(27); // left smiley
                GLCD_putchar(28); // right smiley
                GLCD_putchar(0); // space
                //row 3
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                //row 4
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(1); // 'A'
                GLCD_putchar(12); // 'L'
                GLCD_putchar(12); // 'L'
                GLCD_putchar(0); // space
                GLCD_putchar(15); // 'O'
                GLCD_putchar(6); // 'F'
                GLCD_putchar(6); // 'F'
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space

                // Prepare response message
                strncpy((char*)response, "All LEDs turned off.\n\r", MAX_MESSAGE_SIZE);

            } else if (strcmp((char*)message, "BLINK_LEDS") == 0) {

                // If "BLINK_LEDS" was the message received -> tell main() to blink all LEDs
                blink = true;

                GLCD_clear(); // clear the screen
                //row 1
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                //row 2
                GLCD_putchar(13); // 'M'
                GLCD_putchar(5); // 'E'
                GLCD_putchar(18); // 'R'
                GLCD_putchar(18); // 'R'
                GLCD_putchar(25); // 'Y'
                GLCD_putchar(0); // space
                GLCD_putchar(24); // 'X'
                GLCD_putchar(13); // 'M'
                GLCD_putchar(1); // 'A'
                GLCD_putchar(19); // 'S'
                GLCD_putchar(29); // '!'
                GLCD_putchar(27); // left smiley
                GLCD_putchar(28); // right smiley
                GLCD_putchar(0); // space
                //row 3
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                //row 4
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(1); // 'A'
                GLCD_putchar(12); // 'L'
                GLCD_putchar(12); // 'L'
                GLCD_putchar(0); // space
                GLCD_putchar(2); // 'B
                GLCD_putchar(12); // 'L'
                GLCD_putchar(9); // 'I'
                GLCD_putchar(14); // 'N'
                GLCD_putchar(11); // 'K'
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space

                // Prepare response message
                strncpy((char*)response, "LEDs are now blinking.\n\r", MAX_MESSAGE_SIZE);

            } else {

                GLCD_clear(); // clear the screen
                //row 1
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                //row 2
                GLCD_putchar(13); // 'M'
                GLCD_putchar(5); // 'E'
                GLCD_putchar(18); // 'R'
                GLCD_putchar(18); // 'R'
                GLCD_putchar(25); // 'Y'
                GLCD_putchar(0); // space
                GLCD_putchar(24); // 'X'
                GLCD_putchar(13); // 'M'
                GLCD_putchar(1); // 'A'
                GLCD_putchar(19); // 'S'
                GLCD_putchar(29); // '!'
                GLCD_putchar(27); // left smiley
                GLCD_putchar(28); // right smiley
                GLCD_putchar(0); // space
                //row 3
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                GLCD_putchar(0); // space
                //row 4
                GLCD_putchar(14); // 'N'
                GLCD_putchar(15); // 'O'
                GLCD_putchar(20); // 'T'
                GLCD_putchar(0); // space
                GLCD_putchar(18); // 'R'
                GLCD_putchar(5); // 'E'
                GLCD_putchar(3); // 'C'
                GLCD_putchar(15); // 'O'
                GLCD_putchar(7); // 'G'
                GLCD_putchar(14); // 'N'
                GLCD_putchar(9); // 'I'
                GLCD_putchar(26); // 'Z'
                GLCD_putchar(5); // 'E'
                GLCD_putchar(4); // 'D'

                // Else message was not recognized
                strncpy((char*)response, "Task not recognized.\n\r", MAX_MESSAGE_SIZE);
            }

            // Send the response message to laptop
            HAL_UART_Transmit(&huart2, response, strlen(response), UART_DELAY);

            // Zero out message array and response array to get ready for the next message
            memset(message,  0, sizeof(message));
            memset(response, 0, sizeof(response));
            buffer_position = 0;
        }

        // Restart UART2's receive interrupt to wait for next byte
        HAL_UART_Receive_IT(&huart2, &uart2_byte, 1); //start next byte receive interrupt
    }
}
void SPI_write(unsigned char data){
// Chip Enable (low is asserted)
HAL_GPIO_WritePin(CE_PORT, CE_PIN, GPIO_PIN_RESET);
// Send data over SPI1
HAL_SPI_Transmit(&hspi1, (uint8_t*) &data, 1, HAL_MAX_DELAY);
// Chip Disable
HAL_GPIO_WritePin(CE_PORT, CE_PIN, GPIO_PIN_SET);
}
void GLCD_data_write(unsigned char data){
// Switch to "data" mode (D/C pin high)
HAL_GPIO_WritePin(DC_PORT, DC_PIN, GPIO_PIN_SET);
// Send data over SPI
SPI_write(data);
}
void GLCD_command_write(unsigned char data){
// Switch to "command" mode (D/C pin low)
HAL_GPIO_WritePin(DC_PORT, DC_PIN, GPIO_PIN_RESET);
// Send data over SPI
SPI_write(data);
}
void GLCD_init(void){
// Keep CE high when not transmitting
HAL_GPIO_WritePin(CE_PORT, CE_PIN, GPIO_PIN_SET);
// Reset the screen (low pulse - down & up)
HAL_GPIO_WritePin(RESET_PORT, RESET_PIN, GPIO_PIN_RESET);
HAL_GPIO_WritePin(RESET_PORT, RESET_PIN, GPIO_PIN_SET);
// Configure the screen (according to the datasheet)
GLCD_command_write(0x21); // enter extended command mode
GLCD_command_write(0xB0); // set LCD Vop for contrast (this may be adjusted)
GLCD_command_write(0x04); // set temp coefficient
GLCD_command_write(0x15); // set LCD bias mode (this may be adjusted)
GLCD_command_write(0x20); // return to normal command mode
GLCD_command_write(0x0C); // set display mode normal
}
void GLCD_setCursor(unsigned char x, unsigned char y){
GLCD_command_write(0x80 | x); // column
GLCD_command_write(0x40 | y); // bank
}
void GLCD_clear(void){
int i;
for(i = 0; i < (GLCD_WIDTH * NUM_BANKS); i++){
GLCD_data_write(0x00); // write zeros
}
GLCD_setCursor(0,0); // return cursor to top left
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
  while (1)
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
