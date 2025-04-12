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
#include "SEGGER_RTT.h"
#include <stdbool.h>
#include <stdio.h>
#include "TM1638.h"
#include "TM1638_platform.h"
#include "string.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUF_SIZE 256  // 定义接收缓冲区大�???
//#define BLINK_INTERVAL 500  // 小数点闪烁间隔，单位：毫�???
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
  uint32_t KeyS;
  uint32_t *Keys = &KeyS; // 修正为取 KeyS 的地�????????
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint32_t timer_counter = 0; // 用于计数的变�????????

// 双缓冲区配置
#define RX_BUF_SIZE 256  // 增大缓冲区防止溢�?
uint8_t rx_buf1[RX_BUF_SIZE];
uint8_t rx_buf2[RX_BUF_SIZE];
uint8_t *active_buf = rx_buf1;
uint16_t rx_index = 0;
volatile bool buf_ready = false;
uint8_t processing_buf[RX_BUF_SIZE];

// 新增标志位，用于标记是否接收到有效的 $GNRMC 语句
bool gnrmcReceived = false;

//标志位，用于标记啥时候运行数码管的显示部�?
bool tm1638_operate = false;

// 新增变量用于存储时间信息
int hours, minutes, seconds;

// 新增变量用于记录小数点状�???
bool decimalPointState = true;

// 新增标志位，用于记录是否定位成功
bool position_3d = 0;

// 新增变量用于记录小数点闪烁计�???
volatile uint32_t blinkCounter = 0;

TM1638_Handler_t Handler;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  TM1638_Platform_Init(&Handler);
  TM1638_Init(&Handler, TM1638DisplayTypeComAnode);
  TM1638_ConfigDisplay(&Handler, 4, TM1638DisplayStateON);

  // 使能定时器TIM3的更新中
  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
  // 启动定时器TIM3，开始计
  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // 处理准备好的UART数据
       if (buf_ready) {
    	   uint32_t primask = __get_PRIMASK();  // 保存当前中断状�??//�?个时钟周�?
           __disable_irq();  // 短暂禁用中断保护缓冲区切�?//�?个时钟周�?
           memcpy(processing_buf,
                 (active_buf == rx_buf1) ? rx_buf2 : rx_buf1, //这个刚好和UART中断里面�?前一后，processing_buf会取到和中断里active_buf相反的缓冲区，不记得的时候推演一下就清晰�?
                 RX_BUF_SIZE);//memcpy(256字节)	~800时钟周期
           buf_ready = false;

           __set_PRIMASK(primask);              // 恢复中断状�??  //�?个时钟周�?
          // __enable_irq();                  // 如果是没有顺序需要的话，可以不使用__set_PRIMASK(primask); �?个时钟周�?
         //整个临界区约810个周期（17μs），远小于UART字节间隔�?115200bps时为87μs/字节�?,�?以不会丢失数据，不用保存中断状�?�直接禁�?/�?启中�? 也应该问题不�?

           // 打印原始数据（调试用�?
          // SEGGER_RTT_WriteString(0, "UART Data: ");
          // SEGGER_RTT_WriteString(0, (char*)processing_buf);

           // 解析GNRMC数据
           char* gnrmc_ptr = strstr((char*)processing_buf, "$GNRMC");
           if (gnrmc_ptr != NULL) {
               if (sscanf(gnrmc_ptr + 7, "%2d%2d%2d", &hours, &minutes, &seconds) == 3) {
             hours = (hours + 8) % 24; //切换�?下东八区
             gnrmcReceived = true;
               }
           }
       }

       if (gnrmcReceived) {

     //接收到新的消息要干的�?
           gnrmcReceived = false;
     }


if(tm1638_operate)
      {

     	    // 处理3D定位状�?�显�?
     	    TM1638_SetSingleDigit_HEX(&Handler, position_3d ? 3 : 0, 2);
     	    TM1638_SetSingleDigit_HEX(&Handler, position_3d ? 0x0D : 0, 1);

     	    // 处理按键
     	    TM1638_ScanKeys(&Handler, Keys);
     	    if (*Keys & 0x0001) {
     	        TM1638_SetSingleDigit_HEX(&Handler, 7, 0);
     	        HAL_Delay(200);
     	    }
     	    if (*Keys & 0x0002) {
     	        TM1638_SetSingleDigit(&Handler, 0, 0);
     	        HAL_Delay(200);
     	    }

     	    // 显示时间数据

     	    // 将时间显示在数码管后四位，修正显示顺�???
     	    TM1638_SetSingleDigit_HEX(&Handler, minutes % 10, 4);  // 分钟个位
     	    TM1638_SetSingleDigit_HEX(&Handler, minutes / 10, 5);  // 分钟十位
     	    //TM1638_SetSingleDigit_HEX(&Handler, hours % 10, 6);  // 小时个位
     	    // 根据小数点状态设置第 6 位小数点
     	    if (decimalPointState) {
     	        TM1638_SetSingleDigit_HEX(&Handler, hours %10 | TM1638DecimalPoint, 6);
     	    } else {
     	        TM1638_SetSingleDigit_HEX(&Handler, hours % 10, 6);
     	    }
     	    TM1638_SetSingleDigit_HEX(&Handler, hours / 10, 7);  // 小时十位


     	    decimalPointState = !decimalPointState;  // 切换小数点状,下次就会生效
     	    tm1638_operate = false;                  // �?出循环，等待TIM3�?启为True
        }
  }







    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//  TM1638_DeInit(&Handler);
//  return 0;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
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
  htim3.Init.Prescaler = 4799;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  // �???启接收中�???
  // 确保UART中断优先级高于定时器中断
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  HAL_UART_Receive_IT(&huart1, &active_buf[rx_index], 1);
  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 38400;
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB4 PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// 定时器中断处理函�????????
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim3) {

        timer_counter++;
        blinkCounter++;


        if (timer_counter >= 5) {
          //  SEGGER_RTT_WriteString(0, "Timer interrupt occurred!\n");
            timer_counter = 0;
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

    	    position_3d = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11);
    	    //SEGGER_RTT_printf(0, "position_3d Voltage: %d\r\n", (int)position_3d);
        }

        if (blinkCounter >= 10) {  // 50 是主循环中的延时时间
            blinkCounter = 0;
            tm1638_operate = true;

        }

    }
}


/* UART错误处理 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart1) {
        uint32_t errors = huart->Instance->ISR;

        if (errors & USART_ISR_ORE) SEGGER_RTT_WriteString(0, "UART Overrun\n");
        if (errors & USART_ISR_NE) SEGGER_RTT_WriteString(0, "UART Noise\n");
        if (errors & USART_ISR_FE) SEGGER_RTT_WriteString(0, "UART Framing\n");
        if (errors & USART_ISR_PE) SEGGER_RTT_WriteString(0, "UART Parity\n");

        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);

        rx_index = 0;
        HAL_UART_Receive_IT(&huart1, &active_buf[rx_index], 1);
    }
}
// UART接收中断回调函数
/* UART中断回调 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart1) {
        uint8_t byte = active_buf[rx_index];
        rx_index++;

        if (byte == '\n' || rx_index >= RX_BUF_SIZE - 1) {
            active_buf[rx_index] = '\0';
            buf_ready = true;
            rx_index = 0;
            active_buf = (active_buf == rx_buf1) ? rx_buf2 : rx_buf1;//切换缓冲�?
            SEGGER_RTT_WriteString(0, "UART Data: ");
            SEGGER_RTT_WriteString(0, (char*)active_buf);
        }

        HAL_UART_Receive_IT(&huart1, &active_buf[rx_index], 1);//继续下一个字节的接收
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
