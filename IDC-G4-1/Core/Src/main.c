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
#define RX_BUF_SIZE 200  // 定义接收缓冲区大�?
//#define BLINK_INTERVAL 500  // 小数点闪烁间隔，单位：毫�?
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
  uint32_t KeyS;
  uint32_t *Keys = &KeyS; // 修正为取 KeyS 的地�??????
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint32_t timer_counter = 0; // 用于计数的变�??????

uint8_t rx_buf[RX_BUF_SIZE];
uint16_t rx_index = 0;  // 接收数据的索�?

// 新增标志位，用于标记是否接收到有效的 $GNRMC 语句
bool gnrmcReceived = false;
// 新增变量用于存储时间信息
int hours, minutes, seconds;

// 新增变量用于记录小数点状�?
bool decimalPointState = true;

bool position_3d = 0;
// 新增变量用于记录小数点闪烁计�?
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
    // 阻塞接收
//     HAL_UART_Receive(&huart1, rx_buf, sizeof(rx_buf) - 1, HAL_MAX_DELAY);
//     SEGGER_RTT_WriteString(0, "Received via UART: ");
//     SEGGER_RTT_WriteString(0, (char*)rx_buf);
//     SEGGER_RTT_WriteString(0, "\n");
//
//     memset(rx_buf, 0, sizeof(rx_buf)); // 清空缓冲准备下一次接�??????

	  if(position_3d)
	  {

		  TM1638_SetSingleDigit_HEX(&Handler, 3, 2);  // 小时十位
		  TM1638_SetSingleDigit_HEX(&Handler, 0x0D, 1);  // 小时十位

	  }

	  else
	  {

		  TM1638_SetSingleDigit(&Handler, 0, 2);  // 小时十位
		  TM1638_SetSingleDigit(&Handler, 0, 1);  // 小时十位

	  }








      TM1638_ScanKeys(&Handler, Keys);

//    // �?????? 16 位二进制形式打印按键�??????
//    SEGGER_RTT_WriteString(0, "Scanned key value (16-bit binary): ");
//
//    for (int i = 15; i >= 0; i--) {
//        SEGGER_RTT_PutChar(0, (*Keys & (1 << i)) ? '1' : '0');
//    }
//    SEGGER_RTT_WriteString(0, "\n");

    // 判断按键第一位是否按�??????
    if (*Keys & 0x0001) {
        // 若按下，数码管第�??????位显�?????? 7
        TM1638_SetSingleDigit_HEX(&Handler, 7, 0);
        HAL_Delay(800);
    }

    // 判断按键第一位是否按�??????
    if (*Keys & 0x0002) {
        // 若按下，数码管第�??????位显�?????? 7
        TM1638_SetSingleDigit(&Handler, 0, 0);//没有HEX尾缀的这个函数，输入0的话是关闭数码管
        HAL_Delay(800);
    }

    // TM1638_ConfigDisplay(&Handler, 5, TM1638DisplayStateOFF);
    // 假设数码管有 8 位，逐个设置每个数码管为熄灭状�??
    //    for (int i = 0; i < 8; i++) {
    //        TM1638_SetSingleDigit_HEX(&Handler, 0, i);
    //    }


    // �?查是否接收到有效�? $GNRMC 语句
    if (gnrmcReceived) {
        // 转换为东八区时间
        hours = (hours + 8) % 24;

        // 将时间显示在数码管后四位，修正显示顺�?
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



        // 重置标志�?
        gnrmcReceived = false;
    }

  }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  TM1638_DeInit(&Handler);
  return 0;
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
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
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
  // �?启接收中�?
  HAL_UART_Receive_IT(&huart1, &rx_buf[rx_index], 1);
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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// 定时器中断处理函�??????
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim3) {

        timer_counter++;
        blinkCounter++;

        if (timer_counter >= 5) {
            // 在这里添�?????? 1 秒间隔执行的代码
          //  SEGGER_RTT_WriteString(0, "Timer interrupt occurred!\n");
            timer_counter = 0;
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

    	    position_3d = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11);
    	    SEGGER_RTT_printf(0, "position_3d Voltage: %d\r\n", (int)position_3d);


        }

        if (blinkCounter >= 7) {  // 50 是主循环中的延时时间
            blinkCounter = 0;
            decimalPointState = !decimalPointState;  // 切换小数点状�?
            SEGGER_RTT_WriteString(0, "occurred!\n");
        }
    }
}

// UART接收中断回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart1) {
        if (rx_index < RX_BUF_SIZE - 1) {
            if (rx_buf[rx_index] == '\n') {
                // 判断是否�? $GNRMC 语句
                if (strncmp((const char*)rx_buf, "$GNRMC", 6) == 0) {
                    sscanf((const char*)rx_buf + 7, "%2d%2d%2d", &hours, &minutes, &seconds);
                    // 设置标志位，表示已接收到有效�? $GNRMC 语句
                    gnrmcReceived = true;
                }

                // 接收到换行符，打印接收到的数�?
                SEGGER_RTT_WriteString(0, "Received via UART: ");
                SEGGER_RTT_WriteString(0, (char*)rx_buf);
                SEGGER_RTT_WriteString(0, "\n");

                // 清空缓冲�?
                memset(rx_buf, 0, sizeof(rx_buf));
                rx_index = 0; // 确保rx_index重置�?0
            } else {
                // 接收�?个字节数�?
                rx_index++;
            }
            // 继续�?启下�?个字节的接收中断
            HAL_UART_Receive_IT(&huart1, &rx_buf[rx_index], 1);
        }
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
