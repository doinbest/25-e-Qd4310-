/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "headfile.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t g_oledstring[20];
uint8_t g_usart4_receivedata;

uint8_t pc0_mode;
uint8_t pc1_mode;
uint8_t pc2_mode;
uint8_t pc3_mode;

float battery_voltage;

uint8_t Car_Mode = Run_Mode; // 运行模式，默认为正常模式

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
	delay_init(72);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM6_Init();
  MX_ADC2_Init();
  MX_TIM7_Init();
  MX_ADC1_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
	delay_ms(100);    //延时一下，让OLED正常输出
	OLED_Init();
	delay_ms(100);    //延时一下，让OLED正常输出
  OLED_Clear(); //因为是直接进来的，所以清除一下比较好

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);   //开启TIM2的编码器接口模式 ENA
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);   //开启TIM4的编码器接口模式 ENB
  HAL_TIM_Base_Start_IT(&htim5);
  HAL_UART_Receive_IT(&huart1, &UART1_RxByte, 1);
  maixcam_Init(&huart3);
  HAL_UART_Receive_IT(&huart4, &g_usart4_receivedata, 1); // 串口4接收数据中断

  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);
  Motor_Init();
  Encoder_Init();
  ZigbeeGrey_Init();
  JY61p_HardwareZeroYaw();
  PID_param_init();

  protocol_init(); // 初始化协议栈
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    key_scan(); // 如果 key_scan 里面还会控制 car_go，后面要注意
    ZigbeeGrey_Task_WithTick();
    Protocol_Datas_Proc();

    sprintf((char *)g_oledstring, "Car_Mode:%d", Car_Mode);
    OLED_ShowString(0, 0, g_oledstring, 12);

    sprintf((char *)g_oledstring, "distance:%4.2f", Encoder_GetDistanceMm());
    OLED_ShowString(0, 1, g_oledstring, 12);

    sprintf((char *)g_oledstring, "state:%d", drug_state);
    OLED_ShowString(0, 2, g_oledstring, 12);

    sprintf((char *)g_oledstring, "car_go:%d", car_go);
    OLED_ShowString(0, 3, g_oledstring, 12);

    sprintf((char *)g_oledstring, "white:%4d", ZigbeeGrey_White[0]);
    OLED_ShowString(0, 4, g_oledstring, 12);

    sprintf((char *)g_oledstring, "black:%4d", ZigbeeGrey_Black[0]);
    OLED_ShowString(0, 5, g_oledstring, 12);
    
    sprintf((char *)g_oledstring, "yaw:%f", Yaw);
    OLED_ShowString(0, 6, g_oledstring, 12);



    // /*
    //  * 1. 发车前：识别目标数字
    //  */
    if (car_go == 0 && target_number == 0)
    {
      maix_state = 0; // 发送 0x01，识别目标数字
    }

    /*
     * 2. 已经识别到目标数字，但还没发车
     * 现在这里不是“等待按键发车”，而是等待放药发车
     */
    else if (car_go == 0 && target_number != 0)
    {
      maix_state = 2; // 空闲，等待 drug_scan 判断放药/取药
    }

    /*
     * 药品状态机
     * 负责：
     * 起点放药 -> car_go = 1
     * 病房取药 -> car_go = 1 并跳到倒车状态
     */
    drug_scan();

    /*
     * 3. 发车后，根据目标数字执行路线
     */
    if (car_go == 1)
    {
      if (target_number == 1)
      {
        maix_state = 2;
        task_1();
      }
      else if (target_number == 2)
      {
        maix_state = 2;
        task_2();
      }
      else if (target_number >= 3 && target_number <= 8)
      {
        task_search_middle_then_far();
      }
      else
      {
        maix_state = 2;

        Location_flag = 0;
        Angle_flag = 0;
        Gray_flag = 0;

        Motor_SetSpeed(MOTOR_LEFT, 0);
        Motor_SetSpeed(MOTOR_RIGHT, 0);
      }
    }

    MaixCAM_Proc();

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    // 1. 调用野火协议解析函数，传入刚才接收到的1个字节
    // 该函数内部会在解析到完整一帧后，把 receive_cmd 置为 1
    protocol_data_recv(&UART1_RxByte, 1);

    // // 2. 再次开启中断接收下一个字节
    HAL_UART_Receive_IT(&huart1, &UART1_RxByte, 1);
  }
  if(huart->Instance == USART3) // 判断中断源
  {
    maixcam_RxCpltCallback(huart);
    // maixcam_DecToBin(maixcam_data_buff[2], trace_value); // 将接收到的十进制数转换为二进制数组
  }

  if (huart->Instance == UART4) // 判断中断源
  {
    jy61p_ReceiveData(g_usart4_receivedata);                // 调用数据包处理函数
    HAL_UART_Receive_IT(&huart4, &g_usart4_receivedata, 1); // 继续进行中断接收
  }
}

/* 错误回调函数，处理溢出、帧错误等情况 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    // 如果出错（比如上位机发太快导致 ORE 溢出），不要锁死，重新开启接收即可
    HAL_UART_Receive_IT(&huart1, &UART1_RxByte, 1);
  }
  else if (huart->Instance == UART4)
  {
    __HAL_UART_CLEAR_OREFLAG(huart);
    HAL_UART_Receive_IT(&huart4, &g_usart4_receivedata, 1);
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
