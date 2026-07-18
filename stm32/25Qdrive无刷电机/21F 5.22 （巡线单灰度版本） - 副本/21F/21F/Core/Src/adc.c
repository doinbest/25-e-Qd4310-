/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
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
#include "adc.h"

/* USER CODE BEGIN 0 */
#include <string.h>

/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  HAL_ADCEx_Calibration_Start(&hadc1);

  /* USER CODE END ADC1_Init 2 */

}
/* ADC2 init function */
void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */
  HAL_ADCEx_Calibration_Start(&hadc2);

  /* USER CODE END ADC2_Init 2 */

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PA7     ------> ADC1_IN7
    */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
  else if(adcHandle->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspInit 0 */

  /* USER CODE END ADC2_MspInit 0 */
    /* ADC2 clock enable */
    __HAL_RCC_ADC2_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**ADC2 GPIO Configuration
    PC4     ------> ADC2_IN14
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC2_MspInit 1 */

  /* USER CODE END ADC2_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PA7     ------> ADC1_IN7
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_7);

  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
  else if(adcHandle->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspDeInit 0 */

  /* USER CODE END ADC2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC2_CLK_DISABLE();

    /**ADC2 GPIO Configuration
    PC4     ------> ADC2_IN14
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_4);

  /* USER CODE BEGIN ADC2_MspDeInit 1 */

  /* USER CODE END ADC2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/**************************************************************************
函数功能：AD 采样 (ADC2 专用，且固定使用初始化中配置好的通道14)
入口参数：无
返回  值：AD 转换结果 (0~4095)
**************************************************************************/
// uint16_t Get_Adc(void)
// {
//   // 1. 启动 ADC 转换 (使用初始化里默认配置好的通道)
//   HAL_ADC_Start(&hadc2);

//   // 2. 等待转换完成
//   HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);

//   // 3. 返回读取到的 ADC 值
//   return HAL_ADC_GetValue(&hadc2);
// }

/**************************************************************************
函数功能：读取电池电压
入口参数：无
返回  值：电池电压 单位 V
**************************************************************************/
// float Get_battery_volt(void)
// {
//   // 直接调用无参的 Get_Adc
//   uint16_t adc_val = Get_Adc();

//   // 根据分压公式计算实际电压
//   return (float)adc_val * 3.3f * 11.0f / 4096.0f;
// }

uint16_t adc_getValue()
{
  HAL_ADC_Start(&hadc1); // 先开启ADC
  HAL_ADC_PollForConversion(&hadc1, 1);

  return HAL_ADC_GetValue(&hadc1);
}

uint16_t Get_ADC_Average(uint8_t times)
{
  uint32_t ADC_Sum = 0;
  uint8_t i;
  for (i = 0; i < times; i++)
  {
    ADC_Sum += adc_getValue();
    HAL_Delay(5);
  }
  return ADC_Sum / times;
}

// /**
//  * @brief  ADC1 + DMA 采集 40 次并返回平均值
//  * @note   CubeMX 里 ADC1 必须开启 Continuous Conversion Mode
//  * @retval 0~4095 的 ADC 平均值
//  */
// unsigned int adc_getValue(void)
// {
//   uint32_t sum = 0;
//   uint32_t start_tick = 0;

//   adc_dma_done = 0;
//   adc_dma_error = 0;

//   memset(ADC_VALUE, 0, sizeof(ADC_VALUE));

//   if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_VALUE, ADC_DMA_SAMPLE_NUM) != HAL_OK)
//   {
//     return 0;
//   }

//   start_tick = HAL_GetTick();

//   while ((adc_dma_done == 0) && (adc_dma_error == 0))
//   {
//     if ((HAL_GetTick() - start_tick) > 5)
//     {
//       HAL_ADC_Stop_DMA(&hadc1);
//       return 0;
//     }

//     __WFI();
//   }

//   HAL_ADC_Stop_DMA(&hadc1);

//   if (adc_dma_error)
//   {
//     return 0;
//   }

//   for (uint8_t i = 0; i < ADC_DMA_SAMPLE_NUM; i++)
//   {
//     sum += ADC_VALUE[i];
//   }

//   return (uint16_t)(sum / ADC_DMA_SAMPLE_NUM);
// }

// /**
//  * @brief ADC DMA 采集完成回调
//  */
// void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
// {
//   if (hadc->Instance == ADC1)
//   {
//     adc_dma_done = 1;
//   }
// }

// /**
//  * @brief ADC 错误回调
//  */
// void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
// {
//   if (hadc->Instance == ADC1)
//   {
//     adc_dma_error = 1;
//   }
// }

/* USER CODE END 1 */
