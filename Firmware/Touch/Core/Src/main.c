/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "math.h"


#include "oled.h"
#include "w25qxx.h"
#include "mpu6050.h"
#include "nrf.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

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
#define USE_DMP_EN    1
__IO uint8_t mpu_flag = 0;
char str_buf[120];

void test_mpu(uint8_t i)
{
    float pitch,roll,yaw;       //欧拉角
    short accx,accy,accz;       //加速度传感器原始数据
    short gyrox,gyroy,gyroz;    //陀螺仪原始数据
    short temp;                 //温度

    uint8_t t;
    for(t=0; t<i; t++)
    {
        if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
        {
            temp=MPU_Get_Temperature();                 //得到温度值
            MPU_Get_Accelerometer(&accx,&accy,&accz);   //得到加速度传感器数据
            MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);    //得到陀螺仪数据
        }
        sprintf(str_buf,"A:%d,%d,%d",accx,accy,accz);
        OLED_Show_Str(0,16,str_buf,12,1);
        sprintf(str_buf,"G:%d,%d,%d",gyrox,gyroy,gyroz);
        OLED_Show_Str(0,40,str_buf,12,1);
        OLED_Refresh_Gram();
        HAL_Delay(50);
    }
}

void test_fun(void)
{
    int i;
#if OLED_MOVE_DEMO_EN
    OLED_Clear(0);
    for(i=0; i<46; i++)
    {
        OLED_Show_Str(10,16 * i + 10,(char *)articl[i],16,1);
    }

    OLED_OFFSET_Y = 0;
    OLED_DrawRectangle(0 + 5,0 + 5,OLED_PAGE_H * OLED_W - 1 - 5,OLED_PAGE_V * OLED_H - 1 - 5,1);
    OLED_Draw_Circle(128,128,63,1);
    OLED_Refresh_Gram();

    while(1)
    {
        
        //oled_move_dir = !oled_move_dir;
        for(i=0; i<(OLED_PAGE_V - 1) * OLED_H; i++)
        {
            OLED_CUR_POS_Y = i;
            OLED_CUR_POS_X = 0;
            OLED_Refresh_Gram();
            HAL_Delay(25);
        }
        for(i=(OLED_PAGE_V - 1) * OLED_H - 1; i>0; i--)
        {
            OLED_CUR_POS_Y = i;
            OLED_CUR_POS_X = 0;
            OLED_Refresh_Gram();
            HAL_Delay(5);
        }
        for(i=0; i<(OLED_PAGE_H - 1) * OLED_W; i++)
        {
            OLED_CUR_POS_Y = 0;
            OLED_CUR_POS_X = i;
            OLED_Refresh_Gram();
            HAL_Delay(25);
        }
        for(i=0; i<(OLED_PAGE_V - 1) * OLED_H; i++)
        {
            OLED_CUR_POS_Y = i;
            OLED_CUR_POS_X = 128;
            OLED_Refresh_Gram();
            HAL_Delay(15);
        }
        for(i=(OLED_PAGE_V - 1) * OLED_H - 1; i>0; i--)
        {
            OLED_CUR_POS_Y = i;
            OLED_CUR_POS_X = 128;
            OLED_Refresh_Gram();
            HAL_Delay(5);
        }
        for(i=(OLED_PAGE_H - 1) * OLED_W; i>0; i--)
        {
            OLED_CUR_POS_Y = 0;
            OLED_CUR_POS_X = i;
            OLED_Refresh_Gram();
            HAL_Delay(15);
        }

        for(i=0; i<360; i+=2)
        {
            OLED_CUR_POS_Y = 96 + (sin(i * 3.14 / 180) * 63);
            OLED_CUR_POS_X = 64 + (cos(i * 3.14 / 180) * 63);
            OLED_Refresh_Gram();
            HAL_Delay(20);
        }
    }
#endif
MPUinit:
    Set_MPU(1);
    i = MPU_Init();
    if(i != 0)
    {
        sprintf(str_buf,"MPU1:%3d",i);
        OLED_Show_Str(0,32,str_buf,16,1);
        OLED_Refresh_Gram();
        HAL_Delay(100);
        goto MPUinit;
    }
#if USE_DMP_EN
DMPinit:
    i = mpu_dmp_init();
    if(i != 0)
    {
        sprintf(str_buf,"DMP1:%3d",i);
        OLED_Show_Str(0,32,str_buf,16,1);
        OLED_Refresh_Gram();
        HAL_Delay(100);
        goto DMPinit;
    }
#endif
    mpu_flag = 1;
    if(1)test_mpu(100);
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    int i;
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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
    if(0)
    {
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
    __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_4,50);
    }
    OLED_Init();
    W25QXX_Init();
    OLED_Show_Str(0,0,"正常模式OLED",16,1);
    OLED_Refresh_Gram();
    test_fun();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      sendPower((i ++) % 40 + 60,100);
      HAL_Delay(100);
      if(0)
      {
          for(i=0;i<1000;i++)
          {
              __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_4,i);
              HAL_Delay(1);
          }
          for(i=999;i>0;i--)
          {
              __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_4,i);
              HAL_Delay(1);
          }
      }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

/* USER CODE BEGIN 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
