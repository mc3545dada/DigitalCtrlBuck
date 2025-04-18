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
#include "adc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

#include "OLED.h"
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
struct
{
  double kp;
  double target;

  double kd;
  double err;
  double err_last;

  double ki;
  double inte;
  double inte_max;

  double deadzone;

}PID_data;

char str1[16];
char str2[16];
char str3[16];
char str4[16];
char str5[16];
int16_t test1 ;
float TargetVol;
float CurrentVol;
float CurrentPWM;
float value;
int8_t BuckState;
int8_t ProtectState;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  //0.001s一次
  if (htim->Instance == TIM2)
  {
    //设置占空比
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,CurrentPWM);//左
    //采集电压
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 1000);
    value = HAL_ADC_GetValue(&hadc1);
    //三元线性拟合
    CurrentVol = 0.812*(value /100)+0.000395*(value /100)*(value /100)-0.0000295*(value /100)*(value /100)*(value /100)+0.5945;

    //误差值计算
    PID_data.err = TargetVol - CurrentVol;
    //判断死区，防乱跳
    if (PID_data.err>=PID_data.deadzone||PID_data.err<=-PID_data.deadzone)
    {
      //使只当激活继电器时调节pid，不然关闭继电器后pwm马上会到最高
      if (ProtectState == 1)
      {
        //增量式pid调节
        CurrentPWM += PID_data.err * PID_data.kp;

        //限幅pwm
        if (CurrentPWM > 98)
        {
          CurrentPWM = 98;
        }
        else if (CurrentPWM < 0)
        {
          CurrentPWM = 0;
        }

      }

    }
    //闪灯（虽然1khz看不出来。。。）
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  }

}
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  OLED_Init();
  OLED_Clear();

  //初始参数设置
  TargetVol = 5;
  CurrentVol = 0;
  CurrentPWM = 50;
  BuckState = 0;
  ProtectState = 0;

  PID_data.kp = 1.0;
  PID_data.deadzone =0.10;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {



    //reset低0

    //mode按键控制继电器
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)==RESET)
    {
      HAL_Delay(10);
       while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)==RESET);
      ProtectState = ProtectState==1?0:1 ;
    }
    //激活继电器，开始降压
     if (ProtectState == 0)
     {
       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10,GPIO_PIN_RESET);
     }
     else if (ProtectState == 1)
     {
       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10,GPIO_PIN_SET);
     }

      //手动更改目标电压值
    //pb12加
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)==RESET)
    {
      HAL_Delay(10);
      while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)==RESET);
      TargetVol = TargetVol + 0.2;
    }
    //pb13减
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)==RESET)
    {
      HAL_Delay(10);
      while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)==RESET);
      TargetVol = TargetVol - 0.2;
    }

    //限幅pwm
    if (CurrentPWM > 98)
    {
      CurrentPWM = 98;
    }
    else if (CurrentPWM < 0)
    {
      CurrentPWM = 0;
    }


    //此为栅极驱动使能引脚当前电平状态，貌似没什么用
    BuckState = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);

    sprintf(str1,"%.2f\n",TargetVol);
    sprintf(str2,"%.2f\n",CurrentVol);
    sprintf(str3,"%.2f\n",CurrentPWM);
    sprintf(str4,"%d\n",BuckState);
    // sprintf(str4,"%.2f\n",PID_data.err);
    sprintf(str5,"%d\n",ProtectState);

    OLED_ShowString(1,1,"TargetVol",OLED_6X8);
    OLED_ShowString(1,13,"CurrentVol",OLED_6X8);
    OLED_ShowString(1,26,"CurrentPWM",OLED_6X8);
    OLED_ShowString(1,39,"BuckState",OLED_6X8);
    // OLED_ShowString(1,39,"Test",OLED_6X8);
    OLED_ShowString(1,52,"ProtectState",OLED_6X8);

    OLED_ShowString(79,1,str1,OLED_6X8);
    OLED_ShowString(79,13,str2,OLED_6X8);
    OLED_ShowString(79,26,str3,OLED_6X8);
    OLED_ShowString(79,39,str4,OLED_6X8);
    OLED_ShowString(79,52,str5,OLED_6X8);

    OLED_Update();

    //限制oled刷新速度
    HAL_Delay(10);
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
