/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "svpwm.h"
#include "arm_math.h"
#include "retarget.h"
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
uint16_t t;
uint16_t  encValue = 0,encOld = 0;
uint8_t speedLoopFlag;
int16_t encDiff;
uint8_t pwmSwitch = 0;
float sineWave_Data[200];
uint16_t Tcm[3];
uint8_t sector;
float32_t Ua, Ub, Ia, Ib;
float32_t Ualpha, Ubeta, Ialpha, Ibeta;
float32_t Ud, Uq, Id, Iq;
float32_t theta_m, theta_e;
float32_t omega_m, omega_e;
uint16_t adcValue1 = 0;
uint16_t adcValue2 = 0;

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */
    mystdioInit(&huart1);
    for (uint16_t n = 0; n < 200; n++)
        sineWave_Data[n] = arm_sin_f32(2.0f * n * PI / 200);
    TIM8->CCR4 = 8000 - 3;
    TIM8->CCR1 = 8000-1000;
    TIM8->ARR = 8000 - 1;

    HAL_TIM_Base_Start(&htim8);
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_ADC_Start_DMA(&hadc2,(uint32_t *)&adcValue1,1);
    HAL_ADC_Start_DMA(&hadc3,(uint32_t *)&adcValue2,1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
        if(pwmSwitch)
        {
            HAL_GPIO_WritePin(Led0_GPIO_Port, Led0_Pin, GPIO_PIN_RESET);
            HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);
            HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
            HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
            HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
            HAL_TIMEx_PWMN_Start(&htim8,TIM_CHANNEL_1);
            HAL_TIMEx_PWMN_Start(&htim8,TIM_CHANNEL_2);
            HAL_TIMEx_PWMN_Start(&htim8,TIM_CHANNEL_3);
        }
        else
        {
            HAL_GPIO_WritePin(Led0_GPIO_Port, Led0_Pin, GPIO_PIN_SET);
            HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_4);
            HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);
            HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_2);
            HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_3);
            HAL_TIMEx_PWMN_Stop(&htim8,TIM_CHANNEL_1);
            HAL_TIMEx_PWMN_Stop(&htim8,TIM_CHANNEL_2);
            HAL_TIMEx_PWMN_Stop(&htim8,TIM_CHANNEL_3);

        }
        printf("%f,%f\r\n", (float32_t)adcValue1 * 3.31f / 4095, (float32_t)adcValue2 * 3.31f / 4095);
//        printf("%d,%d,%f,%f\r\n", encValue, encDiff, omega_m, omega_e);
//        printf("%f,%f,%f,%f\r\n", theta_m, theta_e, omega_m, omega_e);
//        for (uint8_t n = 0; n < 200; n++)
//        {
//            U_alpha = 50 * sineWave_Data[(n + 200 / 4) % 200];
//            U_beta = 50 * sineWave_Data[n];
//            sector = svpwm(U_alpha, U_beta, 16000, 310, Tcm);
//            printf("%.10f,%.10f,%d,%d,%d,%d\r\n",Ua, Ub,Tcm[0], Tcm[1], Tcm[2], sector);
//        }

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 160;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */
//void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
//{
//    if(htim == &htim8 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
//    {
//        t++;
//        if(t > 199)
//            t = 0;
//        Ua = 10 * sineWave_Data[t];
//        Ub = 10 * sineWave_Data[(t + 200 * 2 / 3) % 200];
//        arm_clarke_f32(Ua, Ub, &Ualpha, &Ubeta);
//        arm_park_f32(Ualpha, Ubeta, &Ud, &Uq, arm_sin_f32(2 * PI * (float32_t) t / 200.0f),
//                     arm_cos_f32(2 * PI * (float32_t) t / 200.0f));
//        sector = svpwm(Ualpha, Ubeta, 16000, 320, Tcm);
//        TIM8->CCR1 = Tcm[0];
//        TIM8->CCR2 = Tcm[1];
//        TIM8->CCR3 = Tcm[2];
//        speedLoopFlag++;
//        if(speedLoopFlag > 39)
//        {
//            int16_t encDiffTmp[3];
//            speedLoopFlag = 0;
//            encValue = __HAL_TIM_GET_COUNTER(&htim1);
//            encDiffTmp[0] = (int16_t)(encValue - encOld);
//            encDiffTmp[1] = (int16_t)(encDiffTmp[0] + 5000);
//            encDiffTmp[2] = (int16_t)(encDiffTmp[0] - 5000);
//            for(uint8_t i = 0;i<3;i++)
//            {
//                if(encDiffTmp[i] > -2500 && encDiffTmp[i] < 2500 )
//                    encDiff = encDiffTmp[i];
//            }
//            encOld = encValue;
//            omega_m = PI * (float32_t) (encDiff) / 10.0f;
//            omega_e = 4 * omega_m;
//            theta_m = 2 * PI * (float32_t) (encValue % 5000) / 5000.0f;
//            theta_e = 2 * PI * (float32_t) (encValue % 1250) / 1250.0f;
//        }
//
//    }

//}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc == &hadc3) {
        t++;
        if (t > 199)
            t = 0;
        Ua = 10 * sineWave_Data[t];
        Ub = 10 * sineWave_Data[(t + 200 * 2 / 3) % 200];
        arm_clarke_f32(Ua, Ub, &Ualpha, &Ubeta);
        arm_park_f32(Ualpha, Ubeta, &Ud, &Uq, arm_sin_f32(2 * PI * (float32_t) t / 200.0f),
                     arm_cos_f32(2 * PI * (float32_t) t / 200.0f));
        sector = svpwm(Ualpha, Ubeta, 16000, 320, Tcm);
        TIM8->CCR1 = Tcm[0];
        TIM8->CCR2 = Tcm[1];
        TIM8->CCR3 = Tcm[2];
        speedLoopFlag++;
        if (speedLoopFlag > 39) {
            int16_t encDiffTmp[3];
            speedLoopFlag = 0;
            encValue = __HAL_TIM_GET_COUNTER(&htim1);
            encDiffTmp[0] = (int16_t) (encValue - encOld);
            encDiffTmp[1] = (int16_t) (encDiffTmp[0] + 5000);
            encDiffTmp[2] = (int16_t) (encDiffTmp[0] - 5000);
            for (uint8_t i = 0; i < 3; i++) {
                if (encDiffTmp[i] > -2500 && encDiffTmp[i] < 2500)
                    encDiff = encDiffTmp[i];
            }
            encOld = encValue;
            omega_m = PI * (float32_t) (encDiff) / 10.0f;
            omega_e = 4 * omega_m;
            theta_m = 2 * PI * (float32_t) (encValue % 5000) / 5000.0f;
            theta_e = 2 * PI * (float32_t) (encValue % 1250) / 1250.0f;
        }
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == Key0_Pin)
        pwmSwitch = ~pwmSwitch;
    if(GPIO_Pin == EncPhaseZ_Pin)
        __HAL_TIM_SET_COUNTER(&htim1,0);
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
