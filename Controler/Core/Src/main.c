/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "printf.h"
#include "stdlib.h"
#include "motor.h"
#include "move.h"
#include "utils.h"
#include "PID/pid.h"
#include "74HC165/74HC165.h"
#include "Compass/QMC5883L.h"
#include "ST7735-HAL/st7735.h"
#include "ICM42605/ICM42605.h"
#include "StepHerlper/stepHelper.h"
#include "SerialParaChanger/SPChanger.h"
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
extern osSemaphoreId_t bQueuePutHandle;

spChanger_t paraKp, paraKi, paraKd;
char shBuff[128];
char chBuff;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void _putchar(char character) {
    HAL_UART_Transmit(&huart5, (uint8_t *) &character, 1, HAL_MAX_DELAY);
}

extern short uart_charPut(char *data, unsigned short len);
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_UART4_Init();
  MX_SPI3_Init();
  MX_UART5_Init();
  MX_UART8_Init();
  MX_I2C1_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM16_Init();
  MX_TIM3_Init();
  MX_SPI4_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_SPI1_Init();
  MX_TIM15_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
    // Encoders
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_2);

    // Motors
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    // Step Motors

    // Screen
    ST7735_Init();
    ST7735_SetRotation(1);
    ST7735_Backlight_On();
    ST7735_FillScreen(ST7735_WHITE);

    // Sensor Init
    int res = 0;
//    if (PMW3901_Init(&CarInfo.pmw) != 0)
//        while (1);
    while (QMC5883_Init() != 0);

    for (int i = 0; i < 100; i++) {
        QMC5883_GetData(&CarInfo.hmc);
        CarInfo.initYawOffset += atan2f(CarInfo.hmc.Mx, CarInfo.hmc.My) * 0.01;
    }

    if ((res = ICM42605_Init()) != 0)
        while (1)printf("ICM Init Error:%d\n", res);

    // Soft Init
    for (int i = 0; i < 4; i++) {
        PID_Init(&CarInfo.msPid[i], 10, 0.5f, 0);//0, 0.2, 0,
        PID_Init(&CarInfo.mpPid[i], 0.15f, 0, 0);// 0.033, 0, 0,
    }
    PID_Init(&CarInfo.cpPidX, 0, 0, 0);
    PID_Init(&CarInfo.cpPidY, 0, 0, 0);
    PID_Init(&CarInfo.aPid, 0, 0, 0);

    shell.write = uart_charPut;
    shellInit(&shell, shBuff, 128);

    // Interrupt
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_UART_Receive_IT(&huart5, (uint8_t *) &chBuff, 1);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
        Motor_Drive(0, 20, 1);
        Motor_Drive(1, 20, 1);
        Motor_Drive(2, 20, 1);
        Motor_Drive(3, 20, 1);
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
    if (htim->Instance == TIM6) {
        TIM_TypeDef *TIMx;
        static int16_t preCNT[4];
        static int16_t sumX, sumY;

        // Get Infrared
        HC165_Get_Data(&CarInfo.infrared);

        // Get compass
        QMC5883_GetData(&CarInfo.hmc);
        CarInfo.yaw = atan2f(CarInfo.hmc.Mx, CarInfo.hmc.My);
        AngRotate2zero(CarInfo.yaw, CarInfo.initYawOffset);
//        printf("data:%f,%f,%f\n", CarInfo.hmc.Mx, CarInfo.hmc.My, ToDig(atan2f(CarInfo.hmc.Mx, CarInfo.hmc.My)));
//
//        // Get gyro
        ICM42605_GetData(&CarInfo.icm, ICM_MODE_ACC | ICM_MODE_GYRO);

        static int cnt = 0;
        const int period = 10;
        cnt++;
        if (cnt == period) {
            cnt = 0;

            // Get position
            PMW3901_Read_Data(&CarInfo.pmw);
            CarInfo.dx = (float) -CarInfo.pmw.deltaX;
            CarInfo.dy = (float) -CarInfo.pmw.deltaY;
            VecRotate(CarInfo.dx, CarInfo.dy, CarInfo.yaw);
            CarInfo.curX += CarInfo.dx;
            CarInfo.curY += CarInfo.dy;

            // Get motor speed
            for (int16_t i = 0; i < 4; i++) {
                TIM_TypeDef *TIMptr[4] = {TIM8, TIM1, TIM4, TIM3};
                int16_t res = 0;
                if (i == 1 || i == 3)
                    CarInfo.spd[i] = -1 * ((int16_t)(TIMptr[i]->CNT) - preCNT[i]);
                else
                    CarInfo.spd[i] = (int16_t)((TIMptr[i]->CNT) - preCNT[i]);
                preCNT[i] = (int16_t)(TIMptr[i]->CNT);
            }

            // Get Motor Position
            for (int i = 0; i < 4; i++)
                CarInfo.psi[i] += CarInfo.spd[i];

            // Map PID
            if (CarInfo.cPsiCtr && !CarInfo.mPsiCtr) {
                CarInfo.spdX = PID_Realize(&CarInfo.cpPidX, CarInfo.curX);
                CarInfo.spdY = PID_Realize(&CarInfo.cpPidY, CarInfo.curY);
                MapSpeedSet(CarInfo.spdY, CarInfo.spdX);
            }

            // Attitude PID
            CarInfo.aPidOut = PID_RealizeForAngle(&CarInfo.aPid, CarInfo.yaw);

            // Motor PID
            for (int i = 0; i < 4; i++) {
                uint8_t dir = 1;

                // Optional position loop 
                if (CarInfo.mPsiCtr) {
                    CarInfo.spdStep = 0.5f;
                    float *outPtr = &CarInfo.mpPIDout[i];
                    float res = PID_Realize(&CarInfo.mpPid[i], CarInfo.psi[i]);

                    // Limit accelerate
                    if (res > *outPtr && res > 0 && *outPtr >= 0)
                        *outPtr = *outPtr < res ? *outPtr + CarInfo.spdStep : *outPtr;
                    else if (res < *outPtr && res < 0 && *outPtr <= 0)
                        *outPtr = *outPtr > res ? *outPtr - CarInfo.spdStep : *outPtr;
                    else *outPtr = res;

                    // Limit speed max
                    if (CarInfo.mpPIDout[i] > CarInfo.spdLimit)CarInfo.mpPIDout[i] = CarInfo.spdLimit;
                    if (CarInfo.mpPIDout[i] < -CarInfo.spdLimit)CarInfo.mpPIDout[i] = -CarInfo.spdLimit;
                }

                // Merge Motor Position PID and Attitude PID output
                switch (i) {
                    case 0:
                    case 2:
                        CarInfo.msPid[i].ctr.aim = CarInfo.mpPIDout[i] - CarInfo.aPidOut;
                        break;
                    case 1:
                    case 3:
                        CarInfo.msPid[i].ctr.aim = CarInfo.mpPIDout[i] + CarInfo.aPidOut;
                        break;
                }

                // Speed loop
                float duty = PID_Realize(&CarInfo.msPid[i], CarInfo.spd[i]);
                if (duty < 0) {
                    duty = -duty;
                    dir = 0;
                }
                Motor_Drive(i, duty, dir);
            }

            // Judge whether the car is static
            if (CarInfo.spd[0] != 0
                && CarInfo.spd[1] != 0
                && CarInfo.spd[2] != 0
                && CarInfo.spd[3] != 0)
                CarInfo.isCarMoving = CarInfo.isCarMoving << 1 | 1;
            else
                CarInfo.isCarMoving = CarInfo.isCarMoving << 1;
        }
    }
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM17) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
