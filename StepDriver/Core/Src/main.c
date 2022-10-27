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
#include "dma.h"
#include "gpio.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "StepHelper.h"
#include "printf.h"
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
stepTypedef steplist[5];

uint8_t spiByteBuff;
uint8_t Buff[16];
int spiReceiveFlag = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void _putchar(char character)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)&character, 1, HAL_MAX_DELAY);
}

void Data_ReFormatData(char* array, int len)
{
    char record;
    char temp[len];
    int j = 0;

    if (array[0] == 0x55) {
        return;
    } else {
        for (int i = 0;; i++) {
            if (array[i] == 0x55) {
                record = i;
                break;
            }
        }
        for (int i = record; i < len; i++) {
            temp[j++] = array[i];
        }
        for (int i = 0; i < record; i++) {
            temp[j++] = array[i];
        }
        for (int i = 0; i < len; i++) {
            array[i] = temp[i];
        }
    }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* hspi)
{

    uint32_t startSpd = 0, finalSpd = 0, accTime = 0, StepNum = 0;
    uint8_t dir = 0, useDec = 0;

    if (hspi == &hspi1) {
        // Check buffer
        if (Buff[0] != 0x55 && Buff[15] != 0xAA) {
            Data_ReFormatData((char*)Buff, 16);
        }

        // Get ID
        uint8_t motorID = Buff[2];

        // Get Instruction
        uint8_t instruction = Buff[1];

        switch (instruction) {
        case 0x00: // Reset
            HAL_SPI_DMAStop(&hspi1);

            for (int i = 0; i < 5; i++) {
                Step_Init(&steplist[i], steplist[i].phtim, steplist[i].channel, 100, 7000, 5000);
            }

            HAL_SPI_Receive_DMA(&hspi1, Buff, 16);
            break;
        case 0x01: // Init
            // get parameter
            startSpd = *(uint32_t*)&Buff[3];
            finalSpd = *(uint32_t*)&Buff[7];
            accTime = *(uint32_t*)&Buff[11];

            // init
            Step_Init(&steplist[motorID], steplist[motorID].phtim, steplist[motorID].channel, startSpd, finalSpd, accTime);
            break;
        case 0x02: // Drive
            // get parameter
            StepNum = *(uint32_t*)&Buff[3];
            dir = Buff[7];
            useDec = Buff[8];

            // init
            Step_Prefill(&steplist[motorID], StepNum, Decelerate_USE);
            HAL_TIM_PWM_PulseFinishedCallback(steplist[motorID].phtim);
            break;
        case 0x03: // Abort
            Step_Abort(&steplist[motorID]);
            break;

        default:
            break;
        }
    }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef* htim)
{
    for (int i = 0; i < 5; i++) {
        if (htim == steplist[i].phtim) {

            if (Step_IsBuffRdy(&steplist[i])) {
                HAL_TIM_PWM_Start_DMA(steplist[i].phtim, steplist[i].channel,
                    (uint32_t*)Step_GetCurBuffer(&steplist[i]),
                    (uint16_t)Step_BuffUsedLength(&steplist[i]));
                Step_BufferUsed(&steplist[i]);
            } else if (steplist[i].state == Stop) {
                HAL_TIM_PWM_Stop_DMA(steplist[i].phtim, steplist[i].channel);
                Step_Unlock(&steplist[i]);
            }

            Step_BuffFill(&steplist[i]);
        }
    }
}

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
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_SPI1_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM5_Init();
    /* USER CODE BEGIN 2 */
    Step_Init(&steplist[0], &htim1, TIM_CHANNEL_1, 100, 7000, 5000);
    Step_Init(&steplist[1], &htim2, TIM_CHANNEL_1, 100, 7000, 5000);
    Step_Init(&steplist[2], &htim3, TIM_CHANNEL_1, 100, 7000, 5000);
    Step_Init(&steplist[3], &htim4, TIM_CHANNEL_1, 100, 7000, 5000);
    Step_Init(&steplist[4], &htim5, TIM_CHANNEL_1, 100, 7000, 5000);

    HAL_SPI_Receive_DMA(&hspi1, Buff, 16);

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        HAL_Delay(500);
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
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 84;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
        | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM10 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM10) {
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

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
