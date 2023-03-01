/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/*
 * Copyright (C) 2023 Atomseek
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "sinwave.h" //���ұ�
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMP_SIZE 200 // һ��50hz���������ڣ�������pwm�������������ڿ���Ƶ�ʼ���ʱ��1��Ƶ�ʳ���50
#define LOG_SIZE 5000 // ���β���һ�β���������
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t key_state = 0;    // 1��ʾ����spwm
uint8_t charge_state = 0; // 1��ʾ�����
uint8_t switch_state = 0; // �������±�־λ

uint16_t pwm = 0;          // PWM�������ֵ
uint16_t count = 0;        // ��ʱ������
uint16_t pwm_half_arr = 0; // pwm��װֵ��һ�룬���ڲ���spwm����ļ���
uint16_t adc_value = 0;    // ����ADC���飬��һ���ǵ�ص�ѹ���ڶ����������ѹ

char RxBuffer = 0;            // ���ڽ��ջ�����
uint16_t adc_count = 0;       // ���β�������
uint16_t adc[LOG_SIZE] = {0}; // ���β�������

struct
{
    int set;        // �趨��ѹ
    float out;      // ˲ʱ�����ѹ
    float rms;      // �����ѹ��Чֵ
    float rms_show; // �����ѹ��ʾֵ
    float rms_sum;  // ������Чֵ�м���
} V = {0, 0, 0, 0, 0};

struct
{
    float kP;        // PID����ϵ��
    float kI;        // PID����ϵ��
    float kD;        // PID΢��ϵ��
    float integralE; // ������
    float deltaE;    // ��ǰ���
    float MI;        // ���Ʊ�Modulation Index�����ڿ������
} PID = {0, 0.001, 0, 0, 0, 0};

// const float sin_wave[SAMP_SIZE]={0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void USART1_printf(char *format, ...);                                      // printf����ӳ�䵽����
void Rotary(TIM_HandleTypeDef *htim, int *set, int min, int max, int step); // ��ť����
void PID_Controller();                                                      // PID������
void ADC_Collect();                                                         // ��ѹ�ɼ�
void OLED_Show();                                                           // ��Ϣ��ʾ����
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
    MX_DMA_Init();
    MX_I2C1_Init();
    MX_ADC1_Init();
    MX_TIM1_Init();
    MX_TIM3_Init();
    MX_USART1_UART_Init();
    /* USER CODE BEGIN 2 */

    OLED_Init(); // oled��ʼ��
    OLED_ShowString(0, 1, "Initializing", 16);
    OLED_Refreash();

    // ��ʼ����ʱ����PWM
    pwm_half_arr = (__HAL_TIM_GetAutoreload(&htim1) + 1) / 2; // ��ȡ��װ��ֵ����ֵ��������ϵ��Ʊȿ���pwm���
    __HAL_TIM_SET_COUNTER(&htim3, 20000);                     // ����������ֵ�趨Ϊ20000������ʹ��������������
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);           // ����������
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);          // ����PWM���Ϊ0
    HAL_TIM_Base_Start_IT(&htim1);                            // ������ʱ���ж�
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);                 // ����pwm���
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);              // ����pwm�������
    HAL_ADCEx_Calibration_Start(&hadc1);                      // ����ADCУ׼
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&adc_value, 1);     // ����ADC����
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&RxBuffer, 1);    // �������ڽ���
    HAL_Delay(1000);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        // ״̬����
        if (switch_state == 1)
        {
            switch_state = 0;
            // ����PID������
            count = 0;
            PID.MI = 0;
            PID.integralE = 0;
            // �������ƹرյ�ѹ���
            if (key_state == 0)
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        }
        // ���β��������ڽ��յ��ַ�1֮�󣬷���0.5s��5000��������
        if (RxBuffer == '1')
        {
            RxBuffer = 0;
            adc_count = 0;                                // ��ʼ�����ѹ����
            HAL_Delay((20 / SAMP_SIZE) * LOG_SIZE + 100); // �ȴ�������ɣ�ʱ��ΪLOG_SIZE�趨�������������ʱ��
            for (size_t i = 0; i < LOG_SIZE; i++)         // �ѱ���õĲ�������ͨ�����ڷ���
                printf("%d\n", adc[i]);
            // ���¿������ڲ��������ж�
            HAL_UART_Receive_IT(&huart1, (uint8_t *)&RxBuffer, 1);
        }
        Rotary(&htim3, &V.set, 0, 220, 1); // ��ť���õ�ѹ
        OLED_Show();                       // ��Ϣ��ʾ
        HAL_Delay(50);
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

    /** Configure the main internal regulator output voltage
     */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
    RCC_OscInitStruct.PLL.PLLN = 8;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the peripherals clocks
     */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_I2C1 | RCC_PERIPHCLK_ADC;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
    PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_HSI;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
// ��ʱ���жϺ���
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim1)
    {
        count++;       // ��ʱ������
        ADC_Collect(); // ��ѹ����

        if (key_state == 1) // �����������
        {
            // �������pwm���㷨��
            pwm = pwm_half_arr + PID.MI * sin_wave[count];
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm);
            // ÿ���������ں�ִ��һ��PID����
            if (count == 0)
                PID_Controller();
        }
    }
}
void ADC_Collect()
{
    // ��ȡADC��ѹֵ������������ѹ��������ֵ����С�����õ�ʵ��ֵ���Ż���������
    // ԭʼ��ʽ��V.out = 278 * (((float)adc_value / 4096 * 3) - 1.5);
    V.out = 0.208 * adc_value - 426;
    V.rms_sum += V.out * V.out;
    // ���ڿ��Ʊ����������
    if (adc_count < LOG_SIZE)
    {
        adc[adc_count] = adc_value;
        adc_count++;
    }
    // ��ʱ��50hz��һ�����ں�,���ɼ�����Чֵ
    if (count >= SAMP_SIZE)
    {
        count = 0;
        // ������������Чֵ
        V.rms = sqrt(V.rms_sum / SAMP_SIZE);
        V.rms_sum = 0;
    }
}

// PID������
void PID_Controller()
{
    // ���Ϊ�趨ֵ����Чֵ֮��
    PID.deltaE = V.set - V.rms;
    // ������
    PID.integralE += PID.deltaE;

    // �����޷�
    if (PID.integralE < -1000)
        PID.integralE = -1000;
    else if (PID.integralE > 1000)
        PID.integralE = 1000;

    // �����ֿ���
    PID.MI = PID.integralE * PID.kI;

    // ����޷�
    if (PID.MI < 0)
        PID.MI = 0;
    else if (PID.MI > 0.9)
        PID.MI = 0.9;
}

// �ⲿ�жϻص�����
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
    // ����
    HAL_Delay(30);
    // �������
    if (GPIO_Pin == KEY_Pin)
    {
        if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
        {
            switch_state = 1;
            // �����ʱ������������
            if (charge_state != 1)
                key_state = !key_state;
        }
    }
    // �����
    else if (GPIO_Pin == CHARGE_Pin)
    {
        // ���ʱ�ر����
        if (HAL_GPIO_ReadPin(CHARGE_GPIO_Port, CHARGE_Pin) == GPIO_PIN_SET)
        {
            switch_state = 1;
            charge_state = 1;
            key_state = 0;
        }
        else
            charge_state = 0;
    }
}

// ��ʾ����
void OLED_Show()
{
    // ����
    OLED_Clear();
    // ���������л����л���ʾ������
    // ��ʾ��
    if (key_state == 1)
        OLED_ShowCHinese(96, 0, 1);
    // ��ʾ��
    else
        OLED_ShowCHinese(96, 0, 0);
    // ��ʾ��ѹ����ͼ��
    if (V.rms >= 36)
        OLED_ShowCHinese(112, 2, 2);

    // ��ʾ���
    if (charge_state == 1)
        OLED_ShowCHinese(112, 0, 7);
    else
        OLED_ShowCHinese(112, 0, 5);

    // ��ʾ��ֵ
    OLED_ShowString(0, 0, "Set:", 16);
    OLED_ShowFloat(32, 0, V.set, 0);

    OLED_ShowString(0, 2, "Out:", 16);
    // ����һ����ѹ���ڵĲ���
    if ((V.rms_show - V.rms) > 0.05 || (V.rms_show - V.rms) < -0.05)
        V.rms_show = V.rms + 0.05;        // ���𲹳�
    OLED_ShowFloat(32, 2, V.rms_show, 2); // ��ʾ���������Чֵ

    OLED_Refreash();
}

// ��ť����
//  htim:ʹ�õı�������ʱ��
//  set:ʹ����ť�޸ĵ�ֵ
//  min:��ť�������õ���Сֵ
//  max:��ť�������õ����ֵ
//  step:ÿ��תһ�����õĶ�Ӧֵ����Ϊ�������Ըı���ת����
void Rotary(TIM_HandleTypeDef *htim, int *set, int min, int max, int step)
{
    int rotary_count = __HAL_TIM_GET_COUNTER(htim);   // ��ȡ��ť����ֵ
    if (rotary_count < 10000 || 30000 < rotary_count) // �����쳣����
        __HAL_TIM_SET_COUNTER(htim, 20000);
    else if (rotary_count != 20000)
    {
        // ��ֵ����
        int set_temp = *set;
        set_temp += (rotary_count - 20000) * step;
        if (set_temp < min)
            *set = min;
        else if (set_temp > max)
            *set = max;
        else
            *set = set_temp;
        // ����������ť������Ϊ20000�ĳ�ʼֵ
        __HAL_TIM_SET_COUNTER(htim, 20000);
    }
}

// �ض���printf���ڷ�������
#ifdef __GNUC__
int _write(int fd, char *ptr, int len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, 0xFFFF);
    return len;
}
#endif
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
    OLED_Clear();
    OLED_ShowString(0, 1, "Error", 16);
    OLED_Refreash();
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
