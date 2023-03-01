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
#include "sinwave.h" //正弦表
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMP_SIZE 200 // 一个50hz正弦周期内，采样与pwm调整次数，等于开关频率即定时器1的频率除以50
#define LOG_SIZE 5000 // 波形采样一次采样点数量
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t key_state = 0;    // 1表示开启spwm
uint8_t charge_state = 0; // 1表示充电中
uint8_t switch_state = 0; // 按键按下标志位

uint16_t pwm = 0;          // PWM输出计数值
uint16_t count = 0;        // 定时器计数
uint16_t pwm_half_arr = 0; // pwm重装值的一半，用于参与spwm输出的计算
uint16_t adc_value = 0;    // 采样ADC数组，第一个是电池电压，第二个是输出电压

char RxBuffer = 0;            // 串口接收缓存区
uint16_t adc_count = 0;       // 波形采样计数
uint16_t adc[LOG_SIZE] = {0}; // 波形采样数组

struct
{
    int set;        // 设定电压
    float out;      // 瞬时输出电压
    float rms;      // 输出电压有效值
    float rms_show; // 输出电压显示值
    float rms_sum;  // 计算有效值中间量
} V = {0, 0, 0, 0, 0};

struct
{
    float kP;        // PID比例系数
    float kI;        // PID积分系数
    float kD;        // PID微分系数
    float integralE; // 误差积分
    float deltaE;    // 当前误差
    float MI;        // 调制比Modulation Index，用于控制输出
} PID = {0, 0.001, 0, 0, 0, 0};

// const float sin_wave[SAMP_SIZE]={0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void USART1_printf(char *format, ...);                                      // printf函数映射到串口
void Rotary(TIM_HandleTypeDef *htim, int *set, int min, int max, int step); // 旋钮函数
void PID_Controller();                                                      // PID控制器
void ADC_Collect();                                                         // 电压采集
void OLED_Show();                                                           // 信息显示函数
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

    OLED_Init(); // oled初始化
    OLED_ShowString(0, 1, "Initializing", 16);
    OLED_Refreash();

    // 初始化定时器和PWM
    pwm_half_arr = (__HAL_TIM_GetAutoreload(&htim1) + 1) / 2; // 获取重装载值的中值，用以配合调制比控制pwm输出
    __HAL_TIM_SET_COUNTER(&htim3, 20000);                     // 编码器计数值设定为20000，可以使计数能正或负增长
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);           // 开启编码器
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);          // 设置PWM输出为0
    HAL_TIM_Base_Start_IT(&htim1);                            // 开启定时器中断
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);                 // 开启pwm输出
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);              // 开启pwm互补输出
    HAL_ADCEx_Calibration_Start(&hadc1);                      // 开启ADC校准
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&adc_value, 1);     // 开启ADC采样
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&RxBuffer, 1);    // 开启串口接收
    HAL_Delay(1000);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        // 状态更新
        if (switch_state == 1)
        {
            switch_state = 0;
            // 重置PID控制器
            count = 0;
            PID.MI = 0;
            PID.integralE = 0;
            // 按键控制关闭电压输出
            if (key_state == 0)
                __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        }
        // 波形采样，串口接收到字符1之后，发送0.5s内5000个采样点
        if (RxBuffer == '1')
        {
            RxBuffer = 0;
            adc_count = 0;                                // 开始保存电压采样
            HAL_Delay((20 / SAMP_SIZE) * LOG_SIZE + 100); // 等待采样完成，时间为LOG_SIZE设定采样点采完所需时间
            for (size_t i = 0; i < LOG_SIZE; i++)         // 把保存好的采样数据通过串口发送
                printf("%d\n", adc[i]);
            // 重新开启串口采样命令中断
            HAL_UART_Receive_IT(&huart1, (uint8_t *)&RxBuffer, 1);
        }
        Rotary(&htim3, &V.set, 0, 220, 1); // 旋钮设置电压
        OLED_Show();                       // 信息显示
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
// 定时器中断函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim1)
    {
        count++;       // 定时器递增
        ADC_Collect(); // 电压采样

        if (key_state == 1) // 按键控制输出
        {
            // 最终输出pwm的算法：
            pwm = pwm_half_arr + PID.MI * sin_wave[count];
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm);
            // 每个正弦周期后执行一次PID运算
            if (count == 0)
                PID_Controller();
        }
    }
}
void ADC_Collect()
{
    // 读取ADC电压值，计算出输出电压，即采样值乘缩小倍数得到实际值，优化浮点运算
    // 原始公式：V.out = 278 * (((float)adc_value / 4096 * 3) - 1.5);
    V.out = 0.208 * adc_value - 426;
    V.rms_sum += V.out * V.out;
    // 串口控制保存采样数据
    if (adc_count < LOG_SIZE)
    {
        adc[adc_count] = adc_value;
        adc_count++;
    }
    // 计时到50hz的一个周期后,即可计算有效值
    if (count >= SAMP_SIZE)
    {
        count = 0;
        // 均方根计算有效值
        V.rms = sqrt(V.rms_sum / SAMP_SIZE);
        V.rms_sum = 0;
    }
}

// PID控制器
void PID_Controller()
{
    // 误差为设定值与有效值之差
    PID.deltaE = V.set - V.rms;
    // 误差积分
    PID.integralE += PID.deltaE;

    // 积分限幅
    if (PID.integralE < -1000)
        PID.integralE = -1000;
    else if (PID.integralE > 1000)
        PID.integralE = 1000;

    // 纯积分控制
    PID.MI = PID.integralE * PID.kI;

    // 输出限幅
    if (PID.MI < 0)
        PID.MI = 0;
    else if (PID.MI > 0.9)
        PID.MI = 0.9;
}

// 外部中断回调函数
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
    // 消抖
    HAL_Delay(30);
    // 按键检测
    if (GPIO_Pin == KEY_Pin)
    {
        if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
        {
            switch_state = 1;
            // 当充电时不允许控制输出
            if (charge_state != 1)
                key_state = !key_state;
        }
    }
    // 检测充电
    else if (GPIO_Pin == CHARGE_Pin)
    {
        // 充电时关闭输出
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

// 显示函数
void OLED_Show()
{
    // 清屏
    OLED_Clear();
    // 按键功能切换后，切换显示的内容
    // 显示开
    if (key_state == 1)
        OLED_ShowCHinese(96, 0, 1);
    // 显示关
    else
        OLED_ShowCHinese(96, 0, 0);
    // 显示高压警告图标
    if (V.rms >= 36)
        OLED_ShowCHinese(112, 2, 2);

    // 显示充电
    if (charge_state == 1)
        OLED_ShowCHinese(112, 0, 7);
    else
        OLED_ShowCHinese(112, 0, 5);

    // 显示数值
    OLED_ShowString(0, 0, "Set:", 16);
    OLED_ShowFloat(32, 0, V.set, 0);

    OLED_ShowString(0, 2, "Out:", 16);
    // 过滤一定电压以内的波动
    if ((V.rms_show - V.rms) > 0.05 || (V.rms_show - V.rms) < -0.05)
        V.rms_show = V.rms + 0.05;        // 线损补偿
    OLED_ShowFloat(32, 2, V.rms_show, 2); // 显示补偿后的有效值

    OLED_Refreash();
}

// 旋钮函数
//  htim:使用的编码器定时器
//  set:使用旋钮修改的值
//  min:旋钮可以设置的最小值
//  max:旋钮可以设置的最大值
//  step:每旋转一格，设置的对应值，设为负数可以改变旋转方向
void Rotary(TIM_HandleTypeDef *htim, int *set, int min, int max, int step)
{
    int rotary_count = __HAL_TIM_GET_COUNTER(htim);   // 获取旋钮计数值
    if (rotary_count < 10000 || 30000 < rotary_count) // 过滤异常计数
        __HAL_TIM_SET_COUNTER(htim, 20000);
    else if (rotary_count != 20000)
    {
        // 数值调节
        int set_temp = *set;
        set_temp += (rotary_count - 20000) * step;
        if (set_temp < min)
            *set = min;
        else if (set_temp > max)
            *set = max;
        else
            *set = set_temp;
        // 重新设置旋钮计数器为20000的初始值
        __HAL_TIM_SET_COUNTER(htim, 20000);
    }
}

// 重定向printf串口发送数据
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
