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
#include "spi.h"
#include "usart.h"
#include "gpio.h"

//蓝锐鑫到此一游

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Queue.h"
#include "afe4490.h"
#include "stdio.h"
#include "serial.h"
#include "afe4490_agc.h"
#include "FIR.h"
#include "math.h"
#include "calculate.h"
#include "MovingAvg.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
StructCirQue pQue1, pQue2,pQue3,pQue4;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
afe44xx_data data;
int32_t IRABSVAL_data[FFT_SIZE], REDABSVAL_data[FFT_SIZE];
int32_t IR_data[FFT_SIZE], RED_data[FFT_SIZE];

static uint16_t index = 0;
static int32_t ir_accum = 0;
static int32_t red_accum = 0;
static int32_t IRABSVAL_accum = 0;
static int32_t REDABSVAL_accum = 0;
static uint8_t avg_count = 0;

int32_t ir_raw = 0, red_raw = 0;
int32_t IRABSVAL_raw = 0, REDABSVAL_raw = 0;

float g_pga_red = 1.0f;
float R_pga_red = 0.1f;
float g_pga_ir = 1.0f;
float R_pga_ir = 0.1f;
ChannelFilter ir_ma_filter;
ChannelFilter red_ma_filter;
ChannelFilter SPO2_ma_filter;
ChannelFilter HR_ma_filter;
#define AVG_WINDOW 256
float32_t ir_history[AVG_WINDOW] = {0.0f};
float32_t red_history[AVG_WINDOW] = {0.0f};
uint32_t write_idx = 0;
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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  InitQueue(&pQue1, IR_data, FFT_SIZE);
  InitQueue(&pQue2, RED_data, FFT_SIZE);
  InitQueue(&pQue3, IRABSVAL_data, FFT_SIZE);
  InitQueue(&pQue4, REDABSVAL_data, FFT_SIZE);
  hAFE44xx.init();
  FIR_InitFromFdacoefs(2);  // IR + RED
  agc_init();
  Calculate_Init();
  DualMovingAvg_Init(&ir_ma_filter,&red_ma_filter,&SPO2_ma_filter,&HR_ma_filter);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
    {
      /* USER CODE END WHILE */

      /* USER CODE BEGIN 3 */
      if (DeQueue(&pQue1, &ir_raw, 1) && DeQueue(&pQue2, &red_raw, 1)&&DeQueue(&pQue3, &IRABSVAL_raw, 1) && DeQueue(&pQue4, &REDABSVAL_raw, 1))
        {
          ir_accum += ir_raw;
          red_accum += red_raw;
          IRABSVAL_accum+=IRABSVAL_raw;
          REDABSVAL_accum+=REDABSVAL_raw;
          avg_count++;

          if (avg_count >= 8)
            {
              int32_t ir_avg =( ir_accum / 8); 
              int32_t red_avg = (red_accum / 8);
              int32_t IRABSVAL_avg =( IRABSVAL_accum / 8);
              int32_t REDABSVAL_avg = (REDABSVAL_accum / 8);
              ir_accum = 0;
              red_accum = 0;
              IRABSVAL_accum=0;
              REDABSVAL_accum=0;
              avg_count = 0;

              switch(agc.ir_pga)
                {
                case PGA_1X:g_pga_ir = 1.0f;R_pga_ir=0.1f;break;
                case PGA_1_5X:g_pga_ir = 1.5f;R_pga_ir=0.15f;break;
                case PGA_2X:g_pga_ir = 2.0f;R_pga_ir=0.2f;break;
                case PGA_3X:g_pga_ir = 3.0f;R_pga_ir=0.3f;break;
                case PGA_4X:g_pga_ir = 4.0f;R_pga_ir=0.4f;break;
                }
              switch(agc.red_pga)
                {
                case PGA_1X:g_pga_red = 1.0f;R_pga_red=0.1f;break;
                case PGA_1_5X:g_pga_red = 1.5f;R_pga_red=0.15f;break;
                case PGA_2X:g_pga_red = 2.0f;R_pga_red=0.2f;break;
                case PGA_3X:g_pga_red = 3.0f;R_pga_red=0.3f;break;
                case PGA_4X:g_pga_red = 4.0f;R_pga_red=0.4f;break;
                }
//int32_t ir_true  = (int32_t)( (float)ir_avg + 2*(double)agc.ambdac_code * R_pga_ir/(0.000000572f)  );
//int32_t red_true = (int32_t)( (float)red_avg + 2*(double)agc.ambdac_code * R_pga_red/(0.000000572f)  );
              int32_t ir_filt = FIR_ProcessSample(0, (float)(-abs(IRABSVAL_avg)));
              int32_t red_filt = FIR_ProcessSample(1, (float)(-abs(REDABSVAL_avg)));            
              //   afe_adjust(1, PGA_1X, 1, PGA_1X, led_ma_to_code(a), led_ma_to_code(b), agc.ambdac_code);

              float smoothed_ir = DualMovingAvg_ProcessIR(&ir_ma_filter, (float)ir_filt);//fir滤波之后平滑
              float smoothed_red = DualMovingAvg_ProcessRED(&red_ma_filter, (float)red_filt); //fir滤波之后平滑  


              agc_push_sample(abs(IRABSVAL_avg),abs(REDABSVAL_avg),ir_avg,red_avg);  //AGC调光传入参数
              Calculate_Update(abs(IRABSVAL_avg),abs(REDABSVAL_avg), smoothed_ir, smoothed_red);  //血氧以及心率计算
              float P = Calculate_GetPI()*1000;
              float R = Calculate_GetR()*1000;
              float spo2 = Calculate_GetSpO2();
							float smoothed_spo2 ;
              if(spo2>=30&&spo2<=100) smoothed_spo2 = DualMovingAvg_ProcessSPO2(&SPO2_ma_filter, (float)spo2);   //血氧平滑

              uint8_t HR = Calculate_GetHeartRate();
              float smoothed_HR = DualMovingAvg_ProcessHR(&HR_ma_filter, (float)HR);   //心率平滑
							float pi_val = Calculate_GetPI()*1000;
              if(agc.finger_off == false) send_plot_data((int32_t)smoothed_ir, (int32_t)smoothed_red, (int32_t)smoothed_spo2, (int32_t)smoothed_HR);
              if(agc.finger_off == true)  send_plot_data((int32_t)0, (int32_t)0, (int32_t)0, (int32_t)0);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
