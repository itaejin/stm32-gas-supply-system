/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "TS_commu.h"
#include "stm32f1xx_hal_adc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_SENSOR_MIN      819      // 4mA에 해당하는 ADC 값
#define ADC_SENSOR_MAX      4096     // 20mA에 해당하는 ADC 값
#define SENSOR_UPDATE_INTERVAL  50   // 센서 업데이트 주기 (ms)
#define SWITCH_UPDATE_INTERVAL  10   // 스위치 업데이트 주기 (ms)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// 🆕 센서 및 스위치 관련 변수들
uint16_t adcdata1, adcdata2, adcdata3, adcCnt = 0;
uint8_t sw2, sw1, sw0;

// 🆕 주기적 작업을 위한 타이머 변수들
uint32_t last_sensor_update = 0;
uint32_t last_switch_update = 0;
uint32_t last_background_update = 0;

// 🆕 시스템 상태 변수들
uint8_t adc_error_count = 0;
uint8_t system_ready = 0;

// 🆕 외부 변수 선언 (TS_commu.c에서 정의된 변수들)
extern DataPT12_t PT1, PT2;
extern DataPT3_t PT3;
extern GasGraph_t GraphPT1, GraphPT2;
extern FlashParam_t fParam;
extern uint8_t TestFlash;

// 🆕 외부 함수 선언
extern void ProcessBackgroundTasks(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// 🆕 새로운 함수 프로토타입들
void ProcessSensorData(void);
void ProcessSwitchData(void);
void ProcessSystemMaintenance(void);
uint16_t ReadADCChannel(uint32_t channel);
void UpdatePTSensorValue(uint8_t sensor_num, uint16_t adc_value);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//======================================================================================================
// 🆕 센서 데이터 처리 함수
//======================================================================================================
void ProcessSensorData(void)
{
    uint32_t current_time = HAL_GetTick();

    // 50ms마다 센서 데이터 업데이트 (너무 자주 하면 CPU 낭비)
    if(current_time - last_sensor_update < SENSOR_UPDATE_INTERVAL) {
        return;
    }
    last_sensor_update = current_time;

    // ADC 카운터 증가 (디버깅용)
    adcCnt++;

    // 🔍 PT1 센서 (ADC 채널 4) 읽기
    adcdata1 = ReadADCChannel(ADC_CHANNEL_4);
    UpdatePTSensorValue(1, adcdata1);

    // 🔍 PT2 센서 (ADC 채널 5) 읽기
    adcdata2 = ReadADCChannel(ADC_CHANNEL_5);
    UpdatePTSensorValue(2, adcdata2);

    // 🔍 PT3 센서 (ADC 채널 6) 읽기
    adcdata3 = ReadADCChannel(ADC_CHANNEL_6);
    UpdatePTSensorValue(3, adcdata3);

    adcCnt += 7;  // 원래 코드와 호환성 유지
}

//======================================================================================================
// 🆕 ADC 채널 읽기 함수 (에러 처리 강화)
//======================================================================================================
uint16_t ReadADCChannel(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    uint16_t adc_value = 0;

    // ADC 채널 설정
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        adc_error_count++;
        return 0;  // 에러 시 0 반환
    }

    // ADC 변환 시작
    if (HAL_ADC_Start(&hadc1) != HAL_OK) {
        adc_error_count++;
        return 0;
    }

    // 변환 완료 대기 (타임아웃 20ms)
    if (HAL_ADC_PollForConversion(&hadc1, 20) == HAL_OK) {
        adc_value = HAL_ADC_GetValue(&hadc1);
        adc_error_count = 0;  // 성공하면 에러 카운터 리셋
    } else {
        adc_error_count++;
        adc_value = 0;  // 타임아웃 시 0 반환
    }

    HAL_ADC_Stop(&hadc1);  // ADC 정지
    return adc_value;
}

//======================================================================================================
// 🆕 PT 센서 값 업데이트 함수 (스케일링 및 유효성 검사)
//======================================================================================================
void UpdatePTSensorValue(uint8_t sensor_num, uint16_t adc_value)
{
    float scale_factor;
    uint16_t scaled_value;

    switch(sensor_num) {
        case 1:  // PT1 센서
            scale_factor = (float)(PT1.MaxSValue - PT1.MinSValue) / (ADC_SENSOR_MAX - ADC_SENSOR_MIN);

            if(adc_value > ADC_SENSOR_MIN) {
                scaled_value = (uint16_t)(10 * ((adc_value - ADC_SENSOR_MIN) * scale_factor + PT1.MinSValue));
                PT1.Value = scaled_value;
                GraphPT1.Value = scaled_value;  // 그래프 데이터도 업데이트
            } else {
                // 센서 연결 안됨 또는 4mA 이하
                // PT1.Value는 변경하지 않음 (마지막 값 유지)
            }
            break;

        case 2:  // PT2 센서
            scale_factor = (float)(PT2.MaxSValue - PT2.MinSValue) / (ADC_SENSOR_MAX - ADC_SENSOR_MIN);

            if(adc_value > ADC_SENSOR_MIN) {
                scaled_value = (uint16_t)(10 * ((adc_value - ADC_SENSOR_MIN) * scale_factor + PT2.MinSValue));
                PT2.Value = scaled_value;
                GraphPT2.Value = scaled_value;  // 그래프 데이터도 업데이트
            }
            break;

        case 3:  // PT3 센서
            scale_factor = (float)(PT3.MaxSValue - PT3.MinSValue) / (ADC_SENSOR_MAX - ADC_SENSOR_MIN);

            if(adc_value > ADC_SENSOR_MIN) {
                scaled_value = (uint16_t)(10 * ((adc_value - ADC_SENSOR_MIN) * scale_factor + PT3.MinSValue));
                PT3.Value = scaled_value;
            }
            break;
    }
}

//======================================================================================================
// 🆕 스위치 데이터 처리 함수
//======================================================================================================
void ProcessSwitchData(void)
{
    uint32_t current_time = HAL_GetTick();

    // 10ms마다 스위치 상태 확인
    if(current_time - last_switch_update < SWITCH_UPDATE_INTERVAL) {
        return;
    }
    last_switch_update = current_time;

    // GPIO 핀 읽기
    sw0 = 0;
    sw0 |= HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4) ? 0x01 : 0;
    sw0 |= HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_5) ? 0x02 : 0;
    sw0 |= HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_6) ? 0x04 : 0;
    sw0 |= HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7) ? 0x08 : 0;

    sw0 |= HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3) ? 0x10 : 0;
    sw0 |= HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1) ? 0x20 : 0;
    sw0 |= HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) ? 0x40 : 0;
    sw0 |= HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0) ? 0x80 : 0;

    // 스위치 상태 분리
    sw2 = sw0 & 0x0F;        // 하위 4비트
    sw1 = (sw0 >> 4) & 0x0F; // 상위 4비트

    // 🆕 스위치 상태에 따른 동작 (필요시 추가)
    // 예: 비상정지 스위치, 모드 선택 스위치 등
}

//======================================================================================================
// 🆕 시스템 유지보수 함수
//======================================================================================================
void ProcessSystemMaintenance(void)
{
    uint32_t current_time = HAL_GetTick();

    // 1초마다 시스템 상태 체크
    if(current_time - last_background_update < 1000) {
        return;
    }
    last_background_update = current_time;

    // 🏥 시스템 건강 상태 체크
    if(adc_error_count > 10) {
        // ADC 에러가 너무 많으면 시스템 리셋 고려
        // 또는 에러 플래그 설정
        adc_error_count = 0;  // 일단 리셋
    }

    // 📊 시스템 통계 업데이트 (필요시)
    // 메모리 사용량, CPU 사용률 등
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  // 🆕 시스템 초기화 전 준비 작업
  system_ready = 0;

  // TS_commu 초기화를 HAL_Init 이전에 수행
  init_TS_commu();
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
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  start_TS_commu();
  HAL_TIM_Base_Start_IT(&htim2);

  // 🆕 초기 타이머 설정
  last_sensor_update = HAL_GetTick();
  last_switch_update = HAL_GetTick();
  last_background_update = HAL_GetTick();

  // 시스템 준비 완료
  system_ready = 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//==========================================================================================
// 🚀 핵심: ProcessBackgroundTasks() 호출 (가장 중요!)
//==========================================================================================
	  ProcessBackgroundTasks();

//==========================================================================================
// 🔧 Flash 테스트 기능 (기존 코드 유지)
//==========================================================================================
	  if(TestFlash) {
		  if(TestFlash == 1) ReadFlash((uint32_t*)&fParam);
		  else if(TestFlash == 3) EraseFlash();
		  else				WriteFlash((uint32_t*)&fParam);
		  TestFlash = 0;
	  }


	    //==========================================================================================
	    // 🔍 센서 데이터 처리 (주기적 실행)
	    //==========================================================================================
	    ProcessSensorData();

	    //==========================================================================================
	    // 🎛️ 스위치 상태 처리 (주기적 실행)
	    //==========================================================================================
	    ProcessSwitchData();

	    //==========================================================================================
	    // 🏥 시스템 유지보수 (주기적 실행)
	    //==========================================================================================
	    ProcessSystemMaintenance();

	    //==========================================================================================
	    // 💤 CPU 절약을 위한 작은 지연 (선택사항)
	    //==========================================================================================
//	    HAL_Delay(1);  // 1ms 지연으로 CPU 사용률 감소

/*
//	  HAL_ADC_Start_IT(hadc1);
	  ADC_ChannelConfTypeDef sConfig = {0};
	  sConfig.Channel = ADC_CHANNEL_4;
	  sConfig.Rank = ADC_REGULAR_RANK_1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  adcCnt++;
	  HAL_ADC_Start(&hadc1);
	  while(HAL_ADC_PollForConversion(&hadc1, 100)!=HAL_OK);
	  adcdata1 = HAL_ADC_GetValue(&hadc1);

	  sConfig.Channel = ADC_CHANNEL_5;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  adcCnt++;
	  HAL_ADC_Start(&hadc1);
	  while(HAL_ADC_PollForConversion(&hadc1, 100)!=HAL_OK);
	  adcdata2 = HAL_ADC_GetValue(&hadc1);

	  sConfig.Channel = ADC_CHANNEL_6;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  adcCnt++;
	  HAL_ADC_Start(&hadc1);
	  while(HAL_ADC_PollForConversion(&hadc1, 100)!=HAL_OK);
	  adcdata3 = HAL_ADC_GetValue(&hadc1);
	  adcCnt+=7;

	  // 4~20mA : 818 ~ 4096
	  //
	  float PT1scale = (float)(PT1.MaxSValue-PT1.MinSValue)/(4096-819);
	  if(adcdata3 > 819) {
		  PT1.Value = (uint16_t)10*((adcdata3 -819)*PT1scale + PT1.MinSValue);
	  } else {
		  // Sensor not connected !!
	  }
	  sw0 = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4);
	  sw0 |= HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_5) ? 2 : 0;
	  sw0 |= HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_6) ? 4 : 0;
	  sw0 |= HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7) ? 8 : 0;

	  sw0 |= HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3) ? 0x10 : 0;
	  sw0 |= HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1) ? 0x20 : 0;
	  sw0 |= HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) ? 0x40 : 0;
	  sw0 |= HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0) ? 0x80 : 0;
	  sw2 = sw0 & 0x0F;
	  sw1 = (sw0 >> 4) & 0x0F;
*/
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
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
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

  /** Configure the Systick interrupt time
  */
  __HAL_RCC_PLLI2S_ENABLE();
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
