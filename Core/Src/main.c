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
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
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

//#define DHTO_Pin GPIO_PIN_1
//#define DHTO_GPIO_Port GPIOA

uint32_t T1Ch1Data[41]; //41 => 1 start bit + 5*8 data bit ( falling ic pol)
// for rising  polarity  +1 capture is need and Start bit DHT22_MeasureICDMA  need 2
uint32_t TimD[42];
uint8_t ByteData[5];
uint32_t sum;

volatile int TimCb=0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	TimCb=1;
}

void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim){
	TimCb=-1;
}

struct DHTSensor_t {
	uint16_t H; // %  *10 aka .1 unit
	int16_t C;  // C * 10
	uint32_t last;

	TIM_HandleTypeDef *htim;
	//shall be inited as tim capture with  psc to get 1us period
	// input capture Tim Ch direct mode dma (dst mem inc dword) falling  pol (rise  start bit 2 and +1 data in array)
	// neg/Fallinf pol require start bit 1 41 data array
	//both cfg can work

	uint32_t TimCh;
	GPIO_TypeDef *Port; //gpio port to access ic pin (shall be set as inut with pull up defaul)
	uint32_t pin;
};

struct DHTSensor_t _hts = {
		.htim = & htim1,
		.TimCh = TIM_CHANNEL_1,
		.Port = DHTO_GPIO_Port,
		.pin = DHTO_Pin,
};
int DHT22_MeasureICDMA(struct DHTSensor_t *hts){
	int rc;
	int i,t,bit,b,c;
	uint32_t t0,dt;
	int startbit=1; // skip  sensor start bit + potential initial low/high  ic pol
	// with falling pol this is 1 and 2 for rising + 1 extra capture
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	if(   hts->last ) { // dont hold 1st after this logic may get wrogn after 2^32 msec time
		if( HAL_GetTick() <  hts->last+2000  ){
			return 0; // at least 2 sec between measure
		}
	}

	hts->last =HAL_GetTick();
	htim1.Instance->CNT=0;

	HAL_GPIO_WritePin(hts->Port, hts->pin,0);
	GPIO_InitStruct.Pin = hts->pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(hts->Port, &GPIO_InitStruct);
	HAL_GPIO_WritePin(hts->Port, hts->pin,0); //drive 0 with pull
	HAL_Delay(2);
	HAL_GPIO_WritePin(hts->Port, hts->pin,1); // no drive let pull up drive 1 until dht take ovwenrship
	//turn back to input with pull up wo we can sense using CC
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	HAL_GPIO_Init(hts->Port, &GPIO_InitStruct);

	memset(T1Ch1Data, 0, sizeof(T1Ch1Data)); // kill all value
	TimCb=0;
	rc = HAL_TIM_IC_Start_DMA(hts->htim, hts->TimCh,T1Ch1Data, sizeof(T1Ch1Data)/4 );
	t0= HAL_GetTick();
	do {
		dt = HAL_GetTick()-t0;
	}while(dt<20 && TimCb ==0 ); //normaly TimCb shall fire , time check avoid deadlock in case of issues
	//if we had error likley not all data set so time to bti cvt will fail before end and cause error too
	HAL_TIM_IC_Stop_DMA(hts->htim, hts->TimCh);
	for( b=0, sum=0, c=0, i=startbit; i< ARRAY_SIZE(T1Ch1Data)&& c< 5  ; i++ ){
		if( T1Ch1Data[i] == 0)
			break;
		t= TimD[i]=T1Ch1Data[i]-T1Ch1Data[i-1];
		bit=t<80 ? 0 : 1 ;
		b=(b<<1) | bit;
		if( (i-startbit)%8 == 7 ){
			if( c<4)
				sum+=b;
			ByteData[c++]=b;
			b =0;
		}
	}
	if (c ==5 &&  (sum&0xFF) == ByteData[4]){
		//Valid
		hts->H  = ((uint32_t)ByteData[0]<<8 )| ByteData[1] ;
		hts->C =  (((uint32_t)ByteData[2] & 0x7F) << 8) | ByteData[3];
		hts->C *= ByteData[2]&0x80 ? -1 : 1;
		rc =0;
	}
	else
		rc = -2;

	return rc;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
volatile int KickMeas=0;
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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if( KickMeas){ //kick meas is set by debuger or app whenever a new measure required
		  KickMeas =0;
		  DHT22_MeasureICDMA(&_hts);
	  }
	  HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
