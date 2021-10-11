/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
//#include "DHT.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DHT_TIMEOUT 				100000	//Количество итераций, после которых функция вернёт пустые значения
#define DHT_POLLING_CONTROL			1		//Включение проверки частоты опроса датчика
#define DHT_POLLING_INTERVAL_DHT11	2000	//Интервал опроса DHT11 (0.5 Гц по даташиту). Можно поставить 1500, будет работать
#define DHT_POLLING_INTERVAL_DHT22	1000	//Интервал опроса DHT22 (1 Гц по даташиту)

#define lineDown() 		HAL_GPIO_WritePin(sensor->DHT_Port, sensor->DHT_Pin, GPIO_PIN_RESET)
#define lineUp()		HAL_GPIO_WritePin(sensor->DHT_Port, sensor->DHT_Pin, GPIO_PIN_SET)
#define getLine()		(HAL_GPIO_ReadPin(sensor->DHT_Port, sensor->DHT_Pin) == GPIO_PIN_SET)
#define Delay(d)		HAL_Delay(d)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
typedef struct {
	float hum;
	float temp;
} DHT_data;

typedef enum {
	DHT11,
	DHT22
} DHT_type;

typedef struct {
	GPIO_TypeDef *DHT_Port;	//Порт датчика (GPIOA, GPIOB, etc)
	uint16_t DHT_Pin;		//Номер пина датчика (GPIO_PIN_0, GPIO_PIN_1, etc)
	DHT_type type;			//Тип датчика (DHT11 или DHT22)
	uint8_t pullUp;			//Нужна ли подтяжка к питанию (0 - нет, 1 - да)

	//Контроль частоты опроса датчика. Значения не заполнять!
	#if DHT_POLLING_CONTROL == 1
	uint32_t lastPollingTime;//Время последнего опроса датчика
	float lastTemp;			 //Последнее значение температуры
	float lastHum;			 //Последнее значение влажности
	#endif
} DHT_sensor;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void goToOutput(DHT_sensor *sensor) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};

  //По умолчанию на линии высокий уровень
  lineUp();

  //Настройка порта на выход
  GPIO_InitStruct.Pin = sensor->DHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD; 	//Открытый сток
  if(sensor->pullUp == 1) {
	  GPIO_InitStruct.Pull = GPIO_PULLUP;						//Подтяжка к питанию
  } else {
	  GPIO_InitStruct.Pull = GPIO_NOPULL;						//Без подтяжки
  }

  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; //Высокая скорость работы порта
  HAL_GPIO_Init(sensor->DHT_Port, &GPIO_InitStruct);
}

static void goToInput(DHT_sensor *sensor) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  //Настройка порта на вход
  GPIO_InitStruct.Pin = sensor->DHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  if(sensor->pullUp == 1) {
	  GPIO_InitStruct.Pull = GPIO_PULLUP;						//Подтяжка к питанию
  } else {
	  GPIO_InitStruct.Pull = GPIO_NOPULL;						//Без подтяжки
  }
  HAL_GPIO_Init(sensor->DHT_Port, &GPIO_InitStruct);
}

DHT_data DHT_getData(DHT_sensor *sensor) {
	DHT_data data = {0.0f, 0.0f};


	#if DHT_POLLING_CONTROL == 1
	/* Ограничение по частоте опроса датчика */
	//Определение интервала опроса в зависимости от датчика
	uint16_t pollingInterval;
	if (sensor->type == DHT11) {
		pollingInterval = DHT_POLLING_INTERVAL_DHT11;
	} else {
		pollingInterval = DHT_POLLING_INTERVAL_DHT22;
	}

	//Если частота превышена, то возврат последнего удачного значения
	if (HAL_GetTick()-sensor->lastPollingTime < pollingInterval) {
		data.hum = sensor->lastHum;
		data.temp = sensor->lastTemp;
		return data;
	}
	sensor->lastPollingTime = HAL_GetTick();
	#endif

	/* Запрос данных у датчика */
	//Перевод пина "на выход"
	goToOutput(sensor);
	//Опускание линии данных на 15 мс
	lineDown();
	Delay(20);
	//Подъём линии, перевод порта "на вход"
	lineUp();
	goToInput(sensor);

	/* Ожидание ответа от датчика */
	uint16_t timeout = 0;
	//Ожидание спада
	while(getLine()) {
		timeout++;
		if (timeout > DHT_TIMEOUT) return data;
	}
	timeout = 0;
	//Ожидание подъёма
	while(!getLine()) {
		timeout++;
		if (timeout > DHT_TIMEOUT) return data;
	}
	timeout = 0;
	//Ожидание спада
	while(getLine()) {
		timeout++;
		if (timeout > DHT_TIMEOUT) return data;
	}

	/* Чтение ответа от датчика */
	uint8_t rawData[5] = {0,0,0,0,0};
	for(uint8_t a = 0; a < 5; a++) {
		for(uint8_t b = 7; b != 255; b--) {
			uint32_t hT = 0, lT = 0;
			//Пока линия в низком уровне, инкремент переменной lT
			while(!getLine()) lT++;
			//Пока линия в высоком уровне, инкремент переменной hT
			timeout = 0;
			while(getLine()) hT++;
			//Если hT больше lT, то пришла единица
			if(hT > lT) rawData[a] |= (1<<b);
		}
	}
	/* Проверка целостности данных */
	if((uint8_t)(rawData[0] + rawData[1] + rawData[2] + rawData[3]) == rawData[4]) {
		//Если контрольная сумма совпадает, то конвертация и возврат полученных значений
		if (sensor->type == DHT22) {
			data.hum = (float)(((uint16_t)rawData[0]<<8) | rawData[1])*0.1f;
			//Проверка на отрицательность температуры
			if(!(rawData[2] & (1<<7))) {
				data.temp = (float)(((uint16_t)rawData[2]<<8) | rawData[3])*0.1f;
			}	else {
				rawData[2] &= ~(1<<7);
				data.temp = (float)(((uint16_t)rawData[2]<<8) | rawData[3])*-0.1f;
			}
		}
		if (sensor->type == DHT11) {
			data.hum = (float)rawData[0];
			data.temp = (float)rawData[2];;
		}
	}

	#if DHT_POLLING_CONTROL == 1
	sensor->lastHum = data.hum;
	sensor->lastTemp = data.temp;
	#endif

	return data;
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
  /* USER CODE BEGIN 2 */
  //temp_s data;
  //char buf[20] = {0};
  //uint8_t err[] = "Error!404\n\r";
  static DHT_sensor livingRoom = {GPIOB, GPIO_PIN_6, DHT11, 0};
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  char msg[40];
	  //Получение данных с датчика
	  DHT_data d = DHT_getData(&livingRoom);
	  //Печать данных в буффер
	  sprintf(msg, "\fLiving room: Temp %d°C, Hum %d%% \n\r", (uint8_t)d.temp, (uint8_t)d.hum);
	  //Отправка текста в UART
	  for(int k = 0; k<20000; k++){
		  if(k == 19990)
			  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 0xFF);
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

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
