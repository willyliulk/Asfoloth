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
#include "rtc.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;

#define EXT_uart huart1
#define EXT_DMA_RX hdma_usart1_rx
#define EXT_DMA_TX hdma_usart1_tx
#define IMU_uart huart3
#define IMU_DMA_RX hdma_usart3_rx
#define IMU_DMA_TX hdma_usart3_tx

extern RTC_HandleTypeDef hrtc;

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

PUTCHAR_PROTOTYPE {
	HAL_UART_Transmit(&EXT_uart, (uint8_t*) &ch, 1, 0xFFFF);
//	HAL_UART_Transmit_DMA(&EXT_DMA_TX, (uint8_t *)&ch, 1);
	return ch;
}

RTC_DateTypeDef GetDate;
RTC_TimeTypeDef GetTime;

uint8_t check_ver;
uint8_t addrval;
bool testvar;
bool testvar2;
uint32_t costtime;
LoRa myLoRa;
float testfloat;

extern IMU imu;

uint16_t data_counter;
uint8_t data_hour, data_min, data_sec, data_subSec;
float data_PA_temp;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

float hex2float(uint8_t *hexNum);

#define EXT_BUFFER_SIZE 1024
uint8_t EXT_buffer[EXT_BUFFER_SIZE];
bool EXT_got_data;
uint16_t EXT_data_Size;

extern IMU imu;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	IMU_UART_CB(huart, Size);

	if (huart->Instance == USART1) {
		EXT_got_data = true;
		EXT_data_Size = Size;

		HAL_UARTEx_ReceiveToIdle_DMA(&EXT_uart, EXT_buffer, EXT_BUFFER_SIZE);
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
	}

}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef struct {
	uint8_t datas[1024];
	int length;
} IMU_DATA_TO_SEND_t;
//typedef struct IMU_DATA_TO_SEND IMU_DATA_TO_SEND_t;

void proc_data_4(IMU_DATA_TO_SEND_t *data, float value) {
	f32_t f32_value;
	f32_value.f = value;
	data->datas[data->length] = f32_value.u8[3];
	data->length += 1;
	data->datas[data->length] = f32_value.u8[2];
	data->length += 1;
	data->datas[data->length] = f32_value.u8[1];
	data->length += 1;
	data->datas[data->length] = f32_value.u8[0];
	data->length += 1;
}

void proc_data_2(IMU_DATA_TO_SEND_t *data, float value) {
	f16_t f16_value;
	f32_t temp;
	temp.f = value;
	f32_to_f16(&temp, &f16_value);
	data->datas[data->length] = f16_value.u8[1];
	data->length += 1;
	data->datas[data->length] = f16_value.u8[0];
	data->length += 1;
}

void proc_data_2_uint16(IMU_DATA_TO_SEND_t *data, uint16_t value) {
	f16_t temp;
	temp.u16 = value;
	data->datas[data->length] = temp.u8[1];
	data->length += 1;
	data->datas[data->length] = temp.u8[0];
	data->length += 1;
}

void proc_data_1_uint8(IMU_DATA_TO_SEND_t *data, uint8_t value) {
	data->datas[data->length] = value;
	data->length += 1;
}

void imu_data_conv_config(IMU *imu, IMU_DATA_TO_SEND_t *out) {
	out->length = 0;
	proc_data_1_uint8(out, data_hour);
	proc_data_1_uint8(out, data_min);
	proc_data_1_uint8(out, data_sec);
	proc_data_1_uint8(out, data_subSec);
	proc_data_2_uint16(out, data_counter);
	proc_data_2(out, imu->temp);
	proc_data_4(out, imu->quaternionWXYZ[0]);
	proc_data_4(out, imu->quaternionWXYZ[1]);
	proc_data_4(out, imu->quaternionWXYZ[2]);
	proc_data_4(out, imu->quaternionWXYZ[3]);
	proc_data_2(out, imu->rateOfTurnXYZ[0]);
	proc_data_2(out, imu->rateOfTurnXYZ[1]);
	proc_data_2(out, imu->rateOfTurnXYZ[2]);
	proc_data_2(out, imu->freeAccelerationXYZ[0]);
	proc_data_2(out, imu->freeAccelerationXYZ[1]);
	proc_data_2(out, imu->freeAccelerationXYZ[2]);
	proc_data_2(out, imu->accelerationXYZ[0]);
	proc_data_2(out, imu->accelerationXYZ[1]);
	proc_data_2(out, imu->accelerationXYZ[2]);
	proc_data_2(out, imu->positionEcefXYZ[0]);
	proc_data_2(out, imu->positionEcefXYZ[1]);
	proc_data_2(out, imu->positionEcefXYZ[2]);
	proc_data_4(out, imu->latitude);
	proc_data_4(out, imu->longitude);
	proc_data_4(out, imu->altitudeEllip);
	proc_data_2(out, imu->velocityXYZ[0]);
	proc_data_2(out, imu->velocityXYZ[1]);
	proc_data_2(out, imu->velocityXYZ[2]);
	proc_data_4(out, data_PA_temp);

}
void imu_data_conv_onFly(IMU *imu, IMU_DATA_TO_SEND_t *out) {
	out->length = 0;
	proc_data_1_uint8(out, data_hour);
	proc_data_1_uint8(out, data_min);
	proc_data_1_uint8(out, data_sec);
	proc_data_1_uint8(out, data_subSec);
	proc_data_2_uint16(out, data_counter);
	proc_data_4(out, imu->quaternionWXYZ[0]);
	proc_data_4(out, imu->quaternionWXYZ[1]);
	proc_data_4(out, imu->quaternionWXYZ[2]);
	proc_data_4(out, imu->quaternionWXYZ[3]);
	proc_data_2(out, imu->rateOfTurnXYZ[0]);
	proc_data_2(out, imu->rateOfTurnXYZ[1]);
	proc_data_2(out, imu->rateOfTurnXYZ[2]);
	proc_data_2(out, imu->freeAccelerationXYZ[0]);
	proc_data_2(out, imu->freeAccelerationXYZ[1]);
	proc_data_2(out, imu->freeAccelerationXYZ[2]);
	proc_data_2(out, imu->positionEcefXYZ[0]);
	proc_data_2(out, imu->positionEcefXYZ[1]);
	proc_data_2(out, imu->positionEcefXYZ[2]);
	proc_data_2(out, imu->velocityXYZ[0]);
	proc_data_2(out, imu->velocityXYZ[1]);
	proc_data_2(out, imu->velocityXYZ[2]);

}

GPIO_PinState modeSwitch = 0, prevModeSwitch = 0;
typedef enum flymode {
	config, onFly
} flyMode;
flyMode curFlyMode;
uint32_t flyModeDebounce = 0;

bool lora_recv_open = false;

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

	HAL_UARTEx_ReceiveToIdle_DMA(&EXT_uart, EXT_buffer, EXT_BUFFER_SIZE);
	__HAL_DMA_DISABLE_IT(&EXT_DMA_RX, DMA_IT_HT);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_SPI4_Init();
  MX_RTC_Init();
  MX_ADC3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

	/*Lora init==============================================================*/
	myLoRa = newLoRa();

	myLoRa.CS_port = SPI4_CS_GPIO_Port;
	myLoRa.CS_pin = SPI4_CS_Pin;
	myLoRa.reset_port = LoRa_RST_GPIO_Port;
	myLoRa.reset_pin = LoRa_RST_Pin;
	myLoRa.DIO0_port = DIO0_GPIO_Port;
	myLoRa.DIO0_pin = DIO0_Pin;
	myLoRa.hSPIx = &hspi4;
	myLoRa.frequency = 433;             	// default = 433 		MHz
	myLoRa.spredingFactor = SF_7;           // default = SF_7
	myLoRa.bandWidth = BW_125KHz;       	// default = BW_125	KHz
	myLoRa.crcRate = CR_4_5;          		// default = CR_4_5
	myLoRa.power = POWER_20db;      		// default = 20db
	myLoRa.overCurrentProtection = 100;     // default = 100 		mA
	myLoRa.preamble = 10;              		// default = 8;

	uint16_t LoRa_status = LoRa_init(&myLoRa);
	if (LoRa_status == LORA_OK) {            //initialize LoRa configuration
		printf("LoRa is running... \n");
	} else {
		printf("LoRa failed :( \n Error code: %d \n", LoRa_status);
	}
	LoRa_setLowDaraRateOptimization(&myLoRa, 1);
	LoRa_startReceiving(&myLoRa);
	uint8_t received_data[10];
	uint8_t packet_size = 0;
	/*Lora init end===========================================================*/

	IMU_Init();

	/*temp init ##############################################################*/
	uint32_t ADC_read;
	int ADC_resolution=12;
	int Vin_temp = 3300;
	float Vout;//Vout,Vin_temp:in mV
	float Vbias = 500;//TMP36 0 degree bias=500mV
	float OutV_Temp_ratio=10;//OutV:mV,Temp:Celsius
	float data_PA_temp;
	/*temp init end###########################################################*/


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint32_t timer = HAL_GetTick();
	uint32_t loopRunTime = 0;
	bool GPS_no_calied = true;

	IMU_DATA_TO_SEND_t data2Lora;

	data_counter = 0;

	printf("init finish!!!!!!!!!!!!\n");
	while (1) {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		//IMU data gathering
		IMU_process_data();

		IMU_State_mechine();

		/* Get the RTC current Time */
		HAL_RTC_GetTime(&hrtc, &GetTime, RTC_FORMAT_BIN);
		/* Get the RTC current Date */
		HAL_RTC_GetDate(&hrtc, &GetDate, RTC_FORMAT_BIN);

		//Calibrate date ,only run once
		if (	(imu.myGnssData.numSV >= 4)
				&& ((GetDate.Year + 2000) != imu.myGnssData.year)
				&& (GPS_no_calied)) {
			printf("Reset RTC timer\n");
			RTC_TimeTypeDef IMU_time;
			RTC_DateTypeDef IMU_date;

			IMU_date.Year = imu.myGnssData.year - 2000;
			IMU_date.Month = imu.myGnssData.month;
			IMU_date.Date = imu.myGnssData.day;
			IMU_time.Hours = imu.myGnssData.hour;
			IMU_time.Minutes = imu.myGnssData.minute;
			IMU_time.Seconds = imu.myGnssData.second;

			IMU_date.Year = imu.myGnssData.year - 2000;
			IMU_date.Month = imu.myGnssData.month;
			IMU_date.Date = imu.myGnssData.day;
			IMU_time.Hours = imu.myGnssData.hour;
			IMU_time.Minutes = imu.myGnssData.minute;
			IMU_time.Seconds = imu.myGnssData.second;

			HAL_RTC_SetTime(&hrtc, &IMU_time, RTC_FORMAT_BIN);
			HAL_RTC_SetDate(&hrtc, &IMU_date, RTC_FORMAT_BIN);

			GPS_no_calied = false;
		}

		/*check fly mode switch*/
		modeSwitch = HAL_GPIO_ReadPin(Mode_Switch_GPIO_Port,
		Mode_Switch_Pin);
		if (modeSwitch == GPIO_PIN_RESET && prevModeSwitch == GPIO_PIN_SET
				&& HAL_GetTick() > flyModeDebounce) {
			flyModeDebounce = HAL_GetTick() + 1000;
			if (curFlyMode == config) {
				curFlyMode = onFly;
				printf("fly mode now --> on fly\n");
			} else {
				curFlyMode = config;
				printf("fly mode now --> config\n");
			}
		}
		prevModeSwitch = modeSwitch;

		//LoRa_receive()
		HAL_GPIO_WritePin(FEM_CPS_GPIO_Port, FEM_CPS_Pin, GPIO_PIN_SET); //low frequency port switch, RESET for transmit, SET for receive
		packet_size = LoRa_receive(&myLoRa, received_data, 10);
		if (packet_size != 0) {
			printf("LoRa get: %s", received_data);
		}

		/*temp sensor*/
		HAL_ADC_Start(&hadc1);
		ADC_read = HAL_ADC_GetValue(&hadc1);
		Vout = ADC_read/(pow(2,ADC_resolution)-1)*Vin_temp;
		data_PA_temp = (Vout-Vbias)/OutV_Temp_ratio;//temp:in Celsius
		printf("temp: %f\n", data_PA_temp);

		//====================================================================
		if (HAL_GetTick() - timer > 333) {

			data_hour = GetTime.Hours;
			data_min = GetTime.Minutes;
			data_sec = GetTime.Seconds;
			data_subSec = ((float) (255 - GetTime.SubSeconds)) * 1.
					/ ((float) (GetTime.SecondFraction + 1)) * 100;

			//packing data from IMU to send via Lora
			if (curFlyMode == config) {
				imu_data_conv_config(&imu, &data2Lora);
			} else if (curFlyMode == onFly) {
				imu_data_conv_onFly(&imu, &data2Lora);
			}

			//LoRa_transmit()
#if 1
			HAL_GPIO_WritePin(FEM_CPS_GPIO_Port, FEM_CPS_Pin, GPIO_PIN_RESET);
			uint8_t err = LoRa_transmit(&myLoRa, data2Lora.datas, data2Lora.length, TRANSMIT_TIMEOUT);
			if (err == 0) {
				printf("LoRa_transmit timed out\n");
			} else {
				printf("LoRa_transmit seccessed\n");
				HAL_Delay(100);
			}
			HAL_GPIO_WritePin(FEM_CPS_GPIO_Port, FEM_CPS_Pin, GPIO_PIN_SET);
#endif

			loopRunTime = HAL_GetTick() - loopRunTime;
//			printf("acc:%f,%f,%f,%f,%f,%d,%d,%d\n", imu.quaternionWXYZ[0],
//					imu.quaternionWXYZ[1], imu.quaternionWXYZ[2],
//					imu.quaternionWXYZ[3], data_PA_temp, data2Lora.length,
//					loopRunTime, data_counter);


			timer = HAL_GetTick();
			data_counter += 1;
			loopRunTime = HAL_GetTick();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
float hex2float(uint8_t *hexNum) {
	union {
		char conv_char[4];
		float output_float;
	} uni;
	uni.conv_char[3] = (char) (hexNum[3]);
	uni.conv_char[2] = (char) (hexNum[2]);
	uni.conv_char[1] = (char) (hexNum[1]);
	uni.conv_char[0] = (char) (hexNum[0]);

	return uni.output_float;
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
