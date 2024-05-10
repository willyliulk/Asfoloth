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
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "IMU_rel/xsens_mti.h"      // Main library
#include "IMU_rel/xsens_utility.h"  // Needed for quaternion conversion function
#include "IMU_rel/xsens_mdata2.h"
#include "IMU_rel/float16Tool.h"
#include "IMU_rel/gnssPvtDataParser.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define EXT_uart huart1
#define IMU_uart huart3

extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&EXT_uart, (uint8_t *)&ch, 1, 0xFFFF);
//	HAL_UART_Transmit_DMA(&hdma_usart1_rx, (uint8_t *)&ch, 1);
	return ch;
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

//setup UART IMU DMA
#define IMU_BUFFER_SIZE 1024
uint8_t IMU_dma_buffer[IMU_BUFFER_SIZE];
uint8_t IMU_main_buffer[IMU_BUFFER_SIZE];
bool IMU_got_data=false;
uint16_t IMU_data_Size=0;

//setup UART EXT DMA
#define EXT_BUFFER_SIZE 1024
uint8_t EXT_dma_buffer[EXT_BUFFER_SIZE];
uint8_t EXT_main_buffer[EXT_BUFFER_SIZE];
bool EXT_got_data=false;
uint16_t EXT_data_Size=0;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	if(huart->Instance == USART3){
		memcpy(IMU_main_buffer, IMU_dma_buffer, Size);
		IMU_got_data	= true;
		IMU_data_Size 	= Size;

		HAL_UARTEx_ReceiveToIdle_DMA(&IMU_uart, IMU_dma_buffer, IMU_BUFFER_SIZE);
		__HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
	}
	if(huart->Instance == USART1){
		memcpy(EXT_main_buffer, EXT_dma_buffer, Size);
		EXT_got_data	= true;
		EXT_data_Size 	= Size;

		HAL_UARTEx_ReceiveToIdle_DMA(&EXT_uart, EXT_dma_buffer, EXT_BUFFER_SIZE);
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
	}
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Cache a copy of IMU data
typedef struct MyImuDataPack {
	float temp;
	float quaternionWXYZ[4];
	float rateOfTurnXYZ[3];
	float freeAccelerationXYZ[3];
	float accelerationXYZ[3];
	GnssPvtData_t myGnssData;
	uint32_t status;
	float positionEcefXYZ[3];
	float latitude, longitude;//經度 緯度
	float altitudeEllip;
	float velocityXYZ[3];
};
typedef struct MyImuDataPack MyImuDataPack_t;
char myData[95];

MyImuDataPack_t my_imu_data_pack={0};
uint32_t last_measurement_ms = 0;
uint32_t measurement_timer = 0;


// Callback functions we'll use to catch IMU ack packets
void handle_ack_gotoconfig( xsens_packet_buffer_t *packet );
void handle_ack_gotomeasurement( xsens_packet_buffer_t *packet );
void handle_ack_outputconfiguration( xsens_packet_buffer_t *packet );
void myHandle_WAKEUP_cb( xsens_packet_buffer_t *packet );

typedef enum {
    ACK_NONE = 0,
    ACK_CONFIG,
    ACK_MEASUREMENT,
    ACK_CONFIGURED,
	ACK_WAKEUP
} ACKFlags_t;

ACKFlags_t ack_flag = ACK_NONE;

void imu_callback( XsensEventFlag_t event, XsensEventData_t *mtdata );
void imu_send_data( uint8_t *data, uint16_t length );


// The library holds state and pointers to callbacks in this structure
//   - adding the imu_send_data ptr lets the library talk to the IMU
xsens_interface_t imu_interface = XSENS_INTERFACE_RX_TX( &imu_callback, &imu_send_data );

// Simple state handling for configuration flow state-machine
typedef enum {
    STATE_STARTUP = 0,
		STATE_ACK_WAKEUP,
    STATE_REQUEST_CONFIG_MODE,
    STATE_ACK_CONFIG_MODE,
    STATE_SET_OUTPUT_CONFIG,
    STATE_ACK_OUTPUT_CONFIG,
    STATE_REQUEST_MEASUREMENT_MODE,
    STATE_ACK_MEASUREMENT_MODE,
	STATE_RUNNING
} DemoStates_t;

DemoStates_t demo_state     = STATE_STARTUP;
uint32_t     demo_timer     = 0;
bool         configure_fast = false;
bool 		 configure_Gyro = false;

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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_UARTEx_ReceiveToIdle_DMA(&IMU_uart, IMU_dma_buffer, IMU_BUFFER_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);

  HAL_UARTEx_ReceiveToIdle_DMA(&EXT_uart, EXT_dma_buffer, EXT_BUFFER_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);


  // Setup custom handler callbacks to catch acknowledgements from IMU
  xsens_mti_override_id_handler( MT_ACK_GOTOCONFIG, &handle_ack_gotoconfig );
  xsens_mti_override_id_handler( MT_ACK_GOTOMEASUREMENT, &handle_ack_gotomeasurement );
  xsens_mti_override_id_handler( MT_ACK_OUTPUTCONFIGURATION, &handle_ack_outputconfiguration );
  xsens_mti_override_id_handler( MT_ACK_RESET, &myHandle_WAKEUP_cb );

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  printf("Configuration demo starting...");
  demo_timer = HAL_GetTick();
  measurement_timer = HAL_GetTick();
  uint32_t 	configTimeOut_timer = HAL_GetTick();
  uint32_t runningNoDataTimeOut=HAL_GetTick();
  bool rotState=false;
  bool changeRotSetting = false;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(IMU_got_data){
		  for(int i=0; i<IMU_data_Size; i++){
			  xsens_mti_parse( &imu_interface, IMU_main_buffer[i] );
		  }
		  memset(IMU_main_buffer, IMU_data_Size, IMU_BUFFER_SIZE);
		  IMU_got_data=false;
	  }
		if(IMU_data_Size == 1024){
			memset(IMU_main_buffer, 0, IMU_BUFFER_SIZE);
			memset(IMU_dma_buffer, 0, IMU_BUFFER_SIZE);
		}

	    switch( demo_state )
	    {
	        case STATE_STARTUP:
	        	printf("Start up...\n");
//	        	xsens_mti_request(&imu_interface, MT_RESET);
	    		HAL_UARTEx_ReceiveToIdle_DMA(&IMU_uart, IMU_dma_buffer, IMU_BUFFER_SIZE);
	    		__HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);

	        	demo_state = STATE_REQUEST_CONFIG_MODE;
	        break;

	        case STATE_ACK_WAKEUP:
	            // Read configuration packet response
	            // TODO: MDATA2 odd input handling?

	            if( ack_flag == ACK_WAKEUP)
	            {
	                printf("IMU is wake...\n");
	                demo_state = STATE_REQUEST_CONFIG_MODE;
	                ack_flag   = ACK_NONE;
	            }
	        break;

	        case STATE_REQUEST_CONFIG_MODE:
	            printf("Requesting config mode...\n");
	            xsens_mti_request( &imu_interface, MT_GOTOCONFIG );
	            demo_state = STATE_ACK_CONFIG_MODE;
	            HAL_Delay(100);
	            configTimeOut_timer = HAL_GetTick();
	        break;

	        case STATE_ACK_CONFIG_MODE:
	            // Wait for GoToConfigAck to return
	            // handle_gotoconfigack will fire when it does, and set our flag
	            if( ack_flag == ACK_CONFIG)
	            {
	                printf("IMU in config mode\n");
	                demo_state = STATE_SET_OUTPUT_CONFIG;
	                ack_flag   = ACK_NONE;
	            }else if(HAL_GetTick() - configTimeOut_timer > 2000){
	            	configTimeOut_timer = HAL_GetTick();
	            	demo_state = STATE_STARTUP;
	            	printf("again: ");
	            }
	        break;

	        case STATE_SET_OUTPUT_CONFIG:
	        {
	        	printf("changeSetting\n");
	        	if(changeRotSetting){
	        		changeRotSetting=false;
					//rot setting
					float rotEuler[3] = {1.5707963, 0, 0};
					uint8_t rotFrame = 0; //0:rotSensor or 1:rotLocal
					if(rotState){
						printf("to setting 1\n");
						rotEuler[0] = 0;
						rotEuler[1] = 1.5707963;
						rotEuler[2] = 0;
						rotState=false;
					}else{
						printf("to setting 2\n");
						rotEuler[0] = 0;
						rotEuler[1] = 0;
						rotEuler[2] = 0;
						rotState=true;
					}
					//rot setting finish

					float rotQuaternion[4];
					XsensEventData_t rotData={0};
					mdata2_packet_t rotPacket={0};
					xsens_euler_to_quaternion(&rotEuler, &rotQuaternion);
					rotPacket.id = MT_SETALIGNMENTROTATION;
					rotPacket.length = 17;
					rotPacket.payload[0] = 0;
					rotPacket.payload[1] = rotFrame;
					for(int i = 2; i < 18; i+=4){
						rotData.data.f4 = rotQuaternion[(i+3)/4-1];
						rotData.data.f4 = rotQuaternion[(i+3)/4-1];
						rotData.data.f4 = rotQuaternion[(i+3)/4-1];

						rotPacket.payload[i+0] = rotData.data.u4 >> 24;
						rotPacket.payload[i+1] = rotData.data.u4 >> 16;
						rotPacket.payload[i+2] = rotData.data.u4 >>  8;
						rotPacket.payload[i+3] = rotData.data.u4;
					}
					xsens_mti_send(&imu_interface, &rotPacket);
	        	}
	        	uint8_t output_rate = 100;   // Hz acceleration message rate
	            printf("Setting acceleration output to %d Hz\n", output_rate);

	            XsensFrequencyConfig_t settings[] = {
//	                { .id = XDI_PACKET_COUNTER, .frequency = 0xFFFF },
//	                { .id = XDI_SAMPLE_TIME_FINE, .frequency = 0xFFFF },
					{ .id = XDI_TEMPERATURE			, .frequency = output_rate},
	                { .id = XDI_QUATERNION			, .frequency = output_rate },
					{ .id = XDI_RATE_OF_TURN		, .frequency = output_rate},
	                { .id = XDI_FREE_ACCELERATION	, .frequency = output_rate },
	                { .id = XDI_ACCELERATION	, .frequency = output_rate },
					{ .id = XDI_GNSS_PVT_DATA		, .frequency = 4},
					{ .id = XDI_STATUS_WORD			, .frequency = output_rate},
	                { .id = XDI_POSITION_ECEF		, .frequency = output_rate },
					{ .id = XDI_LAT_LON				, .frequency = 4},
					{ .id = XDI_ALTITUDE_ELLIPSOID	, .frequency = 4},
					{ .id = XDI_VELOCITY_XYZ		, .frequency = output_rate},
	            //  { .id = XSENS_IDENTIFIER_FORMAT(XDI_QUATERNION, XSENS_FLOAT_FIXED1220, XSENS_COORD_ENU), .frequency = 100 },
	            };

	            xsens_mti_set_configuration( &imu_interface, settings, XSENS_ARR_ELEM(settings) );

	            demo_state = STATE_ACK_OUTPUT_CONFIG;

	        }
	        break;

	        case STATE_ACK_OUTPUT_CONFIG:
	            // Read configuration packet response
	            // TODO: MDATA2 odd input handling?

	            if( ack_flag == ACK_CONFIGURED)
	            {
	                printf("IMU confirmed config...\n");
	                demo_state = STATE_REQUEST_MEASUREMENT_MODE;
	                ack_flag   = ACK_NONE;
	            }
	        break;

	        case STATE_REQUEST_MEASUREMENT_MODE:
	            printf("Requesting measurement mode...\n");
	            xsens_mti_request( &imu_interface, MT_GOTOMEASUREMENT );
	            demo_state = STATE_ACK_MEASUREMENT_MODE;
	        break;

	        case STATE_ACK_MEASUREMENT_MODE:
	            // Wait x in this mode before attempting to configure different settings

	            if( ack_flag == ACK_MEASUREMENT)
	            {
	                uint32_t duration_ms = HAL_GetTick() - demo_timer;

	                printf("IMU back in measurement mode\n");

	                printf(" - took %d ms\n", duration_ms);

	                // Go back to the start of the settings flow
	                demo_state = STATE_RUNNING;
	                ack_flag   = ACK_NONE;
	                demo_timer = HAL_GetTick();
	                runningNoDataTimeOut = HAL_GetTick();
	            }
	        break;

	        case STATE_RUNNING:



	        	if(IMU_got_data){
//	        		printf("euler%.4f,%.4f,%.4f\n",	my_imu_data_pack.eulerRPY[0],
//													my_imu_data_pack.eulerRPY[1],
//													my_imu_data_pack.eulerRPY[2]);
//	        		printf("freeAccel%.4f,%.4f,%.4f\n",	my_imu_data_pack.freeAccelerationXYZ[0],
//													my_imu_data_pack.freeAccelerationXYZ[1],
//													my_imu_data_pack.freeAccelerationXYZ[2]);
	        		printf("acc: %.4f, %.4f, %.4f\n",
	        				my_imu_data_pack.freeAccelerationXYZ[0],
							my_imu_data_pack.freeAccelerationXYZ[1],
							my_imu_data_pack.freeAccelerationXYZ[2]);

	        		memset(IMU_main_buffer, 0, IMU_BUFFER_SIZE);
	        		IMU_got_data = false;
	        		runningNoDataTimeOut = HAL_GetTick();

	        	}else if(HAL_GetTick() - runningNoDataTimeOut > 1000){
	        		demo_state = STATE_STARTUP;
	        		runningNoDataTimeOut = HAL_GetTick();
	        	}

	        	if(EXT_got_data){
	        		char commandChar[10]={0};
	        		sscanf(EXT_main_buffer, "%s\n", commandChar);
	        		printf("%s\n", commandChar);

	        		if(strcmp(commandChar, "Reset") == 0){
	        			printf("process to reset\n");
						xsens_mti_reset_orientation(&imu_interface, XSENS_ORIENTATION_ALIGNMENT_RESET);

//						f32_t a;
//						f16_t b;
//						f32_to_f16(&a, &b);
//						f16_to_f32(&b, &a);
	        		}

	        		if(strcmp(commandChar, "setRot") == 0){
	        			changeRotSetting=true;
	        			demo_state = STATE_REQUEST_CONFIG_MODE;
	        		}

	        		memset(EXT_main_buffer, 0, EXT_BUFFER_SIZE);
	        		EXT_got_data = false;
	        	}
//	        	if(needChangeConfig){
//	        		updateConfig();
//	        		demo_state = STATE_REQUEST_CONFIG_MODE;
//	        	}

	        break;
	        default:
	            // Oops!
	            demo_state = STATE_STARTUP;
	        break;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
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
// Called when the library decoded an inbound packet
//   - If the packet was an MData2 frame (which contains packed motion data)
//   - the callback is called once for each sub-field
void imu_callback( XsensEventFlag_t event, XsensEventData_t *mtdata )
{
    switch( event )
    {
        case XSENS_EVT_TEMPERATURE:
        	if(mtdata->type == XSENS_EVT_TYPE_FLOAT){
        		my_imu_data_pack.temp = mtdata->data.f4;
//        		printf("get Temp\n");
        	}
        break;

        case XSENS_EVT_QUATERNION:
        	if(mtdata->type == XSENS_EVT_TYPE_FLOAT4){
        		my_imu_data_pack.quaternionWXYZ[0] = mtdata->data.f4x4[0];
        		my_imu_data_pack.quaternionWXYZ[1] = mtdata->data.f4x4[1];
        		my_imu_data_pack.quaternionWXYZ[2] = mtdata->data.f4x4[2];
        		my_imu_data_pack.quaternionWXYZ[3] = mtdata->data.f4x4[3];
//        		printf("get Quat\n");
        	}
        break;

        case XSENS_EVT_RATE_OF_TURN:
        	if(mtdata->type == XSENS_EVT_TYPE_FLOAT3){
        		my_imu_data_pack.rateOfTurnXYZ[0] = mtdata->data.f4x3[0];
        		my_imu_data_pack.rateOfTurnXYZ[1] = mtdata->data.f4x3[1];
        		my_imu_data_pack.rateOfTurnXYZ[2] = mtdata->data.f4x3[2];
//        		printf("get Rot\n");
        	}
		break;

        case XSENS_EVT_FREE_ACCELERATION:
        	if(mtdata->type == XSENS_EVT_TYPE_FLOAT3){
        		my_imu_data_pack.freeAccelerationXYZ[0] = mtdata->data.f4x3[0];
        		my_imu_data_pack.freeAccelerationXYZ[1] = mtdata->data.f4x3[1];
        		my_imu_data_pack.freeAccelerationXYZ[2] = mtdata->data.f4x3[2];
//        		printf("get Facc\n");
        	}
        break;

        case XSENS_EVT_ACCELERATION:
        	if(mtdata->type == XSENS_EVT_TYPE_FLOAT3){
        		my_imu_data_pack.accelerationXYZ[0] = mtdata->data.f4x3[0];
        		my_imu_data_pack.accelerationXYZ[1] = mtdata->data.f4x3[1];
        		my_imu_data_pack.accelerationXYZ[2] = mtdata->data.f4x3[2];
//        		printf("get Facc\n");
        	}
        break;

        case XSENS_EVT_GNSS_PVT_DATA:
        	if(mtdata->type == XSENS_EVT_TYPE_GNSS_DATA){
        		my_imu_data_pack.myGnssData = gnssPvt_parse(mtdata->gnssPvtData);
//        		printf("get Gnss data\n");
        	}
        break;

        case XSENS_EVT_STATUS_WORD:
        	if(mtdata->type == XSENS_EVT_TYPE_U32){
        		my_imu_data_pack.status = mtdata->data.u4;
//        		printf("get Sword\n");
        	}
		break;

        case XDI_POSITION_ECEF:
        	if(mtdata->type == XSENS_EVT_TYPE_FLOAT3){
        		my_imu_data_pack.positionEcefXYZ[0] = mtdata->data.f4x3[0];
        		my_imu_data_pack.positionEcefXYZ[1] = mtdata->data.f4x3[1];
        		my_imu_data_pack.positionEcefXYZ[2] = mtdata->data.f4x3[2];
//        		printf("get Facc\n");
        	}
        break;

        case XSENS_EVT_LAT_LON:
        	if(mtdata->type == XSENS_EVT_TYPE_FLOAT2){
        		my_imu_data_pack.latitude 	= mtdata->data.f4x2[0];
        		my_imu_data_pack.longitude 	= mtdata->data.f4x2[1];
//        		printf("get LatLon\n");
        	}
		break;

        case XSENS_EVT_ALTITUDE_ELLIPSOID:
        	if(mtdata->type == XSENS_EVT_TYPE_FLOAT){
        		my_imu_data_pack.altitudeEllip = mtdata->data.f4;
//        		printf("get Alt\n");
        	}
        break;

        case XSENS_EVT_VELOCITY_XYZ:
        	if(mtdata->type == XSENS_EVT_TYPE_FLOAT3){
        		my_imu_data_pack.velocityXYZ[0] = mtdata->data.f4x3[0];
        		my_imu_data_pack.velocityXYZ[1] = mtdata->data.f4x3[1];
        		my_imu_data_pack.velocityXYZ[2] = mtdata->data.f4x3[2];
//        		printf("get Vel\n");
        	}
		break;
    }
}

// Command ACK callback functions
void handle_ack_gotoconfig( xsens_packet_buffer_t *packet )
{
    ack_flag = ACK_CONFIG;
}

void handle_ack_gotomeasurement( xsens_packet_buffer_t *packet )
{
    ack_flag = ACK_MEASUREMENT;
}

void handle_ack_outputconfiguration( xsens_packet_buffer_t *packet )
{
    ack_flag = ACK_CONFIGURED;
}

void myHandle_WAKEUP_cb( xsens_packet_buffer_t *packet ){
	ack_flag = ACK_WAKEUP;
	printf("get Wake up\n");
}

// The library calls this function to send packets to the IMU
void imu_send_data( uint8_t *data, uint16_t len )
{
    HAL_UART_Transmit_DMA(&IMU_uart, data, len );
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
