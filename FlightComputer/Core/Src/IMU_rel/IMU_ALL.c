/*
 * IMU_ALL.c
 *
 *  Created on: Oct 23, 2023
 *      Author: liu willy
 */
#include "IMU_rel/IMU_All.h"

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;

#define IMU_uart huart3
#define IMU_DMA_RX hdma_usart3_rx
#define IMU_DMA_TX hdma_usart3_tx


IMU imu;

void IMU_UART_CB(UART_HandleTypeDef *huart, uint16_t Size) {
	if (huart->Instance == USART3) {
		imu.IMU_got_data = true;
		imu.IMU_data_Size = Size;
		imu.no_stuck=true;


		HAL_UARTEx_ReceiveToIdle_DMA(&IMU_uart, imu.IMU_buffer, IMU_BUFFER_SIZE);
		__HAL_DMA_DISABLE_IT(&IMU_DMA_RX, DMA_IT_HT);
//		HAL_UARTEx_ReceiveToIdle_DMA(&IMU_uart, IMU_dma_buffer, IMU_BUFFER_SIZE);
//		__HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);

	}
	//for uart data overflow safety
	if (Size >= 1024) {
		memset(imu.IMU_buffer, 0, IMU_BUFFER_SIZE);
		imu.IMU_data_Size=0;
		HAL_UARTEx_ReceiveToIdle_DMA(&IMU_uart, imu.IMU_buffer, IMU_BUFFER_SIZE);
		__HAL_DMA_DISABLE_IT(&IMU_DMA_RX, DMA_IT_HT);

	}
}
void imu_callback( XsensEventFlag_t event, XsensEventData_t *mtdata );
void IMU_Init() {
	printf("IMU initializing......\n");
	//setup DMA
	HAL_UARTEx_ReceiveToIdle_DMA(&IMU_uart, imu.IMU_buffer, IMU_BUFFER_SIZE);
	__HAL_DMA_DISABLE_IT(&IMU_DMA_RX, DMA_IT_HT);

	// Setup custom handler callbacks to catch acknowledgements from IMU
	xsens_mti_override_id_handler(MT_ACK_GOTOCONFIG, 					&handle_ack_gotoconfig);
	xsens_mti_override_id_handler(MT_ACK_GOTOMEASUREMENT,			&handle_ack_gotomeasurement);
	xsens_mti_override_id_handler(MT_ACK_OUTPUTCONFIGURATION, &handle_ack_outputconfiguration);
	xsens_mti_override_id_handler(MT_ACK_RESET, 							&myHandle_WAKEUP_cb);

	xsens_interface_t imu_interface_S = XSENS_INTERFACE_RX_TX( &imu_callback, &imu_send_data );
	imu.imu_interface = imu_interface_S;

	imu.ack_flag = ACK_NONE;

	imu.output_Hz = 100;   // Hz acceleration message rate

	imu.imu_state = STATE_STARTUP;
	imu.calided = false;

	IMU_State_mechine();
	printf("IMU:initialization finish.\n");
}

void IMU_process_data() {
	if (imu.IMU_got_data) {
		for (int i = 0; i < imu.IMU_data_Size; i++) {
			xsens_mti_parse(&imu.imu_interface, imu.IMU_buffer[i]);
		}
		//set
		memset(imu.IMU_buffer, imu.IMU_data_Size, IMU_BUFFER_SIZE);
		imu.IMU_got_data = false;
		imu.IMU_data_Size = 0;
		imu.IMU_prc_data = true;
	}
}

void IMU_State_mechine(){

	switch (imu.imu_state) {
	case STATE_STARTUP:
		printf("IMU:Start up...\n");
//		xsens_mti_request(&imu.imu_interface, MT_RESET);
		HAL_UARTEx_ReceiveToIdle_DMA(&IMU_uart, imu.IMU_buffer, IMU_BUFFER_SIZE);
		__HAL_DMA_DISABLE_IT(&IMU_DMA_RX, DMA_IT_HT);

		imu.imu_state = STATE_REQUEST_CONFIG_MODE;
//			imu.imu_state = STATE_ACK_WAKEUP;
	break;

	case STATE_ACK_WAKEUP:
		// Read configuration packet response
		// TODO: MDATA2 odd input handling?

		if (imu.ack_flag == ACK_WAKEUP) {
			printf("IMU:IMU is wake...\n");
			imu.ack_flag = ACK_NONE;
			imu.imu_state = STATE_REQUEST_CONFIG_MODE;
		}
	break;

	case STATE_REQUEST_CONFIG_MODE:
		printf("IMU:Requesting config mode...\n");
		xsens_mti_request(&imu.imu_interface, MT_GOTOCONFIG);
		imu.imu_state = STATE_ACK_CONFIG_MODE;
//		HAL_Delay(100);
		imu.timer_timeOut = HAL_GetTick();
	break;

	case STATE_ACK_CONFIG_MODE:
		// Wait for GoToConfigAck to return
		// handle_gotoconfigack will fire when it does, and set our flag
		if (imu.ack_flag == ACK_CONFIG) {
			printf("IMU:IMU in config mode\n");
			imu.ack_flag = ACK_NONE;
			imu.imu_state = STATE_SET_OUTPUT_CONFIG;
		} else if (HAL_GetTick() - imu.timer_timeOut > 2000) {
			imu.timer_timeOut = HAL_GetTick();
			imu.imu_state = STATE_STARTUP;
			printf("IMU:again: ");
		}
	break;

	case STATE_SET_OUTPUT_CONFIG:
		printf("IMU:change Rot Setting\n");
		if (imu.changeRotSetting) {
//			changeRotMatrix(imu, bool Frame, float x, float y, float z);
//		changeRotMatrix(1, 0, 0, 0);
		}
//		changeRotMatrix(0, 0, 1.570795, 0);

		printf("IMU:Setting output rate to %d Hz\n", imu.output_Hz);
		changeOutputRate(imu.output_Hz);



		imu.imu_state = STATE_ACK_OUTPUT_CONFIG;
	break;

	case STATE_ACK_OUTPUT_CONFIG:
		// Read configuration packet response
		// TODO: MDATA2 odd input handling?

		if (imu.ack_flag == ACK_CONFIGURED) {
			printf("IMU:IMU confirmed config...\n");
			imu.imu_state = STATE_REQUEST_MEASUREMENT_MODE;
			imu.ack_flag = ACK_NONE;
		}
	break;

	case STATE_REQUEST_MEASUREMENT_MODE:
		printf("IMU:Requesting measurement mode...\n");
		xsens_mti_request(&imu.imu_interface, MT_GOTOMEASUREMENT);
		imu.imu_state = STATE_ACK_MEASUREMENT_MODE;
		break;

	case STATE_ACK_MEASUREMENT_MODE:
		// Wait x in this mode before attempting to configure different settings

		if (imu.ack_flag == ACK_MEASUREMENT) {
			// Go back to the start of the settings flow
			imu.imu_state = STATE_RUNNING;
			imu.ack_flag = ACK_NONE;
			printf("start running\n");
			imu.timer_timeOut = HAL_GetTick();
			imu.no_stuck=true;
		}
	break;

	case STATE_RUNNING:
		if(imu.IMU_prc_data == true){
			imu.timer_timeOut = HAL_GetTick();
			imu.no_stuck=true;
			imu.IMU_prc_data = false;
		}else if((imu.no_stuck == false) && (HAL_GetTick() - imu.timer_timeOut > 2000)){

			printf("recieve error\n");
			imu.imu_state = STATE_REQUEST_CONFIG_MODE;
			imu.timer_timeOut = HAL_GetTick();

		}else{
			imu.no_stuck=false;
		}

		if(imu.calided == false){
			xsens_mti_reset_orientation(&imu.imu_interface, XSENS_ORIENTATION_ALIGNMENT_RESET);
			imu.calided = true;
		}

//
//		if (EXT_got_data) {
//			char commandChar[10] = { 0 };
//			sscanf(EXT_buffer, "%s\n", commandChar);
//			printf("%s\n", commandChar);
//
//			if (strcmp(commandChar, "Reset") == 0) {
//				printf("process to reset\n");
//				xsens_mti_reset_orientation(&imu.imu_interface, XSENS_ORIENTATION_ALIGNMENT_RESET);
//			}
//
//			if (strcmp(commandChar, "setRot") == 0) {
//				printf("process to setRot\n");
//				imu.changeRotSetting = true;
//				imu.imu_state = STATE_REQUEST_CONFIG_MODE;
//			}
//

//		}
	break;
	default:
		// Oops!
		imu.imu_state = STATE_STARTUP;
		break;
	}
}

void imu_callback(XsensEventFlag_t event, XsensEventData_t *mtdata) {
	switch (event) {
	case XSENS_EVT_TEMPERATURE:
		if (mtdata->type == XSENS_EVT_TYPE_FLOAT) {
			imu.temp = mtdata->data.f4;
//			printf("get Temp\n");
		}
	break;

	case XSENS_EVT_QUATERNION:
		if (mtdata->type == XSENS_EVT_TYPE_FLOAT4) {
			imu.quaternionWXYZ[0] = mtdata->data.f4x4[0];
			imu.quaternionWXYZ[1] = mtdata->data.f4x4[1];
			imu.quaternionWXYZ[2] = mtdata->data.f4x4[2];
			imu.quaternionWXYZ[3] = mtdata->data.f4x4[3];
//			printf("get Quat\n");
		}
	break;

	case XSENS_EVT_RATE_OF_TURN:
		if (mtdata->type == XSENS_EVT_TYPE_FLOAT3) {
			imu.rateOfTurnXYZ[0] = mtdata->data.f4x3[0];
			imu.rateOfTurnXYZ[1] = mtdata->data.f4x3[1];
			imu.rateOfTurnXYZ[2] = mtdata->data.f4x3[2];
//      printf("get Rot\n");
		}
	break;

	case XSENS_EVT_FREE_ACCELERATION:
		if (mtdata->type == XSENS_EVT_TYPE_FLOAT3) {
			imu.freeAccelerationXYZ[0] = mtdata->data.f4x3[0];
			imu.freeAccelerationXYZ[1] = mtdata->data.f4x3[1];
			imu.freeAccelerationXYZ[2] = mtdata->data.f4x3[2];
//      printf("get Facc\n");
		}
	break;

	case XSENS_EVT_ACCELERATION:
		if (mtdata->type == XSENS_EVT_TYPE_FLOAT3) {
			imu.accelerationXYZ[0] = mtdata->data.f4x3[0];
			imu.accelerationXYZ[1] = mtdata->data.f4x3[1];
			imu.accelerationXYZ[2] = mtdata->data.f4x3[2];
//       printf("get Facc\n");
		}
	break;

	case XSENS_EVT_GNSS_PVT_DATA:
		if (mtdata->type == XSENS_EVT_TYPE_GNSS_DATA) {
			imu.myGnssData = gnssPvt_parse(mtdata->gnssPvtData);
//       printf("get Gnss data\n");
		}
	break;

	case XSENS_EVT_STATUS_WORD:
		if (mtdata->type == XSENS_EVT_TYPE_U32) {
			imu.status = mtdata->data.u4;
//			printf("get Sword\n");
		}
	break;

	case XDI_POSITION_ECEF:
		if (mtdata->type == XSENS_EVT_TYPE_FLOAT3) {
			imu.positionEcefXYZ[0] = mtdata->data.f4x3[0];
			imu.positionEcefXYZ[1] = mtdata->data.f4x3[1];
			imu.positionEcefXYZ[2] = mtdata->data.f4x3[2];
//      printf("get Facc\n");
		}
	break;

	case XSENS_EVT_LAT_LON:
		if (mtdata->type == XSENS_EVT_TYPE_FLOAT2) {
			imu.latitude = mtdata->data.f4x2[0];
			imu.longitude = mtdata->data.f4x2[1];
//			printf("get LatLon\n");
		}
	break;

	case XSENS_EVT_ALTITUDE_ELLIPSOID:
		if (mtdata->type == XSENS_EVT_TYPE_FLOAT) {
			imu.altitudeEllip = mtdata->data.f4;
//			printf("get Alt\n");
		}
	break;

	case XSENS_EVT_VELOCITY_XYZ:
		if (mtdata->type == XSENS_EVT_TYPE_FLOAT3) {
			imu.velocityXYZ[0] = mtdata->data.f4x3[0];
			imu.velocityXYZ[1] = mtdata->data.f4x3[1];
			imu.velocityXYZ[2] = mtdata->data.f4x3[2];
//			printf("get Vel\n");
		}
	break;

	}
}

// Command ACK callback functions
void handle_ack_gotoconfig(xsens_packet_buffer_t *packet) {
	imu.ack_flag = ACK_CONFIG;
}

void handle_ack_gotomeasurement(xsens_packet_buffer_t *packet) {
	imu.ack_flag = ACK_MEASUREMENT;
}

void handle_ack_outputconfiguration(xsens_packet_buffer_t *packet) {
	imu.ack_flag = ACK_CONFIGURED;
}

void myHandle_WAKEUP_cb(xsens_packet_buffer_t *packet) {
	imu.ack_flag = ACK_WAKEUP;
	printf("get Wake up\n");
}

// The library calls this function to send packets to the IMU
void imu_send_data( uint8_t *data, uint16_t length ) {
	HAL_UART_Transmit_DMA(&IMU_uart, data, length);
}

void changeRotMatrix(bool Frame, float x, float y, float z){
	imu.changeRotSetting = false;
	//rot setting
	float rotEuler[3] = { x, y, z };

	float rotQuaternion[4];
	XsensEventData_t rotData = { 0 };
	mdata2_packet_t rotPacket = { 0 };
	xsens_euler_to_quaternion(&rotEuler, &rotQuaternion);
	rotPacket.id = MT_SETALIGNMENTROTATION;
	rotPacket.length = 17;
	rotPacket.payload[0] = 0;
	rotPacket.payload[1] = Frame;
	for (int i = 2; i < 18; i += 4) {
		rotData.data.f4 = rotQuaternion[(i + 3) / 4 - 1];
		rotData.data.f4 = rotQuaternion[(i + 3) / 4 - 1];
		rotData.data.f4 = rotQuaternion[(i + 3) / 4 - 1];

		rotPacket.payload[i + 0] = rotData.data.u4 >> 24;
		rotPacket.payload[i + 1] = rotData.data.u4 >> 16;
		rotPacket.payload[i + 2] = rotData.data.u4 >> 8;
		rotPacket.payload[i + 3] = rotData.data.u4;
	}
	xsens_mti_send(&imu.imu_interface, &rotPacket);
}

void changeOutputRate(int output_rate){
	XsensFrequencyConfig_t settings[] = {
//			{ .id = XDI_PACKET_COUNTER, .frequency = 0xFFFF },
//			{ .id = XDI_SAMPLE_TIME_FINE, .frequency = 0xFFFF },
		{ .id = XDI_TEMPERATURE, 				.frequency = output_rate },
		{ .id =	XDI_QUATERNION, 				.frequency = output_rate },
		{ .id =	XDI_RATE_OF_TURN, 			.frequency = output_rate },
		{ .id = XDI_FREE_ACCELERATION, 	.frequency = output_rate },
		{ .id = XDI_ACCELERATION, 			.frequency = output_rate },
		{ .id = XDI_GNSS_PVT_DATA, 			.frequency = 4 },
		{ .id = XDI_STATUS_WORD, 				.frequency = output_rate },
		{ .id = XDI_POSITION_ECEF,			.frequency = output_rate },
		{ .id = XDI_LAT_LON, 						.frequency = 4 },
		{ .id = XDI_ALTITUDE_ELLIPSOID, .frequency = 4 },
		{ .id =	XDI_VELOCITY_XYZ, 			.frequency = output_rate },
//		  { .id = XSENS_IDENTIFIER_FORMAT(XDI_QUATERNION, XSENS_FLOAT_FIXED1220, XSENS_COORD_ENU), .frequency = 100 },
	};

	xsens_mti_set_configuration(&imu.imu_interface, settings, XSENS_ARR_ELEM(settings));

}
