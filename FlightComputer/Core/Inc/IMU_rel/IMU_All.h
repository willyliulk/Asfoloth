#ifndef IMU_ALL_H
#define IMU_ALL_H

#include "usart.h"
#include "dma.h"

#include "IMU_rel/xsens_mti.h"      // Main library
#include "IMU_rel/xsens_utility.h"  // Needed for quaternion conversion function
#include "IMU_rel/xsens_mdata2.h"
#include "IMU_rel/float16Tool.h"
#include "IMU_rel/gnssPvtDataParser.h"



#define IMU_BUFFER_SIZE 2048

#define EXT_BUFFER_SIZE 1024
extern uint8_t EXT_buffer[EXT_BUFFER_SIZE];
extern bool EXT_got_data;
extern uint16_t EXT_data_Size;


//IMU ACK message
typedef enum {
    ACK_NONE = 0,
    ACK_CONFIG,
    ACK_MEASUREMENT,
    ACK_CONFIGURED,
	ACK_WAKEUP
} ACKFlags_t;


// IMU state mechine
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
} IMU_State_t;


typedef struct{
	//IMU dma setup
	uint8_t IMU_buffer[IMU_BUFFER_SIZE];
	bool IMU_got_data;
	uint16_t IMU_data_Size;
	UART_HandleTypeDef *IMU_uart_handle;

	// The library holds state and pointers to callbacks in this structure
	//   - adding the imu_send_data ptr lets the library talk to the IMU
	xsens_interface_t imu_interface;

	//IMU ACK message
	ACKFlags_t ack_flag;

	//IMU state mechine
	IMU_State_t imu_state;

	//IMU timer for every timeout record usage
	uint32_t timer_timeOut;
	bool no_stuck;
	bool calided;

	bool changeRotSetting;
	uint8_t output_Hz;


	//IMU data
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

} IMU;


void IMU_UART_CB(UART_HandleTypeDef *huart, uint16_t Size);
void imu_send_data( uint8_t *data, uint16_t length );

void IMU_Init();

void IMU_process_data();
void IMU_State_mechine();


// Called when the library decoded an inbound packet
//   - If the packet was an MData2 frame (which contains packed motion data)
//   - the callback is called once for each sub-field
void imu_callback( XsensEventFlag_t event, XsensEventData_t *mtdata );

// Callback functions we'll use to catch IMU ack packets
void handle_ack_gotoconfig( xsens_packet_buffer_t *packet );
void handle_ack_gotomeasurement( xsens_packet_buffer_t *packet );
void handle_ack_outputconfiguration( xsens_packet_buffer_t *packet );
void myHandle_WAKEUP_cb( xsens_packet_buffer_t *packet );

void changeRotMatrix(bool Frame, float x, float y, float z);
void changeOutputRate(int output_rate);


#endif
