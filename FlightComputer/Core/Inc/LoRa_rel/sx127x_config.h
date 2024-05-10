/*
 * sx127x_config.h
 *
 *  Created on: Aug 9, 2023
 *  last modified:20230809
 *      Author: Alex
 *      version:1.0
 *
 *      description: This file define LoRa object content
 */

#ifndef INC_SX127X_CONFIG_H_
#define INC_SX127X_CONFIG_H_



/* Private includes ----------------------------------------------------------*/
#include "main.h"

#ifndef LoRa_MODE //for FSK modem function


/* Private defines -----------------------------------------------------------*/

//LoRa parameters
#define FSK_MODEM false
#define LORA_MODEM true

#define TRANSMIT_TIMEOUT 500
#define RECEIVE_TIMEOUT 500

typedef struct {
//	bool lora;                 //LoRa modem:true, FSK modem:false
//	bool tcxo;
//	bool pa_boost;
//	bool paDac;                // Enables the +20dBm option on PA_BOOST pin
//	uint8_t frequency;         //MHz
//	uint8_t power;             //dBm
//	uint8_t spreadingFactor;
//	uint8_t bandwidth;
//	uint8_t codingRate;
//	uint8_t payloadLength;
//	bool implicit_header;
//	bool crc_on;
	//pin set
	SPI_HandleTypeDef* hSPIx;
	GPIO_TypeDef*      CS_port;
	uint16_t			     CS_pin;
	GPIO_TypeDef*      reset_port;
	uint16_t			     reset_pin;
	GPIO_TypeDef*      DIO0_port;
	uint16_t			     DIO0_pin;
	GPIO_TypeDef*      DIO1_port;
	uint16_t			     DIO1_pin;
	GPIO_TypeDef*      DIO2_port;
	uint16_t			     DIO2_pin;

	//LoRa parameters
	uint8_t   current_mode;    //current operation mode(standby, sleep,...)
	bool      LoRa_modem;       //LoRa modem:true, FSK modem:false
	uint16_t  frequency;     //MHz
	uint8_t	  bandWidth;
	uint8_t	  crcRate;
	uint8_t   implicit_on;
	uint8_t	  spredingFactor;
	uint16_t  preamble;
	uint8_t   paselect;
	uint8_t   maxpower;
	uint8_t	  outputpower;
	uint8_t   PaDac;
	uint8_t   PaOcp;
	uint8_t   CRCon;
	bool      TCXOon;

	//Packet parameters
	uint8_t packetSize;

#ifdef LORA_MODE
	uint8_t	  bandWidth;
	uint8_t	  crcRate;
	uint16_t  preamble;
#endif

}LoRa;





#endif

#endif /* INC_SX127X_CONFIG_H_ */
