/*
 * sx127x_io.c
 *
 *  Created on: Aug 9, 2023
 *  last modified:20230809
 *      Author: Alex
 *      version:1.0
 */

#include "LoRa_rel/sx127x_io.h"


/* ----------------------------------------------------------------------------- *\
		name        : LoRa_readReg

		description : read a register(s) by an address and a length,
									then store value(s) at outpur array.
		arguments   :
			LoRa* LoRa        --> LoRa object handler
			uint8_t* address  -->	pointer to the beginning of address array
			uint16_t r_length -->	detemines number of addresse bytes that
														you want to send
			uint8_t* output		--> pointer to the beginning of output array
			uint16_t w_length	--> detemines number of bytes that you want to read

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_readReg(LoRa* _LoRa, uint8_t* address, uint16_t r_length, uint8_t* output, uint16_t w_length){
	HAL_GPIO_WritePin(_LoRa->CS_port, _LoRa->CS_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(_LoRa->hSPIx, address, r_length, TRANSMIT_TIMEOUT);
	while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY);
	HAL_SPI_Receive(_LoRa->hSPIx, output, w_length, RECEIVE_TIMEOUT);
	while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(_LoRa->CS_port, _LoRa->CS_pin, GPIO_PIN_SET);

//	HAL_GPIO_WritePin(_LoRa->CS_port, _LoRa->CS_pin, GPIO_PIN_RESET);
//	HAL_SPI_Transmit_DMA(_LoRa->hSPIx, address, r_length);
//	while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY);
//	HAL_SPI_Receive_DMA(_LoRa->hSPIx, output, w_length);
//	while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY);
//	HAL_GPIO_WritePin(_LoRa->CS_port, _LoRa->CS_pin, GPIO_PIN_SET);
}


/* ----------------------------------------------------------------------------- *\
		name        : LoRa_writeReg

		description : write a value(s) in a register(s) by an address

		arguments   :
			LoRa* LoRa        --> LoRa object handler
			uint8_t* address  -->	pointer to the beginning of address array
			uint16_t r_length -->	detemines number of addresse bytes that
														you want to send
			uint8_t* output		--> pointer to the beginning of values array
			uint16_t w_length	--> detemines number of bytes that you want to send

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_writeReg(LoRa* _LoRa, uint8_t* address, uint16_t r_length, uint8_t* values, uint16_t w_length){
	HAL_GPIO_WritePin(_LoRa->CS_port, _LoRa->CS_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(_LoRa->hSPIx, address, r_length, TRANSMIT_TIMEOUT);
	while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY);
	HAL_SPI_Transmit(_LoRa->hSPIx, values, w_length, TRANSMIT_TIMEOUT);
	while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(_LoRa->CS_port, _LoRa->CS_pin, GPIO_PIN_SET);

//	HAL_GPIO_WritePin(_LoRa->CS_port, _LoRa->CS_pin, GPIO_PIN_RESET);
//	HAL_SPI_Transmit_DMA(_LoRa->hSPIx, address, r_length);
//	while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY);
//	HAL_SPI_Transmit_DMA(_LoRa->hSPIx, values, w_length);
//	while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY);
//	HAL_GPIO_WritePin(_LoRa->CS_port, _LoRa->CS_pin, GPIO_PIN_SET);

}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_read

		description : read a register by an address

		arguments   :
			LoRa*   LoRa        --> LoRa object handler
			uint8_t address     -->	address of the register e.g 0x1D

		returns     : register value
\* ----------------------------------------------------------------------------- */
uint8_t LoRa_read_single(LoRa* _LoRa, uint8_t address){
	uint8_t read_data;
	uint8_t data_addr;

	data_addr = address & 0x7F;
	LoRa_readReg(_LoRa, &data_addr, 1, &read_data, 1);
	//HAL_Delay(5);

	return read_data;
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_write

		description : write a value in a register by an address

		arguments   :
			LoRa*   LoRa        --> LoRa object handler
			uint8_t address     -->	address of the register e.g 0x1D
			uint8_t value       --> value that you want to write

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_write_single(LoRa* _LoRa, uint8_t address, uint8_t value){
	uint8_t data;
	uint8_t addr;

	addr = address | 0x80; // address 8 bits + 1000 0000
	data = value;
	LoRa_writeReg(_LoRa, &addr, 1, &data, 1);
	//HAL_Delay(5);
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_BurstWrite

		description : write a set of values in a register by an address respectively

		arguments   :
			LoRa*   LoRa        --> LoRa object handler
			uint8_t address     -->	address of the register e.g 0x1D
			uint8_t *value      --> address of values that you want to write
			uint8_t length      --> length of wrote value

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_BurstWrite(LoRa* _LoRa, uint8_t address, uint8_t *value, uint8_t length){
	uint8_t addr;
	addr = address | 0x80;

//	//NSS = 1
	HAL_GPIO_WritePin(_LoRa->CS_port, _LoRa->CS_pin, GPIO_PIN_RESET);
	//say module that I want to write in RegFiFo
	HAL_SPI_Transmit(_LoRa->hSPIx, &addr, 1, TRANSMIT_TIMEOUT);
	while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY);
	//Write data in FiFo
	HAL_SPI_Transmit(_LoRa->hSPIx, value, length, TRANSMIT_TIMEOUT);
	while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY);
	//NSS = 0
	//HAL_Delay(5);
	HAL_GPIO_WritePin(_LoRa->CS_port, _LoRa->CS_pin, GPIO_PIN_SET);

	//NSS = 1
//	HAL_GPIO_WritePin(_LoRa->CS_port, _LoRa->CS_pin, GPIO_PIN_RESET);
//	//say module that I want to write in RegFiFo
//	HAL_SPI_Transmit_DMA(_LoRa->hSPIx, &addr, 1);
//	while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY);
//	//Write data in FiFo
//	HAL_SPI_Transmit_DMA(_LoRa->hSPIx, value, length);
//	while (HAL_SPI_GetState(_LoRa->hSPIx) != HAL_SPI_STATE_READY);
//	//NSS = 0
//	//HAL_Delay(5);
//	HAL_GPIO_WritePin(_LoRa->CS_port, _LoRa->CS_pin, GPIO_PIN_SET);

}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_transmit

		description : Transmit data

		arguments   :
			LoRa*    LoRa     --> LoRa object handler
			uint8_t  data			--> A pointer to the data you wanna send
			uint8_t	 length   --> Size of your data in Bytes
			uint16_t timeOut	--> Timeout in milliseconds
		returns     : 1 in case of success, 0 in case of timeout
\* ----------------------------------------------------------------------------- */
uint8_t LoRa_transmit(LoRa* _LoRa, uint8_t* data, uint8_t length, uint16_t timeout){
	uint8_t read;

	if(isTransmitting(_LoRa)){
		return 0;
	}

	uint8_t mode = _LoRa->current_mode;
	LoRa_gotoMode(_LoRa, STANDBY_MODE);
	//reset FIFO address and payload length

	//transmit data fifo filling
	read = LoRa_read_single(_LoRa, SX127x_LoRa_FifoTxBaseAddr);  //LoRa_read_single return register value
	LoRa_write_single(_LoRa, SX127x_LoRa_FifoAddrPtr, 0);
	LoRa_write_single(_LoRa, SX127x_LoRa_PayloadLength, length);
	LoRa_BurstWrite(_LoRa, SX127x_Fifo, data, length);
	LoRa_gotoMode(_LoRa, TRANSMIT_MODE);

	uint32_t timer=HAL_GetTick();
	uint32_t cter=0;
	while(1){
		read = LoRa_read_single(_LoRa, SX127x_LoRa_IrqFlags);
		if((read & 0x08)!=0){                                  //only check the 3rd bit of read, if 1 -> Tx done
			LoRa_write_single(_LoRa, SX127x_LoRa_IrqFlags, 0x08);  //clear Tx done flag
			LoRa_gotoMode(_LoRa, mode);
			timer = HAL_GetTick() - timer;
			return 1;
		}
		else{
			if(--timeout==0){
				LoRa_gotoMode(_LoRa, mode);
				timer = HAL_GetTick() - timer;
				return 0;
			}
		}
//		HAL_Delay(1);
		for(uint32_t delay=0; delay<1000; delay++){
			asm("nop");
		}
		cter++;
	}
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_startReceiving

		description : Start receiving continuously

		arguments   :
			LoRa*    LoRa     --> LoRa object handler

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_startReceiving(LoRa* _LoRa){
	LoRa_gotoMode(_LoRa, RXCONTIN_MODE);
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_Receive

		description : Read received data from module

		arguments   :
			LoRa*    LoRa     --> LoRa object handler
			uint8_t  data			--> A pointer to the array that you want to write bytes in it
			uint8_t	 length   --> Determines how many bytes you want to read

		returns     : The number of bytes received
\* ----------------------------------------------------------------------------- */
uint8_t LoRa_receive(LoRa* _LoRa, uint8_t* data, uint8_t length){
	uint8_t read;
	uint8_t number_of_bytes;
	uint8_t min = 0;

	for(int i=0; i<length; i++)
		data[i]=0;

//	LoRa_gotoMode(_LoRa, STANDBY_MODE);
	read = LoRa_read_single(_LoRa, SX127x_LoRa_IrqFlags);
	if(read!= 0){

	}
	if((read & 0x40) != 0){
		LoRa_write_single(_LoRa, SX127x_LoRa_IrqFlags, 0xFF);
		number_of_bytes = LoRa_read_single(_LoRa, SX127x_LoRa_RxNbBytes);
		read = LoRa_read_single(_LoRa, SX127x_LoRa_FifoRxCurrentAddr);
		LoRa_write_single(_LoRa, SX127x_LoRa_FifoAddrPtr, read);
		min = length >= number_of_bytes ? number_of_bytes : length;
		for(int i=0; i<min; i++)
			data[i] = LoRa_read_single(_LoRa, SX127x_Fifo);
	}
	LoRa_gotoMode(_LoRa, RXCONTIN_MODE);
	return min;
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_gotoMode

		description : set LoRa Op mode

		arguments   :
			LoRa* LoRa    --> LoRa object handler
			mode	        --> select from defined modes

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_gotoMode(LoRa* _LoRa, uint8_t mode){
	uint8_t    read;
	uint8_t    data;

	read = LoRa_read_single(_LoRa, SX127x_OpMode);
	data = read;

	if(mode == SLEEP_MODE){
		data = (read & 0xF8) | SLEEP_MODE;        //(x&0xF8): remain first 5 bits state, (x|0x00): change last 3 bits state
		_LoRa->current_mode = SLEEP_MODE;
	}else if (mode == STANDBY_MODE){
		data = (read & 0xF8) | STANDBY_MODE;
		_LoRa->current_mode = STANDBY_MODE;
	}else if (mode == TRANSMIT_MODE){
		data = (read & 0xF8) | TRANSMIT_MODE;
		_LoRa->current_mode = TRANSMIT_MODE;
	}else if (mode == RXCONTIN_MODE){
		data = (read & 0xF8) | RXCONTIN_MODE;
		_LoRa->current_mode = RXCONTIN_MODE;
	}else if (mode == RXSINGLE_MODE){
		data = (read & 0xF8) | RXSINGLE_MODE;
		_LoRa->current_mode = RXSINGLE_MODE;
	}

	LoRa_write_single(_LoRa, SX127x_OpMode, data);
//	HAL_Delay(10);
}





/* ----------------------------------------------------------------------------- *\
		name        : LoRa_init

		description : initialize and set the right setting according to LoRa sruct vars

		arguments   :
			LoRa* LoRa        --> LoRa object handler

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
uint16_t LoRa_init(LoRa* _LoRa){
	uint8_t    data;
	uint8_t    read;

	LoRa_reset(_LoRa);

	while(LoRa_isvalid(_LoRa)!=1);               //wait until LoRa available
	// goto sleep mode:
	LoRa_gotoMode(_LoRa, SLEEP_MODE);
	HAL_Delay(10);

	SX127x_set_Tcxo(_LoRa, _LoRa->TCXOon);

	// turn into LoRa modem/Low frequency mode:
	read = LoRa_read_single(_LoRa, SX127x_OpMode);      //read current state of operation mode
	HAL_Delay(10);
	uint8_t LowFreqModeOn = 0;
	if(_LoRa->frequency<500){
		LowFreqModeOn = 1;
	}
	data = read | (_LoRa->LoRa_modem << 7) | (LowFreqModeOn<<3);             //needed operation mode register value
	LoRa_write_single(_LoRa, SX127x_OpMode, data);
	HAL_Delay(10);

	// set frequency:
	LoRa_setFrequency(_LoRa, _LoRa->frequency);

	// set bandwidth, coding rate and expli
	data = (_LoRa->bandWidth << 4) | (_LoRa->crcRate << 1) | (_LoRa->implicit_on << 0);
	LoRa_write_single(_LoRa, SX127x_LoRa_ModemConfig, data);
	HAL_Delay(10);

	// set spreading factor, CRC on, and Timeout Msb:
//	LoRa_setTOMsb_setCRCon(_LoRa);
	LoRa_setSpreadingFactor(_LoRa, _LoRa->spredingFactor);
	//enable CRC
	LoRa_set_CRCon(_LoRa, _LoRa->CRCon);

	// set preamble:
	LoRa_write_single(_LoRa, SX127x_LoRa_PreambleMsb, _LoRa->preamble >> 8);
	HAL_Delay(10);
	LoRa_write_single(_LoRa, SX127x_LoRa_PreambleLsb, _LoRa->preamble >> 0);
	HAL_Delay(10);

	// set output power gain:
	data = (_LoRa->paselect << 7) | (_LoRa->maxpower << 4) | (_LoRa->outputpower << 0);
	LoRa_write_single(_LoRa, SX127x_PaConfig, data);
	HAL_Delay(10);

    //20dBm output
	LoRa_write_single(_LoRa, SX127x_PaDac, _LoRa->PaDac);

	//set OCP current protect
	data = (0x01 << 5) | (_LoRa->PaOcp);
	LoRa_write_single(_LoRa, SX127x_Ocp, data);
	HAL_Delay(10);
//
//		// set over current protection:
//			LoRa_setOCP(_LoRa, _LoRa->overCurrentProtection);
//
//		// set LNA gain:
//			LoRa_write(_LoRa, RegLna, 0x23);

	//normal I&Q
//	LoRa_write_single(_LoRa, SX127x_LoRa_InvertIQ, 0x26);
//	LoRa_write_single(_LoRa, SX127x_LoRa_InvertIQ2, 0x1d);
	//invert I&Q
	LoRa_write_single(_LoRa, SX127x_LoRa_InvertIQ, 0x67);
	LoRa_write_single(_LoRa, SX127x_LoRa_InvertIQ2, 0x19);
	HAL_Delay(10);

//		// set Timeout Lsb:
//			LoRa_write(_LoRa, RegSymbTimeoutL, 0xFF);

	// set base addresses
	LoRa_write_single(_LoRa, SX127x_LoRa_FifoRxBaseAddr, 0);
	LoRa_write_single(_LoRa, SX127x_LoRa_FifoTxBaseAddr, 0);



//		// DIO mapping:   --> DIO: RxDone
	read = LoRa_read_single(_LoRa, SX127x_DioMapping1);
	data = read | 0x3F;
	LoRa_write_single(_LoRa, SX127x_DioMapping1, data);
//
	// goto standby mode:
	LoRa_gotoMode(_LoRa, STANDBY_MODE);
	_LoRa->current_mode = STANDBY_MODE;
	HAL_Delay(10);

	return LORA_OK;
//
//			read = LoRa_read(_LoRa, RegVersion);
//			if(read == 0x12)
//				return LORA_OK;
//			else
//				return LORA_NOT_FOUND;
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_reset

		description : reset module

		arguments   :
			LoRa* LoRa --> LoRa object handler

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_reset(LoRa* _LoRa){
	HAL_GPIO_WritePin(_LoRa->reset_port, _LoRa->reset_pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(_LoRa->reset_port, _LoRa->reset_pin, GPIO_PIN_SET);
	HAL_Delay(100);
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_setFrequency

		description : set carrier frequency e.g 433 MHz

		arguments   :
			LoRa* LoRa        --> LoRa object handler
			int   freq        --> desired frequency in MHz unit, e.g 434

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_setFrequency(LoRa* _LoRa, uint16_t freq){
	uint8_t  data;
	uint32_t Frf;
	Frf = (freq * 524288) / 32;       //Frf = fRF*(2^19)/F(XOSC), 2^19 = 524288, F(XOSC) usually = 32MHz, Frf = 0xABCDEF

	// write Msb(0xAB):
	data = Frf >> 16;
	LoRa_write_single(_LoRa, SX127x_FrMsb, data);
	HAL_Delay(1);

	// write Mid(0xCD):
	data = Frf >> 8;
	LoRa_write_single(_LoRa, SX127x_FrMid, data);
	HAL_Delay(1);

	// write Lsb(0xEF):
	data = Frf >> 0;
	LoRa_write_single(_LoRa, SX127x_FrLsb, data);
	HAL_Delay(1);
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_setSpreadingFactor

		description : set spreading factor, from 7 to 12.

		arguments   :
			LoRa* LoRa        --> LoRa object handler
			int   SP          --> desired spreading factor e.g 7

		returns     : Nothing
\* ----------------------------------------------------------------------------- */
void LoRa_setSpreadingFactor(LoRa* _LoRa, uint8_t SF){
	uint8_t	data;
	uint8_t	read;

	if (SF < 6) {
		SF = 6;
	} else if (SF > 12) {
		SF = 12;
	}

	if (SF == 6) {
		LoRa_write_single(_LoRa, SX127x_LoRa_DetectOptimize, 0xc5);
		LoRa_write_single(_LoRa, SX127x_LoRa_DetectionThreshold, 0x0c);
	} else {
		LoRa_write_single(_LoRa, SX127x_LoRa_DetectOptimize, 0xc3);
		LoRa_write_single(_LoRa, SX127x_LoRa_DetectionThreshold, 0x0a);
	}

	read = LoRa_read_single(_LoRa, SX127x_LoRa_ModemConfig2);
	HAL_Delay(1);

	data = ((SF << 4)) | (read & 0x0F);
	LoRa_write_single(_LoRa, SX127x_LoRa_ModemConfig2, data);
	HAL_Delay(1);
}

/* ----------------------------------------------------------------------------- *\
		name        : SX127x_set_Tcxo

		description : set the clock reference

		arguments   :
			bool on --> LoRa object handler

		returns     : void
\* ----------------------------------------------------------------------------- */
void SX127x_set_Tcxo(LoRa* _LoRa, bool on)
{
    uint8_t	data;
    data = on << 4;
    LoRa_write_single(_LoRa, SX127x_Tcxo, data);
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_isvalid

		description : check the LoRa instruct values

		arguments   :
			LoRa* LoRa --> LoRa object handler

		returns     : returns 1 if all of the values were given, otherwise returns 0
\* ----------------------------------------------------------------------------- */
void LoRa_set_CRCon(LoRa* _LoRa, uint8_t CRCvalue){
	uint8_t	data;
	uint8_t	read;
	read = LoRa_read_single(_LoRa, SX127x_LoRa_ModemConfig2);
	HAL_Delay(1);

	data = (CRCvalue << 2) | (read | 0x04);
	LoRa_write_single(_LoRa, SX127x_LoRa_ModemConfig2, data);
	HAL_Delay(1);
}

/* ----------------------------------------------------------------------------- *\
		name        : LoRa_isvalid

		description : check the LoRa instruct values

		arguments   :
			LoRa* LoRa --> LoRa object handler

		returns     : returns 1 if all of the values were given, otherwise returns 0
\* ----------------------------------------------------------------------------- */
uint8_t LoRa_isvalid(LoRa* _LoRa){

	return 1;
}

/* ----------------------------------------------------------------------------- *\
		name        : isTransmitting

		description : check LoRa is transmitting or not

		arguments   :
			LoRa* LoRa --> LoRa object handler

		returns     : returns 1 if all of the values were given, otherwise returns 0
\* ----------------------------------------------------------------------------- */
bool isTransmitting(LoRa* _LoRa)
{
  if ((LoRa_read_single(_LoRa, SX127x_OpMode) & 0x07) == TRANSMIT_MODE) {
    return true;
  }

//  if (LoRa_read_single(_LoRa, SX127x_LoRa_IrqFlags) & 0x08) {
//    // clear IRQ's
//	  LoRa_write_single(_LoRa, SX127x_LoRa_IrqFlags, 0x08);
//  }
  return false;
}
