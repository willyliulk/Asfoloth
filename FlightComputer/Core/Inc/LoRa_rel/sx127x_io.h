/*
 * sx127x_io.h
 *
 *  Created on: Aug 9, 2023
 *  last modified:20230809
 *      Author: Alex
 *      version:1.0
 *
 *      description: This file provides LoRa read/write function
 */

#ifndef INC_SX127X_IO_H_
#define INC_SX127X_IO_H_

#include "LoRa_rel/sx127x_config.h"

#ifndef LoRa_MODE //for FSK modem function
//read/write function
//fundamental read function using HAL transmit
void LoRa_readReg(LoRa* _LoRa, uint8_t* address, uint16_t r_length, uint8_t* output, uint16_t w_length);
//fundamental write function using HAL transmit
void LoRa_writeReg(LoRa* _LoRa, uint8_t* address, uint16_t r_length, uint8_t* values, uint16_t w_length);
//packed read function using above LoRa_readReg function, only read one byte data!!!
uint8_t LoRa_read_single(LoRa* _LoRa, uint8_t address);
//packed write function using above LoRa_writeReg function, only write one byte data!!!
void LoRa_write_single(LoRa* _LoRa, uint8_t address, uint8_t value);
//write several bytes into register using HAL transmit
void LoRa_BurstWrite(LoRa* _LoRa, uint8_t address, uint8_t *value, uint8_t length);




//packed function using read/write function
void LoRa_gotoMode(LoRa* _LoRa, uint8_t mode);


//transmit data by RF in LoRa modem
uint8_t LoRa_transmit(LoRa* _LoRa, uint8_t* data, uint8_t length, uint16_t timeout);
//change into continue receive mode
void LoRa_startReceiving(LoRa* _LoRa);
//receive data by RF in LoRa modem
uint8_t LoRa_receive(LoRa* _LoRa, uint8_t* data, uint8_t length);

//initialize LoRa configuration
uint16_t LoRa_init(LoRa* _LoRa);
//reset LoRa
void LoRa_reset(LoRa* _LoRa);
//set LoRa frequency
void LoRa_setFrequency(LoRa* _LoRa, uint16_t freq);
//set spreading factor
void LoRa_setSpreadingFactor(LoRa* _LoRa, uint8_t SP);
//set clock reference
void SX127x_set_Tcxo(LoRa* _LoRa, bool on);
//set CRC on
void LoRa_set_CRCon(LoRa* _LoRa, uint8_t CRCvalue);
//check LoRa is available(need to update)
uint8_t LoRa_isvalid(LoRa* _LoRa);
//check LoRa is transmitting or not
bool isTransmitting(LoRa* _LoRa);

#endif

#endif /* INC_SX127X_IO_H_ */
