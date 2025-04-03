/*
 * TMC2209.cpp
 *
 *  Created on: Mar 23, 2025
 *      Author: henry
 */

#include "TMC2209.hpp"

TMC2209::TMC2209(GPIO_TypeDef *port0, uint16_t ad0, GPIO_TypeDef *port1, uint16_t ad1, uint8_t addr, UART_HandleTypeDef *huart)
{
	HAL_GPIO_WritePin(port0, ad0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(port1, ad1, GPIO_PIN_RESET);
	if(addr & 0x01)
	{
		HAL_GPIO_WritePin(port0, ad0, GPIO_PIN_SET);
	}
	if(addr & 0x10)
	{
		HAL_GPIO_WritePin(port1, ad1, GPIO_PIN_SET);
	}

	this->huart = huart;
	this->addr = addr;
}

datagram64 TMC2209::writeDatagram(uint8_t reg, uint32_t data)
{
    datagram64 datagram;
    datagram.data[0] = SYNC;
    datagram.data[1] = this->addr;
    datagram.data[2] = reg;
    memcpy(datagra.data + 3, reverse(data), 4);
    swuart_calcCRC(datagram.data, datagram.len);
    return datagram;
}

uint32_t TMC2209::reverse(uint32_t data){
	uint32_t res;
	uint8_t rshift;
	uint8_t lshift;

	for(uint8_t i = 0; i < 4; i++)
	{
		rshift = (3 - i) * 8;
		lshift = i * 8;
		res |= ((data >> rshift) & 0xFF) << lshift;
	}
	return res;
}

void TMC2209::swuart_calcCRC(uint8_t* datagram, uint8_t datagramLength)
{
	int i,j;
	uint8_t* crc = datagram + (datagramLength-1); // CRC located in last byte of message
	uint8_t currentByte;
	*crc = 0;
	for (i=0; i<(datagramLength-1); i++) { // Execute for all bytes of a message
		currentByte = datagram[i]; // Retrieve a byte to be sent from Array
		for (j=0; j<8; j++) {
			if ((*crc >> 7) ^ (currentByte&0x01)) // update CRC based result of XOR operation
			{
				*crc = (*crc << 1) ^ 0x07;
			}
			else
			{
				*crc = (*crc << 1);
			}
			currentByte = currentByte >> 1;
		} // for CRC bit
	} // for message byte
}



