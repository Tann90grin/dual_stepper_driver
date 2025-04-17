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

void TMC2209::write(uint8_t reg, uint32_t data)
{
	datagram64 datagram = writeDatagram(reg + 0x80, data);
	HAL_UART_Transmit(this->huart, datagram.data, datagram.len, HAL_MAX_DELAY);
}
uint32_t TMC2209::read(uint8_t reg)
{
	request(reg);
	datagram64 datagram = reply();
	return replyData(datagram);
}

void TMC2209::request(uint8_t reg)
{
	datagram32 datagram = requestDatagram(reg);
	HAL_UART_Transmit(this->huart, datagram.data, datagram.len, HAL_MAX_DELAY);
}

datagram64 TMC2209::reply()
{
	datagram64 datagram;
	HAL_UART_Receive(this->huart, datagram.data, datagram.len, HAL_MAX_DELAY);
	return datagram;
}

uint32_t TMC2209::replyData(datagram64 datagram)
{
	uint32_t res;
	for(uint8_t i = 0; i < 4; i++)
	{
		res |= datagram.data[i] << ((3 - i) * 8);
	}
	return res;
}

datagram64 TMC2209::writeDatagram(uint8_t reg, uint32_t data)
{
    datagram64 datagram;
    datagram.data[0] = SYNC;
    datagram.data[1] = this->addr;
    datagram.data[2] = reg;
    uint32_t rev_data = reverse(data);
    memcpy(datagram.data + 3, &rev_data, 4);
    swuart_calcCRC(datagram.data, datagram.len);
    return datagram;
}

datagram32 TMC2209::requestDatagram(uint8_t reg)
{
	datagram32 datagram;
	datagram.data[0] = SYNC;
	datagram.data[1] = this->addr;
	datagram.data[2] = reg;
	swuart_calcCRC(datagram.data, datagram.len);
	return datagram;
}

uint32_t TMC2209::readData(datagram64 datagram)
{
	uint32_t data = 0x0000;
	uint8_t crc;

	memcpy(datagram.data + 3, &data, 4);
	crc = datagram.data[7];
	swuart_calcCRC(datagram.data, datagram.len);
	if(crc != datagram.data[7])
	{
		return UINT32_MAX;
	}
	return reverse(data);
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

void TMC2209::writeGCONF(uint32_t data)
{
	this->reg.GCONF.data = data;
	write(ADDRESS_GCONF, data);
}

void TMC2209::writeIHOLD_IRUN(uint32_t data)
{
	this->reg.IHOLD_IRUN.data = data;
	write(ADDRESS_IHOLD_IRUN, data);
}

void TMC2209::writeCHOPCONF(uint32_t data)
{
	this->reg.CHOPCONF.data = data;
	write(ADDRESS_CHOPCONF, data);
}
void TMC2209::writePWMCONF(uint32_t data)
{
	this->reg.PWMCONF.data = data;
	write(ADDRESS_PWMCONF, data);
}

void TMC2209::writeCOOLCONF(uint32_t data)
{
	this->reg.COOLCONF.data = data;
	write(ADDRESS_COOLCONF, data);
}

void TMC2209::writeTCOOLTHRS(uint32_t data)
{
	this->reg.TCOOLTHRS.data = data;
	write(ADDRESS_TCOOLTHRS, data);
}

void TMC2209::writeTPWMTHRS(uint32_t data)
{
	this->reg.TPWMTHRS.data = data;
	write(ADDRESS_TPWMTHRS, data);
}

void TMC2209::writeSGTHRS(uint32_t data)
{
	this->SGTHRS = data;
	write(ADDRESS_SGTHRS, data);
}

void TMC2209::writeTPOWERDOWN(uint32_t data)
{
	this->TPOWERDOWN = data;
	write(ADDRESS_TPOWERDOWN, data);
}

uint32_t TMC2209::readIOIN()
{
	uint32_t data = read(ADDRESS_IOIN);
	this->reg.IOIN.data = data;
	return data;
}

uint32_t TMC2209::readSG_RESULT()
{
	uint32_t data = read(ADDRESS_SG_RESULT);
	this->SG_RESULT = data;
	return data;
}

uint32_t TMC2209::readIFCNT()
{
	uint32_t data = read(ADDRESS_IFCNT);
	this->IFCNT = data;
	return data;
}

