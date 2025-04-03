/*
 * TMC2209.hpp
 *
 *  Created on: Mar 23, 2025
 *      Author: Henry
 */

#ifndef INC_TMC2209_HPP_
#define INC_TMC2209_HPP_

#include "stm32f4xx_hal.h"

#define SYNC 0x0005

class TMC2209{
	public:
	TMC2209(GPIO_TypeDef *port0, uint16_t ad0, GPIO_TypeDef *port1, uint16_t ad1, uint8_t addr, UART_HandleTypeDef *huart);


	private:
	typedef	struct
	{
		uint8_t data[4];
		uint8_t len = 4;
	}datagram32;

	typedef	struct
	{
		uint8_t data[8];
		uint8_t len = 8;
	}datagram64;

	void swuart_calcCRC(uint8_t* datagram, uint8_t datagramLength);
	datagram64 writeDatagram(uint8_t reg, uint32_t data);
	uint32_t reverse(uint32_t data);
	UART_HandleTypeDef *huart;
	uint8_t addr;
};

#endif /* INC_TMC2209_HPP_ */
