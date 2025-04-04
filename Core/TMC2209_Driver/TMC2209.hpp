/*
 * TMC2209.hpp
 *
 *  Created on: Mar 23, 2025
 *      Author: Henry
 */

#ifndef INC_TMC2209_HPP_
#define INC_TMC2209_HPP_

#include "stm32f4xx_hal.h"

//Sync byte
#define SYNC 0x0005

//General reg map
#define GCONF 0x00 //Global conf flags
#define GSTAT 0x01 //Global stat flags
#define IFCNT 0x02 //Interface transmission counter
#define NODECONF 0x03 //Send delay for read access
#define OTP_PROG 0x04
#define OTP_READ 0x05
#define IOIN 0X06
#define FACTORY_CONF 0x07

//Velocity Dependent Control Reg
#define IHOLD_IRUN 0x10
#define TPOWERDOWN 0x11
#define TSTEP 0x12
#define TPWMTHRS 0x13
#define VACTUAL 0x22

//StallGaurd Control Reg
#define TCOOLTHRS 0x14
#define SGTHRS 0x40
#define SG_RESULT 0x41
#define COOL_CONF 0x42

//Sequencer Reg
#define MSCNT 0x6A
#define MSCURACT 0x6B

//Chopper Control Reg
#define CHOPCONF 0x6C
#define DRV_STATUS 0x6F
#define PWMCONF 0x70
#define PWM_SCALE 0x71
#define PWM_AUTO 0x72

class TMC2209{
	public:
	TMC2209(GPIO_TypeDef *port0, uint16_t ad0, GPIO_TypeDef *port1, uint16_t ad1, uint8_t addr, UART_HandleTypeDef *huart);
	void write(uint8_t reg, uint32_t data);
	uint32_t request(uint8_t reg);

	private:
	typedef	struct
	{
		/*
		 * Byte 0: Sync + reserved(don't care)
		 * Byte 1: Node addr
		 * Byte 2: RW bit + reg (Add 0x80 to reg address for W)
		 * Byte 3-6: Data (in MSB order 3, 2, 1, 0
		 * Byte 7: CRC
		 */
		uint8_t data[4];
		uint8_t len = 4;
	}datagram32;

	typedef	struct
	{
		/*
		 * Byte 0: Sync + reserved(don't care)
		 * Byte 1: Node addr
		 * Byte 2: RW bit + reg (Add 0x80 to reg address for W)
		 * Byte 3: CRC
		 */
		uint8_t data[8];
		uint8_t len = 8;
	}datagram64;

	void swuart_calcCRC(uint8_t* datagram, uint8_t datagramLength);
	datagram64 writeDatagram(uint8_t reg, uint32_t data);
	datagram32 requestDatagram(uint8_t reg);
	uint32_t requestData(datagram64 datagram);
	uint32_t reverse(uint32_t data);
	UART_HandleTypeDef *huart;
	uint8_t addr;
};

#endif /* INC_TMC2209_HPP_ */
