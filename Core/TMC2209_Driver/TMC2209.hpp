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
constexpr uint8_t SYNC = 0x0005;

//Registers addresses with WRITE access
constexpr uint8_t ADDRESS_GCONF = 0x00;
constexpr uint8_t ADDRESS_SLAVECONF = 0x03;
constexpr uint8_t ADDRESS_IHOLD_IRUN = 0x10;
constexpr uint8_t ADDRESS_TPOWERDOWN = 0x11;
constexpr uint8_t ADDRESS_TPWMTHRS = 0x13;
constexpr uint8_t ADDRESS_TCOOLTHRS = 0x14;
constexpr uint8_t ADDRESS_SGTHRS = 0x40;
constexpr uint8_t ADDRESS_COOLCONF = 0x42;
constexpr uint8_t ADDRESS_CHOPCONF = 0x6C;
constexpr uint8_t ADDRESS_PWMCONF = 0x70;

//Registers addresses with READ access
constexpr uint8_t ADDRESS_GSTAT = 0x01;
constexpr uint8_t ADDRESS_IFCNT = 0x02;
constexpr uint8_t ADDRESS_IOIN = 0x06;
constexpr uint8_t ADDRESS_SG_RESULT = 0x41;
constexpr uint8_t ADDRESS_DRV_STATUS = 0X6F;

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

typedef struct
{

};

class TMC2209{
	public:
	TMC2209(GPIO_TypeDef *port0, uint16_t ad0, GPIO_TypeDef *port1, uint16_t ad1, uint8_t addr, UART_HandleTypeDef *huart);

	void write(uint8_t reg, uint32_t data);
	uint32_t read(uint8_t reg);

    void writeGCONF(uint32_t data);
    void writeIHOLD_IRUN(uint32_t data);
    void writeCHOPCONF(uint32_t data);
    void writePWMCONF(uint32_t data);
    void writeCOOLCONF(uint32_t data);
    void writeTCOOLTHRS(uint32_t data);
    void writeTPWMTHRS(uint32_t data);
    void writeSGTHRS(uint32_t data);
    void writeTPOWERDOWN(uint32_t data);
    uint32_t readIOIN();
    uint32_t readSG_RESULT();
    uint32_t readIFCNT();

	private:


	void request(uint8_t reg);
	void swuart_calcCRC(uint8_t* datagram, uint8_t datagramLength);
	datagram64 writeDatagram(uint8_t reg, uint32_t data);
	datagram32 requestDatagram(uint8_t reg);
	uint32_t readData(datagram64 datagram);
	uint32_t reverse(uint32_t data);
	UART_HandleTypeDef *huart;
	uint8_t addr;
};

#endif /* INC_TMC2209_HPP_ */
