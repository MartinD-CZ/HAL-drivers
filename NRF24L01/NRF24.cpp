/* NRF24L01 library for use with ST's HAL
 * Author: Martin Danek, martin@embedblog.eu
 * License: MIT license
 * Date: February 2020
 * Revision: 1.0
 *
 * HOW TO USE
 * 		- the library requires one SPI peripheral, two GPIO outputs and one GPIO input
 * 		- SPI settings:
 * 			- mode: full duplex master
 * 			- data size: 8 bits
 * 			- first bits: MSB first
 * 			- prescaler: choose so that the SCK frequency is max 8 MHz (absolute maximum 10 MHz)
 * 			- CPOL: low
 * 			- CPHA: one edge
 * 		- GPIO outputs (CE, CSN) settings:
 * 			- speed: high
 * 			- output: push-pull
 * 		- GPIO input (IRQ) settings:
 * 			- enable pullup
 * 		- setup:
 * 			1. pass the address of the SPI periph. and the three GPIO ports and pins to the constructor
 * 			2. call the 'init' function with the desired frequency, datarate and power
 * 			3. call the 'enablePipe' function to enable a pipe and set it's address
 * 			4. do any other changes desired (more RX pipes etc.)
 * 			4. call the 'enterRXmode' function to start receiving (you cannot enable pipes etc. during RX mode!)
 * 			3. call the 'poll' function (periodically) to read existing data (if there are any)
 * 			4. transmit using the 'transmit' function; if the module was in RX mode, it will return to RX mode automatically
 *
 * TESTED ON:
 *		STM32F030K6
 *		STM32F412RE
 *
 * CHANGELOG:
 * 		1.0: initial release
 *
 * TODO:
 *
 */

#include "hal_include.h"
#include "NRF24.h"
#include <stdio.h>

#define COM_R_REGISTER			0b00000000			//0b000AAAAA (A = address)
#define COM_W_REGISTER			0b00100000			//0b001AAAAA (A = address)
#define COM_R_RX_PAYLOAD		0b01100001
#define COM_W_TX_PAYLOAD		0b10100000
#define COM_FLUSH_TX			0b11100001
#define COM_FLUSH_RX			0b11100010
#define COM_REUSE_TX_PL			0b11100011
#define COM_RX_PL_WID			0b01100000
#define COM_W_ACK_PAYLOAD		0b10101000			//0b10101PPP (P = pipe; must be enabled in Feature Reg.!)
#define COM_W_TX_PAYLOAD_NOACK	0b10110000
#define COM_NOP					0b11111111

#define BIT_PWR_UP				1
#define BIT_PRIM_RX				0
#define BIT_RX_DR				6
#define BIT_TX_DS				5
#define BIT_MAX_RT				4
#define BIT_EN_DPL				2
#define BIT_EN_ACK_PAY			1
#define BIT_EN_DYN_ACK			0

// ========================================================================================================
//								REGISTER ACCESS FUNCTIONS

/*
 * @brief: reads contents of a register at a given address
 * @param address: address to read
 * @retval: contents of the register
 */
uint8_t NRF24::readRegister(uint8_t address)
{
	uint8_t tx[2], rx[2];
	tx[0] = COM_R_REGISTER | address;
	tx[1] = COM_NOP;

	HAL_GPIO_WritePin(_csn_port, _csn_pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(_spi, tx, rx, 2, NRF_SPI_TIMEOUT);
	HAL_GPIO_WritePin(_csn_port, _csn_pin, GPIO_PIN_SET);

	return rx[1];
}

/*
 * @brief: write to a register at a given address
 * @param address: address to write
 * @param value: value to write
 */
void NRF24::writeRegister(uint8_t address, uint8_t value)
{
	uint8_t tx[2];
	tx[0] = COM_W_REGISTER | address;
	tx[1] = value;

	HAL_GPIO_WritePin(_csn_port, _csn_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(_spi, tx, 2, NRF_SPI_TIMEOUT);
	HAL_GPIO_WritePin(_csn_port, _csn_pin, GPIO_PIN_SET);
}

/*
 * @brief: set/reset a bit in a register
 * @param address: address of the register
 * @param bit: position of the bit to change
 * @values: 0 to 7
 * @param val: set/reset the bit
 */
void NRF24::writeRegisterBit(uint8_t address, uint8_t bit, bool val)
{
	uint8_t reg = readRegister(address);			//first read out the register
	if (val)										//if we want to write a one to the bit then set the bit in the register we read
		reg |= (1 << bit);
	else
		reg &=~(1 << bit);

	writeRegister(address, reg);
}

/*
 * @brief: read one of the multibyte registers
 * @param address: address of the register
 * @param buf: pointer to a buffer to store received values
 * @param bytes: number of bytes to read
 */
void NRF24::readMultibyteRegister(uint8_t address, uint8_t* buf, uint8_t bytes)
{
	uint8_t tx = COM_R_REGISTER | address;
	HAL_GPIO_WritePin(_csn_port, _csn_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(_spi, &tx, 1, NRF_SPI_TIMEOUT);
	for (int8_t i = bytes - 1; i >= 0 ; i--)
	{
		//note that we read the LSByte first (datash. pg. 48), so this is why we are not reading all bytes with one function
		HAL_SPI_Receive(_spi, &(buf[i]), 1, NRF_SPI_TIMEOUT);
	}
	HAL_GPIO_WritePin(_csn_port, _csn_pin, GPIO_PIN_SET);
}

/*
 * @brief: write one of the multibyte registers
 * @param address: address of the register
 * @param buf: pointer to a buffer with values to write
 * @param bytes: number of bytes to write
 */
void NRF24::writeMultibyteRegister(uint8_t address, uint8_t* buf, uint8_t bytes)
{
	uint8_t tx = COM_W_REGISTER | address;
	HAL_GPIO_WritePin(_csn_port, _csn_pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(_spi, &tx, 1, NRF_SPI_TIMEOUT);
	for (int8_t i = bytes - 1; i >= 0 ; i--)
	{
		//note that we write the LSByte first (datash. pg. 48), so this is why we are not writing all bytes with one function
		HAL_SPI_Transmit(_spi, &(buf[i]), 1, NRF_SPI_TIMEOUT);
	}
	HAL_GPIO_WritePin(_csn_port, _csn_pin, GPIO_PIN_SET);
}

// ========================================================================================================
//								INIT & PARAMETER CHANGE FUNCTIONS

/*
 * @brief: initialize the radio to a known state
 * @param frequency: desired operational frequency
 * @values: from 2400 (MHz) to 2525 (MHz)
 * @param datrate: output data rate
 * @values: DATARATE_250KBPS, DATARATE_1MBPS, DATARATE_2MBPS
 * @param power: desired output power
 * @values: POWER_0DBM, POWER_MINUS6DBM, POWER_MINUS12DBM, POWER_MINUS18DBM
 */
void NRF24::init(uint16_t frequency, datarate datrate, outpower power)
{
	while (HAL_GetTick() < 100);					//wait at least 100 ms since startup for power to stabilize

	readRegister(REG_STATUS);						//dummy read

	setFrequency(frequency);
	setDatarate(datrate);
	setOutputPower(power);

	writeRegister(REG_SETUP_AW, 0b01);				//address width 3 bytes
	setRetransmissions(10, 4);						//max 4 retransmission, delay circa 3 ms
	writeRegister(REG_EN_AA, 0x00);					//disable Enhanced Shock Burst on all pipes
	writeRegister(REG_EN_RXADDR, 0x00);				//disable all RX pipes
	writeRegister(REG_FEATURE, (1 << BIT_EN_DPL) | (1 << BIT_EN_DYN_ACK));	//enable dynamic payload and dynamic acknowledge

	uint8_t defAddress[] = {0xD0, 0xD0, 0xD0};		//this default address does not really matter
	enablePipe(0, defAddress);						//enable pipe 0 (for ACK reception)

	flushTX();										//just for good measure, flush TX and RX
	flushRX();

	writeRegisterBit(REG_CONFIG, BIT_PWR_UP, 1);	//go into standby-I
	HAL_Delay(2);
}

/*
 * @brief: set the delay between retransmissions and retransmission counter
 * @param delay : delay between retransmission, where 0 is 250 us, 1 is 500 us; max 15 (ie 4 ms)
 * @note: you must take into account the time it takes to receive the ACK
 * @param count: maximal number of retransmission, where 0 is no retransmissions; max 15
 */
void NRF24::setRetransmissions(uint8_t delay, uint8_t count)
{
	if (count > 15) count = 15;
	if (delay > 15) delay = 15;
	writeRegister(REG_SETUP_RETR, (delay << 4) | (count << 0));
}

/*
 * @brief: set frequency on which the radio operates
 * @param frequency: desired operational frequency
 * @values: from 2400 (MHz) to 2525 (MHz)
 */
void NRF24::setFrequency(uint16_t frequency)
{
	writeRegister(REG_RF_CH, frequency - 2400);
}

/*
 * @brief: set datarate
 * @param datrate: output data rate
 * @values: DATARATE_250KBPS, DATARATE_1MBPS, DATARATE_2MBPS
 */
void NRF24::setDatarate(datarate datrate)
{
	uint8_t reg = readRegister(REG_RF_SETUP);
	reg &=~0b101000;
	reg |= (datrate << 3);
	writeRegister(REG_RF_SETUP, reg);
}

/*
 * @brief: set output power
 * @param power: desired output power
 * @values: POWER_0DBM, POWER_MINUS6DBM, POWER_MINUS12DBM, POWER_MINUS18DBM
 */
void NRF24::setOutputPower(outpower power)
{
	uint8_t reg = readRegister(REG_RF_SETUP);
	reg &=~(0b11 << 0b110);
	reg |= (power << 1);
	writeRegister(REG_RF_SETUP, reg);
}

/*
 * @brief: enables a pipe with a given address for reception
 * @param pipeNum number of pipe, from 0 to 5
 * @note pipe 0 is used in TX for receiving ACK and should not be used for RX
 * @param pipeAddress pointer to a 3 byte array with the address for the desired pipe
 * @note P0 and P1 support the full 3 byte address; other pipes have MSbyte from P1
 */
void NRF24::enablePipe(uint8_t pipeNum, uint8_t* pipeAddress)
{
	if (pipeNum > 5) return;

	//enable Enhanced ShockBurst, the pipe itself, dynamic payload and store message length
	writeRegisterBit(REG_EN_AA, pipeNum, 1);
	writeRegisterBit(REG_EN_RXADDR, pipeNum, 1);
	writeRegisterBit(REG_DYNPD, pipeNum, 1);

	if (pipeNum < 2)
		writeMultibyteRegister(REG_RX_ADDR_P0 + pipeNum, pipeAddress, 3);
	else
		writeRegister(REG_RX_ADDR_P0 + pipeNum, pipeAddress[0]);
}

// ========================================================================================================
//								RX MODE FUNCTIONS

/*
 * @brief: put the radio into RX mode
 * @note: power consumption about 13 mA
 */
void NRF24::enterRxMode(void)
{
	writeRegisterBit(REG_CONFIG, BIT_PRIM_RX, 1);
	HAL_GPIO_WritePin(_ce_port, _ce_pin, GPIO_PIN_SET);
	_rx_mode = true;
	//theoretically, there should be a 200 us delay
}

/*
 * @brief: put the radio into standy-I mode
 * @note: power consumption about 26 uA
 */
void NRF24::exitRxMode(void)
{
	writeRegisterBit(REG_CONFIG, BIT_PRIM_RX, 0);
	HAL_GPIO_WritePin(_ce_port, _ce_pin, GPIO_PIN_RESET);
	_rx_mode = false;
}

// ========================================================================================================
//								ACK PACKET PAYLOAD FUNCTIONS

/*
 * @brief: enables adding payload to ACK packets
 */
void NRF24::enableAckPayload(void)
{
	writeRegisterBit(REG_FEATURE, BIT_EN_ACK_PAY, 1);
	_ackPayloadEnabled = true;
}

/*
 * @brief: send payload for the ACK packet
 * @param pipeNum: pipe to whose ACK packet the payload will attach
 * @param payload: pointer to the payload
 * @param numBytes: number of bytes in the payload
 */
void NRF24::sendAckPayload(uint8_t pipeNum, uint8_t* payload, uint8_t numBytes)
{
	HAL_GPIO_WritePin(_csn_port, _csn_pin, GPIO_PIN_RESET);
	uint8_t command = COM_W_ACK_PAYLOAD | pipeNum;
	HAL_SPI_Transmit(_spi, &command, 1, NRF_SPI_TIMEOUT);
	HAL_SPI_Transmit(_spi, payload, numBytes, NRF_SPI_TIMEOUT);
	HAL_GPIO_WritePin(_csn_port, _csn_pin, GPIO_PIN_SET);
}

// ========================================================================================================
//								TRANSMIT & RECEIVE FUNCTIONS

/*
 * @brief: transmits data
 * @param pipeAddress: 3 byte array with the address of the target pipe
 * @param buffer: pointer to a buffer with the TX data
 * @param numBytes: number of bytes to send
 * @param ack:
 * 		ACK to request acknowledge
 * 		NACK to 'fire and forget'
 * @retval:
 * 		1 = TX_ACK if the data was acknowledged
 * 		2 = TX_MAX_RT if max retransmits was reached, ie message was not acknowledged by the target
 * 		3 = TX_ERROR if the IRQ pin didn't go low before NRF_TRANSMIT_TIMEOUT was hit
 * 		4 = TX_NACK if started with NACK param
 * 		5 = TX_ACK_PAYLOAD if acknowledged & there's response to read with poll
 */
txStatus_TypeDef NRF24::transmit(uint8_t* pipeAddress, uint8_t* buffer, uint8_t numBytes, txAck_e ack)
{
	bool prev_rx_mode = false;
	if (_rx_mode)													//go into Standby-I
	{
		prev_rx_mode = true;
		exitRxMode();
	}

	writeRegister(REG_STATUS, 0b1110000);							//clear SREG, just to be sure

	writeMultibyteRegister(REG_TX_ADDR, pipeAddress, 3);			//set TX address
	writeMultibyteRegister(REG_RX_ADDR_P0, pipeAddress, 3);			//set pipe 0 address

	//load TX FIFO
	HAL_GPIO_WritePin(_csn_port, _csn_pin, GPIO_PIN_RESET);
	uint8_t command = (ack == ACK ? COM_W_TX_PAYLOAD : COM_W_TX_PAYLOAD_NOACK);
	HAL_SPI_Transmit(_spi, &command, 1, NRF_SPI_TIMEOUT);
	HAL_SPI_Transmit(_spi, buffer, numBytes, NRF_SPI_TIMEOUT);
	HAL_GPIO_WritePin(_csn_port, _csn_pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(_ce_port, _ce_pin, GPIO_PIN_SET);				//high pulse starts TX mode
	HAL_Delay(1);
	HAL_GPIO_WritePin(_ce_port, _ce_pin, GPIO_PIN_RESET);

	if (ack == NACK)												//we don't want any acknowledge
	{
		if (prev_rx_mode) enterRxMode();
		return TX_NACK;
	}

	uint32_t start_ticks = HAL_GetTick();
	while (HAL_GPIO_ReadPin(_irq_port, _irq_pin))
	{
		if (HAL_GetTick() - start_ticks > NRF_TRANSMIT_TIMEOUT) break;
	}

	uint8_t status = readRegister(REG_STATUS);

	if (prev_rx_mode) enterRxMode();						//go back to RX mode, if we started in it


	if (_ackPayloadEnabled && (status & (1 << BIT_TX_DS)) && (status & (1 << BIT_RX_DR)))
	{														//we received payload with the ACK
		return TX_ACK_PAYLOAD;								//read the data using poll
	}
	else if ((status & (1 << BIT_TX_DS)))					//TX_DS (Data Sent) interrupt source set
	{
		writeRegisterBit(REG_STATUS, BIT_TX_DS, 1);			//clear INT bit
		return TX_ACK;
	}
	else if ((status & (1 << BIT_MAX_RT)))					//MAX_RT (Max ReTransmitts) interrupt bit set
	{
		writeRegisterBit(REG_STATUS, BIT_MAX_RT, 1);		//clear INT bit
		flushTX();
		return TX_MAX_RT;
	}
	else return TX_ERROR;
}

/*
 * @brief: checks if there is data available
 * @param rxData pointer to a buffer for storing the read data
 * @pipe pointer to uint8_t, which indicates from pipe the data came; can be nullptr
 * @retval 0 if there is no data, otherwise the number of bytes read
 */
uint8_t NRF24::poll(uint8_t* rxData, uint8_t* pipe)
{
	uint8_t result = 0, pipe_l = 0;
	if (HAL_GPIO_ReadPin(_irq_port, _irq_pin) == GPIO_PIN_RESET)
	{
		uint8_t status = readRegister(REG_STATUS);
		if ((status & (1 << BIT_RX_DR)))			//we have received data!
		{
			pipe_l = (status >> 1) & 0b111;														//decode pipe number

			HAL_GPIO_WritePin(_csn_port, _csn_pin, GPIO_PIN_RESET);								//read the number of bytes in that pipe
			uint8_t tx[2], rx[2];
			tx[0] = COM_RX_PL_WID;
			tx[1] = COM_NOP;
			HAL_SPI_TransmitReceive(_spi, tx, rx, 2, NRF_SPI_TIMEOUT);
			HAL_GPIO_WritePin(_csn_port, _csn_pin, GPIO_PIN_SET);

			if ((pipe_l < 0b110) && (rx[1] > 0))
				result = rx[1];																	//some pipe has more than 0 data
			else
				return 0;

			HAL_GPIO_WritePin(_csn_port, _csn_pin, GPIO_PIN_RESET);								//read the payload
			tx[0] = COM_R_RX_PAYLOAD;
			HAL_SPI_Transmit(_spi, tx, 1, NRF_SPI_TIMEOUT);
			HAL_SPI_Receive(_spi, rxData, result, NRF_SPI_TIMEOUT);
			HAL_GPIO_WritePin(_csn_port, _csn_pin, GPIO_PIN_SET);

			writeRegisterBit(REG_STATUS, BIT_RX_DR, 1);											//clear the interrupt
			if (pipe != nullptr) *pipe = pipe_l;
		}
	}

	return result;
}

// ========================================================================================================
//								OTHER FUNCTIONS

/*
 * @brief: deletes all data in TX FIFOs
 */
void NRF24::flushTX(void)
{
	HAL_GPIO_WritePin(_csn_port, _csn_pin, GPIO_PIN_RESET);
	uint8_t tx = COM_FLUSH_TX;
	HAL_SPI_Transmit(_spi, &tx, 1, NRF_SPI_TIMEOUT);
	HAL_GPIO_WritePin(_csn_port, _csn_pin, GPIO_PIN_SET);
}

/*
 * @brief: deletes all data in RX FIFOs
 */
void NRF24::flushRX(void)
{
	HAL_GPIO_WritePin(_csn_port, _csn_pin, GPIO_PIN_RESET);
	uint8_t tx = COM_FLUSH_RX;
	HAL_SPI_Transmit(_spi, &tx, 1, NRF_SPI_TIMEOUT);
	HAL_GPIO_WritePin(_csn_port, _csn_pin, GPIO_PIN_SET);
}

/*
 * @brief: returns the number of retransmits of the last transmission
 * @retval number of retransmits
 * @note: reset at each transmission start
 */
uint8_t NRF24::readRetransmissions(void)
{
	uint8_t data = readRegister(REG_OBSERVE_TX);
	return (data & 0b1111);
}

/*
 * @brief: returns the total number of unsuccesful transmissions (ie the number MAX_RT was reached)
 * @retval number of unsuccesful transmissions
 * @note: reset by writing to RF_CH
 */
uint8_t NRF24::readLostPackets(void)
{
	uint8_t data = readRegister(REG_OBSERVE_TX);
	return ((data >> 4) & 0b1111);
}

/*
 * @brief: reads all the registers on the NRF24 and prints them out in a human readable form
 * @param: buf pointer to buffer of 460 chars, into which the data will be printed;
 * @retval number of bytes written
 */
uint16_t NRF24::printRegisters(char* buf)
{
	uint8_t pipe0[5], pipe1[5], tx_addr[5];
	readMultibyteRegister(REG_RX_ADDR_P0, pipe0, 5);
	readMultibyteRegister(REG_RX_ADDR_P1, pipe1, 5);
	readMultibyteRegister(REG_TX_ADDR, tx_addr, 5);

	return sprintf(buf, "NRF24 REGISTER READOUT (HEX):\n"
			"CONFIG: %02X\n"
			"EN_AA: %02X\n"
			"EN_RXADDR: %02X\n"
			"SETUP_AW: %02X\n"
			"SETUP_RETR: %02X\n"
			"RF_CH: %02X\n"
			"RF_SETUP: %02X\n"
			"STATUS: %02X\n"
			"OBSERVE_TX: %02X\n"
			"RPD: %02X\n"
			"RX_ADDR_P0: %02X %02X %02X %02X %02X\n"
			"RX_ADDR_P1: %02X %02X %02X %02X %02X\n"
			"RX_ADDR_P2: %02X %02X %02X %02X %02X\n"
			"RX_ADDR_P3: %02X %02X %02X %02X %02X\n"
			"RX_ADDR_P4: %02X %02X %02X %02X %02X\n"
			"RX_ADDR_P5: %02X %02X %02X %02X %02X\n"
			"TX_ADDR: %02X %02X %02X %02X %02X\n"
			"RX_PW_P0: %02X\n"
			"RX_PW_P1: %02X\n"
			"RX_PW_P2: %02X\n"
			"RX_PW_P3: %02X\n"
			"RX_PW_P4: %02X\n"
			"RX_PW_P5: %02X\n"
			"FIFO_STATUS: %02X\n"
			"DYNPD: %02X\n"
			"FEATURE: %02X\n",
			readRegister(REG_CONFIG), readRegister(REG_EN_AA), readRegister(REG_EN_RXADDR), readRegister(REG_SETUP_AW),
			readRegister(REG_SETUP_RETR), readRegister(REG_RF_CH), readRegister(REG_RF_SETUP), readRegister(REG_STATUS),
			readRegister(REG_OBSERVE_TX), readRegister(REG_RPD),
			pipe0[0], pipe0[1], pipe0[2], pipe0[3], pipe0[4],
			pipe1[0], pipe1[1], pipe1[2], pipe1[3], pipe1[4],
			pipe1[0], pipe1[1], pipe1[2], pipe1[3], readRegister(REG_RX_ADDR_P2),
			pipe1[0], pipe1[1], pipe1[2], pipe1[3], readRegister(REG_RX_ADDR_P3),
			pipe1[0], pipe1[1], pipe1[2], pipe1[3], readRegister(REG_RX_ADDR_P4),
			pipe1[0], pipe1[1], pipe1[2], pipe1[3], readRegister(REG_RX_ADDR_P5),
			tx_addr[0], tx_addr[1], tx_addr[2], tx_addr[3], tx_addr[4],
			readRegister(REG_RX_PW_P0), readRegister(REG_RX_PW_P1), readRegister(REG_RX_PW_P2), readRegister(REG_RX_PW_P3),
			readRegister(REG_RX_PW_P4), readRegister(REG_RX_PW_P5), readRegister(REG_FIFO_STATUS), readRegister(REG_DYNPD),
			readRegister(REG_FEATURE));
}

