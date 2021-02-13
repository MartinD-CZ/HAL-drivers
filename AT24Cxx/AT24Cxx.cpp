/* AT24Cxx library for use with ST's HAL
 * Author: Martin Danek, martin@embedblog.eu
 * License: MIT license
 * Date: February 2020
 * Revision: 1.0
 *
 * HOW TO USE
 * 		- the constructor requires one I2C peripheral, memory size (SIZE_x_kb) and either the 8 bit address or the status of the three Ax pins (ie LOW or HIGH)
 * 		- be aware that the SIZE_x_kb is in kilobits, not bytes!
 * 		- I2C settings:
 * 			- speed: 400 kHz for Vcc > 2.7 V
 * 			- everything else default
 * 		- usage:
 *
 * TESTED ON:
 *		STM32F412RE
 *
 * CHANGELOG:
 * 		1.0: initial release
 *
 * DATASHEET:
 * 		http://ww1.microchip.com/downloads/en/devicedoc/doc0180.pdf
 *
 * TODO:
 *		add support for larger memories (128 kb etc.)
 */


#include "hal_include.h"
#include "AT24Cxx.h"

#include <stdlib.h>

/*
 * @brief: test is EEPROM is ready to accept new data
 * @retval: true if EEPROM ready
 */
bool AT24Cxx::isReady(void) const
{
	//uint8_t tx = 0x00;
	if (HAL_I2C_Master_Transmit(_I2C, _address, nullptr, 0, 50) == HAL_OK) return true;
	else return false;
}

/*
 * @brief: returns page size in bytes
 */
constexpr uint8_t AT24Cxx::getPageSize(void) const
{
	if (_size == SIZE_1_kb || _size == SIZE_2_kb) return 8;
	else return 16;
}

/*
 * @brief: returns total EEPROM size in bytes
 */
constexpr uint16_t AT24Cxx::getSizeInBytes(void) const
{
	switch (_size)
	{
	case SIZE_1_kb: return 128;
	case SIZE_2_kb: return 256;
	case SIZE_4_kb: return 512;
	case SIZE_8_kb: return 1024;
	case SIZE_16_kb: return 2048;
	case SIZE_32_kb: return 4096;
	default: return 0;
	}
}

/*
 * @brief: write a variable data to address dataAddress
 * @note: it is recommended to not cross page boundaries, ie try to align 4 byte variables to divisible-by-4 addresses
 * @retval:
 * 		HAL_ERROR if dataAddress is out of range
 * 		HAL_TIMEOUT if there was no answer from the EEPROM
 * 		HAL_OK if data saved
 */
template <typename T>
HAL_StatusTypeDef AT24Cxx::write(uint16_t dataAddress, T data) const
{
	if ((dataAddress + sizeof(data)) > (uint16_t)(getSizeInBytes() - 1)) return HAL_ERROR;		//check for out-of range address

	uint8_t* p = reinterpret_cast<uint8_t*>(&data);
	uint8_t buffer[sizeof(data) + 1];
	buffer[0] = dataAddress;
	for (uint8_t i = 0; i < sizeof(data); i++) buffer[i + 1] = /*p[(sizeof(data) - 1) - i]*/ p[i];

	uint8_t errCounter = 0;
	while (HAL_I2C_Master_Transmit(_I2C, _address | (dataAddress >> 8), buffer, sizeof(data) + 1, 100) != HAL_OK)
	{
		HAL_Delay(1);
		if (errCounter > 9) return HAL_TIMEOUT;
	}

	return HAL_OK;
}

template HAL_StatusTypeDef AT24Cxx::write<uint8_t>(uint16_t dataAddress, uint8_t data) const;
template HAL_StatusTypeDef AT24Cxx::write<uint16_t>(uint16_t dataAddress, uint16_t data) const;
template HAL_StatusTypeDef AT24Cxx::write<uint32_t>(uint16_t dataAddress, uint32_t data) const;
template HAL_StatusTypeDef AT24Cxx::write<uint64_t>(uint16_t dataAddress, uint64_t data) const;
template HAL_StatusTypeDef AT24Cxx::write<float>(uint16_t dataAddress, float data) const;
template HAL_StatusTypeDef AT24Cxx::write<double>(uint16_t dataAddress, double data) const;

/*
 * @brief: read data from address dataAddress
 * @retval: data read
 */
template <typename T>
T AT24Cxx::read(uint16_t dataAddress) const
{
	uint8_t buffer[sizeof(T)];

	uint8_t tx = dataAddress;
	HAL_I2C_Master_Transmit(_I2C, _address | (dataAddress >> 8), &tx, 1, 100);

	HAL_I2C_Master_Receive(_I2C, _address, buffer, sizeof(T), 100);
	T* p = reinterpret_cast<T*>(&buffer[0]);
	return *p;
}

template uint8_t AT24Cxx::read<uint8_t>(uint16_t dataAddress) const;
template uint16_t AT24Cxx::read<uint16_t>(uint16_t dataAddress) const;
template uint32_t AT24Cxx::read<uint32_t>(uint16_t dataAddress) const;
template uint64_t AT24Cxx::read<uint64_t>(uint16_t dataAddress) const;
template float AT24Cxx::read<float>(uint16_t dataAddress) const;
template double AT24Cxx::read<double>(uint16_t dataAddress) const;

/*
 * @brief: write an entire page (8 or 16 bytes)
 * @param pageAddress: address of the page, should be a multiple of 8 or 16
 * @param data: pointer to 8 or 16 byte array with the data
 * @retval:
 * 		HAL_ERROR if pageAddress not aligned to page boundaries
 * 		HAL_TIMEOUT if there was no answer from the EEPROM
 * 		HAL_OK if page saved
 */
HAL_StatusTypeDef AT24Cxx::writePage(uint16_t pageAddress, uint8_t* data) const
{
	if (pageAddress % getPageSize()) return HAL_ERROR;		//check if we are aligned

	uint8_t buffer[getPageSize()];
	buffer[0] = pageAddress;
	for (uint8_t i = 1; i <= getPageSize(); i++) buffer[i] = data[i - 1];

	uint8_t errCounter = 0;
	while (HAL_I2C_Master_Transmit(_I2C, _address | (pageAddress >> 8), buffer, getPageSize() + 1, 100) != HAL_OK)
	{
		HAL_Delay(1);
		if (errCounter > 9) return HAL_TIMEOUT;
	}

	return HAL_OK;
}

/*
 * @brief: read an entire page from EEPROM
 * @param pageAddress: address of the page, should be a multiple of 8 or 16
 * @param data: pointer to 8 or 16 byte array to hold the data
 * @retval:
 * 		HAL_ERROR if pageAddress not aligned to page boundaries
 * 		HAL_TIMEOUT if there was no answer from the EEPROM
 * 		HAL_OK if page saved
 */
HAL_StatusTypeDef AT24Cxx::readPage(uint16_t pageAddress, uint8_t* data) const
{
	if (pageAddress % getPageSize()) return HAL_ERROR;

	uint8_t tx = pageAddress;
	HAL_I2C_Master_Transmit(_I2C, _address | (pageAddress >> 8), &tx, 1, 100);

	return HAL_I2C_Master_Receive(_I2C, _address, data, getPageSize(), 100);
}

/*
 * @brief: write 0x00 to the entire EEPROM
 */
void AT24Cxx::eraseChip(void) const
{
	uint8_t data[getPageSize()];
	for (uint8_t j = 0; j < getPageSize(); j++) data[j] = 0x00;

	for (uint16_t i = 0; i < (getSizeInBytes() / getPageSize()); i++)
	{
		writePage(i * getPageSize(), data);
	}
}

