#ifndef AT24CXX_H_
#define AT24CXX_H_

#ifdef CONFIG_FILE
#include "config.h"
#endif

#ifndef AT24_DEFAULT_ADDRESS
#define AT24_DEFAULT_ADDRESS		0b10100000
#endif

enum memsize_e
{
	SIZE_1_kb,
	SIZE_2_kb,
	SIZE_4_kb,
	SIZE_8_kb,
	SIZE_16_kb,
	SIZE_32_kb,
	/*SIZE_64_Kb,
	SIZE_128_Kb*/
};

class AT24Cxx
{
public:
	AT24Cxx(I2C_HandleTypeDef* I2C, memsize_e size, uint8_t address): _I2C(I2C), _size(size), _address(address) {};
	AT24Cxx(I2C_HandleTypeDef* I2C, memsize_e size, bool A0, bool A1, bool A2): _I2C(I2C), _size(size), _address(AT24_DEFAULT_ADDRESS | (A0) | (A1 << 1) | (A2 << 2)) {};

	bool isReady(void) const;
	constexpr uint8_t getPageSize(void) const;
	constexpr uint16_t getSizeInBytes(void) const;

	template <typename T>
	HAL_StatusTypeDef write(uint16_t dataAddress, T data) const;

	template <typename T>
	T read(uint16_t dataAddress) const;

	HAL_StatusTypeDef writePage(uint16_t pageAddress, uint8_t* data) const;
	HAL_StatusTypeDef readPage(uint16_t pageAddress, uint8_t* data) const;

	void eraseChip(void) const;

private:
	I2C_HandleTypeDef* const _I2C;
	const memsize_e _size;
	const uint8_t _address;
};



#endif /* AT24CXX_H_ */
