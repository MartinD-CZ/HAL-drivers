#ifndef QMC5883_H_
#define QMC5883_H_

#include "hal_include.h"

#ifdef CONFIG_FILE
#include "config.h"
#endif

#ifndef QMC_I2C_TIMEOUT
#define QMC_I2C_TIMEOUT				100			//timeout for SPI in polling mode; in ms
#endif

#ifndef QMC5883_I2C_ADDR
#define QMC5883_I2C_ADDR			(0x0D << 1)
#endif

#define QMC5883_CHIPID				0xFF

#define QMC_STATUS_DRDY_FLAG		0b00000001		//data ready flag
#define QMC_STATUS_OVL_FLAG			0b00000010		//data overflow flag (increase range)
#define QMC_STATUS_DOR_FLAG			0b00000100		//data read skipped

#define QMC_READ_NO_DATA			0
#define QMC_READ_I2C_NACK			1
#define QMC_READ_OVERFLOW			2
#define QMC_READ_OK					3

enum osr_e
{
	OSR_64x = 0b11000000,
	OSR_128x = 0b10000000,
	OSR_256x = 0b01000000,
	OSR_512x = 0b00000000
};

enum rng_e
{
	RNG_2GAUSS = 0b000000,
	RNG_8GAUSS = 0b010000
};

enum odr_e
{
	ODR_10HZ = 0b0000,
	ODR_50HZ = 0b0100,
	ODR_100HZ = 0b1000,
	ODR_200HZ = 0b1100
};

enum mode_e
{
	MODE_STANDBY = 0b00,
	MODE_CONTINUOUS = 0b01
};

enum drdy_e
{
	DRDY_ENABLED = 0b00,
	DRDY_DISABLED = 0b01
};

enum rol_e
{
	ROL_PTN_NORMAL = 0b00000000,
	ROL_PTN_ENABLED = 0b01000000
};


class QMC5883
{
public:
	QMC5883(I2C_HandleTypeDef* I2C):
		_I2C(I2C), _drdy_port(nullptr), _drdy_pin(0xFFFF) {};
	QMC5883(I2C_HandleTypeDef* I2C, GPIO_TypeDef* drdy_port, const uint16_t drdy_pin):
		_I2C(I2C), _drdy_port(drdy_port), _drdy_pin(drdy_pin) {};

	bool init(void);
	void setConfig(osr_e osr, rng_e rng, odr_e odr, mode_e mode, rol_e rol = ROL_PTN_NORMAL);
	void reset(void);
	uint8_t readChipID(void);

	bool dataAvailable(void);
	uint8_t readStatusRegister(void);
	uint8_t readData(int16_t* xout = nullptr, int16_t* yout = nullptr, int16_t* zout = nullptr, int16_t* tout = nullptr);

	int16_t getX(void) const {return _x;};
	int16_t getY(void) const {return _y;};
	int16_t getZ(void) const {return _z;};
	int16_t getTemperature(void) const {return _t;};
	int16_t getHeading() const;

private:
	I2C_HandleTypeDef* const _I2C;
	GPIO_TypeDef* const _drdy_port;
	const uint16_t _drdy_pin;

	int16_t _x, _y, _z, _t;
};



#endif /* QMC5883_H_ */
