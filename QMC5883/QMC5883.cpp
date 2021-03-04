/* QMC5883(L) library for use with ST's HAL
 * Author: Martin Danek, martin@embedblog.eu
 * License: MIT license
 * Date: March 2020
 * Revision: 1.0
 *
 * HOW TO USE
 * 		- the library requires one pre-initialized I2C peripheral and optionally one input GPIO
 * 		- I2C settings: does not really matter, just keep the speed to 400 kHz or less
 *
 * 		- GPIO outputs (CE, CSN) settings:
 * 			- speed: high
 * 			- output: push-pull
 * 		- GPIO input (IRQ) settings:
 * 			- enable pullup
 * 		- usage:
 * 			there are two constructors: with and without the DRDY pin; without it, STATUS register is read every time to make
 * 				sure there is new data
 * 			1. call the init function and optionally the config functions
 * 			2. either poll for data using readData or attach a risign-edge interrupt to the GPIO (the ISR should call readData)
 * 			3. get all data either by pasing pointers to readData or by calling the getX etc. functions
 * 				(the library stores last reading as a private variable)
 *
 * TESTED ON:
 *		STM32G070KB
 *
 * CHANGELOG:
 * 		1.0: initial release
 *

 * TODO:
 *		offset calibration
 *		declination calculation
 */

#include "QMC5883.h"
#include <math.h>

#define REG_XOUT_LSB				0x00
#define REG_XOUT_MSB				0x01
#define REG_YOUT_LSB				0x02
#define REG_YOUT_MSB				0x03
#define REG_ZOUT_LSB				0x04
#define REG_ZOUT_MSB				0x05

#define REG_STATUS					0x06

#define REG_TOUT_LSB				0x07
#define REG_TOUT_MSB				0x08

#define REG_CONFIG1					0x09
#define REG_CONFIG2					0x0A
#define REG_RSTPER					0x0B
#define REG_CHIPID					0x0D

// ========================================================================================================
//								SETUP FUNCTIONS

/*
 * @brief: resets the chip and checks for it's ID
 * @retval: true if connection to chip successfully established
 */
bool QMC5883::init(void)
{
	reset();
	if (readChipID() == QMC5883_CHIPID)
		return true;
	else
		return false;
}

/*
 * @brief: sets the basic configration of the chip
 * @param osr: oversampling rate, higher = lower bandwidth and higher power consumption, OSR_x
 * @param rng: measurement range, RNG_x
 * @param odr: output data rate, ODR_x
 * @param mode: mode of operation, MODE_x
 * @param rol: whether the internal register pointer should roll over int the 0x0:0x6 range
 */
void QMC5883::setConfig(osr_e osr, rng_e rng, odr_e odr, mode_e mode, rol_e rol)
{
	//note: we write 0x01 to REG_RSTPER because datasheet mandates that
	drdy_e drdy = _drdy_pin == 0xFFFF ? DRDY_DISABLED : DRDY_ENABLED;
	uint8_t tx[4] = {REG_CONFIG1, (uint8_t)(osr | rng | odr | mode), (uint8_t)(drdy | rol), 0x01};
	HAL_I2C_Master_Transmit(_I2C, QMC5883_I2C_ADDR, tx, 4, QMC_I2C_TIMEOUT);
}

/*
 * @brief: resets the chip
 */
void QMC5883::reset(void)
{
	uint8_t tx[2] = {REG_CONFIG2, 0x80};
	HAL_I2C_Master_Transmit(_I2C, QMC5883_I2C_ADDR, tx, 2, QMC_I2C_TIMEOUT);
}

/*
 * @brief: reads the chip's ID register
 * @retval: chip's ID, should be equal to QMC5883_CHIPID
 */
uint8_t QMC5883::readChipID(void)
{
	uint8_t tx = REG_CHIPID;
	uint8_t rx = 0;
	HAL_I2C_Master_Transmit(_I2C, QMC5883_I2C_ADDR, &tx, 1, QMC_I2C_TIMEOUT);
	HAL_I2C_Master_Receive(_I2C, QMC5883_I2C_ADDR, &rx, 1, QMC_I2C_TIMEOUT);
	return rx;
}

// ========================================================================================================
//								DATA MANIPULATION FUNCTIONS

/*
 * @brief: checks if new data are available for reading from the chip
 * @retval: true if data are available, otherwise false
 */
bool QMC5883::dataAvailable(void)
{
	if (_drdy_pin == 0xFFFF)
		return (readStatusRegister() & QMC_STATUS_DRDY_FLAG);
	else
		return HAL_GPIO_ReadPin(_drdy_port, _drdy_pin);
}

/*
 * @brief: reads the chip's status register
 * @retval: status register content, can be checked against QMC_STATUS_x flags
 */
uint8_t QMC5883::readStatusRegister(void)
{
	uint8_t tx = REG_STATUS;
	uint8_t rx = 0;
	HAL_I2C_Master_Transmit(_I2C, QMC5883_I2C_ADDR, &tx, 1, QMC_I2C_TIMEOUT);
	HAL_I2C_Master_Receive(_I2C, QMC5883_I2C_ADDR, &rx, 1, QMC_I2C_TIMEOUT);
	return rx;
}

//100 LSB per C

/*
 * @brief: reads the data from the chip and stores them internally
 * @param xout, yout, zout: optional pointer to int16_t to copy the measured valued to; pass nullptr if you are not interested in particular data
 * @param tout: temperature output, 100 LSB/C, but absolute accuracy is uncalibrated
 * @retval: QMC_READ_x
 */
uint8_t QMC5883::readData(int16_t* xout, int16_t* yout, int16_t* zout, int16_t* tout)
{
	if (!dataAvailable()) return QMC_READ_NO_DATA;

	uint8_t tx = REG_XOUT_LSB;
	uint8_t rx[9];
	if (HAL_I2C_Master_Transmit(_I2C, QMC5883_I2C_ADDR, &tx, 1, QMC_I2C_TIMEOUT) != HAL_OK) return QMC_READ_I2C_NACK;
	if (HAL_I2C_Master_Receive(_I2C, QMC5883_I2C_ADDR, rx, 9, QMC_I2C_TIMEOUT) != HAL_OK) return QMC_READ_I2C_NACK;

	_x = (((int16_t)rx[1] << 8) | rx[0]);
	_y = (((int16_t)rx[3] << 8) | rx[2]);
	_z = (((int16_t)rx[5] << 8) | rx[4]);
	_t = (((int16_t)rx[8] << 8) | rx[7]);

	if (xout != nullptr) *xout = _x;
	if (yout != nullptr) *yout = _y;
	if (zout != nullptr) *zout = _z;
	if (tout != nullptr) *tout = _t;

	if (rx[REG_STATUS] & QMC_STATUS_OVL_FLAG) return QMC_READ_OVERFLOW;
	else return QMC_READ_OK;
}

/*
 * @brief: calculates the current heading based on last X and Y values
 * @retval: heading in degrees
 */
int16_t QMC5883::getHeading() const
{
	float heading = atan2((float)_x, (float)_y);
	heading *= 180.0f / (float)M_PI;

	int16_t heading_int = roundf(heading);
	if (heading_int > 360) heading_int -= 360;
	if (heading_int < 0) heading_int += 360;

	return heading_int;
}
