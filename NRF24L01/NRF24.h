#ifndef NRF24_H_
#define NRF24_H_

#ifdef CONFIG_FILE
#include "config.h"
#endif

#ifndef NRF_TRANSMIT_TIMEOUT
#define NRF_TRANSMIT_TIMEOUT		50			//how long we wait for ACK during transmission; in ms
#endif

#ifndef NRF_SPI_TIMEOUT
#define NRF_SPI_TIMEOUT				100			//timeout for SPI in polling mode; in ms
#endif

enum datarate
{
	DATARATE_250KBPS = 0b100,
	DATARATE_1MBPS = 0b000,
	DATARATE_2MBPS = 0b001
};

enum outpower
{
	POWER_MINUS18DBM = 0b00,
	POWER_MINUS12DBM = 0b01,
	POWER_MINUS6DBM = 0b10,
	POWER_0DBM = 0b11
};

enum txStatus_TypeDef
{
	TX_ACK = 1,
	TX_MAX_RT = 2,
	TX_ERROR = 3,
	TX_NACK = 4,
	TX_ACK_PAYLOAD = 5
};

enum txAck_e
{
	ACK,
	NACK
};

#define REG_CONFIG				0x00
#define REG_EN_AA				0x01
#define REG_EN_RXADDR			0x02
#define REG_SETUP_AW			0x03
#define REG_SETUP_RETR			0x04
#define REG_RF_CH				0x05
#define REG_RF_SETUP			0x06
#define REG_STATUS				0x07
#define REG_OBSERVE_TX			0x08
#define REG_RPD					0x09
#define REG_RX_ADDR_P0			0x0A
#define REG_RX_ADDR_P1			0x0B
#define REG_RX_ADDR_P2			0x0C
#define REG_RX_ADDR_P3			0x0D
#define REG_RX_ADDR_P4			0x0E
#define REG_RX_ADDR_P5			0x0F
#define REG_TX_ADDR				0x10
#define REG_RX_PW_P0			0x11
#define REG_RX_PW_P1			0x12
#define REG_RX_PW_P2			0x13
#define REG_RX_PW_P3			0x14
#define REG_RX_PW_P4			0x15
#define REG_RX_PW_P5			0x16
#define REG_FIFO_STATUS			0x17
#define REG_DYNPD				0x1C
#define REG_FEATURE				0x1D

class NRF24
{
public:
	NRF24(SPI_HandleTypeDef* spi, GPIO_TypeDef* ce_port, uint16_t ce_pin, GPIO_TypeDef* csn_port, uint16_t csn_pin, GPIO_TypeDef* irq_port, uint16_t irq_pin):
		_spi(spi), _ce_port(ce_port), _ce_pin(ce_pin), _csn_port(csn_port), _csn_pin(csn_pin), _irq_port(irq_port), _irq_pin(irq_pin) {};

	uint8_t readRegister(uint8_t address);
	void writeRegister(uint8_t address, uint8_t value);
	void writeRegisterBit(uint8_t address, uint8_t bit, bool val);
	void readMultibyteRegister(uint8_t address, uint8_t* buf, uint8_t bytes);
	void writeMultibyteRegister(uint8_t address, uint8_t* buf, uint8_t bytes);

	void init(uint16_t frequency, datarate datrate, outpower power);
	void setRetransmissions(uint8_t delay, uint8_t count);
	void setFrequency(uint16_t frequency);
	void setDatarate(datarate datrate);
	void setOutputPower(outpower power);
	void enablePipe(uint8_t pipeNum, uint8_t* pipeAddress);

	void enterRxMode(void);
	void exitRxMode(void);

	void enableAckPayload(void);
	void sendAckPayload(uint8_t pipeNum, uint8_t* payload, uint8_t numBytes);

	txStatus_TypeDef transmit(uint8_t* pipeAddress, uint8_t* buffer, uint8_t numBytes, txAck_e ack = ACK);
	uint8_t poll(uint8_t* rxData, uint8_t* pipe);

	void flushTX(void);
	void flushRX(void);
	uint8_t readRetransmissions(void);
	uint8_t readLostPackets(void);
	uint16_t printRegisters(char* buf);

private:
	SPI_HandleTypeDef* const _spi;
	GPIO_TypeDef* const _ce_port;
	const uint16_t _ce_pin;
	GPIO_TypeDef* const _csn_port;
	const uint16_t _csn_pin;
	GPIO_TypeDef* const _irq_port;
	const uint16_t _irq_pin;

	bool _rx_mode = false;
	bool _ackPayloadEnabled = false;
};



#endif /* NRF24_H_ */
