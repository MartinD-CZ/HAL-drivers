#ifndef WS2812_H_
#define WS2812_H_

#include <stdlib.h>

#ifdef CONFIG_FILE
#include "config.h"
#endif

#ifndef TIMER_CHANNEL
#define TIMER_CHANNEL		1
#endif

#ifndef F_SYSCLK
#define F_SYSCLK			48000000UL
#endif

#if (TIMER_CHANNEL == 1)
#define ENABLE_CHANNEL		TIM_CCER_CC1E
#define CCRx				CCR1
#elif (TIMER_CHANNEL == 2)
#define ENABLE_CHANNEL		TIM_CCER_CC2E
#define CCRx				CCR2
#elif (TIMER_CHANNEL == 3)
#define ENABLE_CHANNEL		TIM_CCER_CC3E
#define CCRx				CCR3
#elif (TIMER_CHANNEL == 4)
#define ENABLE_CHANNEL		TIM_CCER_CC4E
#define CCRx				CCR4
#endif

class WS2812
{
public:
	WS2812(TIM_HandleTypeDef* timer, uint8_t ledCount) :
		_timer(timer),
		_ledCount(ledCount),
		_bufStart(new uint8_t[ledCount * 3])
		{};
	~WS2812() {free(_bufStart);};

	void setRGB(uint8_t ledPos, uint8_t r, uint8_t g, uint8_t b);
	void setRGB_all(uint8_t r, uint8_t g, uint8_t b);

	uint8_t getR(const uint8_t pos) const {return _bufStart[pos * 3 + 1];};
	uint8_t getG(const uint8_t pos) const {return _bufStart[pos * 3 + 0];};
	uint8_t getB(const uint8_t pos) const {return _bufStart[pos * 3 + 2];};

	void __attribute__((optimize("Ofast"))) display() const;

private:
	const TIM_HandleTypeDef* _timer;
	const uint8_t _ledCount;
	uint8_t* const _bufStart;

	const uint8_t _oneHigh = 700 / (1000000000 / F_SYSCLK);
	const uint8_t _zeroHigh = 350 / (1000000000 / F_SYSCLK);
};



#endif /* WS2812_H_ */
