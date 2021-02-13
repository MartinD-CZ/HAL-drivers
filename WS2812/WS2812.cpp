/* WS2812 library for use with ST's HAL
 * Author: Martin Danek, martin@embedblog.eu
 * License: MIT license
 * Date: February 2020
 * Revision: 1.0
 *
 * HOW TO USE
 * 		- the library uses PWM to drive a string of WS2812 LEDs
 * 		- it requires just a timer with a PWM channel (no DMA required)
 * 		- timer/channel settings:
 * 			- prescaler 0
 * 			- counter period = 1250 / (1000000000 / F_SYSCLK), where F_SYSCLK is the frequency of the input clock to the timer
 * 			- mode: PWM mode 1
 * 			- pulse: 0
 * 			- output compare preload: enabled (important!)
 * 			- fast mode: disabled
 * 		- you'll need to modify two defines in the header file:
 * 			- TIMER_CHANNEL, which is the channel used for the PWM
 * 			- F_SYSCLK, see above
 * 		- the constructor allocates memory using 'new', so be careful of that; also, you'll need to pass the address of the timer to the constructor, as well as the number of LEDs
 * 		- use the setRGB function to set individual colors; alternatively, use setRGB_all to set all LEDs
 * 		- display the values on the LEDs using 'display()'
 * 		- note: because the channel is hard-defined, you can have multiple instances of this class, using multiple timers, but all of them must use the same channel
 *
 * TESTED ON:
 * 		STM32F030K6T6 @ 48 MHz
 *
 * CHANGELOG:
 * 		1.0: initial release
 *
 * TODO:
 *		- support for all channels on a single timer
 *		- gamma correction
 *		- peripheral initialization by library
 */

#include "hal_include.h"		//this just includes the stm32fXxx_hal.h
#include "WS2812.h"

void WS2812::setRGB(uint8_t ledPos, uint8_t r, uint8_t g, uint8_t b)
{
	if (ledPos < _ledCount)			//check that we are not writing outside of allocated memory
	{
		_bufStart[ledPos * 3 + 0] = g;
		_bufStart[ledPos * 3 + 1] = r;
		_bufStart[ledPos * 3 + 2] = b;
	}
}

void WS2812::setRGB_all(uint8_t r, uint8_t g, uint8_t b)
{
	for (uint8_t i = 0; i < _ledCount; i++)
	{
		_bufStart[i * 3 + 0] = g;
		_bufStart[i * 3 + 1] = r;
		_bufStart[i * 3 + 2] = b;
	}
}

void __attribute__((optimize("Ofast"))) WS2812::display() const
{
	if (IS_TIM_BREAK_INSTANCE(_timer->Instance) != RESET) __HAL_TIM_MOE_ENABLE(_timer);

	_timer->Instance->SR = 0x00;				//clear interrupt flag
	_timer->Instance->CCER |= ENABLE_CHANNEL;	//enable CC channel		alternative: TIM_CCxChannelCmd(htim->Instance, Channel, TIM_CCx_ENABLE);
	_timer->Instance->CR1 = TIM_CR1_CEN;		//enable timer

	__disable_irq();
	for (uint8_t i = 0; i < _ledCount * 3; i++)											//send out the data itself
	{
		for (uint8_t mask = 0b10000000; mask > 0; mask >>= 1)
		{
			_timer->Instance->CCRx = _bufStart[i] & mask ? _oneHigh : _zeroHigh;		//compute & store next value
			_timer->Instance->SR = 0x00;												//clear update event flag
			while (!(_timer->Instance->SR & TIM_SR_UIF));								//wait for next update event flag
		}
	}

	_timer->Instance->CCRx = 0x0;			//preload low output
	__enable_irq();

	for (uint8_t i = 0; i < 40; i++)		//a crude ~50 us delay for the 'reset' pulse
	{
		while (!(_timer->Instance->SR & TIM_SR_UIF));
		_timer->Instance->SR = 0x00;
	}
}
