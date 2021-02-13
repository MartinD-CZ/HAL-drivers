#ifndef HAL_INCLUDE_H_
#define HAL_INCLUDE_H_

#define LOW					0
#define HIGH				1

#ifdef STM32F0
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_def.h"
#endif

#ifdef STM32F1
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_def.h"
#endif

#ifdef STM32F3
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_def.h"
#endif

#ifdef STM32F4
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#endif

#ifdef STM32L0
#include "stm32l0xx_hal.h"
#include "stm32l0xx_hal_def.h"
#endif

#ifdef STM32G0
#include "stm32g0xx_hal.h"
#include "stm32g0xx_hal_def.h"
#endif

#endif /* HAL_INCLUDE_H_ */
