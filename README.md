# HAL drivers
A repository with drivers/libraries for various ICs I wrote. All of the drivers are C++ objects and their constructors take pointers to the various HAL peripheral handles - so the drivers are hardware independent as long as you use HAL.

Current drivers (more info in the top of the respective cpp file):
 - NRF24L01(+) *RF chip*
 - AT24Cxx *EEPROM*
 - WS2812(B) *RGB LED*