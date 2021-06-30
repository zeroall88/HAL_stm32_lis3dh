Simple code example for basic LIS3DH accelerometer & ADC read from STM_HAL supported devices

All code is on main.c

Copy/port the blocks below.  Blocks delimited by //~~~~~~~~~~~~~ comments


1 >> Copy #defines and device driver code:
	USER CODE BEGIN 0, //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Driver code for LIS3DH control ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

2 >> Copy device initialization code:
	USER CODE BEGIN 2, //~~~~~~~~~~~~~~~~~~~~~~~~~~ Initialization code for LIS3DH control ~~~~~~~~~~~~~~~~~~~~~~~~~~
	Configure code for device specific SPI interface.

3 >> Copy SPI initilization code:
	/* SPI1 parameter configuration*/, //~~~~~~~~~~~~~~~~~~~~~~~~~~~ STM_HAL SPI initialization code for LIS3DH ~~~~~~~~~~~~~~~~~~~~~~~~~~

