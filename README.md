# LIS3DH_STML475

#### Simple SPI driver (using STM_HAL) and example code for basic LIS3DH accelerometer & ADC read from STM_HAL supported devices.

### All code is in main.c

### Copy/port the blocks below.  Blocks delimited by //~~~~~~~~~~~~~ comments


### 1 >> Copy #defines and device driver code:
	USER CODE BEGIN 0, //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Driver code for LIS3DH control ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

### 2 >> Copy device initialization code:
	USER CODE BEGIN 2, //~~~~~~~~~~~~~~~~~~~~~~~~~~ Initialization code for LIS3DH control ~~~~~~~~~~~~~~~~~~~~~~~~~~
	Configure code for device specific SPI interface.

### 3 >> Copy SPI initilization code (or modify STM32CubeMX configuration and regenerate):
	/* SPI1 parameter configuration*/, //~~~~~~~~~~~~~~~~~~~~~~~~~~~ STM_HAL SPI initialization code for LIS3DH ~~~~~~~~~~~~~~~~~~~~~~~~~~


# HAL_stm32_lis3dh
