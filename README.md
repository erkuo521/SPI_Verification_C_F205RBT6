# SPI_Verification_C_F205RET6

HW: 	STM32F205RBT6 EVK
https://www.waveshare.net/wiki/Core205R
PCB:
https://www.waveshare.net/w/upload/0/05/CorexxxR-Schematic.pdf

SW: Keil v5.14
	SPI1:
	STM32F205RBTx ---- 300evk:	
	PA4-----------------CS
	PA5-----------------SCK
	PA6-----------------MISO
	PA7-----------------MOSI
	GND(near 5VIN)-----GND
	PA2----------------nRST
	PC3----------------DRDY
	 
	UART1:  921600 bps
	stm32F205rtb6------CH340:
	PB7_UART_RX--------CH340_TX
	PB6_UART_TX--------CH340_RX
	-----SWD shoud disconnected if you want UART keep working. restart power------key point
  
  
