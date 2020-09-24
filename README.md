# M480BSP_SPI_Slave
 M480BSP_SPI_Slave

update @ 2020/09/24

1. SPI hw config : PA0MFP_SPI0_MOSI , PA1MFP_SPI0_MISO , PA2MFP_SPI0_CLK , PA3MFP_SPI0_SS

2. Scenario : switch SPI master mode and slave mode per 200ms , only slave will receive and transmit data

3. use 2 defines , ENABLE_SPI_IRQ , ENABLE_SPI_POLLING

4. SPI master test condition : 8M , 512 bytes

5. below is SPI capture log , by polling and IRQ

![image](https://github.com/released/M480BSP_SPI_Slave/blob/master/log_polling.jpg)

![image](https://github.com/released/M480BSP_SPI_Slave/blob/master/log_IRQ.jpg)

