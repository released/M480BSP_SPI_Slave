/*************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    A project template for M480 MCU.
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define LED_R						(PH0)
#define LED_Y						(PH1)
#define LED_G						(PH2)


#define ENABLE_SPI_POLLING
//#define ENABLE_SPI_IRQ

#define MASTER_DATA_NUM			(512)

#define BridgeSpiPortNum				(SPI0)
#define SPI_GET_SSLINE_FLAG(spi)		(((spi)->STATUS & SPI_STATUS_SSLINE_Msk) >> SPI_STATUS_SSLINE_Pos)

#define SPI_GET_SSINAIF_FLAG(spi)		(((spi)->STATUS & SPI_STATUS_SSINAIF_Msk) >> SPI_STATUS_SSINAIF_Pos)
#define SPI_SET_SSINAIF_FLAG(spi)		((spi)->STATUS |= SPI_STATUS_SSINAIF_Msk)

#define SPI_GET_SSACTIF_FLAG(spi)		(((spi)->STATUS & SPI_STATUS_SSACTIF_Msk) >> SPI_STATUS_SSACTIF_Pos)
#define SPI_SET_SSACTIF_FLAG(spi)		((spi)->STATUS |= SPI_STATUS_SSACTIF_Msk)

uint8_t buffer[MASTER_DATA_NUM] = {0};
uint8_t TestCount = 0;
uint8_t RxData = 0;
uint32_t TxDataCount = 0;
uint32_t RxDataCount = 0;

typedef enum{
	flag_DEFAULT = 0 ,

	flag_set_SPI ,
	
	flag_END	
}Flag_Index;

volatile uint32_t BitFlag = 0;
#define BitFlag_ON(flag)							(BitFlag|=flag)
#define BitFlag_OFF(flag)							(BitFlag&=~flag)
#define BitFlag_READ(flag)							((BitFlag&flag)?1:0)
#define ReadBit(bit)								(uint32_t)(1<<bit)

#define is_flag_set(idx)							(BitFlag_READ(ReadBit(idx)))
#define set_flag(idx,en)							( (en == 1) ? (BitFlag_ON(ReadBit(idx))) : (BitFlag_OFF(ReadBit(idx))))


void dump_buffer(uint8_t *pucBuff, int nBytes)
{
    uint16_t i = 0;
    
    printf("dump_buffer : %2d\r\n" , nBytes);    
    for (i = 0 ; i < nBytes ; i++)
    {
        printf("0x%2X," , pucBuff[i]);
        if ((i+1)%8 ==0)
        {
            printf("\r\n");
        }            
    }
    printf("\r\n\r\n");
}

void SPI_Slave_TX(SPI_T *spi , uint8_t *buffer , uint32_t len)
{

	#if 1
    if ((SPI_GET_TX_FIFO_FULL_FLAG(BridgeSpiPortNum) == 0)&& (TxDataCount < len))
    {
        SPI_WRITE_TX(spi, buffer[TxDataCount]);
		TxDataCount++;
    }
	#else
    while (TxDataCount < len)
    {
        SPI_WRITE_TX(spi, buffer[TxDataCount++]);
		while(SPI_IS_BUSY(spi));
    }

	#endif
}

#if defined (ENABLE_SPI_IRQ)
void SPI0_IRQHandler(void)
{
	if (SPI_GET_SSACTIF_FLAG(BridgeSpiPortNum))
	{
		SPI_SET_SSACTIF_FLAG(BridgeSpiPortNum);

//		SPI_ClearRxFIFO(BridgeSpiPortNum);
//		SPI_ClearTxFIFO(BridgeSpiPortNum);

		while (!SPI_GET_SSINAIF_FLAG(BridgeSpiPortNum))
		{
			if (SPI_GET_RX_FIFO_EMPTY_FLAG(BridgeSpiPortNum) == 0)
			{
				buffer[RxDataCount++] = SPI_READ_RX(BridgeSpiPortNum);
			}

			if (RxDataCount == MASTER_DATA_NUM)
			{
				RxDataCount = 0;
			}
			
//			buffer[0] = TestCount+1;
//			buffer[0] = RxData;	
//			SPI_Slave_TX(BridgeSpiPortNum , &buffer[0] , 0x01);
		    if ((SPI_GET_TX_FIFO_FULL_FLAG(BridgeSpiPortNum) == 0) && (TxDataCount < MASTER_DATA_NUM	))
		    {
		        SPI_WRITE_TX(BridgeSpiPortNum, buffer[TxDataCount]);
				TxDataCount++;
		    }
		}

		SPI_SET_SSINAIF_FLAG(BridgeSpiPortNum);
//		SPI_DisableInt(BridgeSpiPortNum,SPI_SSACT_INT_MASK);

		TxDataCount = 0;
		RxDataCount = 0;
		
		printf("SPI_SLAVE : 0x%2X (0x%2X) , TestCount = 0x%2X\r\n" , RxData ,(uint8_t) ~RxData , TestCount);	
		
	}
}
#endif

void SPI_Function_Simulate(void)
{
	#if defined (ENABLE_SPI_POLLING)
	volatile uint8_t res = 0;

	SYS_ResetModule(SPI0_RST);
    SPI_Open(BridgeSpiPortNum, SPI_SLAVE, SPI_MODE_0, 8, (uint32_t)NULL);
	SPI_ClearRxFIFO(BridgeSpiPortNum);
	SPI_ClearTxFIFO(BridgeSpiPortNum);

//	SPI_SetFIFO(BridgeSpiPortNum,0,0);

//	TestCount = 0xAA;
//	buffer[0] = TestCount;
//	SPI_Slave_TX(BridgeSpiPortNum , &buffer[0] , 0x01);

//	buffer[0] = TestCount+1;
//	SPI_Slave_TX(BridgeSpiPortNum , &buffer[0] , 0x01);

	do
	{
//		printf("wait for SPI_GET_SSACTIF_FLAG\r\n");
	}while(SPI_GET_SSACTIF_FLAG(BridgeSpiPortNum) == 0);
	SPI_SET_SSACTIF_FLAG(BridgeSpiPortNum);

	while(SPI_GET_SSINAIF_FLAG(BridgeSpiPortNum) == 0)
	{
//		do
//		{
//			printf("wait for SPI_GET_SSLINE_FLAG\r\n");
//		}while(SPI_GET_SSLINE_FLAG(BridgeSpiPortNum));
	
		if (SPI_GET_RX_FIFO_EMPTY_FLAG(BridgeSpiPortNum) == 0)
		{
			buffer[RxDataCount++] = SPI_READ_RX(BridgeSpiPortNum);
		}

		if (RxDataCount == MASTER_DATA_NUM)
		{
			RxDataCount = 0;
		}		
		
//		buffer[0] = TestCount+1;
//		buffer[0] = RxData;	
//		SPI_Slave_TX(BridgeSpiPortNum , &buffer[0] , 0x01);
	    if ((SPI_GET_TX_FIFO_FULL_FLAG(BridgeSpiPortNum) == 0)&& (TxDataCount < MASTER_DATA_NUM	))
	    {
	        SPI_WRITE_TX(BridgeSpiPortNum, buffer[TxDataCount]);
			TxDataCount++;
	    }
	};
	
	SPI_SET_SSINAIF_FLAG(BridgeSpiPortNum);
//	printf("SPI_SLAVE : 0x%2X (0x%2X) , TestCount = 0x%2X\r\n" , RxData ,(uint8_t) ~RxData , TestCount);	

	TxDataCount = 0;
	RxDataCount = 0;

	#elif defined (ENABLE_SPI_IRQ)

	SYS_ResetModule(SPI0_RST);
    SPI_Open(BridgeSpiPortNum, SPI_SLAVE, SPI_MODE_0, 8, (uint32_t)NULL);

	SPI_ClearRxFIFO(BridgeSpiPortNum);
	SPI_ClearTxFIFO(BridgeSpiPortNum);

	SPI_SetFIFO(BridgeSpiPortNum , 4 , 4);

	SPI_EnableInt(BridgeSpiPortNum,SPI_SSACT_INT_MASK);
    SPI_WRITE_TX(BridgeSpiPortNum, 0xFFFFFFFF);    /* Dummy Write to prevent TX under run */
    NVIC_EnableIRQ(SPI0_IRQn);
	
	#endif

}


void TMR1_IRQHandler(void)
{
	static uint16_t CNT_LED = 0;	
	static uint16_t CNT_SPI = 0;	
	static uint8_t toggle = 0;
	
//	static uint32_t log = 0;	
	
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        TIMER_ClearIntFlag(TIMER1);
	
		if (CNT_SPI++ > 200)
		{		
			CNT_SPI = 0;
			toggle ^= 1;
			
			if (toggle)
			{
				set_flag(flag_set_SPI , ENABLE);
			}
			else
			{
				set_flag(flag_set_SPI , DISABLE);				
			}
		}

		if (CNT_LED++ > 1000)
		{		
			CNT_LED = 0;
//			printf("%s : %2d\r\n" , __FUNCTION__ , log++);
			LED_Y ^= 1;
		}
		
    }
}

void TIMER1_HW_Init(void)
{
    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);
}

void TIMER1_Init(void)
{
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);	
    TIMER_Start(TIMER1);
}

void TIMER0_HW_Init(void)
{
	CLK_EnableModuleClock(TMR0_MODULE);
	CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);
}

void TIMER0_Polling(uint32_t u32Usec)
{
	TIMER_Delay(TIMER0, u32Usec);
}

void LED_Init(void)
{
	GPIO_SetMode(PH,BIT0,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT1,GPIO_MODE_OUTPUT);
	GPIO_SetMode(PH,BIT2,GPIO_MODE_OUTPUT);
	
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable External XTAL (4~24 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(FREQ_192MHZ);
    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART clock source from HXT */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_PCLK1, MODULE_NoMsk);
    CLK_EnableModuleClock(SPI0_MODULE);

//    CLK_EnableModuleClock(PDMA_MODULE);

//	TIMER0_HW_Init();
	TIMER1_HW_Init();
	
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Setup SPI0 multi-function pins */
    SYS->GPA_MFPL |= SYS_GPA_MFPL_PA0MFP_SPI0_MOSI | SYS_GPA_MFPL_PA1MFP_SPI0_MISO | SYS_GPA_MFPL_PA2MFP_SPI0_CLK | SYS_GPA_MFPL_PA3MFP_SPI0_SS;

    /* Enable SPI0 clock pin (PA2) schmitt trigger */
    PA->SMTEN |= GPIO_SMTEN_SMTEN2_Msk;

    /* Enable SPI0 I/O high slew rate */
    GPIO_SetSlewCtl(PA, 0xF, GPIO_SLEWCTL_HIGH);
	
    /* Lock protected registers */
    SYS_LockReg();
}

/*
 * This is a template project for M480 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{
	uint32_t cnt = 0;

    SYS_Init();
    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());

	LED_Init();
	TIMER1_Init();

    /* Got no where to go, just loop forever */
    while(1)
    {
		if (is_flag_set(flag_set_SPI))
		{
			SPI_Function_Simulate();
		}
		else
		{
			SYS_ResetModule(SPI0_RST);
		    SPI_Open(BridgeSpiPortNum, SPI_MASTER, SPI_MODE_0, 8, (uint32_t)NULL);
			LED_R ^= 1;
			printf("SPI_MASTER : 0x%2X \r\n" , cnt++);
			SPI_ClearRxFIFO(BridgeSpiPortNum);
			SPI_ClearTxFIFO(BridgeSpiPortNum);
		}
		
    }
}

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
