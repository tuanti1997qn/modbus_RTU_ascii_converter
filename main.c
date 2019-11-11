#include "stm32f4xx.h"
#include "system_timetick.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define		BUFF_SIZE								200

uint8_t tx_buff_usart3[BUFF_SIZE];
uint8_t tx_buff_usart2[BUFF_SIZE];
uint8_t rx_buff_usart3[BUFF_SIZE];
uint8_t rx_buff_usart2[BUFF_SIZE];
uint8_t dma_rx_buff_usart3[BUFF_SIZE];
uint8_t dma_rx_buff_usart2[BUFF_SIZE];
uint8_t send_buff[BUFF_SIZE];
uint8_t receive_buff[BUFF_SIZE];

uint16_t CRC_Cal;  
uint16_t 	rcv_flag = 0;
uint8_t mode_uart2_master = 0, mode_uart3_master = 0;

//================================Private functions===============================//

							
void delay_us(uint16_t period){
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
  	TIM6->PSC = 83;		// clk = SystemCoreClock / 4 / (PSC+1) *2 = 1MHz
  	TIM6->ARR = period-1;
  	TIM6->CNT = 0;
  	TIM6->EGR = 1;		// update registers;
  	TIM6->SR  = 0;		// clear overflow flag
  	TIM6->CR1 = 1;		// enable Timer6

  	while (!TIM6->SR);
    
  	TIM6->CR1 = 0;		// stop Timer6
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, DISABLE);
}

void delay_01ms(uint16_t period){

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
  	TIM6->PSC = 8399;		// clk = SystemCoreClock / 4 / (PSC+1) *2 = 10KHz
  	TIM6->ARR = period-1;
  	TIM6->CNT = 0;
  	TIM6->EGR = 1;		// update registers;

  	TIM6->SR  = 0;		// clear overflow flag
  	TIM6->CR1 = 1;		// enable Timer6

  	while (!TIM6->SR);
    
  	TIM6->CR1 = 0;		// stop Timer6
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, DISABLE);
}


//==============================End private functions=============================//

//================================MODBUS functions===============================//
static const uint8_t aucCRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40
};

static const uint8_t aucCRCLo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
    0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
    0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9,
    0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
    0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
    0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF,
    0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
    0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 
    0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
    0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,
    0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
    0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,
    0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
    0x41, 0x81, 0x80, 0x40
};

uint16_t usMBCRC16( uint8_t * pucFrame, uint16_t usLen )
{
    uint8_t           ucCRCHi = 0xFF;
    uint8_t           ucCRCLo = 0xFF;
    int             iIndex;

    while( usLen-- )
    {
        iIndex = ucCRCLo ^ *( pucFrame++ );
        ucCRCLo = ( uint8_t )( ucCRCHi ^ aucCRCHi[iIndex] );
        ucCRCHi = aucCRCLo[iIndex];
    }
    return ( uint16_t )( ucCRCHi << 8 | ucCRCLo );
}
uint8_t LRC( uint8_t * pucFrame, uint16_t usLen )
{
    uint8_t           ucLRC = 0;  /* LRC char initialized */

    while( usLen-- )
    {
        ucLRC += *pucFrame++;   /* Add buffer byte without carry */
    }

    /* Return twos complement */
    ucLRC = ( uint8_t ) ( -( ( char ) ucLRC ) );
    return ucLRC;
}

void USART3_Send_Data (uint8_t num_byte)
{
	DMA1_Stream3->NDTR = num_byte;
	DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3);
	DMA_Cmd(DMA1_Stream3, ENABLE);
}

void USART2_Send_Data (uint8_t num_byte)
{
	DMA1_Stream6->NDTR = num_byte;
	DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_TCIF6);
	DMA_Cmd(DMA1_Stream6, ENABLE);
}

uint8_t nu2str (uint8_t nu)
{
	uint8_t str;
	switch (nu)
	{
		case 0:
		{
			str = '0';
			break;
		}
		case 1:
		{
			str = '1';
			break;
		}
		case 2:
		{
			str = '2';
			break;
		}
		case 3:
		{
			str = '3';
			break;
		}
		case 4:
		{
			str = '4';
			break;
		}
		case 5:
		{
			str = '5';
			break;
		}
		case 6:
		{
			str = '6';
			break;
		}
		case 7:
		{
			str = '7';
			break;
		}
		
		case 8:
		{
			str = '8';
			break;
		}
		case 9:
		{
			str = '9';
			break;
		}
		case 10:
		{
			str = 'A';
			break;
		}
		case 11:
		{
			str = 'B';
			break;
		}
		case 12:
		{
			str = 'C';
			break;
		}
		case 13:
		{
			str = 'D';
			break;
		}
		case 14:
		{
			str = 'E';
			break;
		}
		case 15:
		{
			str = 'F';
			break;
		}		
	}
	return str;
}

uint8_t str2nu (uint8_t str)
{
	uint8_t nu;
	switch (str)
	{
		case '0':
		{
			nu = 0;
			break;
		}
		case '1':
		{
			nu = 1;
			break;
		}
		case '2':
		{
			nu = 2;
			break;
		}
		case '3':
		{
			nu = 3;
			break;
		}
		case '4':
		{
			nu = 4;
			break;
		}
		case '5':
		{
			nu = 5;
			break;
		}
		case '6':
		{
			nu = 6;
			break;
		}
		case '7':
		{
			nu = 7;
			break;
		}
		
		case '8':
		{
			nu = 8;
			break;
		}
		case '9':
		{
			nu = 9;
			break;
		}
		case 'A':
		{
			nu = 10;
			break;
		}
		case 'B':
		{
			nu = 11;
			break;
		}
		case 'C':
		{
			nu = 12;
			break;
		}
		case 'D':
		{
			nu = 13;
			break;
		}
		case 'E':
		{
			nu = 14;
			break;
		}
		case 'F':
		{
			nu = 15;
			break;
		}		
	}
	return nu;
}


void Convert_Request_RTU2ASCII(uint8_t *pBufferIn, uint8_t *pBufferOut)
{
	int i;
	uint8_t lrc = 0, temp_lrc_1, temp_lrc_2;
	pBufferOut[0] = ':';
	for (i=0; i<6; i++)
	{
			pBufferOut[2*i+1] =  ((pBufferIn[i] & 0xF0)>>4);
			pBufferOut[2*i+2] =  (pBufferIn[i] & 0x0F);
	}
	//LRC 2 byte
	lrc = LRC(&pBufferIn[0],6);
	temp_lrc_1 = (lrc & 0xF0)>>4;
	pBufferOut[13]= temp_lrc_1;
	temp_lrc_2 = (lrc & 0x0F);
	pBufferOut[14]= temp_lrc_2;

  for (i=1; i<15; i++)
	{
		pBufferOut[i] = nu2str(pBufferOut[i]);		
	} 
	
	pBufferOut[15] = 0x0D;
	pBufferOut[16] = 0x0A;

}

void Convert_Request_ASCII2RTU(uint8_t *pBufferIn, uint8_t *pBufferOut)
{
	int i;
	for (i=1; i<13; i++)
	{
			pBufferIn[i] = str2nu(pBufferIn[i]);
	}
	for (i=0; i<6; i++)
	{
			pBufferOut[i] = (((pBufferIn[2*i+1])<<4)&0xF0) | ((pBufferIn[2*i+2])&0x0F);
	}
	//CRC 2 bytes
	CRC_Cal = usMBCRC16(&pBufferOut[0],6);
	pBufferOut[6] = (uint8_t)CRC_Cal;
	pBufferOut[7] = CRC_Cal>>8;
}


void Convert_Response_RTU2ASCII(uint8_t *pBufferIn, uint8_t *pBufferOut) //chua xai toi
{
	int i;
	//receive modbus rtu
	uint8_t lrc = 0, temp_lrc_1, temp_lrc_2;
	pBufferOut[0] = ':';
	
	for (i=0; i<3; i++)
	{
			pBufferOut[2*i+1] =  nu2str(((pBufferIn[i] & 0xF0)>>4));
			pBufferOut[2*i+2] =  nu2str((pBufferIn[i] & 0x0F));
	}
	for (i=3; i< 3 + pBufferIn[2]; i++)
	{
			pBufferOut[2*i+1] =  nu2str(((pBufferIn[i] & 0xF0)>>4));
			pBufferOut[2*i+2] =  nu2str((pBufferIn[i] & 0x0F));
	}
	//LRC 2 byte
	lrc = LRC(&pBufferIn[0],3 + pBufferIn[2]);
	temp_lrc_1 = (lrc & 0xF0)>>4;
	pBufferOut[pBufferIn[2]*2 + 7]= nu2str(temp_lrc_1);
	temp_lrc_2 = (lrc & 0x0F);
	pBufferOut[pBufferIn[2]*2 + 8]= nu2str(temp_lrc_2);     
	pBufferOut[pBufferIn[2]*2 + 9] = 0x0D;
	pBufferOut[pBufferIn[2]*2 + 10] = 0x0A;
}

void Convert_Response_ASCII2RTU(uint8_t *pBufferIn, uint8_t *pBufferOut)
{
	int j;
	//Receive modbus ascii
	for (j=1;j<7;j++)
	{
		pBufferIn[j] = str2nu(pBufferIn[j]);
	}
	pBufferOut[0] = (((pBufferIn[1])<<4)|0x0F) &( (pBufferIn[2])|0xF0);
	pBufferOut[1] = (((pBufferIn[3])<<4)|0x0F) &( (pBufferIn[4])|0xF0);
	pBufferOut[2] = (((pBufferIn[5])<<4)|0x0F) &( (pBufferIn[6])|0xF0);
	for (j=3; j<(3+pBufferOut[2]);j++)
	{   
			uint8_t temp_data_asc_1, temp_data_asc_2;
			temp_data_asc_1 = str2nu(pBufferIn[2*j+1]);
			temp_data_asc_2 = str2nu(pBufferIn[2*j+2]);
		
			pBufferOut[j] = ((temp_data_asc_1<<4)|0x0F) &( temp_data_asc_2|0xF0);
	}
	//CRC 2 bytes
	CRC_Cal = usMBCRC16(&pBufferOut[0],3+pBufferOut[2]);        
	pBufferOut[pBufferOut[2]+3] = (uint8_t)CRC_Cal;
	pBufferOut[pBufferOut[2]+4] = CRC_Cal>>8;
}
//==============================End MODBUS functions=============================//

//=====================================Init======================================//
void init_main(void)
{
  GPIO_InitTypeDef 	GPIO_InitStructure; 
	GPIO_InitTypeDef	GPIO_Output;
	USART_InitTypeDef USART_InitStructure;   
	DMA_InitTypeDef  	DMA_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;
	
  /* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  /* Enable UART clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	/* Enable DMA1 clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	
  /* Connect USART3 pins & USART2 pins to AF2 */  
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
  
  /* GPIO Configuration for USART3 Tx */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  /* GPIO Configuration for USART3 Rx */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  /* GPIO Configuration for USART2 Tx */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  /* GPIO Configuration for USART2 Rx */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	/* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
	GPIO_Output.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
	GPIO_Output.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Output.GPIO_OType = GPIO_OType_PP;
	GPIO_Output.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Output.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_Output);
	
	/* Configure PC7, PC3 in output pushpull mode */
	GPIO_Output.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_7;
	GPIO_Output.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Output.GPIO_OType = GPIO_OType_PP;
	GPIO_Output.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Output.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_Output);
       
  /* USARTx configured as follow:
		- BaudRate = 115200 baud  
    - Word Length = 8 Bits
    - One Stop Bit
    - No parity
    - Hardware flow control disabled (RTS and CTS signals)
    - Receive and transmit enabled
  */
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);
	USART_Init(USART2, &USART_InitStructure);  
	/* Enable USART3 & USART2 */
	USART_Cmd(USART3, ENABLE);
	USART_Cmd(USART2, ENABLE);
	/* Enable USART3 & USART2 DMA */
	USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE); 
	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE); 
	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
	/* Enable global interrupts for USART */
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
	/* Enable IDLE line detection for USART3 to process DMA transfer completed interrupt */
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
	/* Enable global interrupts for USART */
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
	/* Enable IDLE line detection for UART4 to process DMA transfer completed interrupt */
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);		
	
	/* DMA1 Stream3 Channel4 for USART3 Tx configuration */			
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)tx_buff_usart3;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_BufferSize = BUFF_SIZE;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// co mode circle
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream3, &DMA_InitStructure);
  DMA_Cmd(DMA1_Stream3, ENABLE);
	/* DMA2 Stream2 Channel4 for USART3 Rx configuration */			
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)dma_rx_buff_usart3;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = BUFF_SIZE;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// co mode circle
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream1, &DMA_InitStructure);
  DMA_Cmd(DMA1_Stream1, ENABLE);
	/* Enable DMA Interrupt to the highest priority */
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  /* Transfer complete interrupt mask */
  DMA_ITConfig(DMA1_Stream1, DMA_IT_TC, ENABLE);
  
  	/* DMA1 Stream6 Channel4 for USART2 Tx configuration */			
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)tx_buff_usart2;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_BufferSize = BUFF_SIZE;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// co mode circle
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream6, &DMA_InitStructure);
  DMA_Cmd(DMA1_Stream6, ENABLE);
	/* DMA1 Stream5 Channel4 for USART2 Rx configuration */			
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)dma_rx_buff_usart2;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = BUFF_SIZE;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// co mode circle
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream5, &DMA_InitStructure);
  DMA_Cmd(DMA1_Stream5, ENABLE);
	/* Enable DMA Interrupt to the highest priority */
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  /* Transfer complete interrupt mask */
  DMA_ITConfig(DMA1_Stream5, DMA_IT_TC, ENABLE);
}
//==================================End init=================================//

//=============================Interrupt handlers============================//
void DMA1_Stream1_IRQHandler(void)
{
  uint16_t i;

  /* Clear the DMA1_Stream2 TCIF2 pending bit */
  DMA_ClearITPendingBit(DMA1_Stream1, DMA_IT_TCIF1);
  
	//Process
	for(i=0; i<BUFF_SIZE; i++)
		rx_buff_usart3[i] = dma_rx_buff_usart3[i];
	rcv_flag = 1;
	

	/* Prepare DMA for next transfer */
	/* Important! DMA stream won't start if all flags are not cleared first */
	DMA1->HIFCR = DMA_FLAG_DMEIF1 | DMA_FLAG_FEIF1 | DMA_FLAG_HTIF1 | DMA_FLAG_TCIF1 | DMA_FLAG_TEIF1;
	DMA1_Stream1->M0AR = (uint32_t)dma_rx_buff_usart3;   /* Set memory address for DMA again */
	DMA1_Stream1->NDTR = BUFF_SIZE;    						/* Set number of bytes to receive */
	DMA_Cmd(DMA1_Stream1, ENABLE);           			/* Start DMA transfer */

	//UART3 connect MASTER
	if (mode_uart2_master == 0)
	{	
		if (rx_buff_usart3[0] == ':')
		{
			Convert_Request_ASCII2RTU(rx_buff_usart3,tx_buff_usart2);
			mode_uart3_master = 1;
			GPIO_SetBits(GPIOC,GPIO_Pin_3);
			USART2_Send_Data(10);
		}
		else 
		{
			Convert_Request_RTU2ASCII(rx_buff_usart3,tx_buff_usart2);
			mode_uart3_master = 1;
			GPIO_SetBits(GPIOC,GPIO_Pin_3);
			USART2_Send_Data(19);	
		}
	}
	//UART3 connect SLAVE
	if (mode_uart2_master == 1)
	{
		if (rx_buff_usart3[0] == ':')
		{
			Convert_Response_ASCII2RTU(rx_buff_usart3,tx_buff_usart2);
			mode_uart2_master = 0;
			GPIO_SetBits(GPIOC,GPIO_Pin_3);
			USART2_Send_Data(5+tx_buff_usart3[2]*2);
		}
		else 
		{
			Convert_Response_RTU2ASCII(rx_buff_usart3,tx_buff_usart2);
			mode_uart2_master = 0;
			GPIO_SetBits(GPIOC,GPIO_Pin_3);
			USART2_Send_Data(11+rx_buff_usart3[2]*2 + 2);	//offset
		}
	}
	
	while (DMA_GetFlagStatus(DMA1_Stream5,DMA_FLAG_TCIF6) == 0)
	{
			
	}
	GPIO_ResetBits(GPIOC,GPIO_Pin_3);
}

void DMA1_Stream5_IRQHandler(void)
{
  uint16_t i;

  /* Clear the DMA1_Stream2 TCIF5 pending bit */
  DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);
	
	//Process
	for(i=0; i<BUFF_SIZE; i++)
		rx_buff_usart2[i] = dma_rx_buff_usart2[i];
	rcv_flag = 1;
	
	/* Prepare DMA for next transfer */
	/* Important! DMA stream won't start if all flags are not cleared first */
	DMA1->HIFCR = DMA_FLAG_DMEIF5 | DMA_FLAG_FEIF5 | DMA_FLAG_HTIF5 | DMA_FLAG_TCIF5 | DMA_FLAG_TEIF5;
	DMA1_Stream5->M0AR = (uint32_t)dma_rx_buff_usart2;   /* Set memory address for DMA again */
	DMA1_Stream5->NDTR = BUFF_SIZE;    						/* Set number of bytes to receive */
	DMA_Cmd(DMA1_Stream5, ENABLE);           			/* Start DMA transfer */
	
	//UART2 connect MASTER
	if (mode_uart3_master == 0)
	{	
		if (rx_buff_usart2[0] == ':')
		{
			Convert_Request_ASCII2RTU(rx_buff_usart2,tx_buff_usart3);
			mode_uart2_master = 1;
			GPIO_SetBits(GPIOC,GPIO_Pin_7);
			USART3_Send_Data(10);
		}
		else 
		{
			Convert_Request_RTU2ASCII(rx_buff_usart2,tx_buff_usart3);
			mode_uart2_master = 1;
			GPIO_SetBits(GPIOC,GPIO_Pin_7);
			USART3_Send_Data(19);
		}	
	}
	//UART2 connect SLAVE
	if (mode_uart3_master == 1)
	{
		if (rx_buff_usart2[0] == ':')
		{
			Convert_Response_ASCII2RTU(rx_buff_usart2,tx_buff_usart3);
			mode_uart3_master = 0;			
			GPIO_SetBits(GPIOC,GPIO_Pin_7);
			USART3_Send_Data(5+tx_buff_usart3[2]*2);
		}
		else 
		{
			Convert_Response_RTU2ASCII(rx_buff_usart2,tx_buff_usart3);
			mode_uart3_master = 0;			
			GPIO_SetBits(GPIOC,GPIO_Pin_7);
			USART3_Send_Data(11+rx_buff_usart2[2]*2 + 2);	//offset
		}
	}
		
	while (DMA_GetFlagStatus(DMA1_Stream1,DMA_FLAG_TCIF3) == 0)
	{
			
	}
	GPIO_ResetBits(GPIOC,GPIO_Pin_7);
}


void USART3_IRQHandler(void) 
{
    /* Check for IDLE flag */
    if (USART3->SR & USART_FLAG_IDLE) {         	/* We want IDLE flag only */
        /* This part is important */
        /* Clear IDLE flag by reading status register first */
        /* And follow by reading data register */
        volatile uint32_t tmp;                  /* Must be volatile to prevent optimizations */
        tmp = USART3->SR;                       	/* Read status register */
        tmp = USART3->DR;                       	/* Read data register */
        (void)tmp;                              /* Prevent compiler warnings */
//				USART_GetFlagStatus(UART4, USART_IT_IDLE); 
//				USART_ReceiveData(UART4);
        DMA_Cmd(DMA1_Stream1, DISABLE);       	/* Disabling DMA will force transfer complete interrupt if enabled */
    }

}

void USART2_IRQHandler(void) 
{
    /* Check for IDLE flag */
    if (USART2->SR & USART_FLAG_IDLE) {         	/* We want IDLE flag only */
        /* This part is important */
        /* Clear IDLE flag by reading status register first */
        /* And follow by reading data register */
        volatile uint32_t tmp;                  /* Must be volatile to prevent optimizations */
        tmp = USART2->SR;                       	/* Read status register */
        tmp = USART2->DR;                       	/* Read data register */
        (void)tmp;                              /* Prevent compiler warnings */
//				USART_GetFlagStatus(UART4, USART_IT_IDLE); 
//				USART_ReceiveData(UART4);
        DMA_Cmd(DMA1_Stream5, DISABLE);       	/* Disabling DMA will force transfer complete interrupt if enabled */
    }

}
//================================End interrupt==============================//

//================================Main program===============================//
int main(void)
{
	/* Enable SysTick at 10ms interrupt */
	SysTick_Config(SystemCoreClock/100);

	init_main();
	GPIO_ResetBits(GPIOC,GPIO_Pin_3);	//uart 2
	GPIO_ResetBits(GPIOC,GPIO_Pin_7);	//uart 3

	while(1)
	{		

	}

}
