/**************************************************************************************************
 
  Phyplus Microelectronics Limited confidential and proprietary. 
  All rights reserved.

  IMPORTANT: All rights of this software belong to Phyplus Microelectronics 
  Limited ("Phyplus"). Your use of this Software is limited to those 
  specific rights granted under  the terms of the business contract, the 
  confidential agreement, the non-disclosure agreement and any other forms 
  of agreements as a customer or a partner of Phyplus. You may not use this 
  Software unless you agree to abide by the terms of these agreements. 
  You acknowledge that the Software may not be modified, copied, 
  distributed or disclosed unless embedded on a Phyplus Bluetooth Low Energy 
  (BLE) integrated circuit, either as a product or is integrated into your 
  products.  Other than for the aforementioned purposes, you may not use, 
  reproduce, copy, prepare derivative works of, modify, distribute, perform, 
  display or sell this Software and/or its documentation for any purposes.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  PHYPLUS OR ITS SUBSIDIARIES BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
  
**************************************************************************************************/

/*******************************************************************************
* @file		spi.c
* @brief	Contains all functions support for spi driver
* @version	0.0
* @date		18. Oct. 2017
* @author	qing.han
* 
* Copyright(C) 2016, PhyPlus Semiconductor
* All rights reserved.
*
*******************************************************************************/
#include "spi.h"
#include "error.h"
#include <string.h>
#include "pwrmgr.h"
#include "clock.h"
#include "log.h"

typedef struct _spi_Context{
	spi_Cfg_t  cfg;
	hal_spi_t* spi_info;
	spi_data_t transmit;
}spi_Ctx_t;

static spi_Ctx_t m_spiCtx[2];
extern  uint32_t pclk;

////////////////// SPI  /////////////////////////////////////////
//this function is used just when you want to use two spi master module
int hal_spi_module_select(hal_spi_t* spi_ptr)
{
	if((spi_ptr == NULL) || (spi_ptr->spi_index > 1))
		return PPlus_ERR_INVALID_PARAM;

	if((spi_ptr != m_spiCtx[0].spi_info) && (spi_ptr != m_spiCtx[1].spi_info))
		return PPlus_ERR_NOT_REGISTED;

	HAL_ENTER_CRITICAL_SECTION();
	if(spi_ptr->spi_index == 0)
	{
		AP_PERI_MASTER_SELECT &= ~0x33;
		AP_PERI_MASTER_SELECT |= 0x11;
	}
	else
	{
		AP_PERI_MASTER_SELECT &= ~0x33;
		AP_PERI_MASTER_SELECT |= 0x22;
	}
	HAL_EXIT_CRITICAL_SECTION();
	
	return PPlus_SUCCESS;
}

static void hal_spi_write_fifo(AP_SSI_TypeDef *Ssix,uint8_t len,uint8_t* tx_rx_ptr)
{
	uint8_t i=0;
	HAL_ENTER_CRITICAL_SECTION();
	while(i<len)
	{
		SPI_DATA = *(tx_rx_ptr+i);
		i++;
	}
	HAL_EXIT_CRITICAL_SECTION();
}

static void hal_spi_read_fifo(AP_SSI_TypeDef *Ssix,uint8_t len,uint8_t* tx_rx_ptr)
{
	uint8_t i=0;
	HAL_ENTER_CRITICAL_SECTION();
	while(i<len)
	{
		*(tx_rx_ptr+i) =	SPI_DATA;
		i++;
	}
	HAL_EXIT_CRITICAL_SECTION();
}

void spi_int_enable(hal_spi_t* spi_ptr)
{
	AP_SSI_TypeDef *Ssix = AP_SPI0; 

	if(spi_ptr->spi_index == SPI1)
		Ssix = AP_SPI1;
	
	HAL_ENTER_CRITICAL_SECTION();
	NVIC_EnableIRQ((IRQn_Type)(SPI0_IRQ + spi_ptr->spi_index));
	NVIC_SetPriority((IRQn_Type)(SPI0_IRQ + spi_ptr->spi_index), IRQ_PRIO_HAL);
	HAL_EXIT_CRITICAL_SECTION();

	if(m_spiCtx[spi_ptr->spi_index].cfg.force_cs == true)
		Ssix->IMR = 0x11;
	else
		Ssix->IMR = 0x10;
}

static void spi_int_disable(hal_spi_t* spi_ptr)
{
	AP_SSI_TypeDef *Ssix = AP_SPI0; 

	if(spi_ptr->spi_index == SPI1)
		Ssix = AP_SPI1;

	HAL_ENTER_CRITICAL_SECTION();
	NVIC_DisableIRQ((IRQn_Type)(SPI0_IRQ + spi_ptr->spi_index));
	HAL_EXIT_CRITICAL_SECTION();

	Ssix->IMR = 0x00;
}

static void spi_int_callback(uint8_t spi_index)
{	
	AP_SSI_TypeDef *Ssix = AP_SPI0;
	volatile uint8_t spi_irs_status;
	spi_data_t* trans_ptr;
	uint32_t rx_ftlr;
	spi_Evt_t evt;
	bool complete_flag = false;
	uint16_t rem;
	uint8_t *rxBuf,*txBuf,i,len;
	
	if(((spi_index != 0) && (spi_index != 1)) || (m_spiCtx[spi_index].spi_info == NULL))
		return;
	
	if(spi_index == 1)
		Ssix = AP_SPI1;
	
	trans_ptr = &(m_spiCtx[spi_index].transmit);

	spi_irs_status = Ssix->ISR;
	if(spi_irs_status & TRANSMIT_FIFO_EMPTY)
	{
		if(m_spiCtx[spi_index].cfg.force_cs == true)
		{
			if(trans_ptr->tx_index >= trans_ptr->tx_rx_len)
				Ssix->IMR = 0x10;
		}
	}
	
	if(spi_irs_status & RECEIVE_FIFO_FULL)
	{
		rx_ftlr = Ssix->RXFTLR;
		rxBuf = trans_ptr->rx_ptr + trans_ptr->rx_index;
		
		i=0;
		while(i<(rx_ftlr+1))
		{
			*(rxBuf+i) = SPI_DATA;
			i++;
		}
				
		trans_ptr->rx_index += rx_ftlr+1;
		if(trans_ptr->rx_index < trans_ptr->tx_rx_len)
		{
			rem = trans_ptr->tx_rx_len - trans_ptr->rx_index;
			if(m_spiCtx[spi_index].cfg.force_cs == true)
			{
				len = (rem>=4)?4:rem;
			}
			else
			{
				len = (rem>=8)?8:rem;
			}
	
			if(rx_ftlr != (len-1))
			{
				Ssix->RXFTLR = len-1;
				Ssix->TXFTLR = len-1;
			}
			
			i=0;
			txBuf = trans_ptr->tx_ptr + trans_ptr->tx_index;
			while(i<len)
			{
				SPI_DATA = *(txBuf+i);
				i++;
			}
			trans_ptr->tx_index += len;
		}
		else
		{
			if(m_spiCtx[spi_index].cfg.force_cs == true)
				hal_gpio_fmux(m_spiCtx[spi_index].cfg.ssn_pin, Bit_ENABLE);
			complete_flag = true;
		}
	}
	
	if(complete_flag == true)
	{
		Ssix->IMR = 0x00;
		trans_ptr->idle = true;
		trans_ptr->tx_ptr = NULL;
		trans_ptr->rx_ptr = NULL;
		trans_ptr->tx_index = 0;
		trans_ptr->rx_index = 0;
		trans_ptr->tx_rx_len = 0;
		hal_pwrmgr_unlock((MODULE_e)(MOD_SPI0 + spi_index));
		
		evt.spi = spi_index;
		evt.type = TRANSMIT_COMPLETED;
		m_spiCtx[spi_index].cfg.evt_handler(evt);
	}
}

/**************************************************************************************
 * @fn          hal_SPI0_IRQHandler
 *
 * @brief       This function process for spi0 interrupt,when use int please consummate its callbackfunction
 *
 * input parameters
 *
 * @param       None.      
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
void __attribute__((used)) hal_SPI0_IRQHandler(void)
{
	spi_int_callback(0);
}

/**************************************************************************************
 * @fn          hal_SPI1_IRQHandler
 *
 * @brief       This function process for spi1 interrupt,when use int please consummate its callbackfunction
 *
 * input parameters
 *
 * @param       None.      
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
void __attribute__((used)) hal_SPI1_IRQHandler(void)
{
	spi_int_callback(1);
}

/**************************************************************************************
 * @fn          hal_spi_pin_init
 *
 * @brief       This function process for spi pin initial(4 lines);You can use two spi,spi0 and spi1,should programe by USE_AP_SPIX 
 *
 * input parameters
 *
 * @param       GPIO_Pin_e sck_pin: define sclk pin
 *              GPIO_Pin_e ssn_pin: define ssn pin
 *              GPIO_Pin_e tx_pin: define transmit pin;when use as master,it's mosi pin;corresponding,use as slave,it's miso
 *              GPIO_Pin_e rx_pin: define receive pin;when use as master,it's miso pin;corresponding,use as slave,it's mosi
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
static void hal_spi_pin_init(hal_spi_t* spi_ptr,GPIO_Pin_e sck_pin,GPIO_Pin_e ssn_pin,GPIO_Pin_e tx_pin,GPIO_Pin_e rx_pin)
{
	if((sck_pin >= GPIO_P31) || (sck_pin <= GPIO_P03))
		gpio_pin0to3_pin31to34_control(sck_pin,1);

	if((ssn_pin >= GPIO_P31) || (ssn_pin <= GPIO_P03))
		gpio_pin0to3_pin31to34_control(ssn_pin,1);

	if((tx_pin >= GPIO_P31) || (tx_pin <= GPIO_P03))
		gpio_pin0to3_pin31to34_control(tx_pin,1);

	if((rx_pin >= GPIO_P31) || (rx_pin <= GPIO_P03))
		gpio_pin0to3_pin31to34_control(rx_pin,1);
	
	if(spi_ptr->spi_index == SPI0)
	{
		hal_gpio_fmux_set(sck_pin, SPI_0_SCK);
		hal_gpio_fmux_set(ssn_pin, SPI_0_SSN);
		hal_gpio_fmux_set(tx_pin, SPI_0_TX);
		hal_gpio_fmux_set(rx_pin, SPI_0_RX);
	}
	else if(spi_ptr->spi_index == SPI1)
	{
		hal_gpio_fmux_set(sck_pin, SPI_1_SCK);
		hal_gpio_fmux_set(ssn_pin, SPI_1_SSN);
		hal_gpio_fmux_set(tx_pin, SPI_1_TX);
		hal_gpio_fmux_set(rx_pin, SPI_1_RX);
	}	
}

static void hal_spi_pin_deinit(GPIO_Pin_e sck_pin,GPIO_Pin_e ssn_pin,GPIO_Pin_e tx_pin,GPIO_Pin_e rx_pin)
{
	hal_gpio_fmux(sck_pin, Bit_DISABLE);
	hal_gpio_fmux(ssn_pin, Bit_DISABLE); 
	hal_gpio_fmux(tx_pin, Bit_DISABLE); 
	hal_gpio_fmux(rx_pin, Bit_DISABLE); 
	
	if((sck_pin >= GPIO_P31) || (sck_pin <= GPIO_P03))
		gpio_pin0to3_pin31to34_control(sck_pin,0);

	if((ssn_pin >= GPIO_P31) || (ssn_pin <= GPIO_P03))
		gpio_pin0to3_pin31to34_control(ssn_pin,0);

	if((tx_pin >= GPIO_P31) || (tx_pin <= GPIO_P03))
		gpio_pin0to3_pin31to34_control(tx_pin,0);

	if((rx_pin >= GPIO_P31) || (rx_pin <= GPIO_P03))
		gpio_pin0to3_pin31to34_control(rx_pin,0);
}

/**************************************************************************************
 * @fn          hal_spi_master_init
 *
 * @brief       This function process for spi master initial
 *
 * input parameters
 *
 * @param       uint32_t baud: baudrate select
 *              SPI_SCMOD_e scmod: Serial Clock Polarity and Phase select;	SPI_MODE0,        //SCPOL=0,SCPH=0(default)
 *																																					SPI_MODE1,        //SCPOL=0,SCPH=1
 *																																					SPI_MODE2,        //SCPOL=1,SCPH=0
 *																																					SPI_MODE3,        //SCPOL=1,SCPH=1
 *              SPI_TMOD_e tmod: Transfer Mode                              SPI_TRXD,        //Transmit & Receive(default)
 *                                                                          SPI_TXD,         //Transmit Only
 *                                                                          SPI_RXD,         //Receive Only
 *                                                                          SPI_EEPROM,      //EEPROM Read	
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
static void hal_spi_master_init(hal_spi_t* spi_ptr,uint32_t baud,SPI_SCMOD_e scmod,SPI_TMOD_e tmod)	
{
	uint8_t shift = 0; 							
	AP_SSI_TypeDef *Ssix = AP_SPI0;
	uint16_t baud_temp;

	if(spi_ptr->spi_index == SPI1)
	{
		shift = 1;
		Ssix = AP_SPI1;
	}
	
	DISABLE_SPI;	  
	
	AP_PERI_MASTER_SELECT |= (BIT(shift)|BIT(shift+4));
	Ssix->CR0= ((Ssix->CR0) & 0xfffffc3f)|(scmod<<6)|(tmod<<8);

	baud_temp = (pclk + (baud>>1)) / baud;
	if(baud_temp<2)
	{
		baud_temp = 2;
	}
	else if(baud_temp>65534)
	{
		baud_temp =65534;
	}	
	Ssix->BAUDR= baud_temp;   // set clock(round)
	
	Ssix->TXFTLR=3;    // set fifo threshold to triggle interrupt
	Ssix->RXFTLR=3;

	Ssix->IMR = 0x00;
	Ssix->SER=1;      //enable slave device
	
	ENABLE_SPI;
}

/**************************************************************************************
 * @fn          hal_spi_slave_init
 *
 * @brief       This function process for spi slave initial
 *
 * input parameters
 *
 * @param       uint32_t baud: baudrate select
 *              SPI_SCMOD_e scmod: Serial Clock Polarity and Phase select;	SPI_MODE0,        //SCPOL=0,SCPH=0(default)
 *																																					SPI_MODE1,        //SCPOL=0,SCPH=1
 *																																					SPI_MODE2,        //SCPOL=1,SCPH=0
 *																																					SPI_MODE3,        //SCPOL=1,SCPH=1
 *              SPI_TMOD_e tmod: Transfer Mode                              SPI_TRXD,        //Transmit & Receive(default)
 *                                                                          SPI_TXD,         //Transmit Only
 *                                                                          SPI_RXD,         //Receive Only
 *                                                                          SPI_EEPROM,      //EEPROM Read	
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
/*static*/ void hal_spi_slave_init(hal_spi_t* spi_ptr,uint32_t baud,SPI_SCMOD_e scmod,SPI_TMOD_e tmod)
{
	uint8_t shift = 0;
	AP_SSI_TypeDef *Ssix = AP_SPI0;
	uint16_t baud_temp;	

	if(spi_ptr->spi_index == SPI1)
	{
		shift = 1;
		Ssix = AP_SPI1;
	}
	
	DISABLE_SPI;
	
	AP_PERI_MASTER_SELECT &= ~BIT(shift);
	Ssix->CR0= ((Ssix->CR0) & 0xfffffc3f)|(scmod<<6)|(tmod<<8)|0x400;
	
	baud_temp = (pclk + (baud>>1)) / baud;
	if(baud_temp<2)
	{
		baud_temp = 2;
	}
	else if(baud_temp>65534)
	{
		baud_temp =65534;
	}	
	Ssix->BAUDR= baud_temp;   // set clock(round)
	
	Ssix->TXFTLR=1;    // set fifo threshold to triggle interrupt
	Ssix->RXFTLR=0;
	Ssix->IMR=0x3f;
//	Ssix->SER=1;      //enable slave device
	ENABLE_SPI;
}

/**************************************************************************************
 * @fn          hal_spi_init
 *
 * @brief       This function will config the spi module.
 *
 * input parameters
 *
 * @param       	spi_Cfg_t cfg:there are two parameter you need config to use spi,one is cfg,another is the macro SPI_MASTER_MODE
 *                refer to struct spi_Cfg_t,you will config it right.
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      
 * 							PPlus_SUCCESS:config success.
 * 							PPlus_ERR_BUSY:the spi you want to use is work,please use another or stop it and reuse it.
 * 							PPlus_ERR_INVALID_PARAM:there are two spi in mcu,you parameter is invalid.
 **************************************************************************************/
int hal_spi_bus_init(hal_spi_t* spi_ptr,spi_Cfg_t cfg)
{
	if((spi_ptr == NULL) || (spi_ptr->spi_index > 1))
		return PPlus_ERR_INVALID_PARAM;
	
	if(m_spiCtx[spi_ptr->spi_index].spi_info != NULL)
		return PPlus_ERR_BUSY;

	clk_gate_enable((MODULE_e)(MOD_SPI0 + spi_ptr->spi_index));
	
	hal_spi_pin_init(spi_ptr,cfg.sclk_pin,cfg.ssn_pin,cfg.MOSI,cfg.MISO);

#if SPI_MASTER_MODE
	hal_spi_master_init(spi_ptr,cfg.baudrate, cfg.spi_scmod, cfg.spi_tmod);
#else
	hal_spi_slave_init(spi_ptr,cfg.baudrate, cfg.spi_scmod, cfg.spi_tmod);
#endif
	
	m_spiCtx[spi_ptr->spi_index].cfg = cfg;
	m_spiCtx[spi_ptr->spi_index].transmit.idle = true;
	m_spiCtx[spi_ptr->spi_index].spi_info = spi_ptr;
	
	if(m_spiCtx[spi_ptr->spi_index].cfg.int_mode == true)
		spi_int_enable(spi_ptr);
	else
		spi_int_disable(spi_ptr);
	
	return PPlus_SUCCESS;
}

/**************************************************************************************
 * @fn          hal_spi_deinit
 *
 * @brief       This function will deinit the spi you select.
 *
 * input parameters
 *
 * @param       	hal_spi_t* spi_ptr: spi module handle.

 *
 * output parameters
 *
 * @param       None.
 *
 * @return      
 *              PPlus_SUCCESS
 *              PPlus_ERR_INVALID_PARAM
 **************************************************************************************/
int hal_spi_bus_deinit(hal_spi_t* spi_ptr)
{
	if((spi_ptr == NULL) || (spi_ptr->spi_index > 1))
		return PPlus_ERR_INVALID_PARAM;
	
	if((spi_ptr != m_spiCtx[0].spi_info) && (spi_ptr != m_spiCtx[1].spi_info))
		return PPlus_ERR_NOT_REGISTED;
	
	clk_gate_disable((MODULE_e)(MOD_SPI0 + spi_ptr->spi_index));
	//spi_int_disable(spi_ptr);
	
	hal_spi_pin_deinit(m_spiCtx[spi_ptr->spi_index].cfg.sclk_pin,m_spiCtx[spi_ptr->spi_index].cfg.ssn_pin,m_spiCtx[spi_ptr->spi_index].cfg.MOSI,m_spiCtx[spi_ptr->spi_index].cfg.MISO);
	memset(&m_spiCtx,0,2*sizeof(spi_Ctx_t));
	
	return PPlus_SUCCESS;
}

/**************************************************************************************
 * @fn          hal_spi_tx_rx_single_frame
 *
 * @brief       spi master sends commands and receives response,its length is from 1 to 8.
 *
 * input parameters
 *
 * @param       hal_spi_t* spi_ptr: spi module handle.
 *              uint8_t* tx_buf:tx buf pointer.
 *              uint8_t tx_len:tx buf length,its length is from 1 to 8.          
 *
 * output parameters
 *
 * @param       uint8_t* rx_buf:rx buf pointer.
 *
 * @return      
 *              PPlus_SUCCESS
 *              PPlus_ERR_INVALID_PARAM
 *              PPlus_ERR_BUSY
 **************************************************************************************/
static int hal_spi_tx_rx_single_frame(hal_spi_t* spi_ptr,uint8_t* tx_buf,uint8_t* rx_buf,uint8_t len)
{
	int ret =  PPlus_ERR_BUSY;;
	uint8_t dummy_buf[8] ={0,0,0,0,0,0,0,0};//padding data with your application
	uint8_t* tx_rx_ptr = NULL;
	AP_SSI_TypeDef *Ssix = AP_SPI0; 
	
	if(spi_ptr->spi_index == SPI1)
		Ssix = AP_SPI1;
	
	if(tx_buf != NULL)
		tx_rx_ptr = tx_buf;
	else
		tx_rx_ptr = dummy_buf;
	
	//while(SPI_BUSY);	
	if(TX_FIFO_NOT_FULL)
	{
		hal_spi_write_fifo(Ssix,len,tx_rx_ptr);
		
		SPI_INIT_TOUT(to);
		while(SPI_BUSY)
		{
			if(true == SPI_CHECK_TOUT(to, SPI_OP_TIMEOUT))
				return PPlus_ERR_TIMEOUT;
		}
		ret = PPlus_SUCCESS;
	}
	else
	{
		return ret;
	}
	
	if(rx_buf != NULL)
		tx_rx_ptr = rx_buf;
	else
		tx_rx_ptr = dummy_buf;
	
	//while(SPI_BUSY);
	len = NUMBER_DATA_RX_FIFO;
	hal_spi_read_fifo(Ssix,len,tx_rx_ptr);

	return ret;
}

static void spi0_sleep_handler(void)
{
	if(m_spiCtx[0].spi_info != NULL)
		hal_spi_bus_deinit(m_spiCtx[0].spi_info);		
}

static void spi1_sleep_handler(void)
{
	if(m_spiCtx[1].spi_info != NULL)
		hal_spi_bus_deinit(m_spiCtx[1].spi_info);		
}

/**************************************************************************************
 * @fn          hal_spi_init
 *
 * @brief       it is used to init spi module.
 *
 * input parameters
 * @param       None
 *
 * output parameters
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
void hal_spi_init(void)
{
	hal_pwrmgr_register(MOD_SPI0,spi0_sleep_handler, NULL);
	hal_pwrmgr_register(MOD_SPI1,spi1_sleep_handler, NULL);
	memset(&m_spiCtx,0,2*sizeof(spi_Ctx_t));
}

int hal_spi_transmit(hal_spi_t* spi_ptr,uint8_t* tx_buf,uint8_t* rx_buf,uint16_t len)
{
	uint8_t remainder = 0,temp = 0,trans_len = 4;
	AP_SSI_TypeDef *Ssix = AP_SPI0; 
	uint16_t offset = 0;
	
	if((spi_ptr == NULL) || (spi_ptr->spi_index > 1))
		return PPlus_ERR_INVALID_PARAM;
	
	if((spi_ptr != m_spiCtx[0].spi_info) && (spi_ptr != m_spiCtx[1].spi_info))
		return PPlus_ERR_NOT_REGISTED;
	
	if((tx_buf == NULL) && (rx_buf == NULL))
		return PPlus_ERR_INVALID_PARAM;
	
	if(len == 0)
		return PPlus_ERR_INVALID_PARAM;
	
	if(m_spiCtx[spi_ptr->spi_index].cfg.int_mode == true)//set tx buf first when use int,use hal_spi_int_set_tx_buf
	{
		if((m_spiCtx[spi_ptr->spi_index].transmit.tx_ptr == NULL) || (len > m_spiCtx[spi_ptr->spi_index].transmit.tx_buf_len))
			return PPlus_ERR_INVALID_PARAM;
	}
	
	if(m_spiCtx[spi_ptr->spi_index].transmit.idle == false)
		return PPlus_ERR_BUSY;
	
	
	if(spi_ptr->spi_index == SPI1)
		Ssix = AP_SPI1;
	
	if(m_spiCtx[spi_ptr->spi_index].cfg.force_cs == true)
	{
		hal_gpio_fmux(m_spiCtx[spi_ptr->spi_index].cfg.ssn_pin,Bit_DISABLE);
		hal_gpio_write(m_spiCtx[spi_ptr->spi_index].cfg.ssn_pin,0);
	}
	
	if(m_spiCtx[spi_ptr->spi_index].cfg.int_mode == false)
	{
		while(len > 0)
		{
			temp =(len >= 8)?8:len;
			hal_spi_tx_rx_single_frame(spi_ptr,(tx_buf + offset),(rx_buf + offset),temp);
			
			offset += temp;
			len -= temp;
		}
	
	if(m_spiCtx[spi_ptr->spi_index].cfg.force_cs == true)
		hal_gpio_fmux(m_spiCtx[spi_ptr->spi_index].cfg.ssn_pin,Bit_ENABLE);
	}
	else
	{
		spi_int_disable(spi_ptr);
		
		m_spiCtx[spi_ptr->spi_index].transmit.idle = false;
		m_spiCtx[spi_ptr->spi_index].transmit.rx_index = 0;
		m_spiCtx[spi_ptr->spi_index].transmit.tx_index = 0;
		m_spiCtx[spi_ptr->spi_index].transmit.rx_ptr = rx_buf;
		m_spiCtx[spi_ptr->spi_index].transmit.tx_rx_len = len;
		memcpy(m_spiCtx[spi_ptr->spi_index].transmit.tx_ptr,tx_buf,len);
		
		if(TX_FIFO_NOT_FULL)
		{
			//while(SPI_BUSY);
			hal_pwrmgr_lock((MODULE_e)(MOD_SPI0 + spi_ptr->spi_index));
			
			if(m_spiCtx[spi_ptr->spi_index].cfg.force_cs == true)			
				trans_len = 4;
			else
				trans_len = 8;
		
			remainder = (len >= trans_len)?trans_len:len;
			m_spiCtx[spi_ptr->spi_index].transmit.tx_index = remainder;
			Ssix->TXFTLR=remainder-1;
			Ssix->RXFTLR = remainder -1;
			hal_spi_write_fifo(Ssix,remainder,tx_buf);
			
		}
		//while(SPI_BUSY);
		spi_int_enable(spi_ptr);
	}
	
	return PPlus_SUCCESS;
}

int hal_spi_int_set_tx_buf(hal_spi_t* spi_ptr,uint8_t* tx_buf,uint16_t len)
{
	if((spi_ptr == NULL) || (spi_ptr->spi_index > 1) || (tx_buf == NULL) || (len == 0))
		return PPlus_ERR_INVALID_PARAM;

	if((spi_ptr != m_spiCtx[0].spi_info) && (spi_ptr != m_spiCtx[1].spi_info))
		return PPlus_ERR_NOT_REGISTED;
	 
	HAL_ENTER_CRITICAL_SECTION();
	m_spiCtx[spi_ptr->spi_index].transmit.tx_ptr = tx_buf;//used when tx int
	m_spiCtx[spi_ptr->spi_index].transmit.tx_buf_len = len;
	HAL_EXIT_CRITICAL_SECTION();

	return PPlus_SUCCESS;
}

int hal_spi_set_int_mode(hal_spi_t* spi_ptr,bool en)
{ 
	if((spi_ptr == NULL) || (spi_ptr->spi_index > 1))
		return PPlus_ERR_INVALID_PARAM;

	if((spi_ptr != m_spiCtx[0].spi_info) && (spi_ptr != m_spiCtx[1].spi_info))
		return PPlus_ERR_NOT_REGISTED;
	
	m_spiCtx[spi_ptr->spi_index].cfg.int_mode = en;
	
	if(en)
	{
			m_spiCtx[spi_ptr->spi_index].cfg.int_mode = true;
			spi_int_enable(spi_ptr);
	}
	else
	{
			m_spiCtx[spi_ptr->spi_index].cfg.int_mode = false;
			spi_int_disable(spi_ptr);
	}
	
	return PPlus_SUCCESS;
}

int hal_spi_set_force_cs(hal_spi_t* spi_ptr,bool en)
{
	if((spi_ptr == NULL) || (spi_ptr->spi_index > 1))
		return PPlus_ERR_INVALID_PARAM;

	if((spi_ptr != m_spiCtx[0].spi_info) && (spi_ptr != m_spiCtx[1].spi_info))
		return PPlus_ERR_NOT_REGISTED;
	
	m_spiCtx[spi_ptr->spi_index].cfg.force_cs = en;
	return PPlus_SUCCESS;
}

bool hal_spi_get_transmit_bus_state(hal_spi_t* spi_ptr)
{
	return m_spiCtx[spi_ptr->spi_index].transmit.idle;
}


void hal_spi_TxComplete(hal_spi_t* spi_ptr)
{
	AP_SSI_TypeDef *Ssix = AP_SPI0; 
	
	if((spi_ptr !=  m_spiCtx[0].spi_info)  &&  (spi_ptr !=  m_spiCtx[0].spi_info)){
		return;
	}
	
	if(spi_ptr->spi_index == SPI1){
		Ssix = AP_SPI1;
	}

  SPI_INIT_TOUT(to);
	while(SPI_BUSY)
	{
		if(true == SPI_CHECK_TOUT(to, SPI_OP_TIMEOUT)){
			LOG("timeout\n");
			break;
		}
	}		
}

void hal_spi_send_byte(hal_spi_t* spi_ptr,uint8_t data)
{
	AP_SSI_TypeDef *Ssix = AP_SPI0; 
	
	if((spi_ptr !=  m_spiCtx[0].spi_info)  &&  (spi_ptr !=  m_spiCtx[0].spi_info)){
		return;
	}
	
	if(spi_ptr->spi_index == SPI1){
		Ssix = AP_SPI1;
	}
	
	if(TX_FIFO_NOT_FULL){		 
		SPI_DATA = data & 0xff;
		SPI_INIT_TOUT(to);
		while(SPI_BUSY)
		{
			if(true == SPI_CHECK_TOUT(to, SPI_OP_TIMEOUT)){
				LOG("timeout\n");
				break;
			}
		}			 
	}
}
