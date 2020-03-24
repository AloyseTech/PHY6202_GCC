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
* @file		adc.h
* @brief	Contains all functions support for adc driver
* @version	0.0
* @date		18. Oct. 2017
* @author	qing.han
* 
* Copyright(C) 2016, PhyPlus Semiconductor
* All rights reserved.
*
*******************************************************************************/
#ifndef __ADC__H__
#define __ADC__H__

#include "types.h"

#define    MAX_ADC_SAMPLE_SIZE     32
#define    MAX_ADC_SAMPLE_ID       (MAX_ADC_SAMPLE_SIZE-1)

//#define    BELOW_1V                 0
//#define    UP_1V                    1
//#define    ADC_INPUT_LEVEL          BELOW_1V
//#define    ADC_INPUT_LEVEL          UP_1V


#define    ADC_CH_BASE             (0x40050400UL)
#define    ADC_SAMPLE_AUTO              0
#define    ADC_SAMPLE_MANNUAL           1
#define    ADC_SAMPLE_MODE              SAMPLE_MANNUAL
//#define    ADC_SAMPLE_MODE              SAMPLE_AUTO

#define    ENABLE_ADC_INT      *(volatile unsigned int *)0x40050034 |= 0x000001ff
#define    MASK_ADC_INT        *(volatile unsigned int *)0x40050034 &= 0xfffffe00
#define    ADC_IRQ_ENABLE      *(volatile unsigned int *) 0xe000e100 = BIT(29)
#define    ADC_IRQ_DISABLE     *(volatile unsigned int *) 0xe000e100 = BIT(29)
#define    CLEAR_ADC_INT_CH0   *(volatile unsigned int *)0x40050038 |= BIT(0)
#define    CLEAR_ADC_INT_CH1   *(volatile unsigned int *)0x40050038 |= BIT(1)
#define    CLEAR_ADC_INT_CH2   *(volatile unsigned int *)0x40050038 |= BIT(2)
#define    CLEAR_ADC_INT_CH3   *(volatile unsigned int *)0x40050038 |= BIT(3)
#define    CLEAR_ADC_INT_CH4   *(volatile unsigned int *)0x40050038 |= BIT(4)
#define    CLEAR_ADC_INT_CH5   *(volatile unsigned int *)0x40050038 |= BIT(5)  
#define    CLEAR_ADC_INT_CH6   *(volatile unsigned int *)0x40050038 |= BIT(6)
#define    CLEAR_ADC_INT_CH7   *(volatile unsigned int *)0x40050038 |= BIT(7)
#define    CLEAR_ADC_INT_VOICE *(volatile unsigned int *)0x40050038 |= BIT(8)
#define    CLEAR_ADC_INT(n)    *(volatile unsigned int *)0x40050038 |= BIT(n)

#define    IS_CLAER_ADC_INT_CH0 (*(volatile unsigned int *)0x4005003c) & BIT(0)
#define    IS_CLAER_ADC_INT_CH1 (*(volatile unsigned int *)0x4005003c) & BIT(1)
#define    IS_CLAER_ADC_INT_CH2 (*(volatile unsigned int *)0x4005003c) & BIT(2)
#define    IS_CLAER_ADC_INT_CH3 (*(volatile unsigned int *)0x4005003c) & BIT(3)
#define    IS_CLAER_ADC_INT_CH4 (*(volatile unsigned int *)0x4005003c) & BIT(4)
#define    IS_CLAER_ADC_INT_CH5 (*(volatile unsigned int *)0x4005003c) & BIT(5)
#define    IS_CLAER_ADC_INT_CH6 (*(volatile unsigned int *)0x4005003c) & BIT(6)
#define    IS_CLAER_ADC_INT_CH7 (*(volatile unsigned int *)0x4005003c) & BIT(7)
#define    IS_CLAER_ADC_INT_VOICE (*(volatile unsigned int *)0x4005003c) & BIT(8)
#define    IS_CLAER_ADC_INT(n)   (*(volatile unsigned int *)0x4005003c) & BIT(n)

#define    GET_IRQ_STATUS         ((*(volatile unsigned int *) 0x4005003c) & 0x3ff)
#define    ENABLE_ADC             (*(volatile unsigned int *)0x4000f048 |= BIT(3))
#define    DISABLE_ADC            (*(volatile unsigned int *)0x4000f048 &= ~BIT(3))
#define    ADC_CLOCK_ENABLE       (*(volatile unsigned int *)0x4000f044 |= BIT(13))
#define    ADC_CLOCK_DISABLE       (*(volatile unsigned int *)0x4000f044 &= ~BIT(13))
#define    ADC_DBLE_CLOCK_DISABLE (*(volatile unsigned int *)0x4000f044 &= ~BIT(21))
#define    POWER_DOWN_ADC         (*(volatile unsigned int *)0x4000f048 &= ~BIT(3))
#define    POWER_UP_TEMPSENSOR    (*(volatile unsigned int *)0x4000f048 |= BIT(29))
#define    REG_IO_CONTROL         ((volatile unsigned int  *)0x4000f020)



#define ADCC_REG_BASE   (0x4000F000)



/**************************************************************************************
 * @fn          hal_get_adc_int_source
 *
 * @brief       This function process for get adc interrupt source,such as adc channel NO 
 *
 * input parameters
 *
 * @param       None.      
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      adc interrupt source bit loaction(uint8_t) 
 **************************************************************************************/




typedef enum {
  ADC_CH0 =0,
  ADC_CH1 =1,
  ADC_CH1P =2,  ADC_CH1DIFF = 2, ADC_CH1P_P12=2,
  ADC_CH1N =3,  ADC_CH1N_P11= 3,
  ADC_CH2P =4,  ADC_CH2DIFF = 4, ADC_CH2P_P14=4,
  ADC_CH2N =5,  ADC_CH2N_P13= 5, 
  ADC_CH3P =6,  ADC_CH3DIFF = 6, ADC_CH3P_P20=6,
  ADC_CH3N =7,  ADC_CH3N_P15= 7,
  ADC_CH_VOICE =8,
  ADC_CH_NUM =9,
}adc_CH_t;




enum{
  HAL_ADC_EVT_DATA = 1,
  HAL_ADC_EVT_FAIL = 0xff
};


typedef enum {
	HAL_ADC_CLOCK_80K = 0,
	HAL_ADC_CLOCK_160K = 1,
	HAL_ADC_CLOCK_320K = 2,
}adc_CLOCK_SEL_t;

typedef struct _adc_Cfg_t{
  bool  is_continue_mode; 
  bool  is_differential_mode;
  bool  is_high_resolution; //high resolution for adc sampling
  bool  is_auto_mode; //manual|auto mode
}adc_Cfg_t;


typedef struct _adc_Evt_t{
  int       type;
  adc_CH_t  ch;
  uint16_t* data;
  uint8_t   size; //word size
}adc_Evt_t;

typedef void (*adc_Hdl_t)(adc_Evt_t* pev);


typedef struct _adc_Contex_t{
  bool      enable;
  bool      is_continue_mode;
  bool      is_differential_mode;
  adc_Hdl_t  evt_handler[ADC_CH_NUM];
}adc_Ctx_t;

/**************************************************************************************
 * @fn          hal_adc_init
 *
 * @brief       This function process for adc initial 
 *
 * input parameters
 *
 * @param       ADC_CH_e adc_pin: adc pin select;ADC_CH0~ADC_CH7 and ADC_CH_VOICE
 *              ADC_SEMODE_e semode: single-end mode and diff mode select; 1:SINGLE_END(single-end mode) 0:DIFF(Diff mode)
 *              IO_CONTROL_e amplitude: input signal amplitude, 0:BELOW_1V,1:UP_1V
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
void hal_adc_init(void);

int hal_adc_config_channel(adc_CH_t channel, adc_Cfg_t cfg, adc_Hdl_t evt_handler);

int hal_adc_clock_config(adc_CLOCK_SEL_t clk);

int hal_adc_start(void);

int hal_adc_stop(void);

float hal_adc_value(uint16_t* buf, uint8_t size, bool high_resol, bool diff_mode);

void __attribute__((weak)) hal_ADC_IRQHandler(void);

float hal_adc_value_cal(adc_CH_t ch,uint16_t* buf, uint8_t size, bool high_resol, bool diff_mode);


#endif
