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
* @file   adc.c
* @brief  Contains all functions support for adc driver
* @version  0.0
* @date   18. Oct. 2017
* @author qing.han
*
* Copyright(C) 2016, PhyPlus Semiconductor
* All rights reserved.
*
*******************************************************************************/

#include "error.h"
#include "ap_cp.h"
#include "common.h"
#include "gpio.h"
#include "pwrmgr.h"
#include "clock.h"
#include "adc.h"
#include <string.h>
#include "log.h"

static bool mAdc_init_flg = FALSE;
static adc_Ctx_t mAdc_Ctx;
static uint8_t  adc_cal_read_flag = 0;
static uint16_t adc_cal_postive=0x0fff;
static uint16_t adc_cal_negtive=0x0fff;

const static GPIO_Pin_e s_pinmap[ADC_CH_NUM] = {
  GPIO_DUMMY, //ADC_CH0 =0,
  GPIO_DUMMY, //ADC_CH1 =1,
  P12, //ADC_CH1P =2,  ADC_CH1DIFF = 2,
  P11, //ADC_CH1N =3,
  P14, //ADC_CH2P =4,  ADC_CH2DIFF = 4,
  P13, //ADC_CH2N =5,
  P20, //ADC_CH3P =6,  ADC_CH3DIFF = 6,
  P15, //ADC_CH3N =7,
  GPIO_DUMMY,  //ADC_CH_VOICE =8,
};

static void set_sampling_resolution(adc_CH_t channel, bool is_high_resolution,bool is_differential_mode)
{
  if(is_high_resolution){
    if(is_differential_mode)
    {
        BM_CLR(REG_IO_CONTROL,BIT(7+((channel>>1)<<1)-(channel&0x01)));
        BM_SET(REG_IO_CONTROL,BIT(((channel>>1)<<1)-(channel&0x01)-1));
        BM_CLR(REG_IO_CONTROL,BIT(7+(((channel+1)>>1)<<1)-((channel+1)&0x01)));
        BM_SET(REG_IO_CONTROL,BIT((((channel+1)>>1)<<1)-((channel+1)&0x01)-1));
    }
    else
    {
        BM_CLR(REG_IO_CONTROL,BIT(7+((channel>>1)<<1)-(channel&0x01)));
        BM_SET(REG_IO_CONTROL,BIT(((channel>>1)<<1)-(channel&0x01)-1));
    }
    
  }
  else
  {
    if(is_differential_mode)
    {
        BM_CLR(REG_IO_CONTROL,BIT(((channel>>1)<<1)-(channel&0x01)-1));
        BM_SET(REG_IO_CONTROL,BIT(7+((channel>>1)<<1)-(channel&0x01)));
        BM_CLR(REG_IO_CONTROL,BIT((((channel+1)>>1)<<1)-((channel+1)&0x01)-1));
        BM_SET(REG_IO_CONTROL,BIT(7+(((channel+1)>>1)<<1)-((channel+1)&0x01)));
    }
    else
    {
        BM_CLR(REG_IO_CONTROL,BIT(((channel>>1)<<1)-(channel&0x01)-1));
        BM_SET(REG_IO_CONTROL,BIT(7+((channel>>1)<<1)-(channel&0x01)));
    }   
  }
}

static void set_differential_mode(void)
{
  subWriteReg(0x4000f048,8,8,0);
  subWriteReg(0x4000f048,11,11,0);
}

static void set_single_mode(adc_CH_t channel)
{
  if(channel & 1)
    subWriteReg(0x4000f048,11,11,1);
  else
    subWriteReg(0x4000f048,8,8,1);
}

static void set_channel(adc_CH_t channel)
{
  int index = (int)channel;
  if(index==1){
    subWriteReg(0x4000f048,7,5,1);
    POWER_UP_TEMPSENSOR;
    return;
  }
  GPIO_Pin_e pin = s_pinmap[index];
  if(pin == GPIO_DUMMY)
    return;
  subWriteReg(0x4000f048,7,5,((channel>>1)+1));
}


static void set_analog_pin(adc_CH_t channel)
{
  uint8_t index = (int)channel;
  GPIO_Pin_e pin = s_pinmap[index];
  if(pin == GPIO_DUMMY)
    return;
  hal_gpio_cfg_analog_io(pin,Bit_ENABLE);
}


static void disable_analog_pin(adc_CH_t channel)
{
  int index = (int)channel;
  GPIO_Pin_e pin = s_pinmap[index];
  if(pin == GPIO_DUMMY)
    return;
  hal_gpio_cfg_analog_io(pin,Bit_DISABLE);
  hal_gpio_pin_init(pin,IE);       //ie=0,oen=1 set to imput
  hal_gpio_pull_set(pin,FLOATING);    //set pin pull up/down floating
}

static void clear_adcc_cfg(void)
{
  memset(&mAdc_Ctx, 0, sizeof(mAdc_Ctx));
}


/////////////// adc ////////////////////////////
/**************************************************************************************
 * @fn          hal_ADC_IRQHandler
 *
 * @brief       This function process for adc interrupt
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
void __attribute__((used)) hal_ADC_IRQHandler(void)
{
  int ch;
  uint16_t adc_data[MAX_ADC_SAMPLE_SIZE];
//  LOG("ADC IRQ\n");
  MASK_ADC_INT;
  for (ch = 0; ch < ADC_CH3N+1; ch++) {
    if (GET_IRQ_STATUS & BIT(ch)) {
      int n;
      for (n = 0; n < (MAX_ADC_SAMPLE_SIZE-3); n++) {
        adc_data[n] = (uint16_t)(read_reg(ADC_CH_BASE + (ch * 0x80) + ((n+3) * 4))&0xfff);
      }
      CLEAR_ADC_INT(ch);
      //while (IS_CLAER_ADC_INT(ch));
      if(mAdc_Ctx.enable == FALSE)
        continue;
//      LOG("CH%d\n",ch);
      if (mAdc_Ctx.evt_handler[ch]) {
        adc_Evt_t evt;
        evt.type = HAL_ADC_EVT_DATA;
        evt.ch = (adc_CH_t)ch;
        evt.data = adc_data;
        evt.size = MAX_ADC_SAMPLE_SIZE-3;
        mAdc_Ctx.evt_handler[ch](&evt);
      }
    }
  }

  if(mAdc_Ctx.is_continue_mode == FALSE)
    hal_adc_stop();
  
  ENABLE_ADC_INT;

}

static void adc_wakeup_hdl(void)
{
  NVIC_SetPriority((IRQn_Type)ADCC_IRQ, IRQ_PRIO_HAL);
}

/**************************************************************************************
 * @fn          hal_adc_init
 *
 * @brief       This function process for adc initial
 *
 * input parameters
 *
 * @param       ADC_MODE_e mode: adc sample mode select;1:SAM_MANNUAL(mannual mode),0:SAM_AUTO(auto mode)
 *              ADC_CH_e adc_pin: adc pin select;ADC_CH0~ADC_CH7 and ADC_CH_VOICE
 *              ADC_SEMODE_e semode: signle-ended mode negative side enable; 1:SINGLE_END(single-ended mode) 0:DIFF(Differentail mode)
 *              IO_CONTROL_e amplitude: input signal amplitude, 0:BELOW_1V,1:UP_1V
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      None.
 **************************************************************************************/
void hal_adc_init(void) {
  mAdc_init_flg = TRUE;
  hal_pwrmgr_register(MOD_ADCC,NULL,adc_wakeup_hdl);
  clear_adcc_cfg();
}



int hal_adc_config_channel(adc_CH_t channel, adc_Cfg_t cfg, adc_Hdl_t evt_handler)
{
  if(!mAdc_init_flg){
    return PPlus_ERR_NOT_REGISTED;
  }
  
  if(mAdc_Ctx.enable){
    return PPlus_ERR_BUSY;
  }

  if(evt_handler == NULL)
    return PPlus_ERR_INVALID_PARAM;

  if(channel == ADC_CH_VOICE || channel == ADC_CH0 /*||channel == ADC_CH1*/ ){
    return PPlus_ERR_NOT_SUPPORTED;
  }

  if((channel!=1)&&(cfg.is_differential_mode && (channel & 1))){
    return PPlus_ERR_INVALID_PARAM;
  }

//  if(mAdc_Ctx.is_differential_mode != cfg.is_differential_mode){
//    return PPlus_ERR_IO_CONFILCT;
//  }
  
  mAdc_Ctx.is_differential_mode = cfg.is_differential_mode;

  mAdc_Ctx.evt_handler[channel] = evt_handler;
  
  clk_gate_enable(MOD_ADCC);//enable ADCC clk gated
  
  //CLK_1P28M_ENABLE;
  AP_PCRM->CLKSEL |= BIT(6);
  
	//ENABLE_XTAL_OUTPUT;         //enable xtal 16M output,generate the 32M dll clock
  AP_PCRM->CLKHF_CTL0 |= BIT(18);
  
	//ENABLE_DLL;                  //enable DLL
  AP_PCRM->CLKHF_CTL1 |= BIT(7);
	
	//ADC_DBLE_CLOCK_DISABLE;      //disable double 32M clock,we are now use 32M clock,should enable bit<13>, diable bit<21>
  AP_PCRM->CLKHF_CTL1 &= ~BIT(21);
	
	//ADC_CLOCK_ENABLE;            //adc clock enbale,always use clk_32M
  AP_PCRM->CLKHF_CTL1 |= BIT(13);

  //subWriteReg(0x4000f07c,4,4,1);    //set adc mode,1:mannual,0:auto mode
  AP_PCRM->ADC_CTL4 |= BIT(4);
	//*(volatile unsigned int *) 0x4000f040=0x5014B820;
	//*(volatile unsigned int *) 0x4000f044=0x019028b0;
  //*(volatile unsigned int *) 0x4000f048=0x0000014b;

  set_sampling_resolution(channel, cfg.is_high_resolution,cfg.is_differential_mode);
  
  if(cfg.is_differential_mode){
    set_channel(channel);
    set_differential_mode();
    set_analog_pin(channel);
    set_analog_pin((adc_CH_t)(channel+1));
  }
  else
  {
    set_channel(channel);
    set_single_mode(channel);
    set_analog_pin(channel);
  }

  return PPlus_SUCCESS;
}

int hal_adc_clock_config(adc_CLOCK_SEL_t clk){
  if(!mAdc_init_flg){
    return PPlus_ERR_NOT_REGISTED;
  }
	subWriteReg(0x4000F000 + 0x7c,2,1,clk);
	return PPlus_SUCCESS;
}

int hal_adc_start(void)
{
  if(!mAdc_init_flg){
    return PPlus_ERR_NOT_REGISTED;
  }

  mAdc_Ctx.enable = TRUE;
  hal_pwrmgr_lock(MOD_ADCC);
  //ENABLE_ADC;
  AP_PCRM->ANA_CTL |= BIT(3);
  
  //ADC_IRQ_ENABLE;
  NVIC_EnableIRQ((IRQn_Type)ADCC_IRQ);
  //ENABLE_ADC_INT;
  AP_ADCC->intr_mask = 0x1ff;
  
  //disableSleep();
  return PPlus_SUCCESS;

}

int hal_adc_stop(void)
{
  int i;
  if(!mAdc_init_flg){
    return PPlus_ERR_NOT_REGISTED;
  }
  //MASK_ADC_INT;
  AP_ADCC->intr_mask = 0x1ff;
  
  NVIC_DisableIRQ((IRQn_Type)ADCC_IRQ);
  
  //DISABLE_ADC;
  AP_PCRM->ANA_CTL &= ~BIT(3);

  //ADC_CLOCK_DISABLE;
  AP_PCRM->CLKHF_CTL1 &= ~BIT(13);
  
  for(i =0; i< ADC_CH_NUM; i++){
    if(mAdc_Ctx.evt_handler[i]){
      disable_analog_pin((adc_CH_t)i);
    }
  }
  
  clk_gate_disable(MOD_ADCC);//disable I2C clk gated
  clear_adcc_cfg();
  //enableSleep();
  hal_pwrmgr_unlock(MOD_ADCC);
	return PPlus_SUCCESS;
}



/**************************************************************************************
 * @fn          hal_adc_value
 *
 * @brief       This function process for get adc value
 *
 * input parameters
 *
 * @param       ADC_CH_e adc_pin: adc pin select;ADC_CH0~ADC_CH7 and ADC_CH_VOICE
 *
 * output parameters
 *
 * @param       None.
 *
 * @return      ADC value
 **************************************************************************************/
float hal_adc_value(uint16_t* buf, uint8_t size, bool high_resol, bool diff_mode)
{
  uint8_t i;
  int adc_sum = 0;
  float result = 0.0;
  for (i = 0; i < size; i++) {
    int value;
    if(diff_mode == TRUE){
      value = (buf[i] & 0x0800) ? 0 - (int)(buf[i]&0x7ff) : (int)(buf[i]&0x7ff);
    }
    else{
      value = (int)(buf[i]&0xfff);
    }
    adc_sum += value;
  }

  result = ((float)adc_sum)/size;

  result = (diff_mode) ? (result / 2048 -1) : (result /4096);

  if(high_resol == FALSE)
    result = result * 4;  

  return result;
}

static void hal_adc_load_calibration_value(void)
{
    if(adc_cal_read_flag==FALSE)
    {
        adc_cal_read_flag = TRUE;
        adc_cal_negtive = read_reg(0x11001000)&0x0fff;
        adc_cal_postive = (read_reg(0x11001000)>>16)&0x0fff;
         
        LOG("AD_CAL[%x %x]\n",adc_cal_negtive,adc_cal_postive);
    }
   
}

float hal_adc_value_cal(adc_CH_t ch,uint16_t* buf, uint8_t size, bool high_resol, bool diff_mode)
{
    uint8_t i;
    int adc_sum = 0;
    float result = 0.0;
    for (i = 0; i < size; i++) {
        adc_sum += (buf[i]&0xfff);
    }
  
    hal_adc_load_calibration_value();
  
    result = ((float)adc_sum)/size;
  
    if((adc_cal_postive!=0xfff)&&(adc_cal_negtive!=0xfff)){
        float delta = ((int)(adc_cal_postive-adc_cal_negtive))/2.0;
        if(ch&0x01)
        {
            result = (diff_mode) ? ((result-2048-delta)*2/(adc_cal_postive+adc_cal_negtive)) 
            : ((result+delta) /(adc_cal_postive+adc_cal_negtive));
        }
        else
        {
            result = (diff_mode) ? ((result-2048-delta)*2/(adc_cal_postive+adc_cal_negtive)) 
            : ((result-delta) /(adc_cal_postive+adc_cal_negtive));
        }
        
    }else{
        result = (diff_mode) ? (result / 2048 -1) : (result /4096);
    } 

    if(high_resol == FALSE)
        result = result * 4;  

    return result;
}




