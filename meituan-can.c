/*
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    MKE06Z4_Project.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKE06Z4.h"
#include "fsl_debug_console.h"
#include "fsl_mscan.h"
#include "fsl_adc.h"
#include "fsl_ftm.h"
#include "fsl_flash.h"
#include "fsl_tpm.h"
#include "fsl_gpio.h"
#define MSCAN_CLK_FREQ CLOCK_GetFreq(kCLOCK_BusClk)
#define TPM_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_TimerClk)
#define TPM_TIMER_SOURCE_CLOCK (CLOCK_GetFreq(kCLOCK_TimerClk) / 128)
#define FLASH_CLOCK CLOCK_GetFreq(kCLOCK_FlashClk)
#define APPSENDCANID  0x106
#define MOTORA_MAX  0x18
void StopMotoraRun();
void lock();
void unlock();
void can_send_msg(uint32_t ID,uint8_t *data,uint8_t len);
void SysTick_DelayTicks(uint32_t n);
void set_lock_state();
uint32_t get_adc_value();
void setOTAFlag();
void setSNFlag();
void setJumpBootFlag();
void Write_FLASH();
void Erase_FlASH();
void setFlash();
typedef struct {
	uint8_t HW_ver[4];
	uint32_t ota_flag;
	uint8_t SN_H[8];
	uint8_t SN_L[8];
	uint32_t SN_flag;
	uint32_t jumpappcnt;
	uint8_t boot_ver[4];
	uint16_t perroid_2A0;
	uint16_t perroid_2A1;
	uint16_t perroid_2A2;
	uint16_t perroid_2A3;
	uint32_t jump_boot_flag;
}MYFLASH;
typedef enum {
	UNLOCK,
	LOCK,
	ERROR,
	RUNNING,
	UNLOCK_PRE,
	LOCK_PRE,
	ERROR_TO_UNLOCK,
	ERROR_TO_LOCK
}LOCK_STATE;
typedef enum{
	PLACE,
	PLACE_NO,
	PLACE_ERROR
}HAT_STATE;
typedef enum{
	NO_ERROR,
	MOTORA_ERROR,
	UNLOCK_ERROR,
	LOCK_ERROR=4,
	READID_ERROR=8,
	CAN_TRANS_ERROR=16
}ERROR_STATE;
typedef enum{
	UNLOCKCTRL,
	LOCKCTRL,
	LOCKSTATECHECKCTRLYES,
	LOCKSTATECHECKCTRLNO,
	JUMPTOAPP,
	JUMPTOBOOT,
	SYSTERMRESET,
	TRANSCTRLYES,
	TRANSCTRLNO,
	BROADCASTPERIOD,
	WRITESN,
	OTASTART,
	DEFAULT,
	DEFAULT_1
}FUNC_STATE;
/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */

/*
 * @brief   Application entry point.
 */
static flash_config_t s_flashDriver;
adc_channel_config_t adcChannelConfigStruct;
mscan_handle_t mscanHandle;
mscan_mb_transfer_t txXfer, rxXfer;
mscan_frame_t txFrame, rxFrame;
uint8_t send_data[8]={0};
uint8_t recv_data[8]={0};
uint8_t lock_state;
uint8_t lock_error=0;
uint8_t unlock_error=0;
static uint8_t buf[8]={0};
uint8_t buf_2A3[8] = {0};
uint8_t interrupt_flag=0;
uint8_t tpm1_timer_cnt=0;
uint16_t ftm2_timer_2A0_cnt =0;
uint16_t ftm2_timer_2A1_cnt =0;
uint16_t ftm2_timer_2A2_cnt =0;
uint16_t ftm2_timer_2A3_cnt =0;
uint16_t ftm2_timer_can_error_cnt =0;
uint16_t ftm2_2A0_time=50;
uint16_t ftm2_2A1_time=10000;
uint16_t ftm2_2A2_time=10000;
uint16_t ftm2_2A3_time=10000;
uint16_t check_can_time=10000;
uint8_t ftm2_2A0_flag=1,ftm2_2A1_flag=1,ftm2_2A2_flag=1,ftm2_2A3_flag=1;
uint8_t ftm2_flag=0;
uint16_t SW_ver = 0x1001;
uint8_t A1_flag=0,E2_flag=0;
uint8_t E2_data[18]={0};
uint8_t check_can_error_flag=0;
uint8_t trans_can_ok_flag = 0;
uint8_t motoraRunTime=0;
MYFLASH myflash;
/***************************************************************************
 *
 *
 */
uint8_t read_state=0;
uint8_t read_data[128]={0};
static uint8_t ID_data[4]={0};
void delay_time(uint32_t t)
{
	while(t--)
	{
		__asm volatile ("nop");
	}
}
void clearHatID()
{
	uint8_t i;
	for(i=0;i<4;i++)
	{
		buf[3+i]= 0;
	}
}
void setHatID(uint8_t *data)
{
	uint8_t i;

	for(i=0;i<4;i++)
	{
		buf[3+i]= data[i];
	}
}
void setRunningState(uint8_t state)
{
	buf[0] = state;
}
uint8_t getRunningState()
{
	return buf[0];
}
void setLockState(uint8_t state)
{
	buf[1] &=0xF0;
	buf[1] |=(state&0x0F);
}
uint8_t getLockState()
{
	return buf[1]&0x0F;
}
void setHatState(uint8_t state)
{
	buf[1] &=0x0F;
	buf[1] |=state<<4;
}
uint8_t getHatState()
{
	return (buf[1]>>4)&0x0F;
}
void setErrorCode(uint8_t code)
{
	buf[2] |= code;
	if(buf[2]==NO_ERROR)
	{
		setRunningState(0);
	}
	else if(code == LOCK_ERROR||code == UNLOCK_ERROR)
	{
		setRunningState(1);
	}
}
void clearErrorCode(uint8_t code)
{
	buf[2] &= ~code;
	if(buf[2] == NO_ERROR)
	{
		setRunningState(0);
	}
}
void readdata(uint8_t* analyze_data)
{
	uint8_t cnt=0;
	uint8_t cnt_1=0;
	read_state =1;
	while(GPIO_PinRead(BOARD_INITPINS_RFID_IN_GPIO_PORT,BOARD_INITPINS_RFID_IN_PIN)==1);
	while(GPIO_PinRead(BOARD_INITPINS_RFID_IN_GPIO_PORT,BOARD_INITPINS_RFID_IN_PIN)==0);
	do{
		if(ftm2_flag==0)
			SysTick_DelayTicks(30);
		else
			ftm2_flag=0;
		analyze_data[cnt] = GPIO_PinRead(BOARD_INITPINS_RFID_IN_GPIO_PORT,BOARD_INITPINS_RFID_IN_PIN);
		if(analyze_data[cnt]==0)
		{
			while(GPIO_PinRead(BOARD_INITPINS_RFID_IN_GPIO_PORT,BOARD_INITPINS_RFID_IN_PIN)==0);
		}
		else if(analyze_data[cnt]==1)
		{
			while(GPIO_PinRead(BOARD_INITPINS_RFID_IN_GPIO_PORT,BOARD_INITPINS_RFID_IN_PIN)==1)
			{
				SysTick_DelayTicks(5);
				cnt_1++;
				if(cnt_1>10)
				{
					break;
				}
			}
			if(cnt_1>10)
			{
				break;
			}
			else
				cnt_1 =0;
		}
		cnt++;
	}
	while(cnt<=127);
	read_state =0;
}
uint8_t check_if_card_exist()
{
	uint8_t cnt =0,bit_cnt=0;
	do{
		if(GPIO_PinRead(BOARD_INITPINS_RFID_IN_GPIO_PORT,BOARD_INITPINS_RFID_IN_PIN) == 1)
			bit_cnt++;
		cnt++;
		SysTick_DelayTicks(26);
	}
	while(cnt<=20);
	if(bit_cnt>16)
		return 0;
	else
		return 1;
}
uint8_t  analyzer_IDdata(uint8_t *analyze_data)
{
	uint8_t i;
	uint8_t cnt=0;
	for(i=0;i<128;i++)
	{
		if(analyze_data[i]==1)
		{
			cnt++;
		}
		else
			cnt =0;
		if(cnt==9)
		{
			while(analyze_data[i]==1)
			{
				i++;
			}
			break;
		}

	}
	return i;
}
void print_anaData(uint8_t *data)
{
	uint8_t i;
	for(i=0;i<128;i+=8)
	{
		can_send_msg(0x6,&data[i],8);
	}
}
void print_bitdata()
{
		uint16_t i;
	for(i=0;i<256;i+=8)
	{
		//can_send_msg(0x8,&bitdata[i],8);
	}
}
uint8_t check_data(uint8_t *data)
{
	uint8_t ret=1,i;
	for(i=0;i<50;i+=5)
	{
		//can_send_msg(0x5,data+i,5);
		if(*(data+i+4) !=(*(data+i)^*(data+i+1)^*(data+i+2)^*(data+i+3)))
		{
			ret =0;
			break;
		}
	}

	for(i=0;i<4;i++)
	{
		if(*(data+50+i) !=(*(data+i)^*(data+5+i)^*(data+10+i)^*(data+15+i)^*(data+20+i)
											^*(data+25+i)^*(data+30+i)^*(data+35+i)^*(data+40+i)^*(data+45+i)))
			ret=0;
	}
	if(*(data+54)!=0)
	{
		ret = 0;
	}
	return ret;
}
void get_id_data(uint8_t *data)
{
	uint8_t i;
	for(i=0;i<4;i++)
	{
		ID_data[i] = *(data+10+i*10)<<7|*(data+10+i*10+1)<<6|*(data+10+i*10+2)<<5|*(data+10+i*10+3)<<4
									|*(data+10+i*10+5)<<3|*(data+10+i*10+6)<<2|*(data+10+i*10+7)<<1|*(data+10+i*10+8);
	}
}
uint8_t read_id()
{

	memset(read_data,1,128);
	int i;
	//uint8_t buf;
	uint8_t ret;
	uint8_t cnt = 5;
	uint8_t result=0;
	while(cnt--)
	{

		if(check_if_card_exist())
		{
			readdata(read_data);
			//print_anaData(read_data);
			i = analyzer_IDdata(read_data);
			//can_send_msg(0xEE,(uint8_t *)&i,4);
			if(i==128)
			{
				result=0;
				continue;
			}
			else
			{
				ret = check_data(&read_data[i]);
				if(ret!=0)
				{
					get_id_data(&read_data[i]);
					setHatState(PLACE);
					setHatID(ID_data);
					result = 1;
					break;

				}
				else
				{
					result = 2;
					continue;
				}
			}

		}
		else
			result = 0;

	}
	if(result ==0)
	{
		setHatState(PLACE_NO);
		clearHatID();
	}
	return result;
}
/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief MSCAN Call Back function
 */

volatile uint32_t g_systickCounter;
void SysTick_Handler(void)
{
    if (g_systickCounter != 0U)
    {
        g_systickCounter--;
    }
}

void SysTick_DelayTicks(uint32_t n)
{
    g_systickCounter = n;
    while (g_systickCounter != 0U)
    {
    }
}
void jump_to_boot()
{
	NVIC_SystemReset();
}
void clear_send_data()
{
	memset(send_data,0x55,8);
}
void send_negative_msg(uint8_t error,uint8_t cmd,uint8_t *data)
{
	data[0] = 0x03;
	data[1] = 0x7F;
	data[2] = cmd;
	data[3] = error;
	can_send_msg(APPSENDCANID,data,4);
}
void can_send_msg(uint32_t ID,uint8_t *data ,uint8_t len)
{
	txFrame.ID_Type.ID = ID;
	txFrame.format     = kMSCAN_FrameFormatStandard;
	txFrame.type       = kMSCAN_FrameTypeData;
	txFrame.DLR        = 8;
	clear_send_data();
	memcpy(send_data,data,len);
	//memcpy(txFrame.DSR,send_data,8);

	txFrame.dataByte0=send_data[0];
	txFrame.dataByte1=send_data[1];
	txFrame.dataByte2=send_data[2];
	txFrame.dataByte3=send_data[3];
	txFrame.dataByte4=send_data[4];
	txFrame.dataByte5=send_data[5];
	txFrame.dataByte6=send_data[6];
	txFrame.dataByte7=send_data[7];

	MSCAN_TransferSendBlocking(MSCAN,  &txFrame);
}
uint8_t analyzer_data(uint8_t data[8])
{
	uint8_t ret = DEFAULT;
	uint8_t send_buf[8];
	uint16_t can_ID=0;
	uint16_t package_num;
	uint16_t perroid;
	uint32_t package_size=0;
	if(data[0]<0x08)
	{
		switch(data[1])
		{
			case 0x01:{
				if(data[2]==0x00)
				{
					send_buf[0] = 0x02;
					send_buf[1] = 0x41;
					send_buf[2] = 0x00;
					can_send_msg(APPSENDCANID,send_buf,3);
					ret = LOCKSTATECHECKCTRLNO;
				}
				else if(data[2]==0x01)
				{
					send_buf[0] = 0x03;
					send_buf[1] = 0x7F;
					send_buf[2] = 0x01;
					send_buf[3] = 0x22;
					can_send_msg(APPSENDCANID,send_buf,4);
				}
				else
				{
					send_negative_msg(0x12,0x01,send_buf);
				}
			}break;
			case 0x10:{
				if(data[2]==0x01)
				{
					send_buf[0] = 0x02;
					send_buf[1] = 0x50;
					send_buf[2] = 0x01;
					can_send_msg(APPSENDCANID,send_buf,3);
					ret = JUMPTOAPP;
				}
				else if(data[2] == 0x02)
				{
					send_buf[0] = 0x02;
					send_buf[1] = 0x50;
					send_buf[2] = 0x02;
					can_send_msg(APPSENDCANID,send_buf,3);
					ret = JUMPTOBOOT;
				}
				else
					send_negative_msg(0x12,0x10,send_buf);

			}break;
			case 0x11:{
				send_buf[0] = 0x01;
				send_buf[1] = 0x51;
				can_send_msg(APPSENDCANID,send_buf,2);
				ret = SYSTERMRESET;
			}break;
			case 0x28:{
				if(data[2] == 0x00)
				{
					send_buf[0] = 0x02;
					send_buf[1] = 0x68;
					send_buf[2] = 0x00;
					can_send_msg(APPSENDCANID,send_buf,3);
					ret = TRANSCTRLNO;
				}
				else if(data[2] == 0x01)
				{
					send_buf[0] = 0x02;
					send_buf[1] = 0x68;
					send_buf[2] = 0x01;
					can_send_msg(APPSENDCANID,send_buf,3);
					ret = TRANSCTRLYES;
				}
				else
					send_negative_msg(0x12,0x11,send_buf);
			}break;
			case 0x29:{
				can_ID = data[2]<<8|data[3];
				perroid = data[4]<<8|data[5];
				if(can_ID == 0x2A0)
				{
					if(perroid==0xFFFF)
					{
						ftm2_2A0_flag =0;
					}
					else
					{
						ftm2_2A0_time = perroid;
						myflash.perroid_2A0 = perroid;
						setFlash();
					}
				}
				else if(can_ID == 0x2A1)
				{
					if(perroid==0xFFFF)
					{
						ftm2_2A1_flag =0;
					}
					else
					{
						ftm2_2A1_time = perroid;
						myflash.perroid_2A1 = perroid;
						setFlash();

					}
				}
				else if(can_ID == 0x2A2)
				{
					if(perroid == 0xFFFF)
					{
						ftm2_2A2_flag =0;
					}
					else
					{
						ftm2_2A2_time = perroid;
						myflash.perroid_2A2 = perroid;
						setFlash();
					}
				}
				else if(can_ID == 0x2A3)
				{
					if(perroid == 0xFFFF)
					{
						ftm2_2A3_flag =0;
					}
					else
					{
						ftm2_2A3_time = perroid;
						myflash.perroid_2A3 = perroid;
						setFlash();
					}
				}
				else
				{
					send_negative_msg(0x12,0x29,send_buf);
					break;
				}
				send_buf[0] = 0x05;
				send_buf[1] = 0x69;
				send_buf[2] = data[2];
				send_buf[3] = data[3];
				send_buf[4] = data[4];
				send_buf[5] = data[5];
				can_send_msg(APPSENDCANID,send_buf,6);
			}break;
			case 0x85:{
				send_buf[0] = 0x02;
				send_buf[1] = 0xC5;
				send_buf[2] = data[2];
				can_send_msg(APPSENDCANID,send_buf,3);
				check_can_error_flag = data[2];
			}break;
			case 0xA4:{
				send_buf[0] = 0x02;
				send_buf[1] = 0xE4;
				send_buf[2] = 0x01;
				can_send_msg(APPSENDCANID,send_buf,3);
			}break;
			case 0xFF:
			{
				ret = WRITESN;
			}break;
			default:
			{
				send_negative_msg(0x11,data[1],send_buf);
			}break;

		}
	}
	else if(data[0] == 0x10)
	{
		switch(data[2])
		{
			case 0xA1:
			{
				send_buf[0] = 0x30;
				send_buf[1] = 0x08;
				send_buf[2] = 0x14;
				A1_flag=1;
				can_send_msg(APPSENDCANID,send_buf,3);
			}break;
			case 0x2E:
			{
				send_buf[0] = 0x30;
				send_buf[1] = 0x00;
				send_buf[2] = 0x14;
				E2_flag=1;
				memset(E2_data,0,18);
				memcpy(E2_data,&data[3],5);
				can_send_msg(APPSENDCANID,send_buf,3);
			}break;
			default:
			{
				send_negative_msg(0x11,data[2],send_buf);
			}break;
		}
	}
	else if(data[0]>0x20)
	{
		if(A1_flag)
		{
			package_size = data[1]<<24|data[2]<<16|data[3]<<8|data[4];
			send_buf[0] = 0x07;
			send_buf[1] = 0xE1;
			if(package_size >0x7000)
				send_buf[2]=0x02;
			else
				send_buf[2] = 0x00;
			send_buf[3] = 0x00;
			send_buf[4] = 0x80;
			package_num = package_size/128;
			if(package_size%128!=0)
			{
				package_num++;
			}
			send_buf[5] = (package_num>>8)&0xFF;
			send_buf[6] = package_num&0xFF;
			send_buf[7] = 0x01;
			can_send_msg(APPSENDCANID,send_buf,8);
			A1_flag =0;
			ret = OTASTART;
		}
		else if (E2_flag)
		{
			if(data[0]==0x21)
			{
				memcpy(E2_data+5,&data[1],7);
				send_buf[0] = 0xFF;
				can_send_msg(APPSENDCANID,send_buf,1);
			}
			else if(data[0]==0x22)
			{
				memcpy(E2_data+12,&data[1],6);
				E2_flag=0;
				memcpy(myflash.SN_H,&E2_data[2],8);
				memcpy(myflash.SN_L,&E2_data[10],8);
				SysTick->CTRL &=0xFFF8;
				Erase_FlASH();
				Write_FLASH();
				SysTick->CTRL |=0x07;
				send_buf[0] = 0x03;
				send_buf[1] = 0x6E;
				send_buf[2] = E2_data[0];
				send_buf[3] = E2_data[1];
				can_send_msg(APPSENDCANID,send_buf,4);
			}

		}
		else
			send_negative_msg(0x13,0x00,send_buf);
	}
	else
		send_negative_msg(0x13,data[1],send_buf);
	return ret;
}
uint8_t analyzer_broadcast_data(uint8_t data[8])
{
	trans_can_ok_flag =1;
	if(data[0] ==0x00)
	{
		if(getLockState() != UNLOCK&&lock_state != UNLOCK_PRE)
		{
			if(unlock_error ==0)
				return UNLOCKCTRL;
		}
	}
	else if(data[0] == 0x01)
	{
		if(getLockState() !=LOCK&&lock_state !=LOCK_PRE)
		{
			if(lock_error == 0)
			{
				if(getHatState() == PLACE)
					return LOCKCTRL;
			}
		}
	}
	return 2;
}
void FTM2_IRQHandler(void)
{

    /* Clear interrupt flag.*/
    FTM_ClearStatusFlags(FTM2, kFTM_TimeOverflowFlag);
    ftm2_timer_2A0_cnt ++;
    ftm2_timer_2A1_cnt ++;
    ftm2_timer_2A2_cnt ++;
    ftm2_timer_2A3_cnt ++;
    if(ftm2_timer_2A0_cnt>=ftm2_2A0_time)
    {
    	if(ftm2_2A0_flag)
    		can_send_msg(0x2A0,buf,8);
    	ftm2_timer_2A0_cnt = 0;
    	ftm2_flag=1;
    }
    if(ftm2_timer_2A1_cnt>=ftm2_2A1_time)
    {
    	if(ftm2_2A1_flag)
    		can_send_msg(0x2A1,myflash.SN_H,8);
    	ftm2_timer_2A1_cnt = 0;
    	ftm2_flag=1;
    }
    if(ftm2_timer_2A2_cnt>=ftm2_2A2_time)
    {
    	if(ftm2_2A2_flag)
    		can_send_msg(0x2A2,myflash.SN_L,8);
    	ftm2_timer_2A2_cnt = 0;
    	ftm2_flag=1;
    }
    if(ftm2_timer_2A3_cnt>=ftm2_2A3_time)
    {
    	if(ftm2_2A3_flag)
    		can_send_msg(0x2A3,buf_2A3,8);
    	ftm2_timer_2A3_cnt = 0;
    	ftm2_flag=1;
    }
    if(check_can_error_flag&&(ftm2_timer_can_error_cnt++>=check_can_time))
    {
    	if(trans_can_ok_flag==1)
    	{
    		clearErrorCode(CAN_TRANS_ERROR);
    		trans_can_ok_flag =0;
    	}
    	else
    	{
    		setErrorCode(CAN_TRANS_ERROR);
    	}
    	ftm2_timer_can_error_cnt = 0;
    }
    //interrupt_flag = 1;
    __DSB();
}
void check_motora_state()
{
	uint32_t adcvalue;
	adcvalue = get_adc_value();
	//can_send_msg(0xEE,(uint8_t *)&adcvalue,4);
	if(adcvalue >MOTORA_MAX)
	{
		if(lock_state ==UNLOCK_PRE)
		{
			unlock_error =1;
		}
		else if(lock_state == LOCK_PRE)
		{
			lock_error =1;
		}
		setErrorCode(MOTORA_ERROR);
		setRunningState(1);
		tpm1_timer_cnt = 0;
		TPM_DisableInterrupts(TPM1,kTPM_TimeOverflowInterruptEnable);
		DisableIRQ(TPM1_IRQn);
		TPM_StopTimer(TPM1);
		StopMotoraRun();

	}
	else
		clearErrorCode(MOTORA_ERROR);
}
uint8_t check_lock_state()
{
	uint8_t lock_s,unlock_s,ret=0;
	unlock_s = GPIO_PinRead(BOARD_INITPINS_PLB_1_GPIO_PORT, BOARD_INITPINS_PLB_1_PIN);
	lock_s = GPIO_PinRead(BOARD_INITPINS_PLA_1_GPIO_PORT, BOARD_INITPINS_PLA_1_PIN);
	if(unlock_s ==0&&lock_s==0)
		ret=0;     //running
	else if(unlock_s==1&&lock_s==0)
		ret =1;    //unlock_state
	else if(unlock_s ==0&&lock_s ==1)
		ret=2;     //lock_state
	else if(unlock_s ==1&&lock_s ==1)
		ret=3;  //error
	return ret;
}
void TPM1_IRQHandler(void)
{
	uint32_t ret,ret1=0;
	uint32_t adcvalue;
	TPM_ClearStatusFlags(TPM1, kTPM_TimeOverflowFlag);
	tpm1_timer_cnt++;

	//can_send_msg(0xEE,buf,8);
	if(tpm1_timer_cnt==5)
	{
		set_lock_state();
		check_motora_state();
	}
	else if(tpm1_timer_cnt==7)
	{
		ret1 = check_lock_state();
		if(ret1!=0&&ret1!=3)
		{
			setErrorCode(1<<ret1);
		}
		else if(ret1==3)
		{
			setErrorCode(UNLOCK_ERROR);
			setErrorCode(LOCK_ERROR);
		}
	}
	else if(tpm1_timer_cnt == 10)
	{
		set_lock_state();
		check_motora_state();
	}
	else if(tpm1_timer_cnt == 15)
	{
		set_lock_state();
		check_motora_state();
	}
	else if(tpm1_timer_cnt == 20)
	{
		set_lock_state();
		check_motora_state();
	}
	else if(tpm1_timer_cnt ==25)
	{
		set_lock_state();
		check_motora_state();
	}
	else if(tpm1_timer_cnt>=motoraRunTime)
	{
		adcvalue = get_adc_value();
		//can_send_msg(0x2A4,(uint8_t *)&adcvalue,4);
		TPM_DisableInterrupts(TPM1,kTPM_TimeOverflowInterruptEnable);
		DisableIRQ(TPM1_IRQn);
		TPM_StopTimer(TPM1);
		StopMotoraRun();

		if(lock_state ==UNLOCK_PRE)
		{
			if(adcvalue >MOTORA_MAX)
			{
				unlock_error =1;
				setErrorCode(MOTORA_ERROR);
				setRunningState(1);
				tpm1_timer_cnt = 0;
				return ;
			}
			else
			{
				clearErrorCode(MOTORA_ERROR);
			}
			ret = check_lock_state();
			if(ret==1)
			{
				clearErrorCode(UNLOCK_ERROR);
				lock_state = UNLOCK;
				lock_error=0;
				unlock_error=0;

			}
			else if(ret == 2)
			{
				unlock_error =1;
				tpm1_timer_cnt =0;
				setErrorCode(UNLOCK_ERROR);
				setErrorCode(LOCK_ERROR);
				return;
			}
			else if(ret == 0)
			{
				unlock_error =1;
				tpm1_timer_cnt =0;
				setErrorCode(UNLOCK_ERROR);
				clearErrorCode(LOCK_ERROR);
				return;
			}
			else if(ret == 3)
			{
				unlock_error =1;
				tpm1_timer_cnt =0;
				setErrorCode(LOCK_ERROR);
				clearErrorCode(UNLOCK_ERROR);
				return;
			}
		}
		else if(lock_state == LOCK_PRE)
		{
			if(adcvalue >MOTORA_MAX)
			{
				lock_error=1;
				setErrorCode(MOTORA_ERROR);
				setRunningState(1);
				tpm1_timer_cnt = 0;
				return ;
			}
			else
			{
				clearErrorCode(MOTORA_ERROR);
			}
			ret = check_lock_state();
			if(ret==2)
			{
				clearErrorCode(LOCK_ERROR);
				lock_state = LOCK;
				lock_error=0;
				unlock_error=0;

			}
			else if(ret ==1)
			{
				lock_error=1;
				setErrorCode(LOCK_ERROR);
				setErrorCode(UNLOCK_ERROR);
				tpm1_timer_cnt =0;
				return;
			}
			else if(ret  ==3)
			{
				lock_error=1;
				setErrorCode(UNLOCK_ERROR);
				clearErrorCode(LOCK_ERROR);
				tpm1_timer_cnt =0;
				return;
			}
			else if(ret ==0)
			{
				lock_error=1;
				setErrorCode(LOCK_ERROR);
				clearErrorCode(UNLOCK_ERROR);
				tpm1_timer_cnt =0;
				return;
			}
		}

		tpm1_timer_cnt =0;
	}
	__DSB();
}
void MSCAN_1_IRQHandler(void)
{
	uint8_t data[8]={0};
    /* If new data arrived. */
    if (MSCAN_GetRxBufferFullFlag(MSCAN))
    {
        MSCAN_ReadRxMb(MSCAN, &rxFrame);
        MSCAN_ClearRxBufferFullFlag(MSCAN);
        if(rxFrame.format==kMSCAN_FrameFormatStandard)
        {
        	if(rxFrame.ID_Type.ID==0x06)
        	{
        		if(rxFrame.DLR!=8)
        		{
        			if(rxFrame.dataByte0<0x08)
        				send_negative_msg(rxFrame.dataByte1,0x13,data);
        			else
        				send_negative_msg(rxFrame.dataByte2,0x13,data);
        		}
        		else
        		{
					switch(analyzer_data(rxFrame.DSR))
					{
						case JUMPTOAPP:break;
						case JUMPTOBOOT:setJumpBootFlag();
						case SYSTERMRESET:jump_to_boot();break;
						case TRANSCTRLYES:
						{
							FTM_EnableInterrupts(FTM2, kFTM_TimeOverflowInterruptEnable);
							EnableIRQ(FTM2_IRQn);
							FTM_StartTimer(FTM2, kFTM_SystemClock);
						}break;
						case TRANSCTRLNO:
						{
							FTM_DisableInterrupts(FTM2, kFTM_TimeOverflowInterruptEnable);
							DisableIRQ(FTM2_IRQn);
							FTM_StopTimer(FTM2);
						}break;
						case OTASTART:{
							setOTAFlag();
							jump_to_boot();
						}break;
						case WRITESN:
						{
							setSNFlag();
							jump_to_boot();
						}break;
						default:break;
					}
        		}
        	}
        	else if(rxFrame.ID_Type.ID == 0x206)
        	{
        		if(rxFrame.DLR!=8)
        		{
        		    send_negative_msg(0x00,0x13,data);
        		}
        		else
        		{
					switch(analyzer_broadcast_data(rxFrame.DSR))
					{
						case LOCKCTRL:{
							lock_state = LOCK_PRE;
							lock();
						}break;
						case UNLOCKCTRL:
						{
							lock_state = UNLOCK_PRE;
							unlock();
						}break;
						default:break;
					}
        		}
        	}

        }
    }
    SDK_ISR_EXIT_BARRIER;
}
void MSCAN_init()
{
	mscan_config_t mscanConfig;
	MSCAN_GetDefaultConfig(&mscanConfig);
	mscanConfig.baudRate = 500000U;
	mscanConfig.clkSrc = kMSCAN_ClkSrcBus;
	mscanConfig.filterConfig.u32IDAR0 = 0;
	mscanConfig.filterConfig.u32IDAR1 = 0;
	mscanConfig.filterConfig.u32IDMR0 = 0xffffffff;
	mscanConfig.filterConfig.u32IDMR1 = 0xffffffff;
	MSCAN_Init(MSCAN, &mscanConfig, MSCAN_CLK_FREQ);
	MSCAN_EnableRxInterrupts(MSCAN, kMSCAN_RxFullInterruptEnable);
	EnableIRQ(MSCAN_1_IRQn);
}
void ADC_init()
{
	adc_config_t adcConfigStrcut;

	ADC_GetDefaultConfig(&adcConfigStrcut);
	ADC_Init(ADC, &adcConfigStrcut);
	ADC_EnableHardwareTrigger(ADC, false);

	    /* Configure the user channel and interrupt. */
	adcChannelConfigStruct.channelNumber                        = 5U;
	adcChannelConfigStruct.enableInterruptOnConversionCompleted = false;
	adcChannelConfigStruct.enableContinuousConversion           = false;

	    /* Enable the releated analog pins. */
	ADC_EnableAnalogInput(ADC, 1U << 5U, true);

}
uint32_t get_adc_value()
{
	ADC_SetChannelConfig(ADC, &adcChannelConfigStruct);
	while (!ADC_GetChannelStatusFlags(ADC));
	return ADC_GetChannelConversionValue(ADC);
}
void PWM_init()
{
	tpm_config_t tpmInfo;
	tpm_chnl_pwm_signal_param_t tpmParam;
	tpmParam.chnlNumber       = (tpm_chnl_t)0;
	tpmParam.level            = kTPM_LowTrue;
	tpmParam.dutyCyclePercent = 50U;

	TPM_GetDefaultConfig(&tpmInfo);
	TPM_Init(TPM0, &tpmInfo);

	TPM_SetupPwm(TPM0, &tpmParam, 1U, kTPM_CenterAlignedPwm, 125000U, TPM_SOURCE_CLOCK);

	TPM_StartTimer(TPM0, kTPM_SystemClock);
}
void TIMER_init()
{
	ftm_config_t ftmInfo;

	FTM_GetDefaultConfig(&ftmInfo);
	    /* Divide FTM clock by 4 */
	ftmInfo.prescale = kFTM_Prescale_Divide_128;
	    /* Initialize FTM module */
	FTM_Init(FTM2, &ftmInfo);
	FTM_SetTimerPeriod(FTM2, MSEC_TO_COUNT(1U,TPM_TIMER_SOURCE_CLOCK));
	FTM_EnableInterrupts(FTM2, kFTM_TimeOverflowInterruptEnable);
	EnableIRQ(FTM2_IRQn);
	FTM_StartTimer(FTM2, kFTM_SystemClock);
	//FTM_ClearStatusFlags(FTM2, kFTM_TimeOverflowFlag);
}
void MOTORA_timer_init()
{
	tpm_config_t tpmInfo;
	TPM_GetDefaultConfig(&tpmInfo);
	tpmInfo.prescale = kTPM_Prescale_Divide_128;
	TPM_Init(TPM1, &tpmInfo);
	TPM_SetTimerPeriod(TPM1, MSEC_TO_COUNT(100U, TPM_TIMER_SOURCE_CLOCK));

	//TPM_EnableInterrupts(TPM1, kTPM_TimeOverflowInterruptEnable);

	//EnableIRQ(TPM1_IRQn);

	//TPM_StartTimer(TPM1, kTPM_SystemClock);
}
uint8_t flash_init()
{
	memset(&s_flashDriver, 0, sizeof(flash_config_t));
	memset(&myflash,0,sizeof(myflash));

	FLASH_SetProperty(&s_flashDriver, kFLASH_PropertyFlashClockFrequency, FLASH_CLOCK);

	return FLASH_Init(&s_flashDriver);
}
void comb_2A3buf()
{
	memcpy(buf_2A3,myflash.HW_ver,2);
	memcpy(buf_2A3+2,(uint8_t *)&SW_ver,2);
	memcpy(buf_2A3+4,myflash.boot_ver,2);
	buf_2A3[6]=0x00;
	buf_2A3[7]=0x55;
}
void StopMotoraRun()
{
	GPIO_PinWrite(BOARD_INITPINS_PINA_GPIO_PORT, BOARD_INITPINS_PINA_PIN,0);
	GPIO_PinWrite(BOARD_INITPINS_PINB_GPIO_PORT, BOARD_INITPINS_PINB_PIN,0);
}
void unlock()
{
	//can_send_msg(0xEF,buf,8);
	tpm1_timer_cnt =0;
	GPIO_PinWrite(BOARD_INITPINS_PINB_GPIO_PORT, BOARD_INITPINS_PINB_PIN,1);
	GPIO_PinWrite(BOARD_INITPINS_PINA_GPIO_PORT, BOARD_INITPINS_PINA_PIN,0);
	motoraRunTime = 29;
	//lock_state = RUNNING;
	TPM_EnableInterrupts(TPM1, kTPM_TimeOverflowInterruptEnable);
	EnableIRQ(TPM1_IRQn);
	TPM_StartTimer(TPM1, kTPM_SystemClock);

}
void lock()
{
	//can_send_msg(0xEF,buf,8);
	tpm1_timer_cnt = 0;
	GPIO_PinWrite(BOARD_INITPINS_PINA_GPIO_PORT, BOARD_INITPINS_PINA_PIN,1);
	GPIO_PinWrite(BOARD_INITPINS_PINB_GPIO_PORT, BOARD_INITPINS_PINB_PIN,0);
	//lock_state = RUNNING;
	motoraRunTime = 29;
	TPM_EnableInterrupts(TPM1, kTPM_TimeOverflowInterruptEnable);
	EnableIRQ(TPM1_IRQn);
	TPM_StartTimer(TPM1, kTPM_SystemClock);
}
void set_lock_state()
{
	uint32_t unlockstate,lockstate;
	unlockstate = GPIO_PinRead(BOARD_INITPINS_PLB_1_GPIO_PORT, BOARD_INITPINS_PLB_1_PIN);
	lockstate = GPIO_PinRead(BOARD_INITPINS_PLA_1_GPIO_PORT, BOARD_INITPINS_PLA_1_PIN);
	if(unlockstate==1&&lockstate == 1)
	{
		if(lock_state!=UNLOCK_PRE&&lock_state !=LOCK_PRE)
			lock_state = ERROR;
		setLockState(ERROR);
		setRunningState(1);
	}
	else if(unlockstate == 1)
	{
		if(lock_state!=UNLOCK_PRE&&lock_state !=LOCK_PRE)
			lock_state = UNLOCK;
		setLockState(UNLOCK);
		//clearErrorCode(UNLOCK_ERROR);
	}
	else if(lockstate == 1)
	{
		if(lock_state!=UNLOCK_PRE&&lock_state !=LOCK_PRE)
			lock_state = LOCK;
		setLockState(LOCK);
		//clearErrorCode(LOCK_ERROR);
	}
	else if(lockstate == 0&&unlockstate ==0)
	{
		if(lock_state!=UNLOCK_PRE&&lock_state !=LOCK_PRE)
			lock_state = RUNNING;
		setLockState(RUNNING);
	}

}

void Erase_FlASH()
{
	FLASH_Erase(&s_flashDriver, (uint32_t)0x1FE00, sizeof(myflash), kFLASH_ApiEraseKey);
}
void Write_FLASH()
{
	FLASH_Program(&s_flashDriver, 0x1FE00, (uint32_t *)&myflash, sizeof(myflash));
}
void Read_FLASH()
{
	memset(&myflash,0,sizeof(myflash));
	memcpy(&myflash,(uint8_t *)0x1FE00,sizeof(myflash));
}
void setOTAFlag()
{
	SysTick->CTRL &=0xFFF8;
	myflash.ota_flag = 1;
	Erase_FlASH();
	Write_FLASH();
	SysTick->CTRL |=0x07;
}
void setSNFlag()
{
	SysTick->CTRL &=0xFFF8;
	myflash.SN_flag = 1;
	Erase_FlASH();
	Write_FLASH();
	SysTick->CTRL |=0x07;
}
void setFlash()
{
	SysTick->CTRL &=0xFFF8;
	Erase_FlASH();
	Write_FLASH();
	SysTick->CTRL |=0x07;
}
void setJumpBootFlag()
{
	SysTick->CTRL &=0xFFF8;
	myflash.jump_boot_flag =1;
	Erase_FlASH();
	Write_FLASH();
	SysTick->CTRL |=0x07;
}
void init_flash()
{
	myflash.jumpappcnt = 0;
	if(myflash.perroid_2A0==0)
	{
		ftm2_timer_2A0_cnt = ftm2_2A0_time;
		myflash.perroid_2A0 = ftm2_2A0_time;
	}
	else if(myflash.perroid_2A0 == 0xFFFF)
	{
		ftm2_2A0_flag =0;
	}
	else
	{
		ftm2_2A0_time = myflash.perroid_2A0;
		ftm2_timer_2A0_cnt = ftm2_2A0_time;
	}
	if(myflash.perroid_2A1==0)
	{
		ftm2_timer_2A1_cnt = ftm2_2A1_time;
		myflash.perroid_2A1 = ftm2_2A1_time;
	}
	else if(myflash.perroid_2A1==0xFFFF)
	{
		ftm2_2A1_flag =0;
	}
	else
	{
		ftm2_2A1_time = myflash.perroid_2A1;
		ftm2_timer_2A1_cnt = ftm2_2A1_time;
	}
	if(myflash.perroid_2A2==0)
	{
		ftm2_timer_2A2_cnt = ftm2_2A2_time;
		myflash.perroid_2A2 = ftm2_2A2_time;
	}
	else if(myflash.perroid_2A2==0xFFFF)
	{
		ftm2_2A2_flag =0;
	}
	else
	{
		ftm2_2A2_time = myflash.perroid_2A2;
		ftm2_timer_2A2_cnt = ftm2_2A2_time;
	}
	if(myflash.perroid_2A3==0)
	{
		ftm2_timer_2A3_cnt = ftm2_2A3_time;
		myflash.perroid_2A3 = ftm2_2A3_time;
	}
	else if(myflash.perroid_2A3==0xFFFF)
	{
		ftm2_2A3_flag =0;
	}
	else
	{
		ftm2_2A3_time = myflash.perroid_2A3;
		ftm2_timer_2A3_cnt = ftm2_2A3_time;
	}
}
int main(void) {
	uint8_t status;
    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
    status = flash_init();
    Read_FLASH();
    init_flash();
    SysTick_Config(SystemCoreClock / 100000U);
    NVIC_SetPriority(SysTick_IRQn, 0x01);
    PWM_init();
    MSCAN_init();
    //can_send_msg(0x06,&status,1);
    TIMER_init();
    ADC_init();
    MOTORA_timer_init();
    comb_2A3buf();
    set_lock_state();
    //GPIO_PinWrite(BOARD_INITPINS_PLC_1_GPIO_PORT, BOARD_INITPINS_PLC_1_PIN,0);
    //GPIO_PinWrite(BOARD_INITPINS_PINB_GPIO_PORT, BOARD_INITPINS_PINB_PIN,0);
    //GPIO_PinWrite(BOARD_INITPINS_PINA_GPIO_PORT, BOARD_INITPINS_PINA_PIN,1);
    while(1) {
    	read_id();
    	/*if(lock_state == UNLOCK_PRE)
    	{
    		unlock();
    	}
    	else if(lock_state == LOCK_PRE)
    	{
    		lock();
    	}*/
    	//can_send_msg(0x2A0,buf,8);
    	SysTick_DelayTicks(50);
    }
    return 0 ;
}
