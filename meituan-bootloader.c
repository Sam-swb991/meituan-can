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
 * @file    MKE06Z4_bootloader-mt.c
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
#include "fsl_flash.h"
#define APP_ADDR  0x6000
#define APP_SIZE  0x7000
#define BK_APP_ADDR  0xD000
#define PACKAGE_SIZE 128
#define BOOT_VER  0x100
#define HW_VER 0x100
#define APPSENDCANID 0x106
#define MSCAN_CLK_FREQ CLOCK_GetFreq(kCLOCK_BusClk)
#define FLASH_CLOCK CLOCK_GetFreq(kCLOCK_FlashClk)
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
MYFLASH myflash;
uint8_t package_data[136];
uint8_t bk_data[512];
static flash_config_t s_flashDriver;
mscan_frame_t txFrame, rxFrame;
uint8_t send_data[8]={0};
uint8_t recv_data[8]={0};
uint8_t SN_write_cnt=0;
uint8_t A1_flag=0,A2_flag=0;
uint8_t A1_data[11]={0};
uint8_t A2_num =0;
uint32_t FIRMWARE_SIZE;
uint16_t package_num=0;
uint16_t current_NUM =0;
uint8_t current_write_num =0;
uint8_t package_size=0;
uint8_t A2_package_num;
uint8_t last_A2_package_size;
uint8_t last_data_cnt=0;
uint8_t operate_flag=0;
uint8_t Jump_flag=0;
void clearSNFlag();
void clearOTAFlag();
void clearJumpBootFlag();
void can_send_msg(uint32_t ID,uint8_t *data ,uint8_t len);
void analyze_One_data(uint8_t data[8]);
const uint16_t CrcCcittTable[256] = {
0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};
uint16_t crc=0;
uint16_t GetCRCCode(const uint8_t *pBuf, uint32_t iLen) {
	uint32_t i;
	uint16_t j;
	for( i=0; i<iLen; i++ )
	{
		j = (crc >> 8) ^ pBuf[i];
		crc = (crc << 8) ^ CrcCcittTable[j];
	}
	return crc;
}
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
/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */

/*
 * @brief   Application entry point.
 */
void JumpToImage(uint32_t addr)
{
    uint32_t *vectorTable = (uint32_t*)addr;
    uint32_t sp = vectorTable[0];
    uint32_t pc = vectorTable[1];

    typedef void(*app_entry_t)(void);

    uint32_t s_stackPointer = 0;
    uint32_t s_applicationEntry = 0;
    app_entry_t s_application = 0;

    s_stackPointer = sp;
    s_applicationEntry = pc;
    s_application = (app_entry_t)s_applicationEntry;

    // Change MSP and PSP
    __set_MSP(s_stackPointer);
    __set_PSP(s_stackPointer);

    SCB->VTOR = addr;

    // Jump to application
    s_application();

    // Should never reach here.
    __NOP();
}
void copy_bk_to_image()
{

	FLASH_Erase(&s_flashDriver, (uint32_t)APP_ADDR, APP_SIZE, kFLASH_ApiEraseKey);

	FLASH_Program(&s_flashDriver, APP_ADDR, (uint32_t *)BK_APP_ADDR, APP_SIZE);

}
void copy_app_to_bk()
{
	FLASH_Erase(&s_flashDriver, (uint32_t)BK_APP_ADDR, APP_SIZE, kFLASH_ApiEraseKey);
	FLASH_Program(&s_flashDriver, BK_APP_ADDR, (uint32_t *)APP_ADDR, APP_SIZE);
}
void Erase_FlASH()
{
	FLASH_Erase(&s_flashDriver, (uint32_t)0x1FE00, sizeof(myflash), kFLASH_ApiEraseKey);
}
void Write_FLASH()
{
	FLASH_Program(&s_flashDriver, 0x1FE00, (uint32_t *)&myflash, sizeof(myflash));
}
uint8_t WritePackage(uint16_t num,uint8_t *data)
{
	//FLASH_Erase(&s_flashDriver, APP_ADDR+(num-1)*PACKAGE_SIZE, PACKAGE_SIZE, kFLASH_ApiEraseKey);
	return FLASH_Program(&s_flashDriver, APP_ADDR+(num-1)*PACKAGE_SIZE, (uint32_t *)data, PACKAGE_SIZE);
}
void Read_FLASH()
{
	memset(&myflash,0,sizeof(myflash));
	memcpy(&myflash,(uint8_t *)0x1FE00,sizeof(myflash));
}
uint8_t flash_init()
{
	memset(&s_flashDriver, 0, sizeof(flash_config_t));
	memset(&myflash,0,sizeof(myflash));

	FLASH_SetProperty(&s_flashDriver, kFLASH_PropertyFlashClockFrequency, FLASH_CLOCK);

	return FLASH_Init(&s_flashDriver);
}
void clear_send_data()
{
	memset(send_data,0x55,8);
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
void analyze_data(uint8_t data[8])
{
	uint8_t send_data[8]={0};
	uint8_t ret;
	if(data[0] >= 0x10&&data[0]<=0x1F)
	{
		if(data[2] == 0xA1)
		{
			A1_flag =1;
			memcpy(A1_data,&data[2],6);
			send_data[0] = 0x30;
			send_data[1] = 0x08;
			send_data[2] = 0x14;
			can_send_msg(0x106,send_data,3);
		}
		else if(data[2] == 0xA2)
		{
			memset(package_data,1,136);
			A2_flag = 1;

			package_size = data[1]-6;
			last_A2_package_size = package_size%7;
			if(last_A2_package_size!=0)
				A2_package_num=package_size/7+1;
			current_NUM = data[3]<<8|data[4];
			memcpy(package_data,&data[5],3);
			send_data[0] = 0x30;
			send_data[1] = 0x00;
			send_data[2] = 0x01;
			current_write_num = 3;
			can_send_msg(0x106,send_data,3);
		}
	}
	else if (data[0]>=0x20&&data[0]<=0x2F)
	{
		if(A1_flag&&data[0]==0x21)
		{
			memcpy(A1_data+6,&data[1],5);
			FIRMWARE_SIZE = A1_data[7]<<24|A1_data[8]<<16|A1_data[9]<<8|A1_data[10];
			send_data[0] = 0x07;
			send_data[1] = 0xE1;
			if(FIRMWARE_SIZE >APP_SIZE)
			{
				send_data[2] = 0x02; //too large
			}
			else
				send_data[2] = 0x00;
			package_num = FIRMWARE_SIZE/PACKAGE_SIZE;
			if(FIRMWARE_SIZE%PACKAGE_SIZE!=0)
			{
				package_num++;
			}
			send_data[3] = PACKAGE_SIZE>>8&0xFF;
			send_data[4] = PACKAGE_SIZE&0xFF;
			send_data[5] = package_num>>8&0xFF;
			send_data[6] = package_num&0xFF;
			send_data[7] = 0x02;
			can_send_msg(0x106,send_data,8);
			A1_flag = 0;
			FLASH_Erase(&s_flashDriver, (uint32_t)APP_ADDR, APP_SIZE, kFLASH_ApiEraseKey);
		}
		else if(A2_flag)
		{
			A2_num = data[0]&0x0F;
			memcpy(&package_data[current_write_num],&data[1],7);
			current_write_num+=7;
			//can_send_msg(0x106,&current_write_num,1);
			//can_send_msg(0x106,&A2_num,1);
			if(current_write_num>=package_size+3)
			{
				current_write_num = 0;
				A2_flag=0;
				send_data[0] = 0x04;
				send_data[1] = 0xE2;
				send_data[2] = current_NUM>>8&0xFF;
				send_data[3] = current_NUM&0xFF;
				ret = WritePackage(current_NUM,package_data);
				if(ret ==0)
					send_data[4] = 0x00;
				else
					send_data[4] = 0x01;

				can_send_msg(0x106,send_data,5);
				crc = GetCRCCode(package_data,package_size+3);
				last_data_cnt = 0;
			}
		}
	}
	else if(data[0]<0x08)
	{
		analyze_One_data(data);
	}
}
void analyze_One_data(uint8_t data[8])
{
	uint8_t send_buf[8]={0};
	uint16_t transCRC;
	if(data[0]<0x08)
	{
		switch(data[1])
		{
			case 0x10:
			{
				if(data[2]==0x01)
				{
					send_buf[0] = 0x02;
					send_buf[1] = 0x50;
					send_buf[2] = 0x01;
					can_send_msg(APPSENDCANID,send_buf,3);
					Jump_flag=1;
					clearOTAFlag();
				}
				else if(data[2] == 0x02)
				{
					send_buf[0] = 0x02;
					send_buf[1] = 0x50;
					send_buf[2] = 0x02;
					can_send_msg(APPSENDCANID,send_buf,3);
				}
			}break;
			case 0x11:
			{
				send_buf[0] = 0x01;
				send_buf[1] = 0x51;
				can_send_msg(APPSENDCANID,send_buf,2);
				NVIC_SystemReset();
			}break;
			case 0xA3:
			{
				if(data[2] == 0x01)
				{
					transCRC = data[4]<<8|data[3];
					if(crc == transCRC)
					{
						send_buf[0] = 0x02;
						send_buf[1] = 0xE3;
						send_buf[2] = 0x00;
						can_send_msg(0x106,send_buf,3);
						clearOTAFlag();
						copy_app_to_bk();
						Jump_flag=1;
					}
					else
					{
						//send_buf[0]=crc>>8&0xFF;
						//send_buf[1]=crc&0xFF;
						//can_send_msg(0x106,send_buf,2);
						send_buf[0] = 0x02;
						send_buf[1] = 0xE3;
						send_buf[2] = 0x01;
						can_send_msg(0x106,send_buf,3);
						clearOTAFlag();
						copy_bk_to_image();
						Jump_flag=1;
					}
				}
				else if(data[2]==0x02)
				{
					send_buf[0] = 0x02;
					send_buf[1] = 0xE3;
					send_buf[2] = 0x02;
					can_send_msg(0x106,send_buf,3);
					clearOTAFlag();
					copy_bk_to_image();
					Jump_flag=1;
				}
			}
			case 0xA4:
			{
				send_buf[0] = 0x02;
				send_buf[1] = 0xE4;
				send_buf[2] = 0x00;
				can_send_msg(APPSENDCANID,send_buf,3);
			}break;
		}
	}
}

void MSCAN_1_IRQHandler(void)
{
	if (MSCAN_GetRxBufferFullFlag(MSCAN))
	{
	    MSCAN_ReadRxMb(MSCAN, &rxFrame);
	    MSCAN_ClearRxBufferFullFlag(MSCAN);
	    if(myflash.SN_flag == 1)
	    {
	    	if(SN_write_cnt ==0)
	    	{
	    		memcpy(myflash.SN_H,rxFrame.DSR,8);
	    		SN_write_cnt ++;
	    	}
	    	else if(SN_write_cnt == 1)
	    	{
	    		memcpy(myflash.SN_L,rxFrame.DSR,8);
	    		SN_write_cnt =0;
	    		clearSNFlag();
	    		NVIC_SystemReset();
	    	}
	    }
	    else if(myflash.ota_flag == 1||myflash.jumpappcnt>3)
	    {
	    	if(rxFrame.ID_Type.ID == 0x06&&rxFrame.format==kMSCAN_FrameFormatStandard)
	    	{
	    		operate_flag=1;
	    		analyze_data(rxFrame.DSR);
	    	}
	    }
	    else
	    {
	    	if(rxFrame.ID_Type.ID == 0x06&&rxFrame.format==kMSCAN_FrameFormatStandard)
	    	{
	    		operate_flag=1;
	    		analyze_One_data(rxFrame.DSR);
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
void clearOTAFlag()
{
	myflash.ota_flag = 0;
	Erase_FlASH();
	Write_FLASH();
}
void clearSNFlag()
{
	myflash.SN_flag = 0;
	Erase_FlASH();
	Write_FLASH();
}
void clearJumpBootFlag()
{
	myflash.jump_boot_flag = 0;
	Erase_FlASH();
	Write_FLASH();
}
void init_flash()
{
	memset(&myflash,0,sizeof(myflash));
	myflash.boot_ver[0] = BOOT_VER>>8&0xFF;
	myflash.boot_ver[1] = BOOT_VER&0xFF;
	myflash.HW_ver[0] = HW_VER>>8&0xFF;
	myflash.HW_ver[1] = HW_VER&0xFF;
	Erase_FlASH();
	Write_FLASH();
}
int main(void) {

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
    flash_init();
    Read_FLASH();
    if(myflash.jumpappcnt==0xFFFFFFFF)
    {
    	init_flash();

    }

    if(myflash.jump_boot_flag==1)
    {
    	SysTick_Config(SystemCoreClock / 1000U);
        SysTick_DelayTicks(3000);
        SysTick->CTRL &=0xFFF8;
        clearJumpBootFlag();
        if(operate_flag==0)
        {
        	JumpToImage(APP_ADDR);
        }
    }
    if(myflash.ota_flag==0&&myflash.SN_flag == 0)
    {
    	if(operate_flag==0)
    		JumpToImage(APP_ADDR);
    }
    if(myflash.SN_flag == 1)
    {

    }
    MSCAN_init();
    if(myflash.ota_flag == 1)
    {
    	SysTick_Config(SystemCoreClock / 1000U);
    	SysTick_DelayTicks(2000);
    	SysTick->CTRL &=0xFFF8;
    	if(operate_flag==0)
    	{
    		copy_bk_to_image();
			clearOTAFlag();
    	    JumpToImage(APP_ADDR);
    	}
    }
    /* Force the counter to be placed into memory. */
    volatile static int i = 0 ;
    /* Enter an infinite loop, just incrementing a counter. */
    while(1) {
        i++ ;
        if(Jump_flag)
        	JumpToImage(APP_ADDR);
        /* 'Dummy' NOP to allow source level single stepping of
            tight while() loop */
        __asm volatile ("nop");
    }
    return 0 ;
}
