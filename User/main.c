/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2020/04/30
 * Description        : Main program body.
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/
#include "debug.h"
#include "lcd_st7789.h"
#include "ov.h"

/*
 *@Note
  DVP²Ù×÷OV2640ÉãÏñÍ·ÊÓÆµÄ£Ê½ÔÚLCDÏÔÊ¾Í¼ÏñÀý³Ì£º
  DVP¡ª¡ªPIN£º
    D10¡ª¡ªPD6
    D11¡ª¡ªPD2
    D8¡ª¡ªPC10
    D9¡ª¡ªPC12
    DPWDN¡ª¡ªPC3
    DPCLK¡ª¡ªPA6
    D7¡ª¡ªPB9
    D6¡ª¡ªPB8
    D5¡ª¡ªPB6
    D4¡ª¡ªPC11
    D3¡ª¡ªPC9
    D2¡ª¡ªPC8
    D1¡ª¡ªPA10
    D0¡ª¡ªPA9
    RESET¡ª¡ªPC13
    SDA¡ª¡ªPB11
    HERF¡ª¡ªPA4
    SDCLK¡ª¡ªPB10
    SYNC¡ª¡ªPA5
  LCD¡ª¡ªPIN£º
    PD11¡ª¡ªFSMC_A16
    PD12¡ª¡ªFSMC_A17
    PD5 ¡ª¡ªFSMC_NEW
    PD4 ¡ª¡ªFSMC_NOE
    PA15¡ª¡ªLCDRST#
    PD14¡ª¡ªFSMC_D0
    PD15¡ª¡ªFSMC_D1
    PD0 ¡ª¡ªFSMC_D2
    PD1 ¡ª¡ªFSMC_D3
    PE7 ¡ª¡ªFSMC_D4
    PE8 ¡ª¡ªFSMC_D5
    PE9 ¡ª¡ªFSMC_D6
    PE10¡ª¡ªFSMC_D7
    PE11¡ª¡ªFSMC_D8
    PE12¡ª¡ªFSMC_D9
    PE13¡ª¡ªFSMC_D10
    PE14¡ª¡ªFSMC_D11
    PE15¡ª¡ªFSMC_D12
    PD8 ¡ª¡ªFSMC_D13
    PD9 ¡ª¡ªFSMC_D14
    PD10¡ª¡ªFSMC_D15
    PB14¡ª¡ªIO_BLCTR
    PA8 ¡ª¡ªIO_MISO_NC
    PB3 ¡ª¡ªIO_MOSI_SDA
    PB15¡ª¡ªIO_TKINT
    PC13¡ª¡ªIO_BUSY_NC
    PC0 ¡ª¡ªIO_TKRST#
    PB4 ¡ª¡ªIO_CLK
    ×¢£ºÊ¹ÓÃUART2(PA2)´®¿ÚÊä³ö£¬½«debug.hÖÐ #define DEBUG   DEBUG_UART2,
    UART1(PA9)±»DVPÕ¼ÓÃ
 */

/* Global define */
#define TRUE 1
#define FALSE 0

/* DVP Work Mode */
#define RGB565_MODE 0
/* DVP Work Mode Selection */
#define DVP_Work_Mode RGB565_MODE

UINT32 JPEG_DVPDMAaddr0 = 0x20005000;
UINT32 JPEG_DVPDMAaddr1 = 0x20005000 + OV2640_JPEG_WIDTH;

UINT32 RGB565_DVPDMAaddr0 = 0x2000A000;
UINT32 RGB565_DVPDMAaddr1 = 0x2000A000 + RGB565_COL_NUM * 2; // each byte(D9-D2) will take 2 bytes of RAM

volatile UINT32 frame_cnt = 0;
volatile UINT32 addr_cnt = 0;
volatile UINT32 href_cnt = 0;

void DVP_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*********************************************************************
 * @fn      LCD_Reset_GPIO_Init
 *
 * @brief   Init LCD reset GPIO.
 *
 * @return  none
 */
void LCD_Reset_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_SetBits(GPIOA, GPIO_Pin_15);
}

/*********************************************************************
 * @fn      DMA_SRAMLCD_Init
 *
 * @brief   Init SRAMLCD DMA
 *
 * @param   ddr: DVP data memory base addr.
 *
 * @return  none
 */
void DMA_SRAMLCD_Init(u32 ddr)
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
    DMA_DeInit(DMA2_Channel5);

    DMA_InitTypeDef DMA_InitStructure = {0};
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)LCD_DATA;
    DMA_InitStructure.DMA_MemoryBaseAddr = ddr;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = LCD_W;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Enable;
    DMA_Init(DMA2_Channel5, &DMA_InitStructure);
}

/*********************************************************************
 * @fn      DMA_SRAMLCD_Enable
 *
 * @brief   Enable SRAMLCD DMA transmission
 *
 * @return  none
 */
void DMA_SRAMLCD_Enable(void)
{
    DMA_Cmd(DMA2_Channel5, DISABLE);
    DMA_SetCurrDataCounter(DMA2_Channel5, LCD_W);
    DMA_Cmd(DMA2_Channel5, ENABLE);
}

/*********************************************************************
 * @fn      DVP_Init
 *
 * @brief   Init DVP
 *
 * @return  none
 */
void DVP_Init(void)
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DVP, ENABLE);
    DVP->CR0 &= ~RB_DVP_MSK_DAT_MOD;

    /* VSYNC¡¢HSYNC - High level active */
    DVP->CR0 |= RB_DVP_D10_MOD | RB_DVP_V_POLAR;
    DVP->CR1 &= ~((RB_DVP_ALL_CLR) | RB_DVP_RCV_CLR);
    DVP->ROW_NUM = RGB565_ROW_NUM; // rows
    DVP->COL_NUM = RGB565_COL_NUM; // cols
    DVP->HOFFCNT = (RGB565_COL_NUM - ROI_WIDTH) / 2;
    DVP->VST = (RGB565_ROW_NUM - ROI_HEIGTH) / 2;
    DVP->CAPCNT = ROI_WIDTH;
    DVP->VLINE = ROI_HEIGTH;
    DVP->CR1 |= RB_DVP_CROP;

    DVP->DMA_BUF0 = RGB565_DVPDMAaddr0; // DMA addr0
    DVP->DMA_BUF1 = RGB565_DVPDMAaddr1; // DMA addr1

    /* Set frame capture rate */
    DVP->CR1 &= ~RB_DVP_FCRC;
    DVP->CR1 |= DVP_RATE_100P; // 100%

    // Interrupt Enable
    DVP->IER |= RB_DVP_IE_STP_FRM;
    DVP->IER |= RB_DVP_IE_FIFO_OV;
    DVP->IER |= RB_DVP_IE_FRM_DONE;
    DVP->IER |= RB_DVP_IE_ROW_DONE;
    DVP->IER |= RB_DVP_IE_STR_FRM;

    NVIC_InitTypeDef NVIC_InitStructure = {0};
    NVIC_InitStructure.NVIC_IRQChannel = DVP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    DVP->CR1 |= RB_DVP_DMA_EN; // enable DMA
    DVP->CR0 |= RB_DVP_ENABLE; // enable DVP
}

u32 DVP_ROW_cnt = 0;

/*********************************************************************
 * @fn      DVP_IRQHandler
 *
 * @brief   This function handles DVP exception.
 *
 * @return  none
 */
void DVP_IRQHandler(void)
{

    if (DVP->IFR & RB_DVP_IF_ROW_DONE)
    {
        /* Write 0 clear 0 */
        DVP->IFR &= ~RB_DVP_IF_ROW_DONE; // clear Interrupt

        if (addr_cnt % 2) // buf0 done
        {
            addr_cnt++;
            // data shift
            for (u16 i = 0; i < RGB565_COL_NUM * 2; ++i)
            {
                *(u8 *)(RGB565_DVPDMAaddr0 + i) = (u8)(*(u16 *)(RGB565_DVPDMAaddr0 + i * 2) >> 2);
            }
            // Send DVP data to LCD
            DMA_Cmd(DMA2_Channel5, DISABLE);
            DMA_SetCurrDataCounter(DMA2_Channel5, LCD_W);
            DMA2_Channel5->MADDR = RGB565_DVPDMAaddr0;
            DMA_Cmd(DMA2_Channel5, ENABLE);
        }
        else // buf1 done
        {
            addr_cnt++;
            // data shift
            for (u16 i = 0; i < RGB565_COL_NUM * 2; ++i)
            {
                *(u8 *)(RGB565_DVPDMAaddr1 + i) = (u8)(*(u16 *)(RGB565_DVPDMAaddr1 + i * 2) >> 2);
            }
            // Send DVP data to LCD
            DMA_Cmd(DMA2_Channel5, DISABLE);
            DMA_SetCurrDataCounter(DMA2_Channel5, LCD_W);
            DMA2_Channel5->MADDR = RGB565_DVPDMAaddr1;
            DMA_Cmd(DMA2_Channel5, ENABLE);
        }
        href_cnt++;
    }

    if (DVP->IFR & RB_DVP_IF_FRM_DONE)
    {
        DVP->IFR &= ~RB_DVP_IF_FRM_DONE; // clear Interrupt
        addr_cnt = 0;
        href_cnt = 0;
    }

    if (DVP->IFR & RB_DVP_IF_STR_FRM)
    {
        DVP->IFR &= ~RB_DVP_IF_STR_FRM;
    }

    if (DVP->IFR & RB_DVP_IF_STP_FRM)
    {
        DVP->IFR &= ~RB_DVP_IF_STP_FRM;
        frame_cnt++;
    }

    if (DVP->IFR & RB_DVP_IF_FIFO_OV)
    {
        DVP->IFR &= ~RB_DVP_IF_FIFO_OV;
        printf("DVP FIFO OV\r\n");
    }
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    Delay_Init();
    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n", SystemCoreClock);

    /* LCD reset */
    LCD_Reset_GPIO_Init();
    GPIO_ResetBits(GPIOA, GPIO_Pin_15);
    Delay_Ms(100);
    GPIO_SetBits(GPIOA, GPIO_Pin_15);
    LCD_Init();
    LCD_SetBrightness(75);
    LCD_SetColor(0x18E3, RED);
    LCD_DrawRectangle(40, 80, 200, 160, TRUE);
    LCD_ShowString(64, 104, 32, FALSE, "WAITING");

    printf("LCD width:%02d height:%02d \n", LCD_W, LCD_H);

    /* Camera init */
    while (OV2640_Init())
    {
        printf("Camera Err\r\n");
        Delay_Ms(1000);
    }
    printf("Camera Success\r\n");
    Delay_Ms(1000);

    /* Camera mode */
    RGB565_Mode_Init();
    printf("RGB565_MODE\r\n");
    Delay_Ms(1000);

    /* Video data path */
    LCD_AddressSetWrite(0, 0, LCD_W - 1, LCD_H - 1);
    DMA_SRAMLCD_Init((u32)RGB565_DVPDMAaddr0); // DMA2
    DVP_Init();
    printf("DVP Enable\r\n");

    // Video is transfered through DMA now
    printf("\r\n");
    while (1)
    {
    }
}
