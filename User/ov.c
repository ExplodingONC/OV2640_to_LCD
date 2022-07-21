/********************************** (C) COPYRIGHT *******************************
 * File Name          : ov.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2021/06/06
 * Description        : This file contains the headers of the OV2640.
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/
#include "ov.h"
#include "ov2640_regs.h"

/*********************************************************************
 * @fn      SCCB_GPIO_Init
 *
 * @brief   Init SCCB GPIO.
 *
 * @return  none
 */
void SCCB_GPIO_Init(void)
{
	IIC_SCL_OUT;
	IIC_SDA_OUT;
	IIC_SCL_SET;
	IIC_SDA_SET;
}

/*********************************************************************
 * @fn      SCCB_Start
 *
 * @brief   Start Signal
 *
 * @return  none
 */
void SCCB_Start(void)
{
	IIC_SDA_SET;
	IIC_SCL_SET;
	Delay_Us(50);
	IIC_SDA_CLR;
	Delay_Us(50);
	IIC_SCL_CLR;
}

/*********************************************************************
 * @fn      SCCB_Stop
 *
 * @brief   Stop Signal
 *
 * @return  none
 */
void SCCB_Stop(void)
{
	IIC_SDA_CLR;
	Delay_Us(50);
	IIC_SCL_SET;
	Delay_Us(50);
	IIC_SDA_SET;
	Delay_Us(50);
}

/*********************************************************************
 * @fn      SCCB_No_Ack
 *
 * @brief   NAK Signal
 *
 * @return  none
 */
void SCCB_No_Ack(void)
{
	Delay_Us(50);
	IIC_SDA_SET;
	IIC_SCL_SET;
	Delay_Us(50);
	IIC_SCL_CLR;
	Delay_Us(50);
	IIC_SDA_CLR;
	Delay_Us(50);
}

/*********************************************************************
 * @fn      SCCB_WR_Byte
 *
 * @brief   Write One Byte
 *
 * @param   data
 *
 * @return  0 - Success
 *          other - Err
 */
UINT8 SCCB_WR_Byte(UINT8 data)
{
	UINT8 i, t;

	for (i = 0; i < 8; i++)
	{
		if (data & 0x80)
		{
			IIC_SDA_SET;
		}
		else
		{
			IIC_SDA_CLR;
		}

		data <<= 1;
		Delay_Us(50);
		IIC_SCL_SET;
		Delay_Us(50);
		IIC_SCL_CLR;
	}

	IIC_SDA_IN;
	Delay_Us(50);
	IIC_SCL_SET;
	Delay_Us(50);

	if (SDA_IN_R)
		t = 1;
	else
		t = 0;

	IIC_SCL_CLR;
	IIC_SDA_OUT;

	return t;
}

/*********************************************************************
 * @fn      SCCB_RD_Byte
 *
 * @brief   Read One Byte
 *
 * @return  Read one byte data
 */
UINT8 SCCB_RD_Byte(void)
{
	UINT8 t = 0, i;

	IIC_SDA_IN;

	for (i = 8; i > 0; i--)
	{
		Delay_Us(50);
		IIC_SCL_SET;
		t = t << 1;

		if (SDA_IN_R)
			t++;

		Delay_Us(50);
		IIC_SCL_CLR;
	}

	IIC_SDA_OUT;

	return t;
}

/*********************************************************************
 * @fn      SCCB_WR_Reg
 *
 * @brief   Write camera Register
 *
 * @param   Reg_Adr - Register address
 *          Reg_Val - Register value
 *
 * @return  0 - Success
 *          other - Err
 */
UINT8 SCCB_WR_Reg(UINT8 Reg_Adr, UINT8 Reg_Val)
{
	UINT8 res = 0;

	SCCB_Start();
	if (SCCB_WR_Byte(SCCB_ID))
		res = 1;
	Delay_Us(100);
	if (SCCB_WR_Byte(Reg_Adr))
		res = 1;
	Delay_Us(100);
	if (SCCB_WR_Byte(Reg_Val))
		res = 1;
	SCCB_Stop();

	return res;
}

/*********************************************************************
 * @fn      SCCB_RD_Reg
 *
 * @brief   Read camera Register
 *
 * @return  Camera Register value
 */
UINT8 SCCB_RD_Reg(UINT8 Reg_Adr)
{
	UINT8 val = 0;

	SCCB_Start();
	SCCB_WR_Byte(SCCB_ID);
	Delay_Us(100);
	SCCB_WR_Byte(Reg_Adr);
	Delay_Us(100);
	SCCB_Stop();
	Delay_Us(100);

	SCCB_Start();
	SCCB_WR_Byte(SCCB_ID | 0X01);
	Delay_Us(100);
	val = SCCB_RD_Byte();
	SCCB_No_Ack();
	SCCB_Stop();

	return val;
}

/*********************************************************************
 * @fn      DVP_GPIO_Init
 *
 * @brief   Init DVP GPIO.
 *
 * @return  none
 */
void DVP_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE); //ʹ����ض˿�ʱ��

	// PC3-PC13
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}

/*********************************************************************
 * @fn      OV2640_Init
 *
 * @brief   Init OV2640
 *
 * @return  0 - Success
 *          1 - Err
 */
UINT8 OV2640_Init(void)
{
	UINT16 i = 0;
	UINT16 reg;

	DVP_GPIO_Init();

	OV_PWDN_CLR; // POWER ON
	Delay_Ms(10);
	OV_RESET_CLR; // Reset OV2640
	Delay_Ms(10);
	OV_RESET_SET; // Reset End

	SCCB_GPIO_Init();
	SCCB_WR_Reg(0xFF, 0x01); // Register Bank Select Sensor address
	SCCB_WR_Reg(0x12, 0x80); // Reset All Register

	Delay_Ms(50);

	// Read MID
	reg = SCCB_RD_Reg(0x1C);
	reg <<= 8;
	reg |= SCCB_RD_Reg(0x1D);

	if (reg != OV2640_MID)
	{
		printf("MID:%08x\r\n", reg);
		return 1;
	}

	// Read PID
	reg = SCCB_RD_Reg(0x0A);
	reg <<= 8;
	reg |= SCCB_RD_Reg(0x0B);

	if (reg != OV2640_PID)
	{
		printf("PID:%08x\r\n", reg);
		return 2;
	}

	for (i = 0; i < sizeof(ov2640_init_regs) / 2; i++)
	{
		SCCB_WR_Reg(ov2640_init_regs[i][0], ov2640_init_regs[i][1]);
	}

	return 0;
}

/*********************************************************************
 * @fn      RGB565_Mode_Init
 *
 * @brief   Init RGB565 mode
 *
 * @return  none
 */
void RGB565_Mode_Init(void)
{
	// OV2640_OutSize_Set(OV2640_RGB565_WIDTH,OV2640_RGB565_HEIGHT);
	// OV2640_Speed_Set(28,1);
	OV2640_OutSize_Set_Custom();
	OV2640_RGB565_Mode();
}

/*********************************************************************
 * @fn      OV2640_RGB565_Mode
 *
 * @brief   RGB565 Mode
 *
 * @return  none
 */
void OV2640_RGB565_Mode(void)
{
	UINT16 i = 0;

	// preamble
	for (i = 0; i < (sizeof(ov2640_format_change_preamble_regs) / 2); i++)
	{
		SCCB_WR_Reg(ov2640_format_change_preamble_regs[i][0], ov2640_format_change_preamble_regs[i][1]);
	}
	// RGB565
	for (i = 0; i < (sizeof(ov2640_rgb565_be_regs) / 2); i++)
	{
		SCCB_WR_Reg(ov2640_rgb565_be_regs[i][0], ov2640_rgb565_be_regs[i][1]);
	}
}

/*********************************************************************
 * @fn      OV2640_OutSize_Set
 *
 * @brief   Set Image Resolution
 *
 * @param   Image_width -
 *          Image_height -
 *
 * @return  none
 */
UINT8 OV2640_OutSize_Set(UINT16 Image_width, UINT16 Image_height)
{
	UINT16 Out_Size_Width;
	UINT16 Out_Size_Height;
	UINT8 t;

	if ((Image_width % 4) || (Image_height % 4))
		return 0x01;

	Out_Size_Width = Image_width / 4;
	Out_Size_Height = Image_height / 4;

	SCCB_WR_Reg(0XFF, 0X00);
	SCCB_WR_Reg(0XE0, 0X04);
	SCCB_WR_Reg(0X5A, Out_Size_Width & 0XFF);
	SCCB_WR_Reg(0X5B, Out_Size_Height & 0XFF);
	t = (Out_Size_Width >> 8) & 0X03;
	t |= (Out_Size_Height >> 6) & 0X04;
	SCCB_WR_Reg(0X5C, t);
	SCCB_WR_Reg(0XE0, 0X00);

	return 0;
}

UINT8 OV2640_OutSize_Set_Custom(void)
{
	UINT16 i = 0;

	// preamble
	for (i = 0; i < (sizeof(ov2640_size_change_preamble_regs) / 2); i++)
	{
		SCCB_WR_Reg(ov2640_size_change_preamble_regs[i][0], ov2640_size_change_preamble_regs[i][1]);
	}
	// custom:240*240
	for (i = 0; i < (sizeof(ov2640_qvga_regs) / 2); i++)
	{
		SCCB_WR_Reg(ov2640_qvga_regs[i][0], ov2640_qvga_regs[i][1]);
	}

	return 0;
}

/*********************************************************************
 * @fn      OV2640_Speed_Set
 *
 * @brief   Set DVP PCLK
 *
 * @param   Pclk_Div - DVP output speed ctrl
 *          Xclk_Div - Crystal oscillator input frequency division
 *
 * @return  0 - Success
 *          other - Err
 */
void OV2640_Speed_Set(UINT8 Pclk_Div, UINT8 Xclk_Div)
{
	SCCB_WR_Reg(0XFF, 0X00);
	SCCB_WR_Reg(0XD3, Pclk_Div);

	SCCB_WR_Reg(0XFF, 0X01);
	SCCB_WR_Reg(0X11, Xclk_Div);
}
