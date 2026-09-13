/**
 * ************************************************************************
 * 
 * @file OLED_IIC_Config.c
 * @author zxr
 * @brief IIC初始化配置和驱动OLED屏幕的底层函数
 * 
 * ************************************************************************
 * @copyright Copyright (c) 2024 zxr 
 * ************************************************************************
 */
#include "OLED_IIC_Config.h"
#include "MyDelay.h"

unsigned char  ScreenBuffer[SCREEN_PAGE_NUM][SCREEN_COLUMN];

static void IIC_Start(void)
{
    OLED_W_SDA(1);
    OLED_W_SCL(1);
    MyDelay_us(5);
    OLED_W_SDA(0);
    MyDelay_us(5);
    OLED_W_SCL(0);
    MyDelay_us(5);
}

static void IIC_Stop(void)
{
    OLED_W_SDA(0);
    OLED_W_SCL(1);
    MyDelay_us(5);
    OLED_W_SDA(1);
    MyDelay_us(5);
}

static void IIC_Send_Byte(unsigned char txd)
{
    unsigned char t;
    for(t=0;t<8;t++)
    {
        OLED_W_SDA((txd&0x80)>>7);
        txd<<=1;
        OLED_W_SCL(1);
        MyDelay_us(5);
        OLED_W_SCL(0);
        MyDelay_us(5);
    }
    OLED_W_SDA(1);
    OLED_W_SCL(1);
    MyDelay_us(5);
    OLED_W_SCL(0);
    MyDelay_us(5);
}

void I2C_WriteByte(uint8_t addr, uint8_t data)
{
    IIC_Start();
    IIC_Send_Byte(OLED_ADDRESS);
    IIC_Send_Byte(addr);
    IIC_Send_Byte(data);
    IIC_Stop();
}

/**
 * ************************************************************************
 * @brief 写命令函数
 * @param[in] cmd  写的命令
 * ************************************************************************
 */
void WriteCmd(unsigned char cmd)
{
    I2C_WriteByte(0x00, cmd);
}

/**
 * ************************************************************************
 * @brief 写数据函数
 * @param[in] dat  写的数据
 * ************************************************************************
 */
void WriteDat(unsigned char dat)
{
    I2C_WriteByte(0x40, dat);
}

/**
 * ************************************************************************
 * @brief 开启OLED
 * ************************************************************************
 */
void OLED_ON(void)
{
	WriteCmd(0X8D);  //设置电荷泵
	WriteCmd(0X14);  //开启电荷泵
	WriteCmd(0XAF);  //OLED开启
}

/**
 * ************************************************************************
 * @brief 关闭OLED
 * ************************************************************************
 */
void OLED_OFF(void)
{
	WriteCmd(0X8D);  //设置电荷泵
	WriteCmd(0X10);  //关闭电荷泵
	WriteCmd(0XAE);  //OLED关闭
}

/**
 * ************************************************************************
 * @brief OLED清屏函数
 * ************************************************************************
 */
void OLED_CLS(void)//清屏
{
	unsigned char m,n;
	for(m=0;m<8;m++)
	{
		WriteCmd(0xb0+m);	//page0-page1
		WriteCmd(0x00);		//low column start address
		WriteCmd(0x10);		//high column start address
		for(n=0;n<128;n++)
		{
			WriteDat(0x00);
		}
	}
}

/**
 * ************************************************************************
 * @brief OLED初始化函数
 * ************************************************************************
 */
void OLED_Init(void)
{
	MyDelay_ms(50);   // OLED 上电稳定延时（防止被编译器优化掉空循环）

	WriteCmd(0xAE);
	WriteCmd(0xD5);
	WriteCmd(0x80);
	WriteCmd(0xA8);
	WriteCmd(0x3F);
	WriteCmd(0xD3);
	WriteCmd(0x00);
	WriteCmd(0x40);
	WriteCmd(0xA1);
	WriteCmd(0xC8);
	WriteCmd(0xDA);
	WriteCmd(0x12);
	WriteCmd(0x81);
	WriteCmd(0xCF);
	WriteCmd(0xD9);
	WriteCmd(0xF1);
	WriteCmd(0xDB);
	WriteCmd(0x30);
	WriteCmd(0xA4);
	WriteCmd(0xA6);
	WriteCmd(0x8D);
	WriteCmd(0x14);
	WriteCmd(0xAF);
	OLED_CLS();
}

/**
 * ************************************************************************
 * @brief 更新数据缓冲区函数
 * ************************************************************************
 */
void OLED_RefreshRAM(void)
{
	for(unsigned short int m = 0; m < SCREEN_ROW/8; m++)
	{
		WriteCmd(0xb0+m);		
		WriteCmd(0x00);			
		WriteCmd(0x10);			
		for(unsigned short int n = 0; n < SCREEN_COLUMN; n++)
		{
			WriteDat(ScreenBuffer[m][n]);
		}
	} 
}

/**
 * ************************************************************************
 * @brief 清除数据缓冲区函数
 * ************************************************************************
 */
void OLED_ClearRAM(void)
{
	for(unsigned short int m = 0; m < SCREEN_ROW/8; m++)
	{
		for(unsigned short int n = 0; n < SCREEN_COLUMN; n++)
		{
			ScreenBuffer[m][n] = 0x00;
		}
	}
}

/**
 * ************************************************************************
 * @brief 在指定位置画点函数
 * 
 * @param[in] x  			起始横坐标(x:0~127)
 * @param[in] y  			起始纵坐标(y:0~63)
 * @param[in] set_pixel  	画点状态  SET_PIXEL = 1, RESET_PIXEL = 0
 * 
 * ************************************************************************
 */
void OLED_SetPixel(signed short int x, signed short int y, unsigned char set_pixel)
{ 
	if (x >= 0 && x < SCREEN_COLUMN && y >= 0 && y < SCREEN_ROW) {
		if(set_pixel){
				ScreenBuffer[y/8][x] |= (0x01 << (y%8));
		}  
		else{
				ScreenBuffer[y/8][x] &= ~(0x01 << (y%8));
		}
	}
}

/**
 * ************************************************************************
 * @brief 屏幕内容取反显示
 * 
 * @param[in] mode  模式
 * 					ON	0xA7	取反全显
 *  				OFF	0xA6	默认模式,正常显示
 * 
 * ************************************************************************
 */
void OLED_DisplayMode(unsigned char mode)
{
	WriteCmd(mode);
}

/**
 * ************************************************************************
 * @brief 屏幕亮度调节
 * 
 * @param[in] intensity  亮度大小(0~255),默认值为0x7f
 * 
 * ************************************************************************
 */
void OLED_IntensityControl(unsigned char intensity)
{
	WriteCmd(0x81);
	WriteCmd(intensity);
}

/**
 * ************************************************************************
 * @brief 全屏内容偏移指定距离
 * 
 * @param[in] shift_num  偏移步数(0~63)
 * 
 * ************************************************************************
 */
void OLED_Shift(unsigned char shift_num)
{
	for(unsigned char i = 0; i < shift_num; i++)
		{
			WriteCmd(0xd3);
			WriteCmd(i);
			HAL_Delay(10);
		}
}


/**
 * ************************************************************************
 * @brief 屏幕内容水平方向滚动播放
 * 
 * @param[in] start_page  	起始页码	(0~7)
 * @param[in] end_page  	结束页码	(0~7)
 * @param[in] direction  	滚动方向
 * 							LEFT	0x27
 * 							RIGHT	0x26
 * @note 在起始页码和结束页码之间的内容将被滚动显示,写入的新内容不会被改变
 * ************************************************************************
 */
void OLED_HorizontalShift(unsigned char start_page,unsigned char end_page,unsigned char direction)
{
	WriteCmd(0x2e);  

	WriteCmd(direction);
	WriteCmd(0x00);
	WriteCmd(start_page);
	WriteCmd(0x05);
	WriteCmd(end_page);
	WriteCmd(0x00);
	WriteCmd(0xff);

	WriteCmd(0x2f);
}
