
#include "LCD_DRIVER.h"

#include "glcdfont.c"

#include <arduino.h>


//LCD的画笔颜色和背景色	   
uint16_t POINT_COLOR=0x0000;	//画笔颜色
uint16_t BACK_COLOR=0xFFFF;  //背景色 

inline GFXglyph *pgm_read_glyph_ptr(const GFXfont *gfxFont, uint8_t c) {
#ifdef __AVR__
  return &(((GFXglyph *)pgm_read_pointer(&gfxFont->glyph))[c]);
#else
  // expression in __AVR__ section may generate "dereferencing type-punned
  // pointer will break strict-aliasing rules" warning In fact, on other
  // platforms (such as STM32) there is no need to do this pointer magic as
  // program memory may be read in a usual way So expression may be simplified
  return gfxFont->glyph + c;
#endif //__AVR__
}

inline uint8_t *pgm_read_bitmap_ptr(const GFXfont *gfxFont) {
#ifdef __AVR__
  return (uint8_t *)pgm_read_pointer(&gfxFont->bitmap);
#else
  // expression in __AVR__ section generates "dereferencing type-punned pointer
  // will break strict-aliasing rules" warning In fact, on other platforms (such
  // as STM32) there is no need to do this pointer magic as program memory may
  // be read in a usual way So expression may be simplified
  return gfxFont->bitmap;
#endif //__AVR__
}

void CLCD_Driver::fillScreen(uint16_t color) {
  fillRect(0, 0, _width, _height, color);
}

CLCD_Driver::CLCD_Driver()
{
	
	
  
	m_sLcd.pin_num_cs = 15;
	
	m_sLcd.pin_num_wr = 2; //LCD_WR
	
	m_sLcd.pin_num_rd = 19; //LCD_RD
	
	m_sLcd.pin_num_rs = 4;
	
	m_sLcd.pin_num_rst = 5;
	
	m_sLcd.pin_data_num[0] = 12;//LCD_D0
	
	m_sLcd.pin_data_num[1] = 13;
	
	m_sLcd.pin_data_num[2] = 26;
	
	m_sLcd.pin_data_num[3] = 25;
	
	m_sLcd.pin_data_num[4] = 33;
	
	m_sLcd.pin_data_num[5] = 32;
	
	m_sLcd.pin_data_num[6] = 27;
	
	m_sLcd.pin_data_num[7] = 14;
	
	m_lcddev = {0};

  _width = ILI9341_TFTWIDTH;
  _height = ILI9341_TFTHEIGHT;

  rotation = 0;
  cursor_y = cursor_x = 0;
  textsize_x = textsize_y = 1;
  textcolor = textbgcolor = 0xFFFF;
  wrap = true;
  _cp437 = false;
  gfxFont = NULL;
	
}

void CLCD_Driver::WriteCommand(unsigned char yData)
{
	//digitalWrite(m_sLcd.pin_num_cs, 0);
	
	digitalWrite(m_sLcd.pin_num_rs, 0);
		
	reg_lcd_write_data(&yData, 1);
	
	//digitalWrite(m_sLcd.pin_num_cs, 1);
}

/**************************************************************************/
/*!
    @brief  Print one byte/character of data, used to support print()
    @param  c  The 8-bit ascii character to write
*/
/**************************************************************************/
size_t CLCD_Driver::write(uint8_t c) {

 TFT_DBG_PRINT(String("CLCD_Driver::write:") + String(c));

 if (!gfxFont) { // 'Classic' built-in font

     TFT_DBG_PRINT(String("CLCD_Driver::write:") + String("Classic built-in font"));

    if (c == '\n') {              // Newline?
      cursor_x = 0;               // Reset x to zero,
      cursor_y += textsize_y * 8; // advance y one line
    } else if (c != '\r') {       // Ignore carriage returns
      if (wrap && ((cursor_x + textsize_x * 6) > _width)) { // Off right?
        cursor_x = 0;                                       // Reset x to zero,
        cursor_y += textsize_y * 8; // advance y one line
      }
      drawChar(cursor_x, cursor_y, c, textcolor, textbgcolor, textsize_x,
               textsize_y);
      cursor_x += textsize_x * 6; // Advance x one char
    }

  } else { // Custom font

    TFT_DBG_PRINT(String("CLCD_Driver::write:") + String("Custom font"));

    if (c == '\n') {
      cursor_x = 0;
      cursor_y +=
          (int16_t)textsize_y * (uint8_t)pgm_read_byte(&gfxFont->yAdvance);
    } else if (c != '\r') {
      uint8_t first = pgm_read_byte(&gfxFont->first);
      if ((c >= first) && (c <= (uint8_t)pgm_read_byte(&gfxFont->last))) {
        GFXglyph *glyph = pgm_read_glyph_ptr(gfxFont, c - first);
        uint8_t w = pgm_read_byte(&glyph->width),
                h = pgm_read_byte(&glyph->height);
        if ((w > 0) && (h > 0)) { // Is there an associated bitmap?
          int16_t xo = (int8_t)pgm_read_byte(&glyph->xOffset); // sic
          if (wrap && ((cursor_x + textsize_x * (xo + w)) > _width)) {
            cursor_x = 0;
            cursor_y += (int16_t)textsize_y *
                        (uint8_t)pgm_read_byte(&gfxFont->yAdvance);
          }
          drawChar(cursor_x, cursor_y, c, textcolor, textbgcolor, textsize_x,
                   textsize_y);
        }
        cursor_x +=
            (uint8_t)pgm_read_byte(&glyph->xAdvance) * (int16_t)textsize_x;
      }
    }
  }
  return 1;
}

void CLCD_Driver::LCD_WriteRAM_Prepare(void)
{
 	LCD_cmd(m_lcddev.wramcmd);	  
}

/*****************************************************************************
 * @name       :void LCD_WriteReg(u16 LCD_Reg, u16 LCD_RegValue)
 * @date       :2018-08-09 
 * @function   :Write data into registers
 * @parameters :LCD_Reg:Register address
                LCD_RegValue:Data to be written
 * @retvalue   :None
******************************************************************************/

void CLCD_Driver::LCD_WriteReg(uint16_t LCD_Reg, uint16_t LCD_RegValue)
{
  LCD_cmd(LCD_Reg);
  LCD_data(LCD_RegValue);
}

/*****************************************************************************
 * @name       :void LCD_direction(u8 direction)
 * @date       :2018-08-09 
 * @function   :Setting the display direction of LCD screen
 * @parameters :direction:0-0 degree
                          1-90 degree
													2-180 degree
													3-270 degree
 * @retvalue   :None
******************************************************************************/ 
void CLCD_Driver::LCD_direction(uint8_t direction)
{ 
			m_lcddev.setxcmd=0x2A;
			m_lcddev.setycmd=0x2B;
			m_lcddev.wramcmd=0x2C;
			m_lcddev.rramcmd=0x2E;
			
	switch(direction)
	{		  
		case 0:						 	 		
			m_lcddev.width=LCD_W;
			m_lcddev.height=LCD_H;		
			LCD_WriteReg(0x36,(1<<6)|(1<<3));//0 degree MY=0,MX=0,MV=0,ML=0,BGR=1,MH=0
		break;
		case 1:
			m_lcddev.width=LCD_H;
			m_lcddev.height=LCD_W;
			LCD_WriteReg(0x36,(1<<3)|(1<<4)|(1<<5));//90 degree MY=0,MX=1,MV=1,ML=1,BGR=1,MH=0
		break;
		case 2:						 	 		
			m_lcddev.width=LCD_W;
			m_lcddev.height=LCD_H;	
			LCD_WriteReg(0x36,(1<<3)|(1<<7));//180 degree MY=1,MX=1,MV=0,ML=0,BGR=1,MH=0
		break;
		case 3:
			m_lcddev.width=LCD_H;
			m_lcddev.height=LCD_W;
			LCD_WriteReg(0x36,(1<<3)|(1<<5)|(1<<6)|(1<<7));//270 degree MY=1,MX=0,MV=1,ML=0,BGR=1,MH=0
		break;	
		default:break;
	}		
	
}	

/*****************************************************************************
 * @name       :void LCD_WR_DATA_16Bit(u16 Data)
 * @date       :2018-08-09 
 * @function   :Write an 16-bit command to the LCD screen
 * @parameters :Data:Data to be written
 * @retvalue   :None
******************************************************************************/	 
void CLCD_Driver::LCD_WR_DATA_16Bit(uint16_t Data)
{
	uint8_t yDataValue = 0;
	
	digitalWrite(m_sLcd.pin_num_rs, 1);
	
	yDataValue = (Data>>8)&0xFF;
	
	reg_lcd_write_data(&yDataValue, 1);
	
	yDataValue = Data&0xFF;
	
	reg_lcd_write_data(&yDataValue, 1);
}

/*****************************************************************************
 * @name       :void LCD_Clear(u16 Color)
 * @date       :2018-08-09 
 * @function   :Full screen filled LCD screen
 * @parameters :color:Filled color
 * @retvalue   :None
******************************************************************************/	
void  CLCD_Driver::LCD_Clear(uint16_t Color)
{
	uint16_t i,j;
	
	char ayMsgText[512];
	
	sprintf(ayMsgText, "CLCD_Driver::LCD_Clear -> w(%d),h(%d),(%d)", m_lcddev.width, m_lcddev.height, Color);
	
	TFT_DBG_PRINT(ayMsgText);
	
	LCD_SetWindows(0,0,m_lcddev.width-1,m_lcddev.height-1);	
	
    for(i=0;i<m_lcddev.width;i++)
	 {
	  for (j=0;j<m_lcddev.height;j++)
	   {
        	 LCD_WR_DATA_16Bit(Color);
	   }

	}
}
void  CLCD_Driver::LCD_Draw_rect(int xorg, int yorg, int rect_width,int rect_height,uint16_t Color)
{
	uint16_t i,j;
	
	LCD_SetWindows(xorg, yorg, rect_width,rect_height);
	
    for(i=0;i<rect_width;i++)
	 {
	  for (j=0;j<rect_height;j++)
	   {
		   LCD_WR_DATA_16Bit(Color--);
	   }

	  }
}
void CLCD_Driver::LCD_DRAW_LINE(uint16_t Color){
	uint16_t i;
	LCD_SetWindows(75,75,200/*m_lcddev.width-1*/,75/*m_lcddev.height-1*/);	
    for(i=0;i<200/*m_lcddev.width*/;i++)
	 {
         LCD_WR_DATA_16Bit(Color);
	 }
	  // do
	  // {
		  // delay(1000);
	  // }while(!Serial.available());
	  // serialflash();
}

void CLCD_Driver::LCD_SetWindows(uint16_t xStar, uint16_t yStar, uint16_t xEnd, uint16_t yEnd)
{	
	LCD_cmd(m_lcddev.setxcmd);	
	LCD_data(xStar>>8);
	LCD_data(0x00FF&xStar);		
	LCD_data(xEnd>>8);
	LCD_data(0x00FF&xEnd);

	LCD_cmd(m_lcddev.setycmd);	
	LCD_data(yStar>>8);
	LCD_data(0x00FF&yStar);		
	LCD_data(yEnd>>8);
	LCD_data(0x00FF&yEnd);	

	LCD_WriteRAM_Prepare();	//开始写入GRAM				
}   


/*****************************************************************************
 * @name       :void LCD_DrawPoint(u16 x,u16 y)
 * @date       :2018-08-09 
 * @function   :Write a pixel data at a specified location
 * @parameters :x:the x coordinate of the pixel
                y:the y coordinate of the pixel
 * @retvalue   :None
******************************************************************************/	
void CLCD_Driver::LCD_DrawPoint(uint16_t x,uint16_t y)
{
	LCD_SetWindows(x,y,x,y);//设置光标位置 
	LCD_WR_DATA_16Bit(POINT_COLOR); 	    
} 	 


/*****************************************************************************
 * @name       :void LCD_SetCursor(u16 Xpos, u16 Ypos)
 * @date       :2018-08-09 
 * @function   :Set coordinate value
 * @parameters :Xpos:the  x coordinate of the pixel
								Ypos:the  y coordinate of the pixel
 * @retvalue   :None
******************************************************************************/ 
void CLCD_Driver::LCD_SetCursor(uint16_t Xpos, uint16_t Ypos)
{	  	    			
	LCD_SetWindows(Xpos,Ypos,Xpos,Ypos);	
} 

void CLCD_Driver::GUI_DrawPoint(uint16_t x,uint16_t y,uint16_t color)
{
	LCD_SetCursor(x,y);//设置光标位置 
	LCD_WR_DATA_16Bit(color); 
}




void CLCD_Driver::reg_lcd_write_data(unsigned char *data, uint8_t size) 
{
	digitalWrite( m_sLcd.pin_num_cs, 0 );
	
	
	for (int i=0; i < size; i++)
	{
		digitalWrite(m_sLcd.pin_data_num[0], data[i] & 0x01);
		digitalWrite(m_sLcd.pin_data_num[1], data[i] & 0x02);
		digitalWrite(m_sLcd.pin_data_num[2], data[i] & 0x04);
		digitalWrite(m_sLcd.pin_data_num[3], data[i] & 0x08);
		digitalWrite(m_sLcd.pin_data_num[4], data[i] & 0x10); 
		digitalWrite(m_sLcd.pin_data_num[5], data[i] & 0x20);
		digitalWrite(m_sLcd.pin_data_num[6], data[i] & 0x40);
		digitalWrite(m_sLcd.pin_data_num[7], data[i] & 0x80);	
		
		digitalWrite( m_sLcd.pin_num_wr, 0 ); //pulsing write pin
		digitalWrite( m_sLcd.pin_num_wr, 1 );
	}
}


void CLCD_Driver::WriteData(unsigned char yData)
{
	
	//digitalWrite(m_sLcd.pin_num_cs, 0);
	
	digitalWrite(m_sLcd.pin_num_rs, 1);
		
	reg_lcd_write_data(&yData, 1);
	
	//digitalWrite(m_sLcd.pin_num_cs, 1);

}


void CLCD_Driver::InitPins_mode()
{
    char ayMsg[512];//char data
	pinMode(m_sLcd.pin_num_wr, OUTPUT); //write pin as op(set to 0)
	digitalWrite(m_sLcd.pin_num_wr,1);
	
	pinMode(m_sLcd.pin_num_cs, OUTPUT); //cs as op(set to 0)
	digitalWrite(m_sLcd.pin_num_cs,1);
	
	pinMode(m_sLcd.pin_num_rd, OUTPUT); //read pin as output(set to 1)
	digitalWrite(m_sLcd.pin_num_rd,1);
	
	pinMode(m_sLcd.pin_num_rs, OUTPUT); //register select to command or write mode
	digitalWrite(m_sLcd.pin_num_rs, 1); //reset high
	
	pinMode(m_sLcd.pin_num_rst, OUTPUT); //reset pin
	digitalWrite(m_sLcd.pin_num_rst, 1); //reset high
	
	pinMode(m_sLcd.pin_data_num[0], OUTPUT); //datapins 0-7
	
	pinMode(m_sLcd.pin_data_num[1], OUTPUT);
	
	pinMode(m_sLcd.pin_data_num[2], OUTPUT);
	
	pinMode(m_sLcd.pin_data_num[3], OUTPUT);
	
	pinMode(m_sLcd.pin_data_num[4], OUTPUT);
	
	pinMode(m_sLcd.pin_data_num[5], OUTPUT);
	
	pinMode(m_sLcd.pin_data_num[6], OUTPUT);
	
	pinMode(m_sLcd.pin_data_num[7], OUTPUT);

    


	// sprintf(ayMsg,"d0-D7 = 0x55\r\n Press Enter to continue\r\n");
	// Serial.println(ayMsg);
	// WriteData(0x55);
	// do
	// {
		// delay(1000);
	// }while(!Serial.available());
	// serialflash();
	
	// sprintf(ayMsg,"d0-D7 = 0x55\r\n Press Enter to continue\r\n");
	// Serial.println(ayMsg);
	// LCD_data(0x55);
	// do
	// {
		// delay(1000);
	// }while(!Serial.available());
	// serialflash();

	// sprintf(ayMsg,"d0-D7 = 0xAA\r\n Press Enter to continue\r\n");
	// Serial.println(ayMsg);
	// WriteData(0xAA);
	// do
	// {
		// delay(1000);
	// }while(!Serial.available());
	// serialflash();
	    
    // LCD_cmd(0x01); // Software reset
    // delay(100);
    // LCD_cmd(0x11); // Sleep out
	// LCD_cmd(0x28);//display off
    // do
	// {
		// delay(1000);
	// }while(!Serial.available());
	// serialflash();
    // LCD_cmd(0x29); //dipaly ON
      // do
	// {
		// delay(1000);
	// }while(!Serial.available());
	// serialflash();
	// sprintf(ayMsg,"d0-D7 = 0xAA\r\n Press Enter to continue\r\n");
	// Serial.println(ayMsg);
	// LCD_data(0xAA);
	// do
	// {
		// delay(1000);
	// }while(!Serial.available());
	// serialflash();
//	int yData = 0x55;
	 	
}

void CLCD_Driver::serialflash()
{
	while(Serial.available())
	{
		char t = Serial.read();
	}
}

void CLCD_Driver::LCD_cmd(uint8_t yData)
{
	
	TFT_DBG_PRINT(String("CLCD_Driver::LCD_cmd: 0x") + String(yData, HEX));

	digitalWrite(m_sLcd.pin_num_rs, 0); //enabling the command mode
	
	reg_lcd_write_data(&yData, 1);


}

// reset and init the lcd controller
void CLCD_Driver::init()
{
    /* Start Initial Sequence ----------------------------------------------------*/
    
	LCD_cmd(0XF1);
	LCD_data(0x36);
	LCD_data(0x04);
	LCD_data(0x00);
	LCD_data(0x3C);
	LCD_data(0X0F);
	LCD_data(0x8F);
	LCD_cmd(0XF2);
	LCD_data(0x18);
	LCD_data(0xA3);
	LCD_data(0x12);
	LCD_data(0x02);
	LCD_data(0XB2);
	LCD_data(0x12);
	LCD_data(0xFF);
	LCD_data(0x10);
	LCD_data(0x00);
	LCD_cmd(0XF8);
	LCD_data(0x21);
	LCD_data(0x04);
	LCD_cmd(0XF9);
	LCD_data(0x00);
	LCD_data(0x08);
	LCD_cmd(0x36);
	LCD_data(0x08);
	LCD_cmd(0xB4);
	LCD_data(0x00);
	LCD_cmd(0xC1);
	LCD_data(0x41);
	LCD_cmd(0xC5);
	LCD_data(0x00);
	LCD_data(0x91);
	LCD_data(0x80);
	LCD_data(0x00);
	LCD_cmd(0xE0);
	LCD_data(0x0F);
	LCD_data(0x1F);
	LCD_data(0x1C);
	LCD_data(0x0C);
	LCD_data(0x0F);
	LCD_data(0x08);
	LCD_data(0x48);
	LCD_data(0x98);
	LCD_data(0x37);
	LCD_data(0x0A);
	LCD_data(0x13);
	LCD_data(0x04);
	LCD_data(0x11);
	LCD_data(0x0D);
	LCD_data(0x00);
	LCD_cmd(0xE1);
	LCD_data(0x0F);
	LCD_data(0x32);
	LCD_data(0x2E);
	LCD_data(0x0B);
	LCD_data(0x0D);
	LCD_data(0x05);
	LCD_data(0x47);
	LCD_data(0x75);
	LCD_data(0x37);
	LCD_data(0x06);
	LCD_data(0x10);
	LCD_data(0x03);
	LCD_data(0x24);
	LCD_data(0x20);
	LCD_data(0x00);
	LCD_cmd(0x3A);
	LCD_data(0x55);
	LCD_cmd(0x11);
	LCD_cmd(0x36);
	LCD_data(0x28);
	delay(120);
	LCD_cmd(0x29);
  
  LCD_direction(LCD_USE8BIT_MODEL);

	//LCD_direction(USE_HORIZONTAL);
	
    // digitalWrite(m_sLcd.pin_num_cs, 0);
    // LCD_cmd(0xF1);
    // LCD_data(0x36);
    // LCD_data(0x04);
    // LCD_data(0x00);
    // LCD_data(0x3C);
    // LCD_data(0x0F);
    // LCD_data(0x8F);
    // digitalWrite(m_sLcd.pin_num_cs, 1);


	// digitalWrite(m_sLcd.pin_num_cs, 0);
    // LCD_cmd(0xF2);
    // LCD_data(0x18);
    // LCD_data(0xA3);
    // LCD_data(0x12);
    // LCD_data(0x02);
    // LCD_data(0xb2);
    // LCD_data(0x12);
    // LCD_data(0xFF);
    // LCD_data(0x10);
    // LCD_data(0x00);
	// digitalWrite(m_sLcd.pin_num_cs, 1);

	// digitalWrite(m_sLcd.pin_num_cs, 0);
    // LCD_cmd(0xF8);
    // LCD_data(0x21);
    // LCD_data(0x04);
	// digitalWrite(m_sLcd.pin_num_cs, 1);

	// digitalWrite(m_sLcd.pin_num_cs, 0);
    // LCD_cmd(0xF9);
    // LCD_data(0x00);
    // LCD_data(0x08); 
	// digitalWrite(m_sLcd.pin_num_cs, 1);	

	// digitalWrite(m_sLcd.pin_num_cs, 0);
    // LCD_cmd(0xC0);
    // LCD_data(0x0f); //13
    // LCD_data(0x0f); //10
	// digitalWrite(m_sLcd.pin_num_cs, 1);

	// digitalWrite(m_sLcd.pin_num_cs, 0);
    // LCD_cmd(0xC1);
    // LCD_data(0x42); //43
	// digitalWrite(m_sLcd.pin_num_cs, 1);

	// digitalWrite(m_sLcd.pin_num_cs, 0);
    // LCD_cmd(0xC2);
    // LCD_data(0x22);
	// digitalWrite(m_sLcd.pin_num_cs, 1);

	// digitalWrite(m_sLcd.pin_num_cs, 0);
    // LCD_cmd(0xC5);
    // LCD_data(0x01); //00
    // LCD_data(0x29); //4D
    // LCD_data(0x80);
	// digitalWrite(m_sLcd.pin_num_cs, 1);

	// digitalWrite(m_sLcd.pin_num_cs, 0);
    // LCD_cmd(0xB6);
    // LCD_data(0x00);
    // LCD_data(0x02); //42
    // LCD_data(0x3b);
	// digitalWrite(m_sLcd.pin_num_cs, 1);

	// digitalWrite(m_sLcd.pin_num_cs, 0);
    // LCD_cmd(0xB1);
    // LCD_data(0xB0); //C0
    // LCD_data(0x11);
	// digitalWrite(m_sLcd.pin_num_cs, 1);

	// digitalWrite(m_sLcd.pin_num_cs, 0);
    // LCD_cmd(0xB4);
    // LCD_data(0x02); //01
	// digitalWrite(m_sLcd.pin_num_cs, 1);

	// digitalWrite(m_sLcd.pin_num_cs, 0);
    // LCD_cmd(0xE0);
    // LCD_data(0x0F);
    // LCD_data(0x18);
    // LCD_data(0x15);
    // LCD_data(0x09);
    // LCD_data(0x0B);
    // LCD_data(0x04);
    // LCD_data(0x49);
    // LCD_data(0x64);
    // LCD_data(0x3D);
    // LCD_data(0x08);
    // LCD_data(0x15);
    // LCD_data(0x06);
    // LCD_data(0x12);
    // LCD_data(0x07);
    // LCD_data(0x00);
	// digitalWrite(m_sLcd.pin_num_cs, 1);

	// digitalWrite(m_sLcd.pin_num_cs, 0);
    // LCD_cmd(0xE1);
    // LCD_data(0x0F);
    // LCD_data(0x38);
    // LCD_data(0x35);
    // LCD_data(0x0a);
    // LCD_data(0x0c);
    // LCD_data(0x03);
    // LCD_data(0x4A);
    // LCD_data(0x42);
    // LCD_data(0x36);
    // LCD_data(0x04);
    // LCD_data(0x0F);
    // LCD_data(0x03);
    // LCD_data(0x1F);
    // LCD_data(0x1B);
    // LCD_data(0x00);
	// digitalWrite(m_sLcd.pin_num_cs, 1);

	// digitalWrite(m_sLcd.pin_num_cs, 0);
    // LCD_cmd(0x20);                     // display inversion OFF
  
    // LCD_cmd(0x36);      // MEMORY_ACCESS_CONTROL (orientation stuff)
    // LCD_data(0x48);
	// digitalWrite(m_sLcd.pin_num_cs, 1);
     
	// digitalWrite(m_sLcd.pin_num_cs, 0);
    // LCD_cmd(0x3A);      // COLMOD_PIXEL_FORMAT_SET
    // LCD_data(0x55);     // 16 bit pixel 
	// digitalWrite(m_sLcd.pin_num_cs, 1);

	// digitalWrite(m_sLcd.pin_num_cs, 0);
    // LCD_cmd(0x13); // Nomal Displaymode
    
    // LCD_cmd(0x11);                     // sleep out
    // delay(150);
     
    // LCD_cmd(0x29);                     // display on
    // delay(150);
	// digitalWrite(m_sLcd.pin_num_cs, 1);
}


/**************************************************************************/
/*!
    @brief   Enable/Disable display color inversion
    @param   invert True to invert, False to have normal color
*/
/**************************************************************************/
void CLCD_Driver::invertDisplay(bool invert) 
{
  LCD_cmd(invert ? ILI9341_INVON : ILI9341_INVOFF);
}

void CLCD_Driver::LCD_data(uint8_t yData)
{
	TFT_DBG_PRINT(String("CLCD_Driver::LCD_data: 0x") + String(yData, HEX));
	
	digitalWrite(m_sLcd.pin_num_rs, 1); //enabling the command mode
	
	reg_lcd_write_data(&yData, 1);
	
	//WriteData(ycommand);
	
	// digitalWrite(m_sLcd.pin_num_cs, 1); //enabling the lcd

	// for(int  i = 0; i < 8; i++)
	// {
		// if( ycommand & 0x01)
		// {
			// digitalWrite(m_sLcd.pin_data_num[i], 1);
		// }
		// else
		// {
			// digitalWrite(m_sLcd.pin_data_num[i], 0);
		// }
		
		// ycommand >>= 1;
	// }

    // digitalWrite(m_sLcd.pin_num_wr, 1); //write low
 
	// digitalWrite(m_sLcd.pin_num_cs, 0); //enabling the lcd
	
	// digitalWrite(m_sLcd.pin_num_rs, 1); //enabling the command mode
	
	// digitalWrite(m_sLcd.pin_num_wr, 0);
	
	// delay(1);
	
	// digitalWrite(m_sLcd.pin_num_wr, 1);
	
	// digitalWrite(m_sLcd.pin_num_cs, 1); //enabling the lcd
	
	// digitalWrite(m_sLcd.pin_num_cs, 0);
	// digitalWrite(m_sLcd.pin_num_rs,1); //enabling the data mode
	
	// digitalWrite(m_sLcd.pin_num_wr,1); //enable write pin

	// for(int  i = 0; i < 8; i++)
	// {
		// if((data>>i) & 0x01)
		// {
			// digitalWrite(m_sLcd.pin_data_num[i], 1);
		// }
		// else
		// {
			// digitalWrite(m_sLcd.pin_data_num[i], 0);
		// }
	// }
	// digitalWrite(m_sLcd.pin_num_wr,0); //disable write pin

}
	
CLCD_Driver::~CLCD_Driver()
{
	  
}

/**************************************************************************/
/*!
   @brief    Draw a perfectly vertical line (this is often optimized in a
   subclass!)
    @param    x   Top-most x coordinate
    @param    y   Top-most y coordinate
    @param    h   Height in pixels
   @param    color 16-bit 5-6-5 Color to fill with
*/
/**************************************************************************/
void CLCD_Driver::drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) {
  writeLine(x, y, x, y + h - 1, color);
 
}

/**************************************************************************/
/*!
   @brief    Write a perfectly vertical line, overwrite in subclasses if
   startWrite is defined!
    @param    x   Top-most x coordinate
    @param    y   Top-most y coordinate
    @param    h   Height in pixels
   @param    color 16-bit 5-6-5 Color to fill with
*/
/**************************************************************************/
void CLCD_Driver::writeFastVLine(int16_t x, int16_t y, int16_t h,
                                  uint16_t color) {
  // Overwrite in subclasses if startWrite is defined!
  // Can be just writeLine(x, y, x, y+h-1, color);
  // or writeFillRect(x, y, 1, h, color);
  
  drawFastVLine(x, y, h, color);
}

/**************************************************************************/
/*!
    @brief  Quarter-circle drawer with fill, used for circles and roundrects
    @param  x0       Center-point x coordinate
    @param  y0       Center-point y coordinate
    @param  r        Radius of circle
    @param  corners  Mask bits indicating which quarters we're doing
    @param  delta    Offset from center-point, used for round-rects
    @param  color    16-bit 5-6-5 Color to fill with
*/
/**************************************************************************/
void CLCD_Driver::fillCircleHelper(int16_t x0, int16_t y0, int16_t r,
                                    uint8_t corners, int16_t delta,
                                    uint16_t color) {

  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;
  int16_t px = x;
  int16_t py = y;

  delta++; // Avoid some +1's in the loop

  while (x < y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;
    // These checks avoid double-drawing certain lines, important
    // for the SSD1306 library which has an INVERT drawing mode.
    if (x < (y + 1)) {
      if (corners & 1)
        writeFastVLine(x0 + x, y0 - y, 2 * y + delta, color);
      if (corners & 2)
        writeFastVLine(x0 - x, y0 - y, 2 * y + delta, color);
    }
    if (y != py) {
      if (corners & 1)
        writeFastVLine(x0 + py, y0 - px, 2 * px + delta, color);
      if (corners & 2)
        writeFastVLine(x0 - py, y0 - px, 2 * px + delta, color);
      py = y;
    }
    px = x;
  }
}

/**************************************************************************/
/*!
   @brief    Draw a perfectly horizontal line (this is often optimized in a
   subclass!)
    @param    x   Left-most x coordinate
    @param    y   Left-most y coordinate
    @param    w   Width in pixels
   @param    color 16-bit 5-6-5 Color to fill with
*/
/**************************************************************************/
void CLCD_Driver::drawFastHLine(int16_t x, int16_t y, int16_t w,
                                 uint16_t color) {
  // startWrite();
  writeLine(x, y, x + w - 1, y, color);
  // endWrite();
}

void CLCD_Driver::writePixel(int16_t x, int16_t y, uint16_t color) {
  GUI_DrawPoint(x, y, color);
}


// Draw a character
/**************************************************************************/
/*!
   @brief   Draw a single character
    @param    x   Bottom left corner x coordinate
    @param    y   Bottom left corner y coordinate
    @param    c   The 8-bit font-indexed character (likely ascii)
    @param    color 16-bit 5-6-5 Color to draw chraracter with
    @param    bg 16-bit 5-6-5 Color to fill background with (if same as color,
   no background)
    @param    size_x  Font magnification level in X-axis, 1 is 'original' size
    @param    size_y  Font magnification level in Y-axis, 1 is 'original' size
*/
/**************************************************************************/
void CLCD_Driver::drawChar(int16_t x, int16_t y, unsigned char c,
                            uint16_t color, uint16_t bg, uint8_t size_x,
                            uint8_t size_y) 
{

  TFT_DBG_PRINT(String("\r\n CLCD_Driver::drawChar"));

  
  if (!gfxFont) { // 'Classic' built-in font

    if ((x >= _width) ||              // Clip right
        (y >= _height) ||             // Clip bottom
        ((x + 6 * size_x - 1) < 0) || // Clip left
        ((y + 8 * size_y - 1) < 0))   // Clip top
      return;

    if (!_cp437 && (c >= 176))
      c++; // Handle 'classic' charset behavior

    //startWrite();
    for (int8_t i = 0; i < 5; i++) { // Char bitmap = 5 columns
      uint8_t line = pgm_read_byte(&font[c * 5 + i]);
      for (int8_t j = 0; j < 8; j++, line >>= 1) {
        if (line & 1) {
          if (size_x == 1 && size_y == 1)
            writePixel(x + i, y + j, color);
          else
            writeFillRect(x + i * size_x, y + j * size_y, size_x, size_y,
                          color);
        } else if (bg != color) {
          if (size_x == 1 && size_y == 1)
            writePixel(x + i, y + j, bg);
          else
            writeFillRect(x + i * size_x, y + j * size_y, size_x, size_y, bg);
        }
      }
    }
    if (bg != color) { // If opaque, draw vertical line for last column
      if (size_x == 1 && size_y == 1)
        writeFastVLine(x + 5, y, 8, bg);
      else
        writeFillRect(x + 5 * size_x, y, size_x, 8 * size_y, bg);
    }
    //endWrite();

  } else { // Custom font

    // Character is assumed previously filtered by write() to eliminate
    // newlines, returns, non-printable characters, etc.  Calling
    // drawChar() directly with 'bad' characters of font may cause mayhem!

    c -= (uint8_t)pgm_read_byte(&gfxFont->first);
    GFXglyph *glyph = pgm_read_glyph_ptr(gfxFont, c);
    uint8_t *bitmap = pgm_read_bitmap_ptr(gfxFont);

    uint16_t bo = pgm_read_word(&glyph->bitmapOffset);
    uint8_t w = pgm_read_byte(&glyph->width), h = pgm_read_byte(&glyph->height);
    int8_t xo = pgm_read_byte(&glyph->xOffset),
           yo = pgm_read_byte(&glyph->yOffset);
    uint8_t xx, yy, bits = 0, bit = 0;
    int16_t xo16 = 0, yo16 = 0;

    if (size_x > 1 || size_y > 1) {
      xo16 = xo;
      yo16 = yo;
    }

    // Todo: Add character clipping here

    // NOTE: THERE IS NO 'BACKGROUND' COLOR OPTION ON CUSTOM FONTS.
    // THIS IS ON PURPOSE AND BY DESIGN.  The background color feature
    // has typically been used with the 'classic' font to overwrite old
    // screen contents with new data.  This ONLY works because the
    // characters are a uniform size; it's not a sensible thing to do with
    // proportionally-spaced fonts with glyphs of varying sizes (and that
    // may overlap).  To replace previously-drawn text when using a custom
    // font, use the getTextBounds() function to determine the smallest
    // rectangle encompassing a string, erase the area with fillRect(),
    // then draw new text.  This WILL infortunately 'blink' the text, but
    // is unavoidable.  Drawing 'background' pixels will NOT fix this,
    // only creates a new set of problems.  Have an idea to work around
    // this (a canvas object type for MCUs that can afford the RAM and
    // displays supporting setAddrWindow() and pushColors()), but haven't
    // implemented this yet.

    //startWrite();
    for (yy = 0; yy < h; yy++) {
      for (xx = 0; xx < w; xx++) {
        if (!(bit++ & 7)) {
          bits = pgm_read_byte(&bitmap[bo++]);
        }
        if (bits & 0x80) {
          if (size_x == 1 && size_y == 1) {
            writePixel(x + xo + xx, y + yo + yy, color);
          } else {
            writeFillRect(x + (xo16 + xx) * size_x, y + (yo16 + yy) * size_y,
                          size_x, size_y, color);
          }
        }
        bits <<= 1;
      }
    }
    //endWrite();

  } // End classic vs custom font
}

/**************************************************************************/
/*!
    @brief Set the font to display when print()ing, either custom or default
    @param  f  The GFXfont object, if NULL use built in 6x8 font
*/
/**************************************************************************/
void CLCD_Driver::setFont(const GFXfont *f) {
  if (f) {          // Font struct pointer passed in?
    if (!gfxFont) { // And no current font struct?
      // Switching from classic to new font behavior.
      // Move cursor pos down 6 pixels so it's on baseline.
      cursor_y += 6;
    }
  } else if (gfxFont) { // NULL passed.  Current font struct defined?
    // Switching from new to classic font behavior.
    // Move cursor pos up 6 pixels so it's at top-left of char.
    cursor_y -= 6;
  }
  gfxFont = (GFXfont *)f;
}


/**************************************************************************/
/*!
    @brief   Set text 'magnification' size. Each increase in s makes 1 pixel
   that much bigger.
    @param  s  Desired text size. 1 is default 6x8, 2 is 12x16, 3 is 18x24, etc
*/
/**************************************************************************/
void CLCD_Driver::setTextSize(uint8_t s) { setTextSize(s, s); }

/**************************************************************************/
/*!
    @brief   Set text 'magnification' size. Each increase in s makes 1 pixel
   that much bigger.
    @param  s_x  Desired text width magnification level in X-axis. 1 is default
    @param  s_y  Desired text width magnification level in Y-axis. 1 is default
*/
/**************************************************************************/
void CLCD_Driver::setTextSize(uint8_t s_x, uint8_t s_y) {
  textsize_x = (s_x > 0) ? s_x : 1;
  textsize_y = (s_y > 0) ? s_y : 1;
}


/**************************************************************************/
/*!
   @brief    Write a line.  Bresenham's algorithm - thx wikpedia
    @param    x0  Start point x coordinate
    @param    y0  Start point y coordinate
    @param    x1  End point x coordinate
    @param    y1  End point y coordinate
    @param    color 16-bit 5-6-5 Color to draw with
*/
/**************************************************************************/
void CLCD_Driver::writeLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
                             uint16_t color) {

  int16_t steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    _swap_int16_t(x0, y0);
    _swap_int16_t(x1, y1);
  }

  if (x0 > x1) {
    _swap_int16_t(x0, x1);
    _swap_int16_t(y0, y1);
  }

  int16_t dx, dy;
  dx = x1 - x0;
  dy = abs(y1 - y0);

  int16_t err = dx / 2;
  int16_t ystep;

  if (y0 < y1) {
    ystep = 1;
  } else {
    ystep = -1;
  }

  for (; x0 <= x1; x0++) {
    if (steep) {
      GUI_DrawPoint(y0, x0, color);
    } else {
      GUI_DrawPoint(x0, y0, color);
    }
    err -= dy;
    if (err < 0) {
      y0 += ystep;
      err += dx;
    }
  }
}


/**************************************************************************/
/*!
   @brief    Write a rectangle completely with one color, overwrite in
   subclasses if startWrite is defined!
    @param    x   Top left corner x coordinate
    @param    y   Top left corner y coordinate
    @param    w   Width in pixels
    @param    h   Height in pixels
   @param    color 16-bit 5-6-5 Color to fill with
*/
/**************************************************************************/
void CLCD_Driver::writeFillRect(int16_t x, int16_t y, int16_t w, int16_t h,
                                 uint16_t color) {
  // Overwrite in subclasses if desired!
  fillRect(x, y, w, h, color);
}


/**************************************************************************/
/*!
   @brief    Fill a rectangle completely with one color. Update in subclasses if
   desired!
    @param    x   Top left corner x coordinate
    @param    y   Top left corner y coordinate
    @param    w   Width in pixels
    @param    h   Height in pixels
   @param    color 16-bit 5-6-5 Color to fill with
*/
/**************************************************************************/
void CLCD_Driver::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) 
{
  for (int16_t i = x; i < x + w; i++) {
    writeFastVLine(i, y, h, color);
  }

}


/**************************************************************************/
/*!
   @brief   Draw a rounded rectangle with fill color
    @param    x   Top left corner x coordinate
    @param    y   Top left corner y coordinate
    @param    w   Width in pixels
    @param    h   Height in pixels
    @param    r   Radius of corner rounding
    @param    color 16-bit 5-6-5 Color to draw/fill with
*/
/**************************************************************************/
void CLCD_Driver::fillRoundRect(int16_t x, int16_t y, int16_t w, int16_t h,
                                 int16_t r, uint16_t color) {
  int16_t max_radius = ((w < h) ? w : h) / 2; // 1/2 minor axis
  if (r > max_radius)
    r = max_radius;
  // smarter version
  writeFillRect(x + r, y, w - 2 * r, h, color);
  // draw four corners
  fillCircleHelper(x + w - r - 1, y + r, r, 1, h - 2 * r - 1, color);
  fillCircleHelper(x + r, y + r, r, 2, h - 2 * r - 1, color);
 
}
