#ifndef LCD_DRIVER_H
#define LCD_DRIVER_H

#if ARDUINO >= 100
#include "Arduino.h"
#include "Print.h"
#else
#include "WProgram.h"
#endif

#include "gfxfont.h"
#include <cstdint>
#include <stdint.h>

//#define TFT_DBG_PRINT(x)  Serial.println(x)
#define TFT_DBG_PRINT(x)

// typedef unsigned char uint8_t;
// typedef unsigned short uint16_t;
// typedef unsigned long uint32_t;
// typedef unsigned long long uint64_t;

#define LCD_D0_PIN	12
#define LCD_D1_PIN	13
#define LCD_D2_PIN	26
#define LCD_D3_PIN	25
#define LCD_D4_PIN	17
#define LCD_D5_PIN	16
#define LCD_D6_PIN	27
#define LCD_D7_PIN	14

#define LCD_RD_PIN	2
#define LCD_WR_PIN	4
#define LCD_RS_PIN	15
#define LCD_CS_PIN	33
#define LCD_RESET_PIN 32

  
// #include <ardunio.h>

/**
 * @brief Configuration of i2s lcd mode
 * 
 */
typedef struct {
    uint8_t data_width;           /*!< Parallel data width, 16bit or 8bit available */
    uint8_t pin_data_num[16];     /*!< Parallel data output IO*/
    uint8_t pin_num_wr;           /*!< Write clk io*/
    uint8_t pin_num_rd;
    uint8_t pin_num_cs;           /*!< CS io num */
    uint8_t pin_num_rs;           /*!< RS io num */
	uint8_t pin_num_rst;
	int clk_freq;                /*!< I2s clock frequency */
    //i2s_port_t i2s_port;         /*!< I2S port number */
    bool swap_data;              /*!< Swap the 2 bytes of RGB565 color */
    uint32_t buffer_size;        /*!< DMA buffer size */
} i2s_lcd_config_t;


typedef struct  
{										    
	uint16_t width;			//LCD 宽度
	uint16_t height;			//LCD 高度
	uint16_t id;				//LCD ID
	uint8_t  dir;			//横屏还是竖屏控制：0，竖屏；1，横屏。	
	uint16_t	 wramcmd;		//开始写gram指令
	uint16_t  rramcmd;   //开始读gram指令
	uint16_t  setxcmd;		//设置x坐标指令
	uint16_t  setycmd;		//设置y坐标指令	 
}_lcd_dev; 	

#define USE_HORIZONTAL  	0 
#define LCD_USE8BIT_MODEL   1

#define LCD_W 320
#define LCD_H 480

#define ILI9341_TFTWIDTH 480  ///< ILI9341 max TFT width
#define ILI9341_TFTHEIGHT 320 ///< ILI9341 max TFT height

#define ILI9341_NOP 0x00     ///< No-op register
#define ILI9341_SWRESET 0x01 ///< Software reset register
#define ILI9341_RDDID 0x04   ///< Read display identification information
#define ILI9341_RDDST 0x09   ///< Read Display Status

#define ILI9341_SLPIN 0x10  ///< Enter Sleep Mode
#define ILI9341_SLPOUT 0x11 ///< Sleep Out
#define ILI9341_PTLON 0x12  ///< Partial Mode ON
#define ILI9341_NORON 0x13  ///< Normal Display Mode ON

#define ILI9341_RDMODE 0x0A     ///< Read Display Power Mode
#define ILI9341_RDMADCTL 0x0B   ///< Read Display MADCTL
#define ILI9341_RDPIXFMT 0x0C   ///< Read Display Pixel Format
#define ILI9341_RDIMGFMT 0x0D   ///< Read Display Image Format
#define ILI9341_RDSELFDIAG 0x0F ///< Read Display Self-Diagnostic Result

#define ILI9341_INVOFF 0x20   ///< Display Inversion OFF
#define ILI9341_INVON 0x21    ///< Display Inversion ON
#define ILI9341_GAMMASET 0x26 ///< Gamma Set
#define ILI9341_DISPOFF 0x28  ///< Display OFF
#define ILI9341_DISPON 0x29   ///< Display ON

#define ILI9341_CASET 0x2A ///< Column Address Set
#define ILI9341_PASET 0x2B ///< Page Address Set
#define ILI9341_RAMWR 0x2C ///< Memory Write
#define ILI9341_RAMRD 0x2E ///< Memory Read

#define ILI9341_PTLAR 0x30    ///< Partial Area
#define ILI9341_VSCRDEF 0x33  ///< Vertical Scrolling Definition
#define ILI9341_MADCTL 0x36   ///< Memory Access Control
#define ILI9341_VSCRSADD 0x37 ///< Vertical Scrolling Start Address
#define ILI9341_PIXFMT 0x3A   ///< COLMOD: Pixel Format Set

#define ILI9341_FRMCTR1                                                        \
  0xB1 ///< Frame Rate Control (In Normal Mode/Full Colors)
#define ILI9341_FRMCTR2 0xB2 ///< Frame Rate Control (In Idle Mode/8 colors)
#define ILI9341_FRMCTR3                                                        \
  0xB3 ///< Frame Rate control (In Partial Mode/Full Colors)
#define ILI9341_INVCTR 0xB4  ///< Display Inversion Control
#define ILI9341_DFUNCTR 0xB6 ///< Display Function Control

#define ILI9341_PWCTR1 0xC0 ///< Power Control 1
#define ILI9341_PWCTR2 0xC1 ///< Power Control 2
#define ILI9341_PWCTR3 0xC2 ///< Power Control 3
#define ILI9341_PWCTR4 0xC3 ///< Power Control 4
#define ILI9341_PWCTR5 0xC4 ///< Power Control 5
#define ILI9341_VMCTR1 0xC5 ///< VCOM Control 1
#define ILI9341_VMCTR2 0xC7 ///< VCOM Control 2

#define ILI9341_RDID1 0xDA ///< Read ID 1
#define ILI9341_RDID2 0xDB ///< Read ID 2
#define ILI9341_RDID3 0xDC ///< Read ID 3
#define ILI9341_RDID4 0xDD ///< Read ID 4

#define ILI9341_GMCTRP1 0xE0 ///< Positive Gamma Correction
#define ILI9341_GMCTRN1 0xE1 ///< Negative Gamma Correction
//#define ILI9341_PWCTR6     0xFC

// Color definitions
#define ILI9341_BLACK 0x0000       ///<   0,   0,   0
#define ILI9341_NAVY 0x000F        ///<   0,   0, 123
#define ILI9341_DARKGREEN 0x03E0   ///<   0, 125,   0
#define ILI9341_DARKCYAN 0x03EF    ///<   0, 125, 123
#define ILI9341_MAROON 0x7800      ///< 123,   0,   0
#define ILI9341_PURPLE 0x780F      ///< 123,   0, 123
#define ILI9341_OLIVE 0x7BE0       ///< 123, 125,   0
#define ILI9341_LIGHTGREY 0xC618   ///< 198, 195, 198
#define ILI9341_DARKGREY 0x7BEF    ///< 123, 125, 123
#define ILI9341_BLUE 0x001F        ///<   0,   0, 255
#define ILI9341_GREEN 0x07E0       ///<   0, 255,   0
#define ILI9341_CYAN 0x07FF        ///<   0, 255, 255
#define ILI9341_RED 0xF800         ///< 255,   0,   0
#define ILI9341_MAGENTA 0xF81F     ///< 255,   0, 255
#define ILI9341_YELLOW 0xFFE0      ///< 255, 255,   0
#define ILI9341_WHITE 0xFFFF       ///< 255, 255, 255
#define ILI9341_ORANGE 0xFD20      ///< 255, 165,   0
#define ILI9341_GREENYELLOW 0xAFE5 ///< 173, 255,  41
#define ILI9341_PINK 0xFC18        ///< 255, 130, 198


#ifndef min
#define min(a, b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef _swap_int16_t
#define _swap_int16_t(a, b)                                                    \
  {                                                                            \
    int16_t t = a;                                                             \
    a = b;                                                                     \
    b = t;                                                                     \
  }
#endif

class CLCD_Driver : public Print
{  

  int16_t _width;       ///< Display width as modified by current rotation
  int16_t _height;      ///< Display height as modified by current rotation
  int16_t cursor_x = 0;     ///< x location to start print()ing text
  int16_t cursor_y = 0;     ///< y location to start print()ing text
  bool wrap;            ///< If set, 'wrap' text at right edge of display
  bool _cp437;          ///< If set, use correct CP437 charset (default is off)
 uint8_t rotation;     ///< Display rotation (0 thru 3)
  uint8_t textsize_x;   ///< Desired magnification in X-axis of text to print()
  uint8_t textsize_y;   ///< Desired magnification in Y-axis of text to print()
 uint16_t textcolor;   ///< 16-bit background color for print()
 uint16_t textbgcolor; ///< 16-bit text color for print()
GFXfont *gfxFont = 0;

_lcd_dev m_lcddev;

void serialflash();     



private:

  i2s_lcd_config_t  m_sLcd;

// The class

public:             // Access specifier

  void fillScreen(uint16_t color);

	void WriteCommand(unsigned char yData);
	
	void WriteData(unsigned char yData);
	
	CLCD_Driver();
	~CLCD_Driver();

	void InitPins_mode();
	void LCD_cmd(uint8_t ycommand);
	void LCD_data(uint8_t data);
	
  void invertDisplay(bool invert) ;

	void init();
	
	void LCD_WR_DATA_16Bit(uint16_t Data);
	
	void LCD_Clear(uint16_t Color);
	void LCD_DRAW_LINE(uint16_t Color);
	void LCD_Draw_rect(int xorg, int yorg, int rect_width,int rect_height,uint16_t Color);
	void reg_lcd_write_data(unsigned char *data, uint8_t size);
	
	void LCD_SetWindows(uint16_t xStar, uint16_t yStar,uint16_t xEnd,uint16_t yEnd);
	
	void LCD_WriteRAM_Prepare(void);
	
	void LCD_WriteReg(uint16_t LCD_Reg, uint16_t LCD_RegValue);
	
	void LCD_direction(uint8_t direction);
	
	void GUI_DrawPoint(uint16_t x,uint16_t y,uint16_t color);

  void writePixel(int16_t x, int16_t y, uint16_t color);

    void LCD_DrawPoint(uint16_t x,uint16_t y);

    void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos);

    void fillRoundRect(int16_t x, int16_t y, int16_t w, int16_t h,
                                 int16_t r, uint16_t color);

   void writeLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
                             uint16_t color);

   void writeFillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);

   void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);

   void drawFastVLine(int16_t x, int16_t y, int16_t h,   uint16_t color);

   void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) ;

public:
	
  /**********************************************************************/
  /*!
    @brief   Set text font color with transparant background
    @param   c   16-bit 5-6-5 Color to draw text with
    @note    For 'transparent' background, background and foreground
             are set to same color rather than using a separate flag.
  */
  /**********************************************************************/
  void setTextColor(uint16_t c) { textcolor = textbgcolor = c; }

  /**********************************************************************/
  /*!
    @brief   Set text font color with custom background color
    @param   c   16-bit 5-6-5 Color to draw text with
    @param   bg  16-bit 5-6-5 Color to draw background/fill with
  */
  /**********************************************************************/
  void setTextColor(uint16_t c, uint16_t bg) {
    textcolor = c;
    textbgcolor = bg;
  }

  /**********************************************************************/
  /*!
    @brief  Set text cursor location
    @param  x    X coordinate in pixels
    @param  y    Y coordinate in pixels
  */
  /**********************************************************************/
  void setCursor(int16_t x, int16_t y) {
    cursor_x = x;
    cursor_y = y;
  }
  size_t write(uint8_t c);

   void LCD_ShowString(uint16_t x,uint16_t y, uint8_t uSize, uint8_t *p, uint8_t mode);

   void LCD_ShowChar(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, uint8_t num,uint8_t size,uint8_t mode);
	
	
	void writeFastVLine(int16_t x, int16_t y, int16_t h,
                                  uint16_t color);
	
	void drawChar(int16_t x, int16_t y, unsigned char c,
                            uint16_t color, uint16_t bg, uint8_t size_x,
                            uint8_t size_y) ;
							
	void setFont(const GFXfont *f = 0);

  void setTextSize(uint8_t s);

  void setTextSize(uint8_t s_x, uint8_t s_y);

  void fillCircleHelper(int16_t x0, int16_t y0, int16_t r,
                                    uint8_t corners, int16_t delta,
                                    uint16_t color);

private:
 uint8_t _textsize_x;
  uint8_t _textsize_y;


};

#endif  //LCD_DRIVER_H
