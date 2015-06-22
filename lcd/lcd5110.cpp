//------------------------------------------------------------------------------
/* Nokia 5100 LCD display.
   Graphics driver and PCD8544 interface code for SparkFun's
   84x48 Graphic LCD.
   https://www.sparkfun.com/products/10168

  by: Jim Lindblom, Richard Pecl (accommodation for STM32)
    adapted from code by Nathan Seidle and mish-mashed with
    code from the ColorLCDShield.
  date: October 10, 2013
  license: Beerware. Feel free to use, reuse, and modify this
  code as you see fit. If you find it useful, and we meet someday,
  you can buy me a beer.

  This stuff could all be put into a library, but we wanted to
  leave it all in one sketch to keep it as transparent as possible.

  ------------------------------------------------------------------------------

    Graphic LCD Pin ------------- STM32 Pin
       RST       ----------------  B12
       CE        ----------------  B14
       D/C       ----------------  A8
       DIN(MOSI) ----------------  B15
       SCLK      ----------------  B13
       VCC       ----------------  3.3V
       GND       ----------------  GND
*/
//------------------------------------------------------------------------------

#include "FreeRTOS.h"
#include "task.h"

#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
//#include "stm32f10x_spi.h"

#include "bmpfonts.h"

#include "format.h"
#include "uart1.h"

#include "parameters.h"
#include "main.h"
#include "rf.h"
#include "gps.h"

//------------------------------------------------------------------------------

enum DisplayPage
{
  displ_test,
  displ_start,
  displ_navig,
  displ_rxstats,
};

static DisplayPage activePage = displ_navig;

//------------------------------------------------------------------------------

class Display
{
//------------------------------------------------------------------------------
public:

  static const bool white = false;
  static const bool black = true;

  enum Align
  {
    align_left,
    align_right,
    align_center,
  };

  //----------------------------------------------------------------------------

  static uint8_t Width()
  {
    return display_width;
  }

  static uint8_t Height()
  {
    return display_height;
  }

  static void SetFont(BmpFonts newFont)
  {
    font = &GetBmpFont(newFont);
    letterSpace = font->letterSpace;
    lineSpace = font->lineSpace;
  }

  static const BmpFont& Font()
  {
    return *font;
  }

  static uint8_t LetterSpace()
  {
    return letterSpace;
  }

  static uint8_t LineSpace()
  {
    return lineSpace;
  }

  static uint8_t LetterWidth()
  {
    return font->width + letterSpace;
  }

  static uint8_t LineHeight()
  {
    return font->height + lineSpace;
  }

  // This function sets a pixel on screenMap to your preferred
  // color.
  static void Pixel(int x, int y, bool bw = black)
  {
    // First, double check that the coordinate is in range.
    if ((x >= 0) && (x < Width()) && (y >= 0) && (y < Height()))
    {
      uint8_t shift = y % 8;

      if (bw) // If black, set the bit.
        screenMap[x + (y/8)*Width()] |= 1<<shift;
      else   // If white clear the bit.
        screenMap[x + (y/8)*Width()] &= ~(1<<shift);
    }
  }

  // Line draws a line from x0,y0 to x1,y1 with the set color.
  // This function was grabbed from the SparkFun ColorLCDShield
  // library.
  static void Line(int x0, int y0, int x1, int y1, bool bw = black)
  {
    int dy = y1 - y0; // Difference between y0 and y1
    int dx = x1 - x0; // Difference between x0 and x1
    int stepx, stepy;

    if (dy < 0)
    {
      dy = -dy;
      stepy = -1;
    }
    else
      stepy = 1;

    if (dx < 0)
    {
      dx = -dx;
      stepx = -1;
    }
    else
      stepx = 1;

    dy <<= 1; // dy is now 2*dy
    dx <<= 1; // dx is now 2*dx
    Pixel(x0, y0, bw); // Draw the first pixel.

    if (dx > dy)
    {
      int fraction = dy - (dx >> 1);
      while (x0 != x1)
      {
        if (fraction >= 0)
        {
          y0 += stepy;
          fraction -= dx;
        }
        x0 += stepx;
        fraction += dy;
        Pixel(x0, y0, bw);
      }
    }
    else
    {
      int fraction = dx - (dy >> 1);
      while (y0 != y1)
      {
        if (fraction >= 0)
        {
          x0 += stepx;
          fraction -= dy;
        }
        y0 += stepy;
        fraction += dx;
        Pixel(x0, y0, bw);
      }
    }
  }

  // Rect will draw a rectangle from x0,y0 top-left corner to
  // a x1,y1 bottom-right corner. Can be filled with the fill
  // parameter, and colored with bw.
  // This function was grabbed from the SparkFun ColorLCDShield
  // library.
  static void Rect(int x0, int y0, int x1, int y1, bool fill = true, bool bw = black)
  {
    // check if the rectangle is to be filled
    if (fill == 1)
    {
      int xDiff;

      if(x0 > x1)
        xDiff = x0 - x1; //Find the difference between the x vars
      else
        xDiff = x1 - x0;

      while(xDiff > 0)
      {
        Line(x0, y0, x0, y1, bw);

        if(x0 > x1)
          x0--;
        else
          x0++;

        xDiff--;
      }
    }
    else
    {
      // best way to draw an unfilled rectangle is to draw four lines
      Line(x0, y0, x1, y0, bw);
      Line(x0, y1, x1, y1, bw);
      Line(x0, y0, x0, y1, bw);
      Line(x1, y0, x1, y1, bw);
    }
  }

  // setCircle draws a circle centered around x0,y0 with a defined
  // radius. The circle can be black or white. And have a line
  // thickness ranging from 1 to the radius of the circle.
  // This function was grabbed from the SparkFun ColorLCDShield
  // library.
  static void Circle(int x0, int y0, int radius, int lineThickness, bool bw = black)
  {
    for(int r = 0; r < lineThickness; r++)
    {
      int f = 1 - radius;
      int ddF_x = 0;
      int ddF_y = -2 * radius;
      int x = 0;
      int y = radius;

      Pixel(x0, y0 + radius, bw);
      Pixel(x0, y0 - radius, bw);
      Pixel(x0 + radius, y0, bw);
      Pixel(x0 - radius, y0, bw);

      while(x < y)
      {
        if(f >= 0)
        {
          y--;
          ddF_y += 2;
          f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x + 1;

        Pixel(x0 + x, y0 + y, bw);
        Pixel(x0 - x, y0 + y, bw);
        Pixel(x0 + x, y0 - y, bw);
        Pixel(x0 - x, y0 - y, bw);
        Pixel(x0 + y, y0 + x, bw);
        Pixel(x0 - y, y0 + x, bw);
        Pixel(x0 + y, y0 - x, bw);
        Pixel(x0 - y, y0 - x, bw);
      }
      radius--;
    }
  }

  // This function will draw a char (defined in the font table
  // near the beginning of this sketch) at a defined x and y).
  // The color can be either black (1) or white (0).
  static void Char(char character, int x, int y, bool bw = black)
  {
    uint16_t column; // temp byte to store character's column bitmap

    for (uint16_t i=0; i < font->width; i++) // width columns (x) per character
    {
      uint16_t ofs = (character - 0x20) * font->width + i;
      if (font->height <= 8)
        column = font->data[ofs];
      else
        column = (font->data[2*ofs] << 8) | font->data[2*ofs+1];
      for (int j=0; j < font->height; j++) // height rows (y) per character
      {
        if (column & (0x01 << j)) // test bits to set pixels
          Pixel(x+i, y+j, bw);
        else
          Pixel(x+i, y+j, !bw);
      }
    }
  }

  // String draws a string of characters, calling Char with
  // progressive coordinates until it's done.
  // This function was grabbed from the SparkFun ColorLCDShield
  // library.
  static void String(const char* dString, int x, int y, bool bw = black, Align align = align_left)
  {
    size_t len;

    // for right align at x, calculate leftmost coordinate
    if (align == align_right)
    {
      len = strlen(dString);
      x = x - len * LetterWidth() + letterSpace + 1;
    }
    // for center align at x, calculate leftmost coordinate
    else if (align == align_center)
    {
      len = strlen(dString);
      x = (x - len * LetterWidth() + letterSpace + 1) / 2;
    }

    while (*dString != 0x00) // loop until null terminator
    {
      Char(*dString++, x, y, bw);
      x += font->width;

      // inter-char space
      if ((x + letterSpace) <= Width())
      {
        Rect(x, y, x + letterSpace, y+font->height - 1, true, !bw);
        x += letterSpace;
      }

      // wrap around
      if ((x + font->width) > Width())
      {
        x = 0;
        y += LineHeight();
      }
    }
  }

  // This function will draw an array over the screen. (For now) the
  // array must be the same size as the screen, covering the entirety
  // of the display.
  static void Bitmap(char* bitArray)
  {
    for (int i=0; i<(Width() * Height() / 8); i++)
      screenMap[i] = bitArray[i];
  }

  // This function clears the entire display either white (0) or
  // black (1).
  // The screen won't actually clear until you call Update()!
  static void Clear(bool bw)
  {
    for (int i=0; i<(Width() * Height() / 8); i++)
    {
      if (bw)
        screenMap[i] = 0xFF;
      else
        screenMap[i] = 0;
    }
  }

  // Helpful function to directly command the LCD to go to a
  // specific x,y coordinate.
  static void GotoXY(int x, int y)
  {
    DevWrite(tx_cmd, 0x80 | x);  // Column.
    DevWrite(tx_cmd, 0x40 | y);  // Row.
  }

  // This will actually draw on the display, whatever is currently
  // in the screenMap array.
  static void Update()
  {
    // Prevent the RTOS kernel swapping out the task.
    GotoXY(0, 0);
    for (int i=0; i < (Width() * Height() / 8); i++)
    {
      DevWrite(tx_data, screenMap[i]);
    }
  }

  // Set contrast can set the LCD Vop to a value between 0 and 127.
  // 40-60 is usually a pretty good range.
  static void SetContrast(uint8_t contrast)
  {
    DevWrite(tx_cmd, 0x21); //Tell LCD that extended commands follow
    DevWrite(tx_cmd, 0x80 | contrast); //Set LCD Vop (Contrast): Try 0xB1(good @ 3.3V) or 0xBF if your display is too dark
    DevWrite(tx_cmd, 0x20); //Set display mode
  }

  /* There are two ways to do this. Either through direct commands
  to the display, or by swapping each bit in the screenMap array.
  We'll leave both methods here, comment one or the other out if
  you please. */
  static void Invert()
  {
    /* Direct LCD Command option
    DevWrite(tx_cmd, 0x20); //Tell LCD that extended commands follow
    DevWrite(tx_cmd, 0x08 | 0x05); //Set LCD Vop (Contrast): Try 0xB1(good @ 3.3V) or 0xBF if your display is too dark
    DevWrite(tx_cmd, 0x20); //Set display mode  */

    /* Indirect, swap bits in screenMap option: */
    for (int i=0; i < (Width() * Height() / 8); i++)
    {
      screenMap[i] = ~screenMap[i] & 0xFF;
    }
    Update();
  }

  //------------------------------------------------------------------------------

  // There are two memory banks in the LCD, data/RAM and commands.
  // This function sets the DC pin high or low depending, and then
  // sends the data byte
  //This sends the magical commands to the PCD8544
  static void Init()
  {
    //Configure control pins
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    //RCC_APB2PeriphClockCmd (RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    //RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2 ,ENABLE);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12 | GPIO_Pin_14;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13 | GPIO_Pin_15;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_SetBits(GPIOA, GPIO_Pin_8);
    GPIO_SetBits(GPIOB, GPIO_Pin_13 | GPIO_Pin_15);

    //Reset the device to a known state
    DevDeselect();
    DevReset();

    DevWrite(tx_cmd, 0x21); //Tell LCD extended commands follow
    DevWrite(tx_cmd, 0xC6); //Set LCD Vop (Contrast)
    DevWrite(tx_cmd, 0x06); //Set Temp coefficent
    DevWrite(tx_cmd, 0x15); //LCD bias mode 1:48 (try 0x13)
    DevWrite(tx_cmd, 0x20);
    DevWrite(tx_cmd, 0x0C); //Set display control, normal mode.
  }

//------------------------------------------------------------------------------
protected:

  static const uint8_t display_width  = 84; // x-coordinates go wide
  static const uint8_t display_height = 48; // y-coordinates go high

  static const BmpFont* font;
  static uint8_t letterSpace;
  static uint8_t lineSpace;

  /* The screenMap variable stores a buffer representation of the
  pixels on our display. There are 504 total bits in this array,
  same as how many pixels there are on a 84 x 48 display.

  Each byte in this array covers a 8-pixel vertical block on the
  display. Each successive byte covers the next 8-pixel column over
  until you reach the right-edge of the display and step down 8 rows.

  To update the display, we first have to write to this array, then
  call the Display::Update() function, which sends this whole array
  to the PCD8544.

  Because the PCD8544 won't let us write individual pixels at a
  time, this is how we can make targeted changes to the display. */
  static uint8_t screenMap[display_width * display_height / 8];

  // Transfer type for LCD controller
  enum DevTxType
  {
    tx_data,
    tx_cmd,
  };

  //----------------------------------------------------------------------------

  static void DevSelect()
  {
    GPIO_ResetBits(GPIOB, GPIO_Pin_14);
  }

  static void DevDeselect()
  {
    GPIO_SetBits  (GPIOB, GPIO_Pin_14);
  }

  static void DevReset(void)
  {
    GPIO_ResetBits(GPIOB, GPIO_Pin_12);
    vTaskDelay(10);
    GPIO_SetBits  (GPIOB, GPIO_Pin_12);
  }

  static void DevDataTx(uint8_t data)
  {
    for(char i=0; i < 8; i++)
    {
      GPIO_WriteBit(GPIOB, GPIO_Pin_15, (data & 0x80) ? Bit_SET : Bit_RESET);
      data = data << 1;
      GPIO_WriteBit(GPIOB, GPIO_Pin_13, Bit_RESET);
      GPIO_WriteBit(GPIOB, GPIO_Pin_13, Bit_SET);
    }
  }

  static void DevWrite(DevTxType txType, uint8_t data)
  {
    //Tell the LCD that we are writing either to data or a command
    GPIO_WriteBit(GPIOA, GPIO_Pin_8, (txType == tx_data) ? Bit_SET : Bit_RESET);

    DevSelect();
    //SPI_I2S_SendData(SPI2, data);
    DevDataTx(data);
    DevDeselect();
  }

}; // Display

//static
const BmpFont* Display::font = &GetBmpFont(font_4x6);

//static
uint8_t Display::letterSpace = Display::font->letterSpace;

//static
uint8_t Display::lineSpace = Display::font->lineSpace;;

//static
uint8_t Display::screenMap[display_width * display_height / 8];


//------------------------------------------------------------------------------

// Display test
static void TestPage()
{
  Display::Clear(Display::white);

  for (int x=0; x<Display::Width(); x+=8)
  { // Swipe right black
    Display::Rect(0, 0, x, Display::Height(), 1, Display::black);
    Display::Update();
    vTaskDelay(40);
  }
  for (int x=0; x<Display::Width(); x+=8)
  { // Swipe right white
    Display::Rect(0, 0, x, Display::Height(), 1, Display::white);
    Display::Update();
    vTaskDelay(40);
  }
  for (int x=0; x<12; x++)
  { // Shutter swipe
    Display::Rect( 0, 0, x,    Display::Height(), 1, 1);
    Display::Rect(11, 0, x+12, Display::Height(), 1, Display::black);
    Display::Rect(23, 0, x+24, Display::Height(), 1, Display::black);
    Display::Rect(35, 0, x+36, Display::Height(), 1, Display::black);
    Display::Rect(47, 0, x+48, Display::Height(), 1, Display::black);
    Display::Rect(59, 0, x+60, Display::Height(), 1, Display::black);
    Display::Rect(71, 0, x+72, Display::Height(), 1, Display::black);
    Display::Update();
    vTaskDelay(50);
  }
  // 3 Dee!
  //~ Display::Rect(25, 15, 45, 35, 0, Display::white);
  //~ Display::Rect(35, 25, 55, 45, 0, Display::white);
  //~ Display::Line(25, 15, 35, 25, Display::white);
  //~ Display::Line(45, 35, 55, 45, Display::white);
  //~ Display::Line(25, 35, 35, 45, Display::white);
  //~ Display::Line(45, 15, 55, 25, Display::white);
  //~ Display::Update();

  vTaskDelay(2000);
}

//------------------------------------------------------------------------------

// LCD test
static void StartPage()
{
  Display::Clear(Display::white);
  //Display::Rect(0, 0, Display::Width()-1, y_ofs - 1, 1, Display::black);
  //     "----------------"
  Display::String("OGN Tracker v0.1", 0, 0, Display::white);
  Display::Update();
  vTaskDelay(1500);
}

//------------------------------------------------------------------------------

// Navigation info page (speed, altitude...)
// GPS||||  00:00:00
//  spd alt hdg
// gld CECECE
static void NavigPage()
{
  static uint8_t period = 0;
  char buf[50];
  char* text;
  uint8_t x, y, h;

  Display::Clear(Display::white);

  const OgnPosition& gps = GetGPSStatus();

  // blink 'GPS' text until 3-d fix
  if (gps.FixMode >= 3 || (period & 1) == 0)
  {
    text = buf;
    text += Format_String(text, "GPS");
    *text = '\0';
    Display::String(buf, 0, 0);
  }

  // GPS indicator
  x = 3 * Display::LetterWidth() + 1;
  h = Display::Font().height;
  switch (gps.FixMode)
  {
    case 3:
      if (gps.Satellites >= 6)
        Display::Line(x+9, 0, x+9, h-1);
      Display::Line(x+6, h/4, x+6, h-1);
      // fall through
    case 2:
      Display::Line(x+3, h/2, x+3, h-1);
      // fall through
    case 1:
      Display::Line(x+0, (3*h)/4, x+0, h-1);
      // fall through
    case 0:
    default:
      break;
  }

  if (gps.isTimeValid())
  {
    text = buf;
    text += Format_UnsDec(text, gps.Hour, 2);
    *text++ = ':';
    text += Format_UnsDec(text, gps.Min, 2);
    *text++ = ':';
    text += Format_UnsDec(text, gps.Sec, 2);
    *text = '\0';
    Display::String(buf, Display::Width() - 1, 0, Display::black, Display::align_right);
  }
  else
    Display::String("--:--:--", Display::Width() - 1, 0, Display::black, Display::align_right);

  Display::SetFont(font_7x15);

  y = 8;
  text = buf;
  text += Format_String(text, "alt ");
  if (gps.Altitude >= 0)
    text += Format_UnsDec(text, gps.Altitude / 10);
  else
    text += Format_String(text, "?", 4);
  text += Format_String(text, " m");
  *text = '\0';
  // display at center, but maintain digit and unit posistion when order changes
  Display::String(buf, (Display::Width()+5*Display::LetterWidth())/2, y, Display::black, Display::align_right);

  // "123456789"
  // "123kn 360"
  y += Display::Font().height;
  text = buf;
  if (gps.Speed >= 0)
    text += Format_UnsDec(text, gps.Speed / 10);
  else
    text += Format_String(text, "?");
  text += Format_String(text, "kn");
  *text = '\0';
  Display::String(buf, 5*Display::LetterWidth() - Display::LetterSpace() - 1, y, Display::black, Display::align_right);

  text = buf;
  if (gps.Heading >= 0)
    text += Format_UnsDec(text, gps.Heading / 10);
  else
    text += Format_String(text, "?");
  text += Format_String(text, "n");
  *text = '\0';
  Display::String(buf, Display::Width()-1, y, Display::black, Display::align_right);

  Display::SetFont(font_4x6);

  y = Display::Height() - Display::Font().height;
  text = buf;
  text += Format_String(text, GetAcftTypeShort(Parameters.getAcftType()));
  *text++ = ' ';
  text += Format_Hex(text, Parameters.getAddress(), 6);
  *text = '\0';
  Display::String(buf, 0, y);

  Display::Update();

  period++;
}

//------------------------------------------------------------------------------

// Show status page (GPS & Rx stats)
static void RxStatsPage()
{
  char buf[50];
  char* text;
  uint8_t y = 0;

  Display::Clear(Display::white);

  text = buf;
  text += Format_Hex(text, Parameters.getAddress(), 6);
  *text = '\0';
  Display::String(buf, 0, y);
  y += Display::LineHeight() + 4;

  const RFStatus& rf = GetRFStatus();

  // "----------------"
  // "rx=25 l=25 +14dB"
  text = buf;
  text += Format_String(text, "rx=");
  text += Format_UnsDec(text, rf.RX_Packets % 100, 2);
  text += Format_String(text, " l=");
  text += Format_UnsDec(text, rf.RX_Idle % 100, 2);
  text += Format_String(text, " ");
  text += Format_SignDec(text, Parameters.getTxPower());
  text += Format_String(text, "dB");
  *text = '\0';
  Display::String(buf, 0, y);
  y += Display::LineHeight();

  text = buf;
  text += Format_String(text, " ssL=");
  text += Format_UnsDec(text, rf.RX_RssiLow, 3);
  text += Format_String(text, " ssU=");
  text += Format_UnsDec(text, rf.RX_RssiUpp, 3);
  *text = '\0';
  Display::String(buf, 0, y);
  y += Display::LineHeight();

  const OgnPosition& gps = GetGPSStatus();
  text = buf;
  text += Format_String(text, "fix=");
  text += Format_UnsDec(text, gps.FixMode);
  if (gps.FixMode > 0)
  {
    text += Format_String(text, " s=");
    text += Format_UnsDec(text, gps.Satellites);
  }
  *text = '\0';
  Display::String(buf, 0, y);
  y += Display::LineHeight();

  if (gps.isDateValid())
  {
    text = buf;
    text += Format_UnsDec(text, gps.Day);
    *text++ = '.';
    text += Format_UnsDec(text, gps.Month);
    *text++ = '.';
    *text = '\0';
    Display::String(buf, 0, y);
  }

  if (gps.isTimeValid())
  {
    text = buf;
    text += Format_UnsDec(text, gps.Hour, 2);
    *text++ = ':';
    text += Format_UnsDec(text, gps.Min, 2);
    *text++ = ':';
    text += Format_UnsDec(text, gps.Sec, 2);
    *text = '\0';
    Display::String(buf, Display::Width() - 1, y, Display::black, Display::align_right);
  }
  y += Display::LineHeight();

  Display::Update();
}

//------------------------------------------------------------------------------

extern "C"
void vTaskLcd(void* pvParameters)
{
  xSemaphoreTake(UART1_Mutex, portMAX_DELAY);
  Format_String(UART1_Write, "TaskLCD: 5110\n");
  xSemaphoreGive(UART1_Mutex);

  Display::Init(); // This will setup our pins, and initialize the display
  Display::SetContrast(55); // Pretty good value, play around with it

  //TestPage();
  StartPage();

  while (true)
  {
    switch (activePage)
    {
      default:
      case displ_navig:   NavigPage();   break;
      case displ_rxstats: RxStatsPage(); break;
    }
    vTaskDelay(1000);
  }

  Display::Clear(Display::white);
  Display::Update();
}
