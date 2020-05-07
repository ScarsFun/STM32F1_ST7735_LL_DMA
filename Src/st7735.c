/*
 * Fast ST7735 DMA Library for STM32F10x LL (Low Layer drivers)
 *
 PB15 BL
 PB14 CS
 PB13 DC
 PB12 RST
 PA5  SCK
 PA7  MOSI
 */

#include "st7735.h"
#include "stdlib.h"

#define TFT_BL_H() LL_GPIO_SetOutputPin(ST7735_CS_GPIO_Port, ST7735_BL_Pin)
#define TFT_BL_L() LL_GPIO_ResetOutputPin(ST7735_CS_GPIO_Port, ST7735_BL_Pin)
#define TFT_CS_H() LL_GPIO_SetOutputPin(ST7735_CS_GPIO_Port, ST7735_CS_Pin)
#define TFT_CS_L() LL_GPIO_ResetOutputPin(ST7735_CS_GPIO_Port, ST7735_CS_Pin)
#define TFT_DC_D() LL_GPIO_SetOutputPin(ST7735_CS_GPIO_Port, ST7735_DC_Pin)
#define TFT_DC_C() LL_GPIO_ResetOutputPin(ST7735_CS_GPIO_Port, ST7735_DC_Pin)
#define TFT_RES_H() LL_GPIO_SetOutputPin(ST7735_CS_GPIO_Port, ST7735_RES_Pin)
#define TFT_RES_L() LL_GPIO_ResetOutputPin(ST7735_CS_GPIO_Port, ST7735_RES_Pin)

#define SWAP_INT16_T(a, b) \
    {                      \
        int16_t t = a;     \
        a = b;             \
        b = t;             \
    }
#define DELAY 0x80

#if defined(ST7735_1_8_DEFAULT_ORIENTATION) || defined(ST7735S_1_8_DEFAULT_ORIENTATION)
static uint8_t _data_rotation[4] = { ST7735_MADCTL_MX, ST7735_MADCTL_MY, ST7735_MADCTL_MV, ST7735_MADCTL_BGR };
#endif

#if defined(ST7735_1_44_DEFAULT_ORIENTATION) || defined(ST7735_MINI_DEFAULT_ORIENTATION)
static uint8_t _data_rotation[4] = { ST7735_MADCTL_MX, ST7735_MADCTL_MY, ST7735_MADCTL_MV, ST7735_MADCTL_BGR };
#endif

static uint8_t _value_rotation = 0;
static int16_t _height = ST7735_HEIGHT, _width = ST7735_WIDTH;
static uint8_t _xstart = ST7735_XSTART, _ystart = ST7735_YSTART;
volatile uint32_t flag_DMA_CH3_bsy = 0;

// based on Adafruit ST7735 library for Arduino
static const uint8_t
    init_cmds1[] = { // Init for 7735R, part 1 (red or green tab)
        15, // 15 commands in list:
        ST7735_SWRESET, DELAY, //  1: Software reset, 0 args, w/delay
        150, //     150 ms delay
        ST7735_SLPOUT, DELAY, //  2: Out of sleep mode, 0 args, w/delay
        255, //     500 ms delay
        ST7735_FRMCTR1, 3, //  3: Frame rate ctrl - normal mode, 3 args:
        0x01, 0x2C, 0x2D, //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
        ST7735_FRMCTR2, 3, //  4: Frame rate control - idle mode, 3 args:
        0x01, 0x2C, 0x2D, //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
        ST7735_FRMCTR3, 6, //  5: Frame rate ctrl - partial mode, 6 args:
        0x01, 0x2C, 0x2D, //     Dot inversion mode
        0x01, 0x2C, 0x2D, //     Line inversion mode
        ST7735_INVCTR, 1, //  6: Display inversion ctrl, 1 arg, no delay:
        0x07, //     No inversion
        ST7735_PWCTR1, 3, //  7: Power control, 3 args, no delay:
        0xA2,
        0x02, //     -4.6V
        0x84, //     AUTO mode
        ST7735_PWCTR2, 1, //  8: Power control, 1 arg, no delay:
        0xC5, //     VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
        ST7735_PWCTR3, 2, //  9: Power control, 2 args, no delay:
        0x0A, //     Opamp current small
        0x00, //     Boost frequency
        ST7735_PWCTR4, 2, // 10: Power control, 2 args, no delay:
        0x8A, //     BCLK/2, Opamp current small & Medium low
        0x2A,
        ST7735_PWCTR5, 2, // 11: Power control, 2 args, no delay:
        0x8A, 0xEE,
        ST7735_VMCTR1, 1, // 12: Power control, 1 arg, no delay:
        0x0E,
        ST7735_INVOFF, 0, // 13: Don't invert display, no args, no delay
        ST7735_MADCTL, 1, // 14: Memory access control (directions), 1 arg:
        ST7735_DATA_ROTATION, //     row addr/col addr, bottom to top refresh
        ST7735_COLMOD, 1, // 15: set color mode, 1 arg, no delay:
        0x05
    }, //     16-bit color

#if (defined(ST7735_IS_128X128) || defined(ST7735_IS_160X128))
    init_cmds2[] = { // Init for 7735R, part 2 (1.44" display)
        2, //  2 commands in list:
        ST7735_CASET, 4, //  1: Column addr set, 4 args, no delay:
        0x00, 0x00, //     XSTART = 0
        0x00, 0x7F, //     XEND = 127
        ST7735_RASET, 4, //  2: Row addr set, 4 args, no delay:
        0x00, 0x00, //     XSTART = 0
        0x00, 0x7F
    }, //     XEND = 127
#endif // ST7735_IS_128X128

#ifdef ST7735_IS_160X80
    init_cmds2[] = { // Init for 7735S, part 2 (160x80 display)
        3, //  3 commands in list:
        ST7735_CASET, 4, //  1: Column addr set, 4 args, no delay:
        0x00, 0x00, //     XSTART = 0
        0x00, 0x4F, //     XEND = 79
        ST7735_RASET, 4, //  2: Row addr set, 4 args, no delay:
        0x00, 0x00, //     XSTART = 0
        0x00, 0x9F, //     XEND = 159
        ST7735_INVON, 0
    }, //  3: Invert colors
#endif

    init_cmds3[] = { // Init for 7735R, part 3 (red or green tab)
        4, //  4 commands in list:
        ST7735_GMCTRP1, 16, //  1: Magical unicorn dust, 16 args, no delay:
        0x02, 0x1c, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2d, 0x29, 0x25, 0x2B, 0x39, 0x00, 0x01, 0x03, 0x10, ST7735_GMCTRN1, 16, //  2: Sparkles and rainbows, 16 args, no delay:
        0x03, 0x1d, 0x07, 0x06, 0x2E, 0x2C, 0x29, 0x2D, 0x2E, 0x2E, 0x37, 0x3F, 0x00, 0x00, 0x02, 0x10, ST7735_NORON, DELAY, //  3: Normal display on, no args, w/delay
        10, //     10 ms delay
        ST7735_DISPON, DELAY, //  4: Main screen turn on, no args w/delay
        100
    }; //     100 ms delay

//static void ST7735_GPIO_Init(void);
static void ST7735_WriteCommand(uint8_t cmd);
static void ST7735_WriteData(uint8_t* buff, size_t buff_size);
static void ST7735_ExecuteCommandList(const uint8_t* addr);
static void ST7735_SetAddressWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);

static void Send_Data8(uint8_t data);
static void Send_Data16(uint16_t data);
static void Send_DMA_Data16(uint16_t*, uint16_t);
static void Send_DMA_Data8(uint8_t*, uint16_t);

static void Send_DMA_Data16(uint16_t* buff, uint16_t dataSize)
{
    // LL_SPI_SetTransferBitOrder   (SPI1, LL_SPI_LSB_FIRST );
    LL_SPI_SetDataWidth(SPI1, LL_SPI_DATAWIDTH_16BIT);

    LL_SPI_Disable(SPI1);
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
    LL_DMA_ClearFlag_TC3(DMA1);
    LL_DMA_ClearFlag_TE3(DMA1);
    LL_SPI_EnableDMAReq_TX(SPI1);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_3);
    LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_3);
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, dataSize);
    LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_3, (uint32_t)buff, LL_SPI_DMA_GetRegAddr(SPI1), LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3));
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
    LL_SPI_Enable(SPI1);
    while (!flag_DMA_CH3_bsy) {
    }
    flag_DMA_CH3_bsy = 0;

    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
    LL_SPI_Disable(SPI1);
    LL_DMA_ClearFlag_TC3(DMA1);
    LL_DMA_ClearFlag_TE3(DMA1);
    LL_SPI_DisableDMAReq_TX(SPI1);
    LL_DMA_DisableIT_TC(DMA1, LL_DMA_CHANNEL_3);
    LL_DMA_DisableIT_TE(DMA1, LL_DMA_CHANNEL_3);
    LL_SPI_Enable(SPI1);
    LL_SPI_SetDataWidth(SPI1, LL_SPI_DATAWIDTH_8BIT);
    //LL_SPI_SetTransferBitOrder   (SPI1, LL_SPI_MSB_FIRST );
}

static void Send_DMA_Data8(uint8_t* buff, uint16_t dataSize)
{
    //LL_SPI_SetTransferBitOrder   (SPI1, LL_SPI_LSB_FIRST );
    //LL_SPI_SetDataWidth(SPI1, LL_SPI_DATAWIDTH_16BIT);

    LL_SPI_Disable(SPI1);
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
    LL_DMA_ClearFlag_TC3(DMA1);
    LL_DMA_ClearFlag_TE3(DMA1);
    LL_SPI_EnableDMAReq_TX(SPI1);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_3);
    LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_3);
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_3, dataSize);
    LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_3, (uint32_t)buff, LL_SPI_DMA_GetRegAddr(SPI1), LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3));
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
    LL_SPI_Enable(SPI1);
    while (!flag_DMA_CH3_bsy) {
    }
    flag_DMA_CH3_bsy = 0;

    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
    LL_SPI_Disable(SPI1);
    LL_DMA_ClearFlag_TC3(DMA1);
    LL_DMA_ClearFlag_TE3(DMA1);
    LL_SPI_DisableDMAReq_TX(SPI1);
    LL_DMA_DisableIT_TC(DMA1, LL_DMA_CHANNEL_3);
    LL_DMA_DisableIT_TE(DMA1, LL_DMA_CHANNEL_3);
    LL_SPI_Enable(SPI1);
    //LL_SPI_SetDataWidth(SPI1, LL_SPI_DATAWIDTH_8BIT);
    //LL_SPI_SetTransferBitOrder   (SPI1, LL_SPI_MSB_FIRST );
}

static void Send_Data8(uint8_t data)
{
    while (!LL_SPI_IsActiveFlag_TXE(SPI1)) {
    }
    LL_SPI_TransmitData8(SPI1, data);
    // while(!LL_SPI_IsActiveFlag_RXNE(SPI1)) {}
    // (void) SPI1->DR; //fake Rx read;
    while (LL_SPI_IsActiveFlag_BSY(SPI1))
        ;
}
static void Send_Data16(uint16_t data)
{
    while (!LL_SPI_IsActiveFlag_TXE(SPI1)) {
    }
    LL_SPI_TransmitData16(SPI1, data);
    while (!LL_SPI_IsActiveFlag_RXNE(SPI1)) {
    }
    // (void) SPI1->DR; //fake Rx read;
    //while (LL_SPI_IsActiveFlag_BSY(SPI1));
}

static void ST7735_Reset()
{
    TFT_RES_L();
    LL_mDelay(20);
    TFT_RES_H();
}

static void ST7735_WriteCommand(uint8_t cmd)
{
    TFT_DC_C();
    Send_Data8(cmd);
}

static void ST7735_WriteData(uint8_t* buff, size_t buff_size)
{

    TFT_DC_D();

    for (uint16_t index = 0; index < buff_size; index++) {
        Send_Data8(buff[index]);
    }
}

static void ST7735_ExecuteCommandList(const uint8_t* addr)
{
    uint8_t numCommands, numArgs;
    uint16_t ms;

    numCommands = *addr++;
    while (numCommands--) {
        uint8_t cmd = *addr++;
        ST7735_WriteCommand(cmd);

        numArgs = *addr++;
        // If high bit set, delay follows args
        ms = numArgs & DELAY;
        numArgs &= ~DELAY;
        if (numArgs) {
            ST7735_WriteData((uint8_t*)addr, numArgs);
            addr += numArgs;
        }

        if (ms) {
            ms = *addr++;
            if (ms == 255)
                ms = 500;
            LL_mDelay(ms);
        }
    }
}

static void ST7735_SetAddressWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
    // column address set
    ST7735_WriteCommand(ST7735_CASET);
    uint8_t data[] = { 0x00, x0 + _xstart, 0x00, x1 + _xstart };
    ST7735_WriteData(data, sizeof(data));

    // row address set
    ST7735_WriteCommand(ST7735_RASET);
    data[1] = y0 + _ystart;
    data[3] = y1 + _ystart;
    ST7735_WriteData(data, sizeof(data));

    // write to RAM
    ST7735_WriteCommand(ST7735_RAMWR);
}

void ST7735_Init()
{
    //ST7735_GPIO_Init();
    TFT_CS_L();
    ST7735_Reset();
    ST7735_ExecuteCommandList(init_cmds1);
    ST7735_ExecuteCommandList(init_cmds2);
    ST7735_ExecuteCommandList(init_cmds3);
    TFT_CS_H();
}

void ST7735_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
    if ((x >= _width) || (y >= _height))
        return;

    TFT_CS_L();

    ST7735_SetAddressWindow(x, y, x + 1, y + 1);
    uint8_t data[] = { color >> 8, color & 0xFF };
    ST7735_WriteData(data, sizeof(data));

    TFT_CS_H();
}

void ST7735_FillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    // clipping
    if ((x >= _width) || (y >= _height))
        return;
    if ((x + w - 1) >= _width)
        w = _width - x;
    if ((y + h - 1) >= _height)
        h = _height - y;

    TFT_CS_L();
    ST7735_SetAddressWindow(x, y, x + w - 1, y + h - 1);
    TFT_DC_D();
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MEMORY_NOINCREMENT);
    uint16_t tbuf = color;
    Send_DMA_Data16(&tbuf, w * h);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MEMORY_INCREMENT);
    TFT_CS_H();

    /*
    TFT_DC_D();
    uint16_t tbuf[w];
    for (int x = w ; x >= 0; x--) 
            tbuf[x] = color;
    for (y = h; y > 0; y--) 
        Send_DMA_Data16(tbuf,sizeof(tbuf)/2);
    TFT_CS_H();
    */
}

void ST7735_FillScreen(uint16_t color)
{
    ST7735_FillRectangle(0, 0, _width, _height, color);
}

void ST7735_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* data)
{
    if ((x >= _width) || (y >= _height))
        return;
    if ((x + w - 1) >= _width)
        return;
    if ((y + h - 1) >= _height)
        return;

    TFT_CS_L();
    ST7735_SetAddressWindow(x, y, x + w - 1, y + h - 1);
    TFT_DC_D();
    Send_DMA_Data16((uint16_t*)data, w * h);
    TFT_CS_H();
}

void ST7735_InvertColors(bool invert)
{
    TFT_CS_L();
    ST7735_WriteCommand(invert ? ST7735_INVON : ST7735_INVOFF);
    TFT_CS_H();
}

void ST7735_Backlight_On(void)
{
    TFT_BL_H();
}

void ST7735_Backlight_Off(void)
{
    TFT_BL_L();
}

/***************************************************************************************
** Function name:           drawCircle
** Description:             Draw a circle outline
***************************************************************************************/
void ST7735_DrawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -r - r;
    int16_t x = 0;

    ST7735_DrawPixel(x0 + r, y0, color);
    ST7735_DrawPixel(x0 - r, y0, color);
    ST7735_DrawPixel(x0, y0 - r, color);
    ST7735_DrawPixel(x0, y0 + r, color);

    while (x < r) {
        if (f >= 0) {
            r--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        ST7735_DrawPixel(x0 + x, y0 + r, color);
        ST7735_DrawPixel(x0 - x, y0 + r, color);
        ST7735_DrawPixel(x0 - x, y0 - r, color);
        ST7735_DrawPixel(x0 + x, y0 - r, color);

        ST7735_DrawPixel(x0 + r, y0 + x, color);
        ST7735_DrawPixel(x0 - r, y0 + x, color);
        ST7735_DrawPixel(x0 - r, y0 - x, color);
        ST7735_DrawPixel(x0 + r, y0 - x, color);
    }
}

/***************************************************************************************
** Function name:           drawCircleHelper
** Description:             Support function for circle drawing
***************************************************************************************/
void ST7735_DrawCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint16_t color)
{
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;

    while (x < r) {
        if (f >= 0) {
            r--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;
        if (cornername & 0x8) {
            ST7735_DrawPixel(x0 - r, y0 + x, color);
            ST7735_DrawPixel(x0 - x, y0 + r, color);
        }
        if (cornername & 0x4) {
            ST7735_DrawPixel(x0 + x, y0 + r, color);
            ST7735_DrawPixel(x0 + r, y0 + x, color);
        }
        if (cornername & 0x2) {
            ST7735_DrawPixel(x0 + r, y0 - x, color);
            ST7735_DrawPixel(x0 + x, y0 - r, color);
        }
        if (cornername & 0x1) {
            ST7735_DrawPixel(x0 - x, y0 - r, color);
            ST7735_DrawPixel(x0 - r, y0 - x, color);
        }
    }
}

/***************************************************************************************
** Function name:           fillCircle
** Description:             draw a filled circle
***************************************************************************************/
void ST7735_FillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
    ST7735_DrawFastVLine(x0, y0 - r, r + r + 1, color);
    ST7735_FillCircleHelper(x0, y0, r, 3, 0, color);
}

/***************************************************************************************
** Function name:           fillCircleHelper
** Description:             Support function for filled circle drawing
***************************************************************************************/
// Used to do circles and roundrects
void ST7735_FillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, int16_t delta, uint16_t color)
{
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -r - r;
    int16_t x = 0;

    delta++;
    while (x < r) {
        if (f >= 0) {
            r--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        if (cornername & 0x1) {
            ST7735_DrawFastVLine(x0 + x, y0 - r, r + r + delta, color);
            ST7735_DrawFastVLine(x0 + r, y0 - x, x + x + delta, color);
        }
        if (cornername & 0x2) {
            ST7735_DrawFastVLine(x0 - x, y0 - r, r + r + delta, color);
            ST7735_DrawFastVLine(x0 - r, y0 - x, x + x + delta, color);
        }
    }
}

/***************************************************************************************
** Function name:           drawEllipse
** Description:             Draw a ellipse outline
***************************************************************************************/
void ST7735_DrawEllipse(int16_t x0, int16_t y0, int16_t rx, int16_t ry, uint16_t color)
{
    if (rx < 2)
        return;
    if (ry < 2)
        return;
    int16_t x, y;
    int32_t rx2 = rx * rx;
    int32_t ry2 = ry * ry;
    int32_t fx2 = 4 * rx2;
    int32_t fy2 = 4 * ry2;
    int32_t s;

    for (x = 0, y = ry, s = 2 * ry2 + rx2 * (1 - 2 * ry); ry2 * x <= rx2 * y; x++) {
        ST7735_DrawPixel(x0 + x, y0 + y, color);
        ST7735_DrawPixel(x0 - x, y0 + y, color);
        ST7735_DrawPixel(x0 - x, y0 - y, color);
        ST7735_DrawPixel(x0 + x, y0 - y, color);
        if (s >= 0) {
            s += fx2 * (1 - y);
            y--;
        }
        s += ry2 * ((4 * x) + 6);
    }

    for (x = rx, y = 0, s = 2 * rx2 + ry2 * (1 - 2 * rx); rx2 * y <= ry2 * x; y++) {
        ST7735_DrawPixel(x0 + x, y0 + y, color);
        ST7735_DrawPixel(x0 - x, y0 + y, color);
        ST7735_DrawPixel(x0 - x, y0 - y, color);
        ST7735_DrawPixel(x0 + x, y0 - y, color);
        if (s >= 0) {
            s += fy2 * (1 - x);
            x--;
        }
        s += rx2 * ((4 * y) + 6);
    }
}

/***************************************************************************************
** Function name:           fillEllipse
** Description:             draw a filled ellipse
***************************************************************************************/
void ST7735_FillEllipse(int16_t x0, int16_t y0, int16_t rx, int16_t ry, uint16_t color)
{
    if (rx < 2)
        return;
    if (ry < 2)
        return;
    int16_t x, y;
    int32_t rx2 = rx * rx;
    int32_t ry2 = ry * ry;
    int32_t fx2 = 4 * rx2;
    int32_t fy2 = 4 * ry2;
    int32_t s;

    for (x = 0, y = ry, s = 2 * ry2 + rx2 * (1 - 2 * ry); ry2 * x <= rx2 * y; x++) {
        ST7735_DrawFastHLine(x0 - x, y0 - y, x + x + 1, color);
        ST7735_DrawFastHLine(x0 - x, y0 + y, x + x + 1, color);

        if (s >= 0) {
            s += fx2 * (1 - y);
            y--;
        }
        s += ry2 * ((4 * x) + 6);
    }

    for (x = rx, y = 0, s = 2 * rx2 + ry2 * (1 - 2 * rx); rx2 * y <= ry2 * x; y++) {
        ST7735_DrawFastHLine(x0 - x, y0 - y, x + x + 1, color);
        ST7735_DrawFastHLine(x0 - x, y0 + y, x + x + 1, color);

        if (s >= 0) {
            s += fy2 * (1 - x);
            x--;
        }
        s += rx2 * ((4 * y) + 6);
    }
}

/***************************************************************************************
** Function name:           drawRect
** Description:             Draw a rectangle outline
***************************************************************************************/
// Draw a rectangle
void ST7735_DrawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    ST7735_DrawFastHLine(x, y, w, color);
    ST7735_DrawFastHLine(x, y + h - 1, w, color);
    ST7735_DrawFastVLine(x, y, h, color);
    ST7735_DrawFastVLine(x + w - 1, y, h, color);
}

/***************************************************************************************
** Function name:           drawRoundRect
** Description:             Draw a rounded corner rectangle outline
***************************************************************************************/
// Draw a rounded rectangle
void ST7735_DrawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color)
{
    // smarter version
    ST7735_DrawFastHLine(x + r, y, w - r - r, color); // Top
    ST7735_DrawFastHLine(x + r, y + h - 1, w - r - r, color); // Bottom
    ST7735_DrawFastVLine(x, y + r, h - r - r, color); // Left
    ST7735_DrawFastVLine(x + w - 1, y + r, h - r - r, color); // Right
    // draw four corners
    ST7735_DrawCircleHelper(x + r, y + r, r, 1, color);
    ST7735_DrawCircleHelper(x + r, y + h - r - 1, r, 8, color);
    ST7735_DrawCircleHelper(x + w - r - 1, y + r, r, 2, color);
    ST7735_DrawCircleHelper(x + w - r - 1, y + h - r - 1, r, 4, color);
}

/***************************************************************************************
** Function name:           fillRoundRect
** Description:             Draw a rounded corner filled rectangle
***************************************************************************************/
// Fill a rounded rectangle
void ST7735_FillRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color)
{
    // smarter version
    ST7735_FillRectangle(x + r, y, w - r - r, h, color);

    // draw four corners
    ST7735_FillCircleHelper(x + w - r - 1, y + r, r, 1, h - r - r - 1, color);
    ST7735_FillCircleHelper(x + r, y + r, r, 2, h - r - r - 1, color);
}

/***************************************************************************************
** Function name:           drawTriangle
** Description:             Draw a triangle outline using 3 arbitrary points
***************************************************************************************/
// Draw a triangle
void ST7735_DrawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color)
{
    ST7735_DrawLine(x0, y0, x1, y1, color);
    ST7735_DrawLine(x1, y1, x2, y2, color);
    ST7735_DrawLine(x2, y2, x0, y0, color);
}

/***************************************************************************************
** Function name:           fillTriangle
** Description:             Draw a filled triangle using 3 arbitrary points
***************************************************************************************/
// Fill a triangle - original Adafruit function works well and code footprint is small
void ST7735_FillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color)
{
    int16_t a, b, y, last;

    // Sort coordinates by Y order (y2 >= y1 >= y0)
    if (y0 > y1) {
        SWAP_INT16_T(y0, y1);
        SWAP_INT16_T(x0, x1);
    }

    if (y1 > y2) {
        SWAP_INT16_T(y2, y1);
        SWAP_INT16_T(x2, x1);
    }

    if (y0 > y1) {
        SWAP_INT16_T(y0, y1);
        SWAP_INT16_T(x0, x1);
    }

    if (y0 == y2) { // Handle awkward all-on-same-line case as its own thing
        a = b = x0;
        if (x1 < a)
            a = x1;
        else if (x1 > b)
            b = x1;
        if (x2 < a)
            a = x2;
        else if (x2 > b)
            b = x2;
        ST7735_DrawFastHLine(a, y0, b - a + 1, color);
        return;
    }

    int16_t
        dx01 = x1 - x0,
        dy01 = y1 - y0,
        dx02 = x2 - x0,
        dy02 = y2 - y0,
        dx12 = x2 - x1,
        dy12 = y2 - y1,
        sa = 0,
        sb = 0;

    // For upper part of triangle, find scanline crossings for segments
    // 0-1 and 0-2.  If y1=y2 (flat-bottomed triangle), the scanline y1
    // is included here (and second loop will be skipped, avoiding a /0
    // error there), otherwise scanline y1 is skipped here and handled
    // in the second loop...which also avoids a /0 error here if y0=y1
    // (flat-topped triangle).
    if (y1 == y2)
        last = y1; // Include y1 scanline
    else
        last = y1 - 1; // Skip it

    for (y = y0; y <= last; y++) {
        a = x0 + sa / dy01;
        b = x0 + sb / dy02;
        sa += dx01;
        sb += dx02;

        if (a > b)
            SWAP_INT16_T(a, b);
        ST7735_DrawFastHLine(a, y, b - a + 1, color);
    }

    // For lower part of triangle, find scanline crossings for segments
    // 0-2 and 1-2.  This loop is skipped if y1=y2.
    sa = dx12 * (y - y1);
    sb = dx02 * (y - y0);
    for (; y <= y2; y++) {
        a = x1 + sa / dy12;
        b = x0 + sb / dy02;
        sa += dx12;
        sb += dx02;

        if (a > b)
            SWAP_INT16_T(a, b);
        ST7735_DrawFastHLine(a, y, b - a + 1, color);
    }
}

/***************************************************************************************
** Function name:           drawLine
** Description:             draw a line between 2 arbitrary points
***************************************************************************************/

// Slower but more compact line drawing function
void ST7735_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
    int16_t steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep) {
        SWAP_INT16_T(x0, y0);
        SWAP_INT16_T(x1, y1);
    }

    if (x0 > x1) {
        SWAP_INT16_T(x0, x1);
        SWAP_INT16_T(y0, y1);
    }

    int16_t dx, dy;
    dx = x1 - x0;
    dy = abs(y1 - y0);

    int16_t err = dx / 2;
    int16_t ystep;

    if (y0 < y1) {
        ystep = 1;
    }
    else {
        ystep = -1;
    }

    for (; x0 <= x1; x0++) {
        if (steep) {
            ST7735_DrawPixel(y0, x0, color);
        }
        else {
            ST7735_DrawPixel(x0, y0, color);
        }
        err -= dy;
        if (err < 0) {
            y0 += ystep;
            err += dx;
        }
    }
}

/***************************************************************************************
** Function name:           drawFastVLine
** Description:             draw a vertical line
***************************************************************************************/
void ST7735_DrawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color)
{
    // Rudimentary clipping
    if ((x >= _width) || (y >= _height))
        return;
    if ((y + h - 1) >= _height)
        h = _height - y;

#ifdef USE_SPI_DMA
    ST7735_FillRectangle(x, y, 1, h, color);
#else
    ST7735_DrawLine(x, y, x, y + h - 1, color);
#endif
}

/***************************************************************************************
** Function name:           drawFastHLine
** Description:             draw a horizontal line
***************************************************************************************/
void ST7735_DrawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color)
{
    // Rudimentary clipping
    if ((x >= _width) || (y >= _height))
        return;
    if ((x + w - 1) >= _width)
        w = _width - x;

#ifdef USE_SPI_DMA
    ST7735_FillRectangle(x, y, w, 1, color);
#else
    ST7735_DrawLine(x, y, x + w - 1, y, color);
#endif
}

/***************************************************************************************
** Function name:           setRotation
** Description:             rotate the screen orientation m = 0-3
***************************************************************************************/
void ST7735_SetRotation(uint8_t m)
{
    _value_rotation = m % 4;

    TFT_CS_L();

    ST7735_WriteCommand(ST7735_MADCTL);

    switch (_value_rotation) {
    case 0: {
        uint8_t d_r = (_data_rotation[0] | _data_rotation[1] | _data_rotation[3]);
        ST7735_WriteData(&d_r, sizeof(d_r));
        _width = ST7735_WIDTH;
        _height = ST7735_HEIGHT;
        _xstart = ST7735_XSTART;
        _ystart = ST7735_YSTART;
    } break;
    case 1: {
        uint8_t d_r = (_data_rotation[1] | _data_rotation[2] | _data_rotation[3]);
        ST7735_WriteData(&d_r, sizeof(d_r));
        _width = ST7735_HEIGHT;
        _height = ST7735_WIDTH;
        _xstart = ST7735_YSTART;
        _ystart = ST7735_XSTART;
    } break;
    case 2: {
        uint8_t d_r = _data_rotation[3];
        ST7735_WriteData(&d_r, sizeof(d_r));
        _width = ST7735_WIDTH;
        _height = ST7735_HEIGHT;
        _xstart = ST7735_XSTART;
        _ystart = ST7735_YSTART;
    } break;
    case 3: {
        uint8_t d_r = (_data_rotation[0] | _data_rotation[2] | _data_rotation[3]);
        ST7735_WriteData(&d_r, sizeof(d_r));
        _width = ST7735_HEIGHT;
        _height = ST7735_WIDTH;
        _xstart = ST7735_YSTART;
        _ystart = ST7735_XSTART;
    } break;
    }
    TFT_CS_H();
}

uint8_t ST7735_GetRotation(void)
{
    return _value_rotation;
}

int16_t ST7735_GetHeight(void)
{
    return _height;
}

int16_t ST7735_GetWidth(void)
{
    return _width;
}

uint8_t get_bit_from_byte(uint8_t byte, uint8_t bit)
{
    return byte & (1 << bit);
}

void ST7735_DrawChar(char ch, const uint8_t font[], uint16_t X, uint16_t Y, uint16_t color, uint16_t bgcolor)
{
    if ((ch < 31) || (ch > 127))
        return;

    uint8_t fOffset, fWidth, fHeight, fBPL;
    uint8_t* tempChar;

    fOffset = font[0];
    fWidth = font[1];
    fHeight = font[2];
    fBPL = font[3];

    tempChar = (uint8_t*)&font[((ch - 0x20) * fOffset) + 4]; /* Current Character = Meta + (Character Index * Offset) */

    /* Clear background first */
    ST7735_FillRectangle(X, Y, fWidth, fHeight, bgcolor);

    for (int j = 0; j < fHeight; j++) {
        for (int i = 0; i < fWidth; i++) {
            uint8_t z = tempChar[fBPL * i + ((j & 0xF8) >> 3) + 1]; /* (j & 0xF8) >> 3, increase one by 8-bits */
            uint8_t b = 1 << (j & 0x07);
            if ((z & b) != 0x00) {
                ST7735_DrawPixel(X + i, Y + j, color);
            }
        }
    }
}

void ST7735_DrawText(const char* str, const uint8_t font[], uint16_t X, uint16_t Y, uint16_t color, uint16_t bgcolor)
{
    uint8_t charWidth; /* Width of character */
    uint8_t fOffset = font[0]; /* Offset of character */
    uint8_t fWidth = font[1]; /* Width of font */

    while (*str) {
        ST7735_DrawChar(*str, font, X, Y, color, bgcolor);

        /* Check character width and calculate proper position */
        uint8_t* tempChar = (uint8_t*)&font[((*str - 0x20) * fOffset) + 4];
        charWidth = tempChar[0];

        if (charWidth + 2 < fWidth) {
            /* If character width is smaller than font width */
            X += (charWidth + 2);
        }
        else {
            X += fWidth;
        }

        str++;
    }
}

uint16_t ST7735_Color565(uint8_t r, uint8_t g, uint8_t b)
{
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}
