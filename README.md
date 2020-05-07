# STM32F10x_DMA_LL_ST7735
Fast ST7735 DMA Library for STM32F10x LL (Low Layer drivers)

References
1. https://github.com/eziya/STM32_HAL_ILI9341
2. https://os.mbed.com/users/dreschpe/code/SPI_TFT_ILI9341/
3. https://www.mikroe.com/glcd-font-creator

Library supports C font array created by GLCD Font Creator by MikroElektronika.<br>
https://www.mikroe.com/glcd-font-creator<br>

Please refer to the following link to get more details about fonts.<br>
https://os.mbed.com/users/dreschpe/code/SPI_TFT_ILI9341/<br>

[ How to add new fonts ]
1. Run GLCD Font Creator
2. Click File-New Font-Import An Existing System Font
3. Select font, style and size from font dialog.
4. GLCD Font Cretor makes Bitmap fonts
5. Click Export for GLCD menu
6. Select mikroC tab.
7. Copy generated code to fonts.c file
8. Modify data type from unsigned short to uint8_t
9. Add optional bytes (offset, width, height, bpl) to the array !!! IMPORTANT !!!
10. Add extern declaration to fonts.h file

