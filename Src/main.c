/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Fast ST7735 DMA Library for STM32F10x LL (Low Layer drivers)
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "st7735.h"
#include "testimg.h"
#include <stdlib.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t r = 0, randomChar, nchars;
extern volatile uint32_t flag_DMA_CH3_bsy;
uint32_t ticksStart, ticksEnd;
char str_time[10];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void demo2(void)
{
    ST7735_FillScreen(ST7735_BLACK);
    ST7735_FillRectangle(0, 0, 127, 53, ST7735_RED);
    ST7735_DrawText("RED", FONT5, 15, 20, ST7735_WHITE, ST7735_RED);
    ST7735_FillRectangle(0, 53, 127, 106, ST7735_BLUE);
    ST7735_DrawText("BLUE", FONT5, 15, 60, ST7735_WHITE, ST7735_BLUE);
    ST7735_FillRectangle(0, 106, 127, 160, ST7735_GREEN);
    ST7735_DrawText("GREEN", FONT5, 15, 120, ST7735_WHITE, ST7735_GREEN);
    LL_mDelay(2000);
}

void demoTFT(void)
{
    ST7735_SetRotation(r);

    ST7735_FillScreen(ST7735_BLACK);

    ST7735_FillScreen(ST7735_BLACK);

    for (int i = 0; i < ST7735_GetHeight(); i += 4) {
        ST7735_DrawFastHLine(0, i, ST7735_GetWidth() - 1, ST7735_WHITE);
    }

    for (int i = 0; i < ST7735_GetWidth(); i += 4) {
        ST7735_DrawFastVLine(i, 0, ST7735_GetHeight() - 1, ST7735_WHITE);
    }

    LL_mDelay(2000);

    // Check colors
    ST7735_FillScreen(ST7735_BLACK);
    ticksEnd = SysTick->VAL;
    ST7735_DrawText("BLACK", FONT5, 5, 1, ST7735_WHITE, ST7735_BLACK);
    LL_mDelay(500);

    ST7735_FillScreen(ST7735_Color565(0, 0, 255));
    ST7735_DrawText("BLUE", FONT5, 5, 1, ST7735_WHITE, ST7735_BLUE);
    LL_mDelay(500);

    ST7735_FillScreen(ST7735_Color565(255, 0, 0));
    ST7735_DrawText("RED", FONT5, 5, 1, ST7735_WHITE, ST7735_RED);
    LL_mDelay(500);

    ST7735_FillScreen(ST7735_Color565(0, 255, 0));
    ;
    ST7735_DrawText("GREEN", FONT5, 5, 1, ST7735_WHITE, ST7735_GREEN);
    LL_mDelay(500);

    ST7735_FillScreen(ST7735_CYAN);
    ST7735_DrawText("CYAN", FONT5, 5, 1, ST7735_WHITE, ST7735_CYAN);
    LL_mDelay(500);

    ST7735_FillScreen(ST7735_MAGENTA);
    ST7735_DrawText("MAGENTA", FONT5, 5, 1, ST7735_WHITE, ST7735_MAGENTA);
    LL_mDelay(500);

    ST7735_FillScreen(ST7735_YELLOW);
    ST7735_DrawText("YELLOW", FONT5, 5, 1, ST7735_WHITE, ST7735_YELLOW);
    LL_mDelay(500);

    ST7735_FillScreen(ST7735_WHITE);
    ST7735_DrawText("WHITE", FONT5, 5, 1, ST7735_BLACK, ST7735_WHITE);
    LL_mDelay(500);

    // Draw circles
    ST7735_FillScreen(ST7735_BLACK);
    for (int i = 0; i < ST7735_GetHeight() / 2; i += 2) {
        ST7735_DrawCircle(ST7735_GetWidth() / 2, ST7735_GetHeight() / 2, i, ST7735_YELLOW);
    }
    LL_mDelay(1000);

    ST7735_FillScreen(ST7735_BLACK);
    ST7735_FillTriangle(0, 0, ST7735_GetWidth() / 2, ST7735_GetHeight(), ST7735_GetWidth(), 0, ST7735_RED);
    LL_mDelay(1000);

    ST7735_FillScreen(ST7735_BLACK);

    SysTick->CTRL = 0; // Disable SysTick
    SysTick->LOAD = 0xFFFFFF; // Count down from maximum value
    SysTick->VAL = 0; // Clear current value to 0
    SysTick->CTRL = 0x5; // Enable SysTick,  and use processor clock
    while (SysTick->VAL == 0)
        ; // Wait until SysTick reloaded
    ticksStart = SysTick->VAL; // Read start time value

    ST7735_DrawImage(0, 0, 128, 128, (uint16_t*)test_img_128x128);

    ticksEnd = SysTick->VAL;
    SysTick->CTRL = 0; // Disable SysTick
    if ((SysTick->CTRL & 0x10000) == 0) // if no overflow
        nchars = sprintf(str_time, "Ticks: %lu", ticksStart - ticksEnd); // Calculate total cycles
    else
        nchars = sprintf(str_time, "overflow");

    ST7735_DrawText(str_time, FONT3, 5, 110, ST7735_WHITE, ST7735_BLACK);
    SysTick->LOAD = 0x1193F;
    SysTick->CTRL = 0x5; // Enable SysTick
    LL_mDelay(3000);

    ST7735_FillScreen(ST7735_BLACK);
    ST7735_DrawText("Custom font", FONT6, 5, 15, ST7735_WHITE, ST7735_BLACK);
    ST7735_DrawText("ABCDEFGHI", FONT6, 5, 40, ST7735_WHITE, ST7735_BLACK);
    ST7735_DrawText("lmnopqrst", FONT6, 5, 65, ST7735_WHITE, ST7735_BLACK);
    ST7735_DrawText("0123456789", FONT6, 5, 90, ST7735_WHITE, ST7735_BLACK);
    LL_mDelay(2000);
    r++;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    /* System interrupt init*/

    /** NOJTAG: JTAG-DP Disabled and SW-DP Enabled 
  */
    LL_GPIO_AF_Remap_SWJ_NOJTAG();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_SPI1_Init();
    /* USER CODE BEGIN 2 */
    LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_3);
    LL_SPI_Enable(SPI1);
    ST7735_Init();
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    //demo2();
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

        demoTFT();
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);

    if (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2) {
        Error_Handler();
    }
    LL_RCC_HSE_Enable();

    /* Wait till HSE is ready */
    while (LL_RCC_HSE_IsReady() != 1) {
    }
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
    LL_RCC_PLL_Enable();

    /* Wait till PLL is ready */
    while (LL_RCC_PLL_IsReady() != 1) {
    }
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

    /* Wait till System clock is ready */
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {
    }
    LL_Init1msTick(72000000);
    LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
    LL_SetSystemCoreClock(72000000);
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

    /* USER CODE BEGIN SPI1_Init 0 */

    /* USER CODE END SPI1_Init 0 */

    LL_SPI_InitTypeDef SPI_InitStruct = { 0 };

    LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    /* Peripheral clock enable */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
    /**SPI1 GPIO Configuration  
  PA5   ------> SPI1_SCK
  PA7   ------> SPI1_MOSI 
  */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_5 | LL_GPIO_PIN_7;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* SPI1 DMA Init */

    /* SPI1_TX Init */
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PDATAALIGN_HALFWORD);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MDATAALIGN_HALFWORD);

    /* USER CODE BEGIN SPI1_Init 1 */

    /* USER CODE END SPI1_Init 1 */
    /* SPI1 parameter configuration*/
    SPI_InitStruct.TransferDirection = LL_SPI_HALF_DUPLEX_TX;
    SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
    SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
    SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
    SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
    SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
    SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2;
    SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
    SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
    SPI_InitStruct.CRCPoly = 10;
    LL_SPI_Init(SPI1, &SPI_InitStruct);
    /* USER CODE BEGIN SPI1_Init 2 */

    /* USER CODE END SPI1_Init 2 */
}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

    /* Init with LL driver */
    /* DMA controller clock enable */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

    /* DMA interrupt init */
    // Assign request line
    // LL_DMA_SetPeriphRequest(DMA1,LL_DMA_CHANNEL_3, LL_DMA_REQUEST_1);

    // Enable DMA interrupts on DMA Tx complete and DMA error
    //LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_3);  // DMA Tx Complete IT
    //LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_3);  // DMA error
    // note: this will set IT flags internally to DMA periph
    // but won't trigger IT on Cortex core as long as NVIC not set
    /* DMA1_Channel3_IRQn interrupt configuration */
    NVIC_SetPriority(DMA1_Channel3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(DMA1_Channel3_IRQn);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    /* GPIO Ports Clock Enable */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

    /**/
    LL_GPIO_ResetOutputPin(GPIOB, ST7735_RES_Pin | ST7735_DC_Pin | ST7735_CS_Pin | ST7735_BL_Pin);

    /**/
    GPIO_InitStruct.Pin = ST7735_RES_Pin | ST7735_DC_Pin | ST7735_CS_Pin | ST7735_BL_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
void DMA1_Channel3_TransferComplete(void)
{
    LL_DMA_ClearFlag_GI3(DMA1);
    flag_DMA_CH3_bsy = 1;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */

    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
