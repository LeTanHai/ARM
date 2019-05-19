/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "tft_spi.h"
#include "TSC2046.h"
#include "dwt_delay.h"
#include "string.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
GPIO_InitTypeDef GPIO_InitStruct;
extern uint16_t BACK_COLOR, POINT_COLOR;
extern __IO uint32_t systimedelay;
float temp;
TS_TOUCH_DATA_Def my_TS_Handle;
char DATA[10];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
extern void Delay_10us(uint32_t t);

extern void Delay_10us(uint32_t t)
{
	systimedelay = t;
	while(systimedelay != 0);
}

void gpio_set_input (void)
{
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB,&GPIO_InitStruct);
}
void gpio_set_output (void)
{
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB,&GPIO_InitStruct);
}
uint8_t DS18B20_ReadByte(void)
{   
    uint8_t i=0;
    uint8_t data=0;      
      
    for(i=8;i>0;i--)
    {    
        gpio_set_output();   // Cau hinh chan DQ la OUPUT       
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,0);  // Keo chan DQ xuong muc '0'
        data>>=1;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);  // Keo chan DQ len muc '1'      
        gpio_set_input();   // Cau hinh chan DQ la INPUT
        if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6))data|=0x80;   // Nhan du lieu tra ve tu DS18B20
        DWT_Delay(100);                                            
    }
    return(data);    
}
void DS18B20_WriteByte(uint8_t data)
{
    uint8_t i=0;
    gpio_set_output();   // Cau hinh chan DQ la OUTPUT
    for (i=8;i>0;i--)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,0);  // Keo chan DQ xuong muc '0'
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,data&0x01); // Viet du lieu vao DS18B20
				DWT_Delay(40);     
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);  // Keo chan DQ len muc '1'
        data>>=1;
    }    
}
void DS18B20_Init(void)
{
    gpio_set_output();   // Cau hinh chan DQ la OUTPUT
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,0);  // Keo DQ xuong muc '0' trong khoang 480us
    DWT_Delay(480);            
    gpio_set_input();   // Cau hinh chan DQ la INPUT trong khoang 480us
    DWT_Delay(480);              
}
void DS18B20_WriteFunc(uint8_t byte_func)
{
    DS18B20_Init();                 // Khoi tao DS18B20
    DS18B20_WriteByte(0xCC);  // Truy cap thang den cac lenh chuc nang bo nho cua DS18B20
    DS18B20_WriteByte(byte_func);   // Viet ma lenh chuc nang
}
void DS18B20_Config(uint8_t temp_low, uint8_t temp_high, uint8_t resolution)
{   
    resolution=(resolution<<5)|0x1f;  
    DS18B20_WriteFunc(0x4E);        // Cho phep ghi 3 byte vao bo nho nhap:    
		DS18B20_WriteByte(temp_high);   // byte 2: Th
		DS18B20_WriteByte(temp_low);    // byte 3: Tl 
		DS18B20_WriteByte(resolution);  // byte 4: configuration register
    DS18B20_WriteFunc(0x48);        // Ghi vao EEPROM
}
float DS18B20_ReadTemp(void)
{
	//HAL_RTC_GetTime(&hrtc,&sTime,RTC_FORMAT_BIN);
  float temp;
	uint8_t buff_temp1,buff_temp2;   
	DS18B20_WriteFunc(0x44);  // Khoi dong qua trinh do va chuyen doi nhiet do ra so nhi phan
  DWT_Delay(180);     
	DS18B20_WriteFunc(0xBE);   // Doc du lieu tu bo nho DS18b20   
	buff_temp1 = DS18B20_ReadByte(); 
	temp=((float)(buff_temp1&0x0f))/16;		    // Lay phan thuc cua gia tri nhiet do
	buff_temp2 = DS18B20_ReadByte(); 				
	buff_temp1 =((buff_temp1&0xf0)>>4)|((buff_temp2&0x0f)<<4) ;	// Lay phan nguyen cua gia tri nhiet do
	temp=temp+buff_temp1;
	return temp;	   
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();

  /* USER CODE BEGIN 2 */
	
	tft_init();
	TSC2046_Begin(&hspi2,TS_CS_GPIO_Port,TS_CS_Pin);
	tft_clear(DARKBLUE);
	//TSC2046_Calibrate();
	DWT_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		/*my_TS_Handle = TSC2046_GetTouchData();
		if(my_TS_Handle.X > 40)
			{
				BACK_COLOR = WHITE;
				POINT_COLOR = LBBLUE;
				tft_draw_circle(20,20,5);
			}*/
		temp=DS18B20_ReadTemp();
		BACK_COLOR = DARKBLUE;
		POINT_COLOR = BROWN;
		tft_puts14x24(130,45,(int8_t*) "TEMPERATURE",TFT_STRING_MODE_BACKGROUND);
		BACK_COLOR = RED;
		POINT_COLOR = BLACK;
		sprintf(DATA,"%.02f",temp);
		tft_puts26x48(170,54,(int8_t*) DATA,TFT_STRING_MODE_BACKGROUND);
		tft_putchar26x48(170,60,223,0);
		/*BACK_COLOR = WHITE;
		POINT_COLOR = RED;
		tft_puts14x24(25,3,(int8_t*) "Thai Tong Son",TFT_STRING_MODE_BACKGROUND);
		
		BACK_COLOR = WHITE;
		POINT_COLOR = GREEN;
		tft_puts26x48(60,3,(int8_t*) "Thanh Hai",TFT_STRING_MODE_BACKGROUND);
		
		BACK_COLOR = WHITE;
		POINT_COLOR = BLACK;
		tft_puts18x32(120,3,(int8_t*) "Quang Phuoc",TFT_STRING_MODE_BACKGROUND);
		
		BACK_COLOR = WHITE;
		POINT_COLOR = LIGHTBLUE;
		tft_puts14x24(170,3,(int8_t*) "Minh Hieu",TFT_STRING_MODE_BACKGROUND);*/
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, B0_Pin|B1_Pin|TS_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B0_Pin */
  GPIO_InitStruct.Pin = B0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(B0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : B1_Pin TS_CS_Pin */
  GPIO_InitStruct.Pin = B1_Pin|TS_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
