/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "string.h"
#include "stdlib.h"
#include "stdbool.h"
#include "i2c-lcd.h"
/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_uart4_rx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_UART4_Init(void);
void lcd_write(int row,char* data);
int Check_Temp(void);
bool	GetNewFinger(uint16_t	Location);
void ClearBuffer();
void	SearchFinger(void);
uint8_t NullBuf[64],RxBuf[64],TxBuf[18];
uint8_t RxTemp[1];
int iRx=0;
uint16_t num=1;
uint16_t fcount;
int16_t sf;
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_UART4_Init();
	lcd_init();
	lcd_clear_display();
	HAL_Delay(100);
	HAL_UART_Receive_IT(&huart4,RxTemp,1);
	fcount=Check_Temp();
	if(fcount==0) num=0;
	else num=fcount+1;
	//ClearBuffer();
	//for(int i=0;i<20;i++) {RxBuf[i]=0xff;TxBuf[i]=0xff;}
	HAL_Delay(100);
	lcd_goto_XY(1,0);
	lcd_write(1,"Chon chuc nang");
	HAL_Delay(1000);
	//char* dt="Temp"&(char*)ct;
	//lcd_write(2,(char*)ct);//atoi((const char *)text))
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3)==GPIO_PIN_RESET){
			lcd_clear_display();
			HAL_Delay(10);
			lcd_write(1,"Nhap van tay");
			GetNewFinger(num);
			num++;
			//for(int i=0;i<20;i++) {RxBuf[i]=0xff;TxBuf[i]=0xff;}
			lcd_write(1,"Chon chuc nang");
			lcd_write(2," ");
		}
		if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4)==GPIO_PIN_RESET){
			lcd_clear_display();
			HAL_Delay(10);
			lcd_write(1,"Tim van tay");
			SearchFinger();
			lcd_write(1,"Chon chuc nang");
			lcd_write(2," ");
			//for(int i=0;i<20;i++) {RxBuf[i]=0xff;TxBuf[i]=0xff;}
		}
  }
  /* USER CODE END 3 */
}

//User function

//Set RX Callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance==huart4.Instance){
		RxBuf[iRx]=RxTemp[0];
		iRx++;
		HAL_UART_Receive_IT(&huart4,RxTemp,1);
		if(iRx==64) 
		{ClearBuffer();}
	}
}
void lcd_write(int row,char* data){ //
	lcd_goto_XY(row,0);
	lcd_send_string("                ");
	HAL_Delay(10);
	lcd_goto_XY(row,0);
	lcd_send_string(data);
}
int Check_Temp(void){
	TxBuf[0]=FINGERPRINT_STARTCODE_BYTE0;
	TxBuf[1]=FINGERPRINT_STARTCODE_BYTE1;
	TxBuf[2]=0xFF;
	TxBuf[3]=0xFF;
	TxBuf[4]=0xFF;
	TxBuf[5]=0xFF;
	TxBuf[6]=FINGERPRINT_COMMANDPACKET;
	TxBuf[7]=0x00;
	TxBuf[8]=0x03;
	TxBuf[9]=FINGERPRINT_TEMPLATECOUNT;
	TxBuf[10]=0x00;
	TxBuf[11]=0x21;
	HAL_UART_Transmit_IT(&huart4,TxBuf,12);
	HAL_Delay(500);
	if((RxBuf[6]==0x07) && (RxBuf[7]==0x00) && (RxBuf[8]==0x05))
		{
			uint16_t Temp=((RxBuf[10]<<8)|RxBuf[11]);
			return Temp;
		}
	else return -1;
}
bool	GetNewFinger(uint16_t	Location)
{
	uint8_t WaitForFingerInSecond=20;
	uint8_t TimeOut;
	lcd_write(2,"Lay mau lan 1");
	HAL_Delay(10);
	//+++ take Image
	TxBuf[0]=FINGERPRINT_STARTCODE_BYTE0;
	TxBuf[1]=FINGERPRINT_STARTCODE_BYTE1;
	TxBuf[2]=0xFF;
	TxBuf[3]=0xFF;
	TxBuf[4]=0xFF;
	TxBuf[5]=0xFF;
	TxBuf[6]=FINGERPRINT_COMMANDPACKET;
	TxBuf[7]=0x00;
	TxBuf[8]=0x03;
	TxBuf[9]=FINGERPRINT_GETIMAGE;
	TxBuf[10]=0x00;
	TxBuf[11]=0x05;	
	TimeOut=0;
	HAL_Delay(100);
	while(1){
		ClearBuffer();
		HAL_UART_Transmit_IT(&huart4,TxBuf,12);
		HAL_Delay(500);
		if((RxBuf[6]==0x07) && (RxBuf[7]==0x00) && (RxBuf[8]==0x03)){
			if(RxBuf[9]==0x00) break;
			else {
				TimeOut++;
			}
		}
		if(TimeOut==WaitForFingerInSecond)
			goto Faild;
	}
	HAL_Delay(100);
	//--- take Image
	//+++	put image to buffer 1
	TxBuf[8]=0x04;
	TxBuf[9]=FINGERPRINT_IMAGE2TZ;
	TxBuf[10]=1;
	TxBuf[11]=0x00;
	TxBuf[12]=0x08;
	TimeOut=0;
	while(1)
	{
		ClearBuffer();
		HAL_UART_Transmit_IT(&huart4,TxBuf,13);
		HAL_Delay(500);
		if((RxBuf[6]==0x07) && (RxBuf[7]==0x00) && (RxBuf[8]==0x03)){
			if(RxBuf[9]==0x00) break;
			else {
				TimeOut++;
			}
		}
		if(TimeOut==WaitForFingerInSecond)
			goto Faild;
	}
	//---	put image to buffer 1	
	//+++ Wait for put your finger up
	lcd_write(2,"Bo tay ra");
	HAL_Delay(2000);
	//--- Wait for put your finger up
	lcd_write(2,"Lay mau lan 2");
	//+++ take Image
	TxBuf[8]=0x03;
	TxBuf[9]=FINGERPRINT_GETIMAGE;
	TxBuf[10]=0x00;
	TxBuf[11]=0x05;	
	TimeOut=0;
	while(1){
		ClearBuffer();
		HAL_UART_Transmit_IT(&huart4,TxBuf,12);
		HAL_Delay(500);
		if((RxBuf[6]==0x07) && (RxBuf[7]==0x00) && (RxBuf[8]==0x03)){
			if(RxBuf[9]==0x00) break;
			else {
				TimeOut++;
			}
		}
		if(TimeOut==WaitForFingerInSecond)
			goto Faild;
	}
	lcd_write(2,"Dang luu van tay");
	//--- take Image
	//+++	put image to buffer 2
	TxBuf[8]=0x04;
	TxBuf[9]=FINGERPRINT_IMAGE2TZ;
	TxBuf[10]=2;
	TxBuf[11]=0x00;
	TxBuf[12]=0x09;
	TimeOut=0;
	while(1)
	{
		ClearBuffer();
		HAL_UART_Transmit_IT(&huart4,TxBuf,13);
		HAL_Delay(500);
		if((RxBuf[6]==0x07) && (RxBuf[7]==0x00) && (RxBuf[8]==0x03)){
			if(RxBuf[9]==0x00) break;
			else {
				TimeOut++;
			}
		}
		if(TimeOut==WaitForFingerInSecond)
			goto Faild;
	}
	//---	put image to buffer 2
	HAL_Delay(2000);
	//+++ Create Register model
	TxBuf[8]=0x03;
	TxBuf[9]=FINGERPRINT_REGMODEL;
	TxBuf[10]=0x00;
	TxBuf[11]=0x09;	
	TimeOut=0;
	while(1)
	{
		ClearBuffer();
		HAL_UART_Transmit_IT(&huart4,TxBuf,12);
		HAL_Delay(500);
		if((RxBuf[6]==0x07) && (RxBuf[7]==0x00) && (RxBuf[8]==0x03)){
			if(RxBuf[9]==0x00) break;
			else {
				TimeOut++;
			}
		}
		if(TimeOut==WaitForFingerInSecond)
			goto Faild;
	}	
	//---	Create Register model
	//+++ Store in memory
	TxBuf[8]=0x06;
	TxBuf[9]=FINGERPRINT_STORE;
	TxBuf[10]=0x01;
	TxBuf[11]=Location>>8;	
	TxBuf[12]=Location&0xFF;
	uint8_t	Checksum=0;
	for(uint8_t i=0;i<9; i++)
		Checksum+=TxBuf[i+6];
	TxBuf[13]=Checksum>>8;
	TxBuf[14]=Checksum&0xFF;
	while(1)
	{
		ClearBuffer();
		HAL_UART_Transmit_IT(&huart4,TxBuf,15);
		HAL_Delay(500);
		if((RxBuf[6]==0x07) && (RxBuf[7]==0x00) && (RxBuf[8]==0x03)){
			if(RxBuf[9]==0x00) {
				lcd_write(2,"Luu thanh cong");
				HAL_Delay(2000);
				break;
			}
			else {
				lcd_write(2,"Luu that bai");
				HAL_Delay(2000);
				goto Faild;
			}
		}
	}				
	//--- Store in memory
	return true;
	Faild:
	return false;
}
void	SearchFinger(void)
{
	uint8_t WaitForFingerInSecond=10;
	uint8_t TimeOut=10, cstr[2];
	HAL_Delay(100);
	lcd_write(2,"Dat tay vao cb");
	//+++ take Image
	TxBuf[0]=FINGERPRINT_STARTCODE_BYTE0;
	TxBuf[1]=FINGERPRINT_STARTCODE_BYTE1;
	TxBuf[2]=0xFF;
	TxBuf[3]=0xFF;
	TxBuf[4]=0xFF;
	TxBuf[5]=0xFF;
	TxBuf[6]=FINGERPRINT_COMMANDPACKET;
	TxBuf[7]=0x00;
	TxBuf[8]=0x03;
	TxBuf[9]=FINGERPRINT_GETIMAGE;
	TxBuf[10]=0x00;
	TxBuf[11]=0x05;	
	TimeOut=0;
	HAL_Delay(100);
	while(1)
	{
		ClearBuffer();
		HAL_UART_Transmit_IT(&huart4,TxBuf,12);
		HAL_Delay(500);
		if((RxBuf[6]==0x07) && (RxBuf[7]==0x00) && (RxBuf[8]==0x03)){
			if(RxBuf[9]==0x00) break;
			else TimeOut++;
		}
		if(TimeOut==WaitForFingerInSecond)
			break;
	}
	//--- take Image

	//+++	put image to buffer 1
	TxBuf[8]=0x04;
	TxBuf[9]=FINGERPRINT_IMAGE2TZ;
	TxBuf[10]=1;
	TxBuf[11]=0x00;
	TxBuf[12]=0x08;
	HAL_Delay(100);
	TimeOut=0;
	while(1)
	{
		ClearBuffer();
		HAL_UART_Transmit(&huart4,TxBuf,13,100);
		HAL_Delay(500);
		if((RxBuf[6]==0x07) && (RxBuf[7]==0x00) && (RxBuf[8]==0x03)){
			if(RxBuf[9]==0x00) break;
			else TimeOut++;
		}
		if(TimeOut==WaitForFingerInSecond)
			break;
	}
	//---	put image to buffer 1	
	lcd_write(2,"Dang tim...");
	HAL_Delay(1000);
	//+++	Searching
	TxBuf[8]=0x08;
	TxBuf[9]=FINGERPRINT_SEARCH;
	TxBuf[10]=0x01;
	TxBuf[11]=0x00;
	TxBuf[12]=0x00;
	TxBuf[13]=0x00;
	TxBuf[14]=0xA3;
	uint16_t	Checksum=0;
	for(uint8_t i=0;i<11; i++)
		Checksum+=TxBuf[i+6];
	TxBuf[15]=Checksum>>8;
	TxBuf[16]=Checksum&0xFF;
	TimeOut=0;
	HAL_Delay(500);
	while(1)
	{
		ClearBuffer();
		HAL_UART_Transmit_IT(&huart4,TxBuf,17);
		HAL_Delay(1000);
		if((RxBuf[6]==0x07) && (RxBuf[7]==0x00) && (RxBuf[8]==0x07)){
			if(RxBuf[9]==0x00)
			{
				cstr[0]=RxBuf[10]<<8;
				cstr[1]=RxBuf[11];
				sprintf((char*)cstr,"%d",RxBuf[11]);
				lcd_write(2,(char*)(cstr));
				HAL_Delay(3000);
				break;
			}
			if((RxBuf[9]==0x01)||TimeOut==WaitForFingerInSecond){
				lcd_write(2,"Khong tim thay");
				HAL_Delay(3000);
				break;
			}
			TimeOut++;
		}
		TimeOut++;
		if(TimeOut==10){
			lcd_write(2,"Khong tim thay");
			HAL_Delay(3000);
			break;
		}
	}
	//---	Searching	
}

void ClearBuffer(){
	iRx=0; memcpy(RxBuf,NullBuf,64);
}
/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* UART4 init function */
static void MX_UART4_Init(void)
{

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 57600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PE3 PE4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

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
