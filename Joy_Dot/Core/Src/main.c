/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "am2320.h"
#include "FLASH_PAGE_F1.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint32_t joy_value[2];
uint8_t sw_value;
uint8_t input_delay =0;
uint8_t state =0;

uint8_t dot_flag1, dot_flag2 ,dot_flag3 ,dot_flag4;
uint8_t button_flag=0;

int row = 4;
int col = 4;

char time[30];
char date[30];

RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_RTC_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t disp1ay[47][8]={
{0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00}, //.
{0x01,0x62,0x64,0x08,0x10,0x26,0x46,0x80}, // if wirte '/' -> %
{0x3C,0x42,0x42,0x42,0x42,0x42,0x42,0x3C},//0
{0x10,0x30,0x50,0x10,0x10,0x10,0x10,0x7c},//1
{0x7E,0x2,0x2,0x7E,0x40,0x40,0x40,0x7E},//2
{0x3E,0x2,0x2,0x3E,0x2,0x2,0x3E,0x0},//3
{0x8,0x18,0x28,0x48,0xFE,0x8,0x8,0x8},//4
{0x3C,0x20,0x20,0x3C,0x4,0x4,0x3C,0x0},//5
{0x3C,0x20,0x20,0x3C,0x24,0x24,0x3C,0x0},//6
{0x3E,0x22,0x4,0x8,0x8,0x8,0x8,0x8},//7
{0x0,0x3E,0x22,0x22,0x3E,0x22,0x22,0x3E},//8
{0x3E,0x22,0x22,0x3E,0x2,0x2,0x2,0x3E},//9
{0x00,0x18,0x18,0x00,0x00,0x18,0x18,0x00}, // :
{0x00,0x18,0x18,0x10,0x00,0x00,0x00,0x00}, // if wirte ';' -> '''
{0x08,0x10,0x20,0x40,0x20,0x10,0x08,0x00}, // <
{0x00,0x3C,0x3C,0x00,0x00,0x3C,0x3C,0x00}, // =
{0x20,0x10,0x08,0x04,0x08,0x10,0x20,0x00}, // >
{0x3C,0x24,0x24,0x08,0x10,0x10,0x00,0x10}, // ?
{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // if write '@' -> space
{0x18,0x24,0x42,0x42,0x7E,0x42,0x42,0x42},//A
{0x3C,0x22,0x22,0x3c,0x22,0x22,0x3C,0x0},//B
{0x3C,0x40,0x40,0x40,0x40,0x40,0x40,0x3C},//C
{0x7C,0x22,0x22,0x22,0x22,0x22,0x22,0x7C},//D
{0x7C,0x40,0x40,0x7C,0x40,0x40,0x40,0x7C},//E
{0x7C,0x40,0x40,0x7C,0x40,0x40,0x40,0x40},//F
{0x3C,0x40,0x40,0x40,0x4c,0x44,0x44,0x3C},//G
{0x44,0x44,0x44,0x7C,0x44,0x44,0x44,0x44},//H
{0x7C,0x10,0x10,0x10,0x10,0x10,0x10,0x7C},//I
{0x3C,0x8,0x8,0x8,0x8,0x8,0x48,0x30},//J
{0x0,0x24,0x28,0x30,0x20,0x30,0x28,0x24},//K
{0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x7C},//L
{0x81,0xC3,0xA5,0x99,0x81,0x81,0x81,0x81},//M
{0x0,0x42,0x62,0x52,0x4A,0x46,0x42,0x0},//N
{0x3C,0x42,0x42,0x42,0x42,0x42,0x42,0x3C},//O
{0x3C,0x22,0x22,0x22,0x3C,0x20,0x20,0x20},//P
{0x1C,0x22,0x22,0x22,0x22,0x26,0x22,0x1D},//Q
{0x3C,0x22,0x22,0x22,0x3C,0x24,0x22,0x21},//R
{0x0,0x1E,0x20,0x20,0x3E,0x2,0x2,0x3C},//S
{0x0,0x3E,0x8,0x8,0x8,0x8,0x8,0x8},//T
{0x42,0x42,0x42,0x42,0x42,0x42,0x22,0x1C},//U
{0x42,0x42,0x42,0x42,0x42,0x42,0x24,0x18},//V
{0x0,0x49,0x49,0x49,0x49,0x2A,0x1C,0x0},//W
{0x0,0x41,0x22,0x14,0x8,0x14,0x22,0x41},//X
{0x41,0x22,0x14,0x8,0x8,0x8,0x8,0x8},//Y
{0x0,0x7F,0x2,0x4,0x8,0x10,0x20,0x7F},//Z
};

/***************************************/
uint8_t coordinate[9] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t map[9] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

uint32_t Read_Data[9];
uint32_t Upload_Data[9];

/*****************dot matrix function*******************/

void write_byte (uint8_t byte)
{
	for (int i =0; i<8; i++)
	{
		HAL_GPIO_WritePin (GPIOC, clock_Pin, 0);  // pull the clock pin low
		HAL_GPIO_WritePin (GPIOC, data_Pin, byte&0x80);  // write the MSB bit to the data pin
		byte = byte<<1;  // shift left
		HAL_GPIO_WritePin (GPIOC, clock_Pin, 1);  // pull the clock pin HIGH
	}
}

void write_max (uint8_t address, uint8_t data)
{
	HAL_GPIO_WritePin (GPIOC, cs_Pin, 0);  // pull the CS pin LOW
	write_byte (address);
	write_byte (data);
	HAL_GPIO_WritePin (GPIOC, cs_Pin, 1);  // pull the CS pin HIGH
}

// function for init
void max_init(void)
{
 write_max(0x09, 0x00);       //  no decoding
 write_max(0x0a, 0x03);       //  brightness intensity
 write_max(0x0b, 0x07);       //  scan limit = 8 LEDs
 write_max(0x0c, 0x01);       //  power down =0,normal mode = 1
 write_max(0x0f, 0x00);       //  no test display
}

void write_string (char *str)
{
	while (*str)
	{
		for(int i=1;i<9;i++)
			   {
			       write_max (i,disp1ay[(*str - 46)][i-1]);
			   }
		str++;
		HAL_Delay (500);
	}
}

uint8_t MSB_check(uint8_t check, uint16_t col)
{
	uint8_t temp = (check >>col)&0x01;
	if(temp == 0x01) // if bit is 1
		return 1;
	else if(temp == 0x00) // if bit is 0
		return 0;
}

void write_string_shift(char *str)
{
	char now_display[8];
	char next_display[8];

	char str_length = strlen(str);
	char count =0; // check null

	while(*str)
	{
		for (int temp = 0; temp < 8; temp++)
		{
			now_display[temp] = disp1ay[(*str - 46)][temp];
		}
		str++; count++;
		for (int temp = 0; temp < 8; temp++)
		{
			next_display[temp] = disp1ay[(*str - 46)][temp];
		}
		// now char, next char copy

		for (int k = 7; k >= 0; k--) // shift
		{
			for (int i = 1; i < 9; i++)
			{
				if (MSB_check(next_display[i-1], k) == 1)
					now_display[i-1] = (now_display[i-1] << 1) + 1;
				else if (MSB_check(next_display[i-1], k) == 0)
					now_display[i-1] = now_display[i-1] << 1;
				write_max(i, now_display[i-1]);
			}
			HAL_Delay(100);

			if(HAL_GPIO_ReadPin(GPIOB, joysw_Pin) == 0)
						return;
		} //end of display
		if(HAL_GPIO_ReadPin(GPIOB, joysw_Pin) == 0)
			break;
		if(count==str_length -1)
			str++;
	} // end of while


}


uint8_t bit_check(uint8_t check)
{
	uint8_t temp = (check >>col)&0x01;
	if(temp == 0x01) // if bit is 1
		return 1;
	else if(temp == 0x00) // if bit is 0
		return 0;
}
/*****************temp function*******************/


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint16_t cnt =0;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_RTC_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  printf("Start\n");
  max_init ();
  Am2320_HandleTypeDef Am2320_;
  Am2320_ = am2320_Init(&hi2c1, 0x5C << 1);
  float temperature, humidity;

  HAL_TIM_Base_Start_IT(&htim3);
  HAL_ADC_Start_DMA(&hadc1, joy_value, 2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		if (state == 0) // state 1 : normal control
		{
			if (input_delay == 1)
			{
				if (joy_value[1] == 0) //up
				{
					printf("up\r\n");
					dot_flag1 = 1; //dot matrix flag-> up
					input_delay = 0;
				}
				if (joy_value[1] == 4095) //down
				{
					printf("down\r\n");
					dot_flag2 = 1; //dot matrix flag-> down
					input_delay = 0;
				}
				if (joy_value[0] == 0) //left
				{
					printf("left\r\n");
					dot_flag3 = 1; //dot matrix flag-> left
					input_delay = 0;
				}
				if (joy_value[0] == 4095) //right
				{
					printf("right\r\n");
					dot_flag4 = 1; //dot matrix flag-> right
					input_delay = 0;
				}

				if (HAL_GPIO_ReadPin(GPIOB, joysw_Pin) == 0) // button push
				{
					printf("push\r\n");
					button_flag = 1;
					input_delay = 0;

				}

			}// end  of 'input_delay'
			if (dot_flag1 == 1) //up
					{
				coordinate[row] &= ~(1 << col); // pre-coordinate off

				if (row == 0)
					row = 7;
				else
					row--;
				coordinate[row] |= (1 << col);  //cur-coordinate on

				dot_flag1 = 0;
			}
			if (dot_flag2 == 1)  //down
					{
				coordinate[row] &= ~(1 << col); // pre-coordinate off
				if (row == 7)
					row = 0;
				else
					row++;
				coordinate[row] |= (1 << col);

				dot_flag2 = 0;
			}
			if (dot_flag3 == 1) //left
					{
				coordinate[row] &= ~(1 << col); // pre-coordinate off
				if (col == 7)
					col = 0;
				else
					col++;
				coordinate[row] |= (1 << col);

				dot_flag3 = 0;
			}
			if (dot_flag4 == 1) //right
					{
				coordinate[row] &= ~(1 << col); // pre-coordinate off
				if (col == 0)
					col = 7;
				else
					col--;
				coordinate[row] |= (1 << col);
				dot_flag4 = 0;
			}
			if (button_flag == 1) {
				uint8_t bit_state = bit_check(map[row]);
				if (bit_state == 0) { // if bit is 0
					map[row] |= (1 << col); // led on
				} else if (bit_state == 1) { // if bit is 1
					map[row] &= ~(1 << col); //led off
				}

				button_flag = 0;
			}



			while (!(HAL_GPIO_ReadPin(GPIOB, joysw_Pin))) // push button while 5sec;
			{
				cnt++;
				HAL_Delay(100);
				if (cnt > 10) {
					printf("Mode change to 1\r\n");
					cnt = 0;
					state = 1; // change to state1
					write_string("1");
					HAL_Delay(1000);
				}
			}
		}// end of 'state0'

		if (state == 1) // state 1 : display current time
		{
			HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);


			//sprintf(date,"Date: %02d.%02d.%02d\t",sDate.Date,sDate.Month,sDate.Year);
			sprintf(time,"%02d:%02d:%02d",sTime.Hours,sTime.Minutes,sTime.Seconds);
			write_string_shift(time);

			while (!(HAL_GPIO_ReadPin(GPIOB, joysw_Pin))) // push button while 5sec;
			{
				cnt++;
				HAL_Delay(100);
				if (cnt > 10)
				{
					printf("Mode change to 2\r\n");
					cnt = 0;
					state = 2; // change to state0
					write_string("2");
					HAL_Delay(1000);
					max_init ();
				}
			}
		 } // end of 'state1'

		if (state == 2) // state 2 : display temp and hum
		{
			//Am2320_ = am2320_Init(&hi2c1, 0x5C << 1);
			am2320_GetTemperatureAndHumidity(&Am2320_, &temperature, &humidity);
			printf("=====================================\r\n");
			printf("Temperature: %.1fºC\r\n", temperature);
			printf("Humidity: %.1f%%\r\n", humidity);

			sprintf(time,"%.1f;C@%.1f/",temperature,humidity);
			write_string_shift(time);

			while (!(HAL_GPIO_ReadPin(GPIOB, joysw_Pin))) // push button while 5sec;
			{
				cnt++;
				HAL_Delay(100);
				if (cnt > 10)
				{
					printf("Mode change to 3\r\n");
					cnt = 0;
					state = 3; // change to state0
					write_string("3");
					HAL_Delay(1000);
					max_init ();
				}
			}
		 } // end of 'state2'

		if (state == 3) // state 3 : save to Flash Memory
		{
			for(int i =0; i<8; i++)
			{
				Upload_Data[i] = (uint32_t)map[i];
			}
			if (input_delay == 1) {
				if (joy_value[1] == 0) //up
						{
					printf("up\r\n");
					Flash_Write_Data(0x0801F010 , Upload_Data,8);

					//Flash_Write_Data(0x08004410 , Upload_Data,8);
					input_delay = 0;
					write_string_shift("SAVE1");
				}
				if (joy_value[1] == 4095) //down
						{
					printf("down\r\n");
					Flash_Write_Data(0x0801F410 , Upload_Data,8);
					//Flash_Write_Data(0x0800430 , Upload_Data,8);
					input_delay = 0;
					write_string_shift("SAVE2");
				}
				if (joy_value[0] == 0) //left
						{
					printf("left\r\n");
					Flash_Write_Data(0x0801F810 , Upload_Data,8);
					//Flash_Write_Data(0x08004450 , Upload_Data,8);
					input_delay = 0;
					write_string_shift("SAVE3");
				}
				if (joy_value[0] == 4095) //right
						{
					printf("right\r\n");
					Flash_Write_Data(0x0801FC10 , Upload_Data,8);
					//Flash_Write_Data(0x08004470 , Upload_Data,8);
					input_delay = 0;
					write_string_shift("SAVE4");
				}

				if (HAL_GPIO_ReadPin(GPIOB, joysw_Pin) == 0) // button push
						{
					printf("push\r\n");
					button_flag = 1;
					input_delay = 0;

				}
			}

			while (!(HAL_GPIO_ReadPin(GPIOB, joysw_Pin))) // push button while 5sec;
			{
				cnt++;
				HAL_Delay(100);
				if (cnt > 10)
				{
					printf("Mode change to 4\r\n");
					cnt = 0;
					state = 4; // change to state4
					write_string("4");
					HAL_Delay(1000);
					max_init();
				}
			}
		} // end of 'state3'

		if (state == 4) // state 4 : display saved map
		{
			if (input_delay == 1) {
				if (joy_value[1] == 0) //up
						{
					printf("up\r\n");
					Flash_Read_Data(0x0801F010, Read_Data, 8);

					//Flash_Read_Data(0x08004410, Read_Data, 8);
					for(int i =0; i<8; i++)
					{
						map[i] = (uint8_t)Read_Data[i];
					}
					write_string_shift("READ1");
					input_delay = 0;
				}
				if (joy_value[1] == 4095) //down
						{
					printf("down\r\n");
					Flash_Read_Data(0x0801F410, Read_Data, 8);
					//Flash_Read_Data(0x08004430, Read_Data, 8);
					for(int i =0; i<8; i++)
					{
						map[i] = (uint8_t)Read_Data[i];
					}
					write_string_shift("READ2");
					input_delay = 0;
				}
				if (joy_value[0] == 0) //left
						{
					printf("left\r\n");
					Flash_Read_Data(0x0801F810, Read_Data, 8);
					//Flash_Read_Data(0x08004450, Read_Data, 8);
					for(int i =0; i<8; i++)
					{
						map[i] = (uint8_t)Read_Data[i];
					}
					write_string_shift("READ3");
					input_delay = 0;
				}
				if (joy_value[0] == 4095) //right
						{
					printf("right\r\n");
					Flash_Read_Data(0x0801FC10, Read_Data, 8);
					//Flash_Read_Data(0x08004470, Read_Data, 8);
					for(int i =0; i<8; i++)
					{
						map[i] = (uint8_t)Read_Data[i];
					}
					write_string_shift("READ4");
					input_delay = 0;
				}
//				if (HAL_GPIO_ReadPin(GPIOB, joysw_Pin) == 0) // button push
//				{
//					printf("push\r\n");
//					for (int i = 0; i < 8; i++)
//					{
//						map[i] = Display_Data[i];
//					}
//					input_delay = 0;
//
//				}
			}

			while (!(HAL_GPIO_ReadPin(GPIOB, joysw_Pin))) // push button while 5sec;
			{
				cnt++;
				HAL_Delay(100);
				if (cnt > 10)
				{
					printf("Mode change to 0\r\n");
					cnt = 0;
					state = 0; // change to state0
					write_string("0");
					HAL_Delay(1000);
					max_init();
				}
			}
		} // end of 'state4'

		for (int i = 1; i < 9; i++) // display coordinate
		{
			write_max(i, coordinate[i - 1]);
			write_max(i, map[i - 1]);
		}

//sw_value = HAL_GPIO_ReadPin(GPIOB,JOYSW_Pin);
//printf("x : %ld, y : %ld, sw : %d\n", joy_value[0], joy_value[1], sw_value);
//HAL_Delay(500);



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//		for (int i = 1; i < 9; i++) // display map
//		{
//			write_max(i, map[i - 1]);
//		}


  }//end of while
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x19;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_SUNDAY;
  DateToUpdate.Month = RTC_MONTH_MAY;
  DateToUpdate.Date = 0x1;
  DateToUpdate.Year = 0x22;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7200-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 5000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, clock_Pin|cs_Pin|data_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : clock_Pin cs_Pin data_Pin */
  GPIO_InitStruct.Pin = clock_Pin|cs_Pin|data_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : joysw_Pin */
  GPIO_InitStruct.Pin = joysw_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(joysw_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
PUTCHAR_PROTOTYPE {
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART1 and Loop until the end of transmission */
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 0xFFFF);

	return ch;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	input_delay =1;
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
