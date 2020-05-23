/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "string.h"

#include "stdio.h" //https://www.youtube.com/watch?v=5usorjLqtHk for HC-SR04
#include "mp3_lib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define echo_PB12 PB12 // пин на линии EXTI12
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
volatile char sim='q';
uint8_t transmitBuffer[10];	//for uart
uint8_t receiveBuffer[10];	//for uart
GPIO_PinState triger_line_status; //for GPIO_PinState = GPIO_PIN_RESET = 0u, GPIO_PIN_SET

uint32_t delay;
uint32_t echo1;
uint32_t echo2;
float dis1;
float dis2;
uint32_t start = 0;
uint32_t start1 = 0;
uint32_t start2 = 0;
uint32_t start3 = 0;
uint32_t startSum=0;
uint32_t echoCount=0;
uint32_t delta=0;
float range=0.f;
float range2=0.f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
//void log(char msg[]);
void Log(char *msg);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write (int fd, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len, 0xFFFF);

    return len;
}

int _read (int fd, char *ptr, int len)
{

    *ptr = 0x00; // Flush the character buffer

    HAL_UART_Receive(&huart2, (uint8_t*) ptr, 1, 0xFFFF);

return 1;
}
//http://www.count-zero.ru/2016/stm32_uart/
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */



unsigned count = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	char xxx[] = "interrupt comes from TIM1\r\n";


	//uint16_t yyy = strlen(xxx);
	//uint32_t zzz = 100;
	//char uartString[] = "Hello from TIM";
	//char uartString2[] = "1\r\n";
	//bcopy(&uartString2,&uartString,0);

        if(htim->Instance == TIM1) //check if the interrupt comes from TIM1
        {
        	//strcat(&uartString,"1\r\n");
//        	if(/*GPIO_PinState */HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)==0u)
//        	{
        	//	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//        	}
//        	if(/*GPIO_PinState */HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)!=0u)
//        	        	{
//        	        		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//        	        	}

        	//HAL_UART_Transmit(&huart2, uartString, strlen(uartString), 100);

                //посылает в 1й UART текст, у меня он выводится
                //в proteus-е (virtual terminal)

        	/*Log(xxx);

        	triger_line_status = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
        	if(triger_line_status == GPIO_PIN_SET) {Log("GPIO_PIN_SET\r\n");}
        	else {
				Log("GPIOB, GPIO_PIN_13:  GPIO_PIN_RESET\r\n\r\n");
			}*/



        }
//        if(htim->Instance == TIM3) //check if the interrupt comes from TIM3
//        {

//                //HAL_UART_Transmit(&huart2, "interrupt comes from TIM3", strlen("interrupt comes from TIM3"), 100);
//                //посылает в 2й UART текст, у меня он выводится
//                //в proteus-е (virtual terminal)
//        }

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
          if(huart == &huart2)
          	  {
                  HAL_UART_Transmit(&huart2, (uint8_t*)&sim, 1, 10);
                  HAL_UART_Receive_IT(&huart2, (uint8_t*)&sim, 1);
          	  }
          if(huart == &huart1)
                    {
                            //HAL_UART_Transmit(&huart2, (uint8_t*)&sim, 1, 10);
        	  	  	  	  	HAL_UART_Transmit(&huart2, (uint8_t*)&sim, 1,1);
                            HAL_UART_Receive_IT(&huart1, (uint8_t*)&sim, 1);

                    }
          HAL_UART_Receive_IT(&huart1, (uint8_t*)&sim, 1);
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_8)
	{
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

	}
}

//{
//	uint8_t xxx[] = "EXTI15_10_IRQHandler\r\n";
//	HAL_UART_Transmit(&huart2, xxx, strlen(xxx), 100);
//}



//void USART2_IRQHandler(void){
//	uint8_t xxx[] = "USART1_IRQHandler TIM1\r\n";
//	//HAL_UART_Receive_IT(&huart2, receiveBuffer, 2);
//	//HAL_UART_Transmit_IT(&huart2, receiveBuffer, 2);
//	HAL_UART_Transmit(&huart2, xxx, strlen(xxx), 100);
////	UART_Receive_IT(&huart2){
////
////	}
//}

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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1);//запустим таймер1 в режиме прерывания:
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  TIM2->CCR4 = 3;
  HAL_TIM_IC_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_IC_Start(&htim4, TIM_CHANNEL_2);



  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  //HAL_UART_IRQHandler(&huart2);

//  HAL_GPIO_EXTI_IRQHandler(GPIOB,GPIO_PIN_12){
//
//  }

//  for (unsigned char i = 0; i < 2; i++)
//    {
//      //transmitBuffer[i] = i + 1;
//      receiveBuffer[i] = 0;
//    }

    //HAL_UART_Receive_IT(&huart2, receiveBuffer, 32);
    //HAL_UART_Transmit_IT(&huart2, transmitBuffer, 32);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // Play single file from folder
      // This command start playing file 032.mp3 from folder 05
       //MP3_send_cmd(MP3_PLAY_FOLDER_FILE, 1, 1); //folder 01..99, file 001..255
      // MP3_send_cmd(MP3_PLAY_FOLDER_FILE, 1, 1);
      //Make Voice QUEUE
      //MP3_say(MP3_NO_VALUE, 3148, MP3_NO_VALUE);
      //MP3_say(MP3_NO_VALUE, -35, MP3_NO_VALUE);
      //MP3_say(100, 153, 103);
      //MP3_say(MP3_NO_VALUE, 715, MP3_NO_VALUE);



  unsigned char next[] = {0x7E,0xFF,0x06,0x01,0x00,0x00,0x00,0xEF};
  unsigned char play[] = {0x7E,0xFF,0x06,0x03,0x00,0x00,0x00,0xFE,0xee,0xEF};
  	  	  	  	  	   //{ 0X7E, 0xFF, 0x06, 0X03, 00, 00, 00, 0xFE, 0xee, 0XEF};
  unsigned char play2[] = {0x7E,0xFF,0x06,0x0D,0x00,0x00,0x00,0xEF};
  unsigned char flash[] = {0x7E,0xFF,0x06,0x09,0x00,0x01,0x01,0xFF,0xDD,0xEF};
  unsigned char quaryVolume[] = {0x7E,0xFF,0x06,0x43,0x01,0x00,0x00,0xEF};
  unsigned char runsend[] = {0x00,0x00,0xFF};
  //u_char next2[]={0x7E,0xFF};

  char mp3[10];

  char buf[20];

  //char temp[255]=" <-- start in main while\r\n";
//  uint8_t *pdata;
  //uint32_t start = 0;
  float val=0.f;
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

  HAL_UART_Receive_IT(&huart1, (uint8_t*)&sim, 1);

//  	  HAL_Delay(3000);
//  	  HAL_UART_Transmit_IT(&huart2, (uint8_t *)&runsend, 3);
//  	  HAL_Delay(200);
//	  MP3_send_cmd(0x43, 0, 0);//Querycurrent status

  HAL_Delay(6000);
	  HAL_UART_Transmit_IT(&huart2, (uint8_t *)&next, 8);

  	  HAL_Delay(3000);
  	  MP3_send_cmd(0x06, 0, 10);//Specifiedvolumeislevel ...

  	  HAL_Delay(3000);
	  MP3_send_cmd(0x03, 0, 0);//SpecifieddeviceismicroSD

	  HAL_Delay(3000);
  	  HAL_UART_Transmit_IT(&huart2, (uint8_t *)&runsend, 3);
  	  HAL_Delay(3000);
	  MP3_send_cmd(0x4C, 0, 0);//Querycurrent trackinthe microSDcard

	  HAL_Delay(3000);
  	  HAL_UART_Transmit_IT(&huart2, (uint8_t *)&runsend, 3);
  	  HAL_Delay(200);
	  MP3_send_cmd(0x0C, 0, 0);//Reset
	  HAL_Delay(5000);
  	  HAL_UART_Transmit_IT(&huart2, (uint8_t *)&runsend, 3);
  	  //HAL_Delay(1000);
	  //MP3_send_cmd(0x0D, 0, 0);//PLAY
  while (1)
  {
	  HAL_Delay(500);
	  MP3_send_cmd(0x0D, 0, 0);//PLAY
	  HAL_Delay(5000);
	  MP3_send_cmd(0x0C, 0, 0);//Reset
	  	  HAL_Delay(5000);

	  //HAL_GPIO_TogglePin(GPIOС, GPIO_PIN_13);
	  if(0)
	  {
	  //HAL_Delay(6000);
	  //HAL_UART_Transmit(&huart2, &mp3, strlen(mp3), 20);
	  //HAL_UART_Transmit_IT(&huart2, &next, 10);
	  //HAL_UART_Transmit_IT(&huart1, &flash, (uint16_t)10);
	  //HAL_UART_Transmit(&huart2, &newstr, strlen(newstr),10);
	  //HAL_UART_Transmit_IT(&huart1, &play2, 10);
	  HAL_Delay(3000);
	  MP3_send_cmd(0x43, 0, 0);//Querycurrent status

	  HAL_Delay(3000);
	  MP3_send_cmd(0x09, 0, 2);//SpecifieddeviceismicroSD

	  HAL_Delay(3000);
	  MP3_send_cmd(0x4C, 0, 0);//Querycurrent trackinthe microSDcard

	  HAL_Delay(3000);
	  MP3_send_cmd(0x0C, 0, 0);//Reset
	  HAL_Delay(5000);
	  MP3_send_cmd(0x0D, 0, 0);//PLAY
	  HAL_Delay(10000);
	  MP3_send_cmd(MP3_NEXT, 0, 0);
	  HAL_Delay(3000);
	  MP3_send_cmd(MP3_PLAYBACK, MP3_PLAYBACK_MODE_Repeat, MP3_PLAYBACK_SOURCE_TF);
	  HAL_Delay(3000);
	  MP3_send_cmd(MP3_PLAYBACK, MP3_PLAYBACK_SOURCE_TF, MP3_PLAYBACK_MODE_Repeat);
	  HAL_Delay(3000);
	  MP3_send_cmd(MP3_PLAYBACK, 0, 0);
	  HAL_Delay(3000);
	  MP3_send_cmd(MP3_NEXT, 0, 0);
	  HAL_Delay(3000);
	  MP3_send_cmd(MP3_NEXT, 0, 0);
	  HAL_Delay(3000);
	  MP3_send_cmd(MP3_NEXT, 0, 0);
  }
	  //HAL_UART_Transmit(&huart2, &next2, strlen(next2), 20);
	  //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	  if(0)
	  {
	  if(HAL_GetTick()-delay>1000){
		  delay = HAL_GetTick();
		  echo1=HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_1);
		  echo2=HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_2);
		  dis1=echo1 / 58.0f;
		  dis2=echo2 / 58.0f;
		  printf("echo1: %u;  dis1: %.6f; echo2: %u; dis2: %.6f\r\n",echo1, dis1, echo2, dis2);


	  }
	  }

	  if(0){
	  printf("value: %4.2f\n",val);
	  val =val+0.37f;
	  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
//	  start = HAL_GetTick();

	  //start = TIM3->CNT;
	  start = DWT->CYCCNT;
	  //HAL_Delay(8);
	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
	  HAL_Delay(10);
	  //HAL_TIM_Base_Start(&htim3);
	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
	  HAL_Delay(100);
	  //HAL_TIM_Base_Stop(&htim3);
	  utoa(start,buf,10);
	  	  Log(buf);
	  	  //Log(itoa(start1,buf,10));
	  	  Log("<--start\r\n");
	  	  utoa(start1,buf,10);
	  	  Log(buf);
	  //	  Log(itoa(start2,buf,10));
	  	  Log("<--start1\r\n");
	  	  utoa(start2,buf,10);
	  	  Log(buf);
	  //	  Log(itoa(start3,buf,10));
	  	  Log("<--start2\r\n");
	  	  utoa(start3,buf,10);
	  	  Log(buf);
	  	  Log("<--start3\r\n");

	  	  startSum=start2-start1;
	  	  utoa(startSum,buf,10);
	  	  Log(buf);
	  	  Log("<--startSum\r\n");

	  	  //startSum=startSum/71999999*34000/2;
	  	  range=0.000236*startSum;
	  	  printf("time range: %4.8f\r\n",range);
	  	  //utoa(range,buf,10);
	  	  //buf=(char)range;
	  	  //Log(buf);
	  	  //Log("<--Lens cm\r\n");

	  	  utoa(echoCount,buf,10);
	  	  Log(buf);
	  	  Log("<--echoCount\r\n");

	  	  if(delta!=(uint32_t)0) {
	  		delta=start-delta;
	  	  }
	  	  if(delta==(uint32_t)0) {delta=start;}

	  	  utoa(delta,buf,10);
	  	  Log(buf);

	  	  Log("<--delta\r\n");
	  	  Log("\r\n\r\n");
	  	  start1=0u;
	  	  start2=0u;
	  	  start3=0u;
	  	  start=0u;
	  	  delta=0u;
	  	  echoCount=0u;
	  	  startSum=0u;


	  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
	  //-start;
	  //HAL_Delay(500);
	  //itoa(HAL_GetTick(),www,10);
	  //HAL_UART_Transmit(&huart2, &pdata, strlen(&pdata),10);
	  //Log(itoa(start,buf,20));
	  //Log(temp);
	  //getTime(&start);


/*	  utoa(start,buf,10);
	  Log(buf);
	  //Log(itoa(start1,buf,10));
	  Log("<--start\r\n");
	  utoa(start1,buf,10);
	  Log(buf);
//	  Log(itoa(start2,buf,10));
	  Log("<--start1\r\n");
	  utoa(start2,buf,10);
	  Log(buf);
//	  Log(itoa(start3,buf,10));
	  Log("<--start2\r\n");
	  utoa(start3,buf,10);
	  Log(buf);
	  Log("<--start3\r\n");
	  startSum=start1-start;
	  utoa(startSum,buf,10);
	  Log(buf);
	  Log("<--startSum\r\n");
	  utoa(echoCount,buf,10);
	  Log(buf);
	  Log("<--echoCount\r\n");
	  Log("\r\n\r\n");
	  start1=0u;
	  start2=0u;
	  start3=0u;
	  start=0u;
	  echoCount=0u;
	  startSum=0u;
*/

	  /*if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)==GPIO_PIN_RESET)
	  {
		  for(int i=1;i<5;i++)
		  {
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			  HAL_Delay(200);
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
			  HAL_Delay(200);
  		  }
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	  }*/

	  HAL_Delay(759);
	  //HAL_UART_Transmit(&huart2, uartString, strlen(uartString), 500);
	  //HAL_Delay(500);
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
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

  /** Initializes the CPU, AHB and APB busses clocks 
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
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 10;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 179;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 39999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI2FP2;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void Log(char *msg)
{
	//uint8_t pData = *msg;
	uint16_t size = strlen(msg);
	HAL_UART_Transmit(&huart2, msg, size, 20);
}

//extern uint32_t start1 = 0;
//extern uint32_t start2 = 0;
//extern uint32_t start3 = 0;


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	char err[] = "void Error_Handler";
  /* User can add his own implementation to report the HAL error return state */
	Log(err);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
