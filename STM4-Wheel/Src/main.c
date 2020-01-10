/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
//-----CAN VARIABLES-----//
CAN_HandleTypeDef hcan1;
CAN_TxHeaderTypeDef Message_Header; //declare a specific header for message transmittions
CAN_RxHeaderTypeDef Filter_Header; //declare header for message reception
uint32_t TxMailbox;
uint8_t a,r; //declare a send byte
CAN_FilterTypeDef sFilterConfig1; //declare CAN filter structure
CAN_FilterTypeDef sFilterConfig2; //declare CAN filter structure
//button state variables
bool rts; 
bool lts;  
bool CC_enable;
bool CC_inc;
bool CC_dec; //also the set button
bool horn;
bool running_lights;


//used t opreform debounce
bool rts_debounce;
bool lts_debounce;
bool CC_enable_debounce;
bool CC_inc_debounce;
bool CC_dec_debounce;
bool horn_debounce;
bool running_lights_holder;


//these remember previous button states. Used to detect a change
bool lts_holder;
bool rts_holder;
bool CC_enable_holder;
bool CC_inc_holder;
bool CC_dec_holder;
bool horn_holder;
bool running_lights_debounce;

bool kill_switch; //TODO: wtf is this?
int state;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
static void USER_GPIO_Init(void);
void msDelay(uint32_t msTime);
int Debounced_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void CAN_Send(uint8_t message, uint16_t address);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_CAN1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	USER_GPIO_Init();
	
	//-----CAN SETUP-----//
	
	//Header Config
  Message_Header.DLC=1; //give message size of 1 byte
  Message_Header.IDE=CAN_ID_STD; //set identifier to standard
  Message_Header.RTR=CAN_RTR_DATA; //set data type to remote transmission request?
  
	//---General Purpose Filter---//
  sFilterConfig1.FilterFIFOAssignment=CAN_FILTER_FIFO0; //set fifo assignment
	sFilterConfig1.FilterMode = CAN_FILTERMODE_IDLIST;  //list mode
	sFilterConfig1.FilterBank = 1;											//set filter bank
  sFilterConfig1.FilterIdHigh=0x400<<5; //the ID that the filter looks for (THIS IS YOUR ADDRESS)
  sFilterConfig1.FilterIdLow=0;
  sFilterConfig1.FilterScale=CAN_FILTERSCALE_32BIT; //set filter scale
  sFilterConfig1.FilterActivation=ENABLE;
	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig1); //configure CAN filter
	
	//---Mass Communications Filter---//
  sFilterConfig2.FilterFIFOAssignment=CAN_FILTER_FIFO0; //set fifo assignment
	sFilterConfig2.FilterMode = CAN_FILTERMODE_IDMASK;  //mask mode
	sFilterConfig2.FilterBank = 1;											//set filter bank
  sFilterConfig2.FilterIdHigh=0x4EE<<5; //the ID that the filter looks for (THIS IS YOUR ADDRESS)
  sFilterConfig2.FilterIdLow=0;
  sFilterConfig2.FilterMaskIdHigh=0x0FF<<5;
  sFilterConfig2.FilterMaskIdLow=0x0FF<<5;
  sFilterConfig2.FilterScale=CAN_FILTERSCALE_32BIT; //set filter scale
  sFilterConfig2.FilterActivation=ENABLE;
  HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig2); //configure CAN filter

  HAL_CAN_Start(&hcan1); //start CAN
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); //enable interrupts
 
 
  //start ADC and timer it relies on
	HAL_TIM_Base_Start(&htim2);
	HAL_ADC_Start_IT(&hadc1);
	

	GPIOC->ODR=0xFFFF;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		switch(state){
	
  		//states 0-2 preform a level triggered debounce on all inputs
			//must be done like this or else debounce times will stack
			case 0:
				lts_debounce=HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0);	
			  rts_debounce=HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1);
				CC_enable_debounce=HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2);
			  CC_inc_debounce=HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3);
			  CC_dec_debounce=HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4);
			  horn_debounce=HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5);
			  running_lights_debounce=HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_6);
			  msDelay(100);
			  state++;
				break;
			
			case 1:
				if(lts_debounce==HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0)){
				  lts=lts_debounce;
				  }
				if(rts_debounce==HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1)){
				  rts=rts_debounce;
				  }
				if(CC_enable_debounce==HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2)){
				  CC_enable=CC_enable_debounce;
				  }
				if(CC_inc_debounce==HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3)){
				  CC_inc=CC_inc_debounce;
				  }
				if(CC_dec_debounce==HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4)){
				  CC_dec=CC_dec_debounce;
				  }
				if(horn_debounce==HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5)){
				  horn=horn_debounce;
				  }				
				if(running_lights_debounce==HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_6)){
				  running_lights=running_lights_debounce;
				  }
				state++;
				break;
			
		  case 2:
				//TODO: write all CAN messages!!!
			  //if low to high
			  if(lts==!lts_holder && lts_holder==0){
					//send lts on
				  CAN_Send(0x03, 0x240);
					lts_holder=lts;
					//if high to low
				  }else if(lts==!lts_holder && lts_holder==1){
					//send lts off
				  CAN_Send(0x04, 0x240);
					lts_holder=lts;			
					}
				
        //if low to high					
			  if(rts==!rts_holder && rts_holder==0){
					//send rts off
					CAN_Send(0x02, 0x240);
					rts_holder=rts;		
					//if high to low
				  }else if(rts==!rts_holder && rts_holder==1){
					//send rts on
					CAN_Send(0x01, 0x240);
					rts_holder=rts;
					}
			  
				//if low to high	
				if(CC_enable==!CC_enable_holder && CC_enable_holder==0){
				  //send CC enable
					CAN_Send(0x07, 0x200);
					CC_enable_holder=CC_enable;
					//if high to low
				  }else if(CC_enable==!CC_enable_holder && CC_enable_holder==1){
				  //send CC disable
					CC_enable_holder=CC_enable;			
					}
			  
				//if low to high	
				if(CC_inc==!CC_inc_holder && CC_inc_holder==0){
				  //send CC inc
					CAN_Send(0x08, 0x200);
					CC_inc_holder=CC_dec;
					//if high to low
				  }else if(CC_dec==!CC_inc_holder && CC_inc_holder==1){
				  //pass
					CC_inc_holder=CC_inc;			
					}
					
				//if low to high	
			  if(CC_dec==!CC_dec_holder && CC_dec_holder==0){
				  //send CC dec (Also CC set)
					CAN_Send(0x09, 0x200);
					CC_dec_holder=CC_dec;
					//if high to low
				  }else if(CC_dec==!CC_dec_holder && CC_dec_holder==1){
				  //pass
					CC_dec_holder=CC_dec;			
					}
					
				//if low to high	
			  if(horn==!horn_holder && horn_holder==0){
				  //send horn off(if this one breaks we're fucked lmao)
					CAN_Send(0x11, 0x300);
					GPIOD->ODR=0x0000;
				  horn_holder=horn;		
					//if high to low
				  }else if(horn==!horn_holder && horn_holder==1){
					//send horn on											
					CAN_Send(0x10, 0x300);
					GPIOD->ODR=0xFFFF;
					horn_holder=horn;	
					}					
				state=0;
				
				//if low to high
			  if(running_lights==!running_lights_holder && running_lights_holder==0){
				  CAN_Send(0x21, 0x300);
					running_lights_holder=running_lights;		
					//if high to low
				  }else if(running_lights==!running_lights_holder && running_lights_holder==1){
					CAN_Send(0x20, 0x300);
					
					running_lights_holder=running_lights;						
					}
				break;

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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 2;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 60;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
static void USER_GPIO_Init(void){
	//enable clocks
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

	//enable PD12-PD15 debugging lights
  GPIOD->MODER=0x55FFFFFF;
	//enable PE0-PE5 as inputs
	GPIOE->MODER=0xFFFFC000;
	
	//DELETE
	GPIOC->MODER=0x55555555;
	
	//ensure all pulldown resistors are activated
  GPIOE->PUPDR=0xAAAA5555; 
  }
//simple debounced read (using HAL)
int Debounced_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin){
  int pin_holder=HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
  msDelay(100);
  if(pin_holder==HAL_GPIO_ReadPin(GPIOx, GPIO_Pin)){
		return pin_holder;
		}
	return 0;
}


//funtion to send CAN message (using HAL)
void CAN_Send(uint8_t message_out, uint16_t address){
	Message_Header.StdId=address;//define a standard identifier, used for message identification by filters
	HAL_CAN_AddTxMessage(&hcan1, &Message_Header, &message_out, &TxMailbox);	
}

//function uses clock cycles to create a delay
void msDelay(uint32_t msTime){
	for(uint32_t i=0; i<msTime*4000;i++);
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
