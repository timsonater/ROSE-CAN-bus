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
CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

//-----CAN VARIABLES-----//
//Define CAN variables
CAN_HandleTypeDef hcan1;
CAN_TxHeaderTypeDef Message_Header; //declare a specific header for message transmittions
CAN_RxHeaderTypeDef Filter_Header; //declare header for message reception
uint32_t TxMailbox;//mailbox for pending transmitions 
uint8_t message_in; //declare a send and receive bytes

//Declare CAN filter structures
CAN_FilterTypeDef sFilterConfig1; //general header
CAN_FilterTypeDef sFilterConfig2; //header lights messages 
CAN_FilterTypeDef sFilterConfig3; //error messages

//define a state variable
volatile int state=0;
volatile bool fault_flag = 0;
int precharge_timeout=50;  //5 second delay
bool precharge_status=0;

//---FLAG DEFINIIONS---///
//startup flags
bool battery_ok_input;
bool precharge_ok_input;

//these control whether has indicated something should turn on or off
volatile bool lts_on_flag;
volatile bool lts_off_flag;
volatile bool rts_on_flag;
volatile bool rts_off_flag;
volatile bool haz_on_flag;
volatile bool haz_off_flag;


//these indicate whether something is currently activated
volatile bool rts_state;
volatile bool lts_state;
volatile bool haz_state;

//these are for the specific state of the physical light itself
volatile bool lts_light_state;
volatile bool rts_light_state;
volatile bool haz_light_state;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
static void User_GPIO_Init(void);
int Debounced_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void msDelay(uint32_t msTime);
void CAN_Send(uint8_t message_out, uint16_t address);

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
	msDelay(500);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	User_GPIO_Init();
	
	
	//-----CAN SETUP-----//
  Message_Header.DLC=1; //give message size of 1 byte
  Message_Header.IDE=CAN_ID_STD; //set identifier to standard
  Message_Header.RTR=CAN_RTR_DATA; //set data type to remote transmission request?
  
	//---General Purpose Filter---//
  sFilterConfig1.FilterFIFOAssignment=CAN_FILTER_FIFO0; //set fifo assignment
	sFilterConfig2.FilterMode = CAN_FILTERMODE_IDLIST;  //mask mode
	sFilterConfig2.FilterBank = 1;											//set filter bank
  sFilterConfig1.FilterIdHigh=0x300<<5; //the ID that the filter looks for (THIS IS YOUR ADDRESS)
  sFilterConfig1.FilterIdLow=0;
  sFilterConfig1.FilterScale=CAN_FILTERSCALE_32BIT; //set filter scale
  sFilterConfig1.FilterActivation=ENABLE;
	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig1); //configure CAN filter
	
	//---Blinker Control Filter---//
  sFilterConfig2.FilterFIFOAssignment=CAN_FILTER_FIFO0; //set fifo assignment
	sFilterConfig2.FilterMode = CAN_FILTERMODE_IDMASK;  //mask mode
	sFilterConfig2.FilterBank = 2;											//set filter bank
  sFilterConfig2.FilterIdHigh=0x340<<5; //the ID that the filter looks for (THIS IS YOUR ADDRESS)
  sFilterConfig2.FilterIdLow=0;
  sFilterConfig2.FilterMaskIdHigh=0x0FF<<5;
  sFilterConfig2.FilterMaskIdLow=0x0FF<<5;
  sFilterConfig2.FilterScale=CAN_FILTERSCALE_32BIT; //set filter scale
  sFilterConfig2.FilterActivation=ENABLE;
  HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig2); //configure CAN filter

  //---Error Messages Filter---//
  sFilterConfig3.FilterFIFOAssignment=CAN_FILTER_FIFO0; //set fifo assignment
	sFilterConfig2.FilterMode = CAN_FILTERMODE_IDMASK;  //mask mode
	sFilterConfig2.FilterBank = 3;											//set filter bank
  sFilterConfig3.FilterIdHigh=0x3EE<<5; //the ID that the filter looks for (THIS IS YOUR ADDRESS)
  sFilterConfig3.FilterIdLow=0;
  sFilterConfig3.FilterMaskIdHigh=0x0FF<<5;
  sFilterConfig3.FilterMaskIdLow=0x7FF<<5;
  sFilterConfig3.FilterScale=CAN_FILTERSCALE_32BIT; //set filter scale
  sFilterConfig3.FilterActivation=ENABLE;
  HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig3); //configure CAN filter
	
  HAL_CAN_Start(&hcan1); //start CAN
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); //enable interrupts

	//start blinker control timers
	HAL_TIM_Base_Start_IT(&htim4);
	//start battery fault timer
	HAL_TIM_Base_Start_IT(&htim3);
	
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		switch(state){

			//----PRECHARGE STATE----//	
			case 0:	
				if(fault_flag){
					break;
				}
			
				//step 1: check BMS signal to ensure battery is safe (PE10)
				//(should be grounded, may not be usable)
				battery_ok_input = ((GPIOE->IDR >> 10) & 0x0001);  
				if(battery_ok_input){
					state=4;
					break;				
				}
				
				//activate precharge PC10
				GPIOC->BSRR = 0x00000400;
				
				if(fault_flag){
					break;
				}
				msDelay(1500);

				//make sure precharge is ok PC0
				precharge_ok_input = (GPIOC->IDR &0x0001);
				if(!precharge_ok_input){
					state=4;
					break;
				}
				
				//close the positive contactor PC11 and PC9
				GPIOC->BSRR = 0x00000A00;
				
				if(fault_flag){
					break;
				}
				msDelay(500);
				//cut precharge board off PC10
				GPIOC->BSRR = 0x04000000;
				//set PRB to Main pack by setting PE6 high
				GPIOE->BSRR = 0x00000040;
				msDelay(1000);
				//cut auxillary power by seting PD9 low
				GPIOD->BSRR = 0x02000000;
				
				//EMERGENCY POWER SWITCH TEST
				//hold 24V-12V DC DC high: PD9
				//GPIOD->BSRR=0x00000200;
				//msDelay(200);
				//cut off positive contactor PC11 and PC9
				//GPIOC->BSRR = 0x0A000000;
				//set PRB to Auxillary power by setting PE6 low
				//GPIOE->BSRR = 0x00400000;
				
				
				//main power is now on enter normal operation states
				state=1;
				GPIOD->ODR |= 0x1000;
				break;
				
			//----NORMAL OPERATION STATES----//		
			//check right turn signals (PC9)
			case 1:
				if(rts_on_flag==1){
					
					//ensure other turn signal is off
					GPIOC->BSRR=0x03000000;
					lts_state=0;
					lts_light_state=0;
					
					//set state to 1
					rts_state=1;
					//timer will interrupt immediately so set this to turn on turn signal
					rts_light_state=0;				
					//reload prescaler by forcing update event
					TIM4->EGR |=TIM_EGR_UG;	
				
					rts_on_flag=0;
					
					}
				if(rts_off_flag==1 && lts_state==0){
					
					//set state to 0
					rts_state=0;
					//ensure lights are off
					GPIOC->BSRR=0x02000000;
					rts_off_flag=0;
					}
				state++;
				break;
					
			//check left turn signals	(PC8)
			case 2:
				if(lts_on_flag==1){
			
					//ensure other turn signal is off
					GPIOC->BSRR=0x03000000;
					rts_state=0;
					rts_light_state=0;
					
					//set state to 1
					lts_state=1;
					//timer will interrupt immediately so set this to turn on turn signal
					lts_light_state=0;
					//reset the turn signal timer
					TIM4->EGR |=TIM_EGR_UG;	//reload prescaler by forcing update event
					
					lts_on_flag=0;
					}
				
				if(lts_off_flag==1 && rts_state==0){
					
					//ensure other turn signal is off
					GPIOC->BSRR=0x03000000;
							
					//set state to 0
					lts_state=0;
					//ensure lights are off
					GPIOC->BSRR=0x01000000;
					lts_off_flag=0;
					}
				state++;
				break;
					
			//check the hazards
			case 3:
				if(haz_on_flag==1){
					
					//set state to 1
					haz_state=1;
					//timer will interrupt immediately so set these to turn on turn signals
					haz_light_state=0;
					//reset the turn signal timer
					TIM4->EGR |=TIM_EGR_UG;
					
					haz_on_flag=0;
					}
				if(haz_off_flag==1){

					//set states to 0
					haz_state=0;	
					//ensure lights are off
					GPIOC->BSRR=0x03000000;
					//stop turn signals
					haz_off_flag=0;
					}
				state=1;
				break;
					
				
					
					
			 //-----FAULT STATES-----//
			
			//turn off contactors
			case 4:
				//cut off positive contactor PC11 and PC9
				GPIOC->BSRR = 0x0A000000;
				//cut precharge board off (if its on) PC10
				GPIOC->BSRR = 0x04000000;
				//return to normal states to retain some operation of the car
				state=1;
				break;
				}
			}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
			
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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  htim3.Init.Prescaler = 100;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 550;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 6300;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 550;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

static void User_GPIO_Init(void){
	//enable clocks
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	
	//enable PD12-PD15 debugging lights as outputs and PD9 as output
  GPIOD->MODER=0x55F7FFFF;
	//enable PC8-PC11, PC4 as outputs, PC0 as input
	GPIOC->MODER=0xFF55F7FC;
	//enable PE0, PE10 as input, PE6, PE14 as output
	GPIOE->MODER=0xDFCFDFFC;	
	//enable PA0 as input
	GPIOA->MODER=0xA8000000; //reset state
	
	
	//enable pullup on PE10, switching from pulldown to force rising edge interrupt
	GPIOE->PUPDR=0x55555555;
	//enable pullup on PA0
	GPIOA->PUPDR=0x64000001;
  }
//TODO: if this reads two different things it returns 0, that could be bad
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

//this function makes sure the main power is still
int Check_Main_Power(void){
  int main_power=Debounced_ReadPin(GPIOE, GPIO_PIN_0);
	//make sure power is on
	if(main_power==1){
		//increment state if the car just turned on
		if(state==0){
		  state++;
		  }
		return 1;
	}else{
		//enter fault state
	  state=10;
		return 0;
	}
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
