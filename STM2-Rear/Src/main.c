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

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */

//-----CAN variables-----//
CAN_HandleTypeDef hcan1;
CAN_TxHeaderTypeDef Message_Header; //declare a specific header for message transmittions
CAN_RxHeaderTypeDef Filter_Header; //declare header for message reception
uint32_t TxMailbox;//mailbox for pending transmitions
//uint8_t message_out; 
uint8_t message_in; //declare a send and receive bytes

//Declare CAN filter structures
CAN_FilterTypeDef sFilterConfig1; //general header
CAN_FilterTypeDef sFilterConfig2; //header lights messages (from STM4)
CAN_FilterTypeDef sFilterConfig3; //throttle position (from STM1)
CAN_FilterTypeDef sFilterConfig4; //regen position (from STM1)
CAN_FilterTypeDef sFilterConfig5; //error messages

bool kill_switch=0;

//---FLAG DEFINIIONS---///
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
volatile bool haz_state=0;

//these are for the specific state of the physical light itself
volatile bool lts_light_state;
volatile bool rts_light_state;
volatile bool haz_light_state;


volatile int state;
volatile bool fault_flag;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM2_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */
static void User_GPIO_Init(void);
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
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_DAC_Init();
  MX_TIM2_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
	
	//Setup GPIO
	User_GPIO_Init();
	//Added delay to allow motor controller to start up
	msDelay(5000);
	//start DACs
	HAL_DAC_Start(&hdac, DAC1_CHANNEL_1);
	HAL_DAC_Start(&hdac, DAC1_CHANNEL_2);
	
  //-----CAN SETUP-----//
  Message_Header.DLC=1; //give message size of 1 byte
  Message_Header.IDE=CAN_ID_STD; //set identifier to standard
  Message_Header.RTR=CAN_RTR_DATA; //set data type to remote transmission request?
  
	//---General Purpose Filter---//
  sFilterConfig1.FilterFIFOAssignment=CAN_FILTER_FIFO0; //set fifo assignment
	sFilterConfig1.FilterMode = CAN_FILTERMODE_IDLIST;  //list mode
	sFilterConfig1.FilterBank = 1;											//set filter bank
  sFilterConfig1.FilterIdHigh=0x200<<5; //the ID that the filter looks for (THIS IS YOUR ADDRESS)
  sFilterConfig1.FilterIdLow=0;
  sFilterConfig1.FilterScale=CAN_FILTERSCALE_32BIT; //set filter scale
  sFilterConfig1.FilterActivation=ENABLE;
	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig1); //configure CAN filter
	
	//---Blinker Control Filter---//
  sFilterConfig2.FilterFIFOAssignment=CAN_FILTER_FIFO0; //set fifo assignment
	sFilterConfig2.FilterMode = CAN_FILTERMODE_IDMASK;  //mask mode
	sFilterConfig2.FilterBank = 2;											//set filter bank
  sFilterConfig2.FilterIdHigh=0x240<<5; //the ID that the filter looks for (THIS IS YOUR ADDRESS)
  sFilterConfig2.FilterIdLow=0;
  sFilterConfig2.FilterMaskIdHigh=0x0FF<<5;
  sFilterConfig2.FilterMaskIdLow=0x0FF<<5;
  sFilterConfig2.FilterScale=CAN_FILTERSCALE_32BIT; //set filter scale
  sFilterConfig2.FilterActivation=ENABLE;
  HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig2); //configure CAN filter

  //---Throttle Position Filter---//
  sFilterConfig3.FilterFIFOAssignment=CAN_FILTER_FIFO0; //set fifo assignment
	sFilterConfig3.FilterMode = CAN_FILTERMODE_IDLIST;  //list mode
	sFilterConfig3.FilterBank = 3;											//set filter bank
  sFilterConfig3.FilterIdHigh=0x210<<5; //the ID that the filter looks for (THIS IS YOUR ADDRESS)
  sFilterConfig3.FilterIdLow=0;
  sFilterConfig3.FilterScale=CAN_FILTERSCALE_32BIT; //set filter scale
  sFilterConfig3.FilterActivation=ENABLE;
  HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig3); //configure CAN filter


  //---Regen Position Filter---//
  sFilterConfig4.FilterFIFOAssignment=CAN_FILTER_FIFO0; //set fifo assignment
	sFilterConfig4.FilterMode = CAN_FILTERMODE_IDLIST;  //list mode
	sFilterConfig4.FilterBank = 4;											//set filter bank
  sFilterConfig4.FilterIdHigh=0x220<<5; //the ID that the filter looks for (THIS IS YOUR ADDRESS)
  sFilterConfig4.FilterIdLow=0;
  sFilterConfig4.FilterScale=CAN_FILTERSCALE_32BIT; //set filter scale
  sFilterConfig4.FilterActivation=ENABLE;
  HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig4); //configure CAN filter


	//---Error Messages Filter---//
  sFilterConfig5.FilterFIFOAssignment=CAN_FILTER_FIFO0; //set fifo assignment
	sFilterConfig5.FilterMode = CAN_FILTERMODE_IDMASK;  //mask mode
	sFilterConfig5.FilterBank = 5;											//set filter bank
  sFilterConfig5.FilterIdHigh=0x2EE<<5; //the ID that the filter looks for (THIS IS YOUR ADDRESS)
  sFilterConfig5.FilterIdLow=0;
  sFilterConfig5.FilterMaskIdHigh=0x7FF<<5;
  sFilterConfig5.FilterMaskIdLow=0x7FF<<5;
  sFilterConfig5.FilterScale=CAN_FILTERSCALE_32BIT; //set filter scale
  sFilterConfig5.FilterActivation=ENABLE;
  HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig5); //configure CAN filter


  HAL_CAN_Start(&hcan1); //start CAN
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); //enable interrupts
  
	
	//TODO: DELETE THIS!!!
	HAL_DAC_SetValue(&hdac, DAC1_CHANNEL_1, DAC_ALIGN_8B_R, 0x00);
	HAL_DAC_SetValue(&hdac, DAC1_CHANNEL_2, DAC_ALIGN_8B_R, 0x00);

	//start speed collection timers
	HAL_TIM_Base_Start_IT(&htim5);
	HAL_TIM_Base_Start(&htim2);
	
	//start blinker control timers
	HAL_TIM_Base_Start_IT(&htim4);
	
	//GPIOD->ODR = 0xFFFFFFFF;
	
	GPIOD->BSRR = 0x00000100;
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	switch(state){
		//matt=matt
		//-----NORMAL OPERATION STATES-----//
		case 0:
			//check to see if the killswitch is high
			kill_switch = Debounced_ReadPin(GPIOE, GPIO_PIN_0);
			//if killswitch is high tell STM3 to turn off the car
			if(kill_switch==1){
				  //broadcast error message
					CAN_Send(0xE1, 0x0EE);
					//stop DACs
					HAL_DAC_Stop(&hdac, DAC1_CHANNEL_1);
					HAL_DAC_Stop(&hdac, DAC1_CHANNEL_2);
					//trip fault flag
					fault_flag=1;
					}
			state++;
			break;
					
    //check right turn signals
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
				//force update event

				//ensure lights are off
				GPIOC->BSRR=0x02000000;
			  //stop turn signals
				rts_off_flag=0;
			  }
			state++;
			break;
				
		//check left turn signals	
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
			  //stop turn signals
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
			
			state=0;
			//enter fault states if needed
			if(fault_flag == 1)
				state=4;
			
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
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /**DAC Initialization 
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /**DAC channel OUT1 config 
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /**DAC channel OUT2 config 
  */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_TI2FP2;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_FALLING;
  sSlaveConfig.TriggerFilter = 15;
  if (HAL_TIM_SlaveConfigSynchronization(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

static void User_GPIO_Init(void){
  //enable clocks
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
	
	//enable PD12-PD15 debugging lights
  GPIOD->MODER=0x55555555;
	//enable PC5, PC7, PC8-PC12 as outputs
	GPIOC->MODER=0xFD5577FF;
	//enable PE0 as an input
	GPIOE->MODER=0xFFFFFFFC;
	//ensure all pulldown resistors are activated
  GPIOE->PUPDR=0xAAAAAAAA; 
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
