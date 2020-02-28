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
#include "stm32f4xx_it.h"

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

//----CAN VARIABLES-----//
CAN_HandleTypeDef hcan1;
CAN_TxHeaderTypeDef Message_Header; //declare a specific header for message transmittions
CAN_RxHeaderTypeDef Filter_Header; //declare header for message reception
uint32_t TxMailbox;//mailbox for pending transmitions
uint8_t message_out, message_in; //declare a send and receive bytes
CAN_FilterTypeDef sFilterConfig1; //general purpose filter
CAN_FilterTypeDef sFilterConfig2; //error messages filter



//----FLAG DEFINITIONS-----//
volatile bool CAN_check_result=0;
volatile bool precharge_ok=0;

bool main_pwr;
bool haz;
bool motor_on;
bool forward_reverse;
bool power_eco;
bool break_lights;
bool power_off;

bool main_pwr_holder;
bool haz_holder;
bool motor_on_holder;
bool forward_reverse_holder;
bool power_eco_holder;
bool break_lights_holder;
bool power_off_holder;

//debounce variables
bool main_pwr_debounce;
bool haz_debounce;
bool motor_on_debounce;
bool forward_reverse_debounce;
bool power_eco_debounce;
bool break_lights_debounce;
bool power_off_debounce;


//define state machine state
//NOTE: Starting at 4 to skip precharge commands
volatile int state=4;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

int Debounced_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
int Check_Main_Power(void);
int CAN_check(void);
void msDelay(uint32_t msTime);
void CAN_Send(uint8_t message_out, uint16_t address);
static void User_GPIO_Init(void);



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
	
	//initialize GPIO
	User_GPIO_Init();

  
  //-----CAN SETUP-----//
	Message_Header.DLC=1; //give message size of 1 byte
  Message_Header.IDE=CAN_ID_STD; //set identifier to standard
  Message_Header.RTR=CAN_RTR_DATA; //set data type to remote transmission request?
  
  //---General Purpose Filter---//
  sFilterConfig1.FilterFIFOAssignment=CAN_FILTER_FIFO0; //set fifo assignment
	sFilterConfig1.FilterMode = CAN_FILTERMODE_IDLIST;  //list mode
	sFilterConfig1.FilterBank = 1;											//set filter bank
  sFilterConfig1.FilterIdHigh=0x100<<5; //the ID that the filter looks for (THIS IS YOUR ADDRESS)
  sFilterConfig1.FilterIdLow=0;
  sFilterConfig1.FilterScale=CAN_FILTERSCALE_32BIT; //set filter scale
  sFilterConfig1.FilterActivation=ENABLE;
  HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig1); //configure CAN filter
  
  //---Error Messages Filter---//
  sFilterConfig2.FilterFIFOAssignment=CAN_FILTER_FIFO0; //set fifo assignment
	sFilterConfig2.FilterMode = CAN_FILTERMODE_IDMASK;  //mask mode
	sFilterConfig2.FilterBank = 2;											//set filter bank
  sFilterConfig2.FilterIdHigh=0x1EE<<5; //the ID that the filter looks for (THIS IS YOUR ADDRESS)
  sFilterConfig2.FilterIdLow=0;
  sFilterConfig2.FilterMaskIdHigh=0x0FF<<5;
  sFilterConfig2.FilterMaskIdLow=0x0FF<<5;
  sFilterConfig2.FilterScale=CAN_FILTERSCALE_32BIT; //set filter scale
  sFilterConfig2.FilterActivation=ENABLE;
  HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig2); //configure CAN filter	

  //start CAN
  HAL_CAN_Start(&hcan1); 
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); //enable interrupts


	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
	switch(state){
		
			//NOTE: Commented out since precharge is currently done by BMS
		
			/*
			//power has just turned on, check to ensure main power input is high, and fault state is low
			case 0:
		    Check_Main_Power();
			  break;
			
			//do a quick check of the CAN network
			case 1:
				//small wait so all STMs can power on
			  HAL_Delay(200);
				
				//transmit CAN check message which travels to all STMs, starting with STM1
			  //TODO: make this actuall go to all of them (they all need to be powered lol)
				CAN_Send(0xFF, 0x244);
			  			 	
			  //wait a small amount of time and ensure the CAN check worked
			  msDelay(10);
			  //CAN_check_result=1;
			  if(CAN_check_result==1){
					state++;
					//reset the check result
					CAN_check_result=0;
				}else{
					//enter fault state
				  state=10;
				}
				break;
			
			//check BMS communications
			//TODO: finish this lol
			case 2:
				//pass (for now)
			  GPIOD->ODR=0x1000;
			  msDelay(1000);
			  state++;
			  break;
			
			//precharge state: turn on relay, check if its complete
			case 3:
				
				//send message: Start Precharge to STM3
        CAN_Send(0xFE, 0x300);
			  Check_Main_Power();
				*/
			  /*TODO: wait here should act as a reasonable amount of time the precharge should take,
			          otherwise the car should enter a fault state*/
			  /*
				msDelay(100);
			  //check on our main power while waiting for precharge to finish
			  while(1){
					int power_ok=Check_Main_Power();
				  if(power_ok==0){
						//send error message
						CAN_Send(0xE2, 0x0EE);
						state=10;
						break;
					}
					GPIOD->ODR=0x2000;
					if(precharge_ok==1){
					   state++;
						 break;
					}
				}
				break;
			*/
				
			//begin throttle position broadcast	
			case 4:
				//start ADC and timer it relies on
			  HAL_TIM_Base_Start(&htim2);
	      HAL_ADC_Start_IT(&hadc1);
			  state++;
				break;
			
			//----NORMAL OPERATION STATES----//
			
			//states 5-7 preform a level triggered debounce on all inputs
			//must be done like this or else debounce times will stack
			case 5:
				main_pwr_debounce=HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0);
			  forward_reverse_debounce=HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1);
			  motor_on_debounce=HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4);
			  haz_debounce=HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_6);
			  power_eco_debounce=HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_7);
				break_lights_debounce=HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10);   //PE10!!!
				power_off_debounce=HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9);
			  msDelay(100);
			  state++;
				break;
			
			case 6:
				if(main_pwr_debounce==HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0)){
				  main_pwr=main_pwr_debounce;
				  }
				if(forward_reverse_debounce==HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2)){
				  forward_reverse=forward_reverse_debounce;
				  }
				if(motor_on_debounce==HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4)){
				  motor_on=motor_on_debounce;
				  }
				if(haz_debounce==HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_6)){
				  haz=haz_debounce;
				  }
				if(power_eco_debounce==HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_7)){
				  power_eco=power_eco_debounce;
				  }
				if(break_lights_debounce==HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10)){
				  break_lights=break_lights_debounce;
				  }
				if(break_lights_debounce==HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_8)){
				  break_lights=break_lights_debounce;
				  }
				if(power_off_debounce==HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_9)){
				  power_off=power_off_debounce;
				  }
				state++;
				break;
			
		  case 7:
				//TODO: write all CAN messages!!!
			
			  //if low to high
			  if(main_pwr==!main_pwr_holder && main_pwr_holder==0){
					//broadcast error message
					//CAN_Send(0x0EE, 0xE2);
					main_pwr_holder=main_pwr;	
					//if high to low
				  }else if(main_pwr==!main_pwr_holder && main_pwr_holder==1){
					//pass (this shouldn't even happen/matter)
					main_pwr_holder=main_pwr;
					}
					
	
			  //if low to high
				//TODO: figure out exactly how all these switches work
			  if(forward_reverse==!forward_reverse_holder && forward_reverse_holder==0){
				  CAN_Send(0x16, 0x200);
					forward_reverse_holder=forward_reverse;
					//if high to low
				  }else if(forward_reverse==!forward_reverse_holder && forward_reverse_holder==1){
				  CAN_Send(0x17, 0x200);
					forward_reverse_holder=forward_reverse;		
					}
					
					
			  //if low to high
			  if(motor_on==!motor_on_holder && motor_on_holder==0){
				  CAN_Send(0x18, 0x200); //send motor on
					motor_on_holder=motor_on;
					GPIOD->ODR=0xFFFF;
					//if high to low
				  }else if(motor_on==!motor_on_holder && motor_on_holder==1){
				  CAN_Send(0x19, 0x200);  //send motor off
						GPIOD->ODR=0x0000;
					motor_on_holder=motor_on;
					}
					
				//if low to high
			  if(haz==!haz_holder && haz_holder==0){
					//send Haz off
				  CAN_Send(0x06, 0x240);
					haz_holder=haz;
					//if high to low
				  }else if(haz==!haz_holder && haz_holder==1){
					//send Haz on
				  CAN_Send(0x05, 0x240);
					haz_holder=haz;					
					}

			  //if low to high
			  if(power_eco==!power_eco_holder && power_eco_holder==0){
					//send power mode to stm2
				  CAN_Send(0x14, 0x200);
					power_eco_holder=power_eco;
					//if high to low
				  }else if(power_eco==!power_eco_holder && power_eco_holder==1){
				  //send eco mode to stm2
					CAN_Send(0x15, 0x200);
					power_eco_holder=power_eco;
					}


			  //if low to high
			  if(break_lights==!break_lights_holder && break_lights_holder==0){
				  CAN_Send(0x12, 0x200); //send "brake lights on" to turn on break lights
					GPIOD->ODR = 0xFFFF;
					break_lights_holder=break_lights;
					//if high to low
				  }else if(break_lights==!break_lights_holder && break_lights_holder==1){
					CAN_Send(0x13, 0x200);  //sending "brake lights off" to disable CC
						GPIOD->ODR = 0x0000;
					break_lights_holder=break_lights;					
					}					
					
				//if low to high
			  if(power_off==!power_off_holder && power_off_holder==0){
					power_off_holder=power_off;
					//if high to low
				  }else if(power_off==!power_off_holder && power_off_holder==1){
					CAN_Send(0xF1, 0x300);	//send signal to turn  
					power_off_holder=power_off;					
					}					
				state=5;
				break;
					
			//fault state (hangup)
		  case 10:
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
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
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
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
  htim2.Init.Prescaler = 20;
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

static void User_GPIO_Init(void){
	//enable clocks
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	
	//enable PD12-PD15 debugging lights
  GPIOD->MODER=0x55FFFFFF;
	//enable PE0-PE10 as inputs
	GPIOE->MODER=0xFFC00000;

  //ensure all pulldown resistors are activated (CHECK PE9 works)
  GPIOE->PUPDR=0xAAAA5AAA; 
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
