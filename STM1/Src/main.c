#include "main.h"

//function declarations
static void GPIO_Init(void);
void SystemClock_Config(void);
static void MX_CAN1_Init(void);
void msDelay(uint32_t msTime);

//Define CAN variables
CAN_HandleTypeDef hcan1;
CAN_TxHeaderTypeDef Message_Header; //declare a specific header for message transmittions
CAN_RxHeaderTypeDef Filter_Header; //declare header for message reception
uint32_t TxMailbox;//mailbox for pending transmitions
uint8_t message_out, message_in; //declare a send and receive bytes
CAN_FilterTypeDef sFilterConfig; //declare CAN filter structure



int count; //keep track of time
int lts_count; //keep track of left turn signal time
int rts_count; //keep track of right turn signal time

//these keep track of which state the turn signals are changing too
//"high to low"(0), or "low to high"(1)
int lts_state;
int rts_state;

extern uint8_t rts; //right turn signal byte
extern uint8_t lts; //left turn signal byte
extern uint8_t horn; //horn


int main(void)
{
  // Reset of all peripherals, Initializes the Flash interface and the Systick.
  HAL_Init();
  //Configure the system clock
  SystemClock_Config();
  //initialize GPIO
  GPIO_Init();
	//initialize CAN
  MX_CAN1_Init();

	//CAN header setup
  Message_Header.DLC=1; //give message size of 1 byte
  Message_Header.IDE=CAN_ID_STD; //set identifier to standard
  Message_Header.RTR=CAN_RTR_DATA; //set data type to remote transmission request?
  Message_Header.StdId=0x245; //define a standard identifier, used for message identification by filters (switch this for the other microcontroller)
	
	//CAN Filter setup
	sFilterConfig.FilterFIFOAssignment=CAN_FILTER_FIFO0; //set fifo assignment
	sFilterConfig.FilterIdHigh=0x244<<5; //the ID that the filter looks for (switch this for the other microcontroller)
	sFilterConfig.FilterIdLow=0;
	sFilterConfig.FilterMaskIdHigh=0;
	sFilterConfig.FilterMaskIdLow=0;
	sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT; //set filter scale
	sFilterConfig.FilterActivation=ENABLE;
	
	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig); //configure CAN filter	
	HAL_CAN_Start(&hcan1); //start CAN
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); //enable interrupts
	GPIOD->ODR=0x1000;

  while (1)
  {
		//---------------LEFT TURN SIGNAL---------------//
		//if lts is high, blink lts
  	if(lts==0xFF || (lts==0xF0 && count==lts_count)){

			//if first set lts to "not first"
			if(lts==0xFF){
				lts=0xF0;
				//set the count the lts will blink on
				lts_count=count;
				//set the state
				lts_state=1;
			}
			
			
			//TODO: redefine lts to 0xF0 in the lower if statement
			//put lts high if its set to
			if(lts_state==1){
				GPIOC->ODR|=(0x0001);
				lts_state=0;
			//put lts low if its set to
			}else{			
				GPIOC->ODR=(GPIOC->ODR-0x0001);
				lts_state=1;
			}
  	}
		
		
		//if lts off signal is recieved, turn off lts
		if(lts==0x0F){
			GPIOC->ODR=(GPIOC->ODR-0x0001);
			lts=0x00; //set lts to idle
		}
	 
		
		//---------------RIGHT TURN SIGNAL---------------//
		
		//if rts is high, blink rts
  	if(rts==0xFF || (rts==0xF0 && count==rts_count)){
			
			//if first set rts to "not first"
			if(rts==0xFF){
				rts=0xF0;
				//set the count the rts will blink on
				rts_count=count;
				//set the state
				rts_state=1;
			}
			
			//put rts high if its set to
			if(rts_state==1){
				GPIOC->ODR|=0x0002;
				rts_state=0;
			//put rts low if its set to
			}else{
				GPIOC->ODR=(GPIOC->ODR-0x0002);
				rts_state=1;
			}
  	}
		
		//if rts off signal is recieved, turn off rts
		//TODO: make sure you don't subtract is you don't need to.
		if(rts==0x0F){
			GPIOC->ODR=(GPIOC->ODR-0x0002);
			rts=0x00; //set rts to idle
		}
		
		
		
		//--------------------HORN----------------------//
		//if horn is high, start honking horn
		if(horn==0xFF){
			GPIOC->ODR|=0x0004;
		}	
		//if horn is low, stop honking horn
		if(horn==0x00){
			GPIOC->ODR=(GPIOC->ODR-0x0004);
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
		}
		
		
		/*
	  GPIOD->ODR=0xF000; //flash on board LEDs
		msDelay(200);
		GPIOD->ODR=0x0000; //flash on board LEDs
		msDelay(200);
		*/
		
		msDelay(50);
		count++;
		
		//reset count at 500ms
		if(count==10){
			//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);
			count=0;
		}
  }
}


//configure the system clocks (with HAL)
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  //Configure the main internal regulator output voltage 
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  //Initializes the CPU, AHB and APB busses clocks
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  //Initializes the CPU, AHB and APB busses clocks
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}



//Initialize CAN (using HAL)
static void MX_CAN1_Init(void)
{
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 21;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
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
}

//initialize GPIO pins
static void GPIO_Init(void)
{
  //Enable GPIO Ports Clock (using HAL)
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

	//enable PD12-PD15 debugging lights
  GPIOD->MODER=0x55000000;
	//enable PC0 PC1 PC2 as outputs
  GPIOC->MODER=0x00000015;
}


//Simple timing function used for delays
void msDelay(uint32_t msTime){
	for(uint32_t i=0; i<msTime*4000;i++);
}

//error handler (dont delete)
void Error_Handler(void)
{
	//pass
}

/****END OF FILE****/
