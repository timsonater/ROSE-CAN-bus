#include "main.h"
#include "stdbool.h"

//Define CAN variables
CAN_HandleTypeDef hcan1;
CAN_TxHeaderTypeDef Message_Header; //declare a specific header for message transmittions
CAN_RxHeaderTypeDef Filter_Header; //declare header for message reception
uint32_t TxMailbox;//mailbox for pending transmitions
uint8_t message_out, message_in; //declare a send and receive bytes
CAN_FilterTypeDef sFilterConfig; //declare CAN filter structure

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void GPIO_Init(void);
static void MX_CAN1_Init(void);
int Debounced_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void check_ok(void);

//define state machine state
int state=0;

//import external CAN variables from interrupt file
extern bool CAN_check_result;



int main(void)
{
  // Reset of all peripherals, Initializes the Flash interface and the Systick.
  HAL_Init();
  //Configure the system clock 
  SystemClock_Config();


  /* Initialize all configured peripherals */
  GPIO_Init();
  MX_CAN1_Init();

  Message_Header.DLC=1; //give message size of 1 byte
  Message_Header.IDE=CAN_ID_STD; //set identifier to standard
  Message_Header.RTR=CAN_RTR_DATA; //set data type to remote transmission request?
  Message_Header.StdId=0x244; //define a standard identifier, used for message identification by filters
  
  //filter one (stack light blink)
  sFilterConfig.FilterFIFOAssignment=CAN_FILTER_FIFO0; //set fifo assignment
  sFilterConfig.FilterIdHigh=0x242<<5; //the ID that the filter looks for (THIS IS YOUR ADDRESS)
  sFilterConfig.FilterIdLow=0;
  sFilterConfig.FilterMaskIdHigh=0;
  sFilterConfig.FilterMaskIdLow=0;
  sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT; //set filter scale
  sFilterConfig.FilterActivation=ENABLE;
  
  HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig); //configure CAN filter
  HAL_CAN_Start(&hcan1); //start CAN
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); //enable interrupts
  

  while (1)
  {
		HAL_Delay(1000);
		switch(state){
			
			//power has just turned on, check to ensure main power input is high, and fault state is low
			case 0:
		    check_ok();
			  break;
			
			//do a quick check of the CAN network
			case 1:
				
			  /*TODO: potentially make this process a function depending on how we need to
                do it in the main state*/			
				//transmit CAN check message which travels to all STMs, starting with STM1
				Message_Header.StdId=0x244;
			  message_out=0xFF;
			  HAL_CAN_AddTxMessage(&hcan1, &Message_Header, &message_out, &TxMailbox);
			  GPIOD->ODR=0xF000;
			
			  //wait a small amount of time and ensure the CAN check worked
				HAL_Delay(200);
			  if(CAN_check_result==1){
					state++;
					//reset the check result
					CAN_check_result=0;
				}else{
					//enter fault state
				  state=10;
				}
				break;
			
			case 2:
				GPIOD->ODR=0xB000;
			//fault state
			case 10:
				GPIOD->ODR=0xA000;
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

//intitialize CAN (with HAL)
static void MX_CAN1_Init(void)
{
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 21;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
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
  
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

	
	//enable PD12-PD15 debugging lights
  GPIOD->MODER=0x55FFFFFF;
	
	//enable PE0 PD1 PD2 as inputs
	GPIOE->MODER=0xFFFFFFC0;
  //ensure all pulldown resistors are activated
  GPIOE->PUPDR=0xAAAAAAAA; 
}

//simple debounced read
int Debounced_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin){
  int pin_holder=HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0);
  HAL_Delay(100);
  if(pin_holder==HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0)){
		return pin_holder;
		}
	return 0;
}


//this function makes sure the main power is still on and the kill switch is not high
/*TODO: maybe have a small amount of tolerance for each of main power, a bad connection could cut off the car
	      also this could allow for the removal of the debouncing wait*/
void check_ok(void){
  int main_power=Debounced_ReadPin(GPIOE, GPIO_PIN_0);
	int kill_switch=HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1);
	//make sure power is on
	if(main_power==1 && kill_switch==0){
		//if not the first state
		if(state==0){
		  state++;
		  }
		return;
	}else{
		//enter fault state
	  state=10;
	}
}

//error handler (dont delete)
void Error_Handler(void)
{
  //pass
}
/*****END OF FILE****/
