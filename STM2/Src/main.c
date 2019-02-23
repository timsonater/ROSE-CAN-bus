#include "main.h"

//Define CAN variables
CAN_HandleTypeDef hcan1;
CAN_TxHeaderTypeDef pHeader; //declare a specific header for message transmittions
CAN_RxHeaderTypeDef pRxHeader; //declare header for message reception
uint32_t TxMailbox;
uint8_t a,r; //declare a send byte
CAN_FilterTypeDef sFilterConfig; //declare CAN filter structure


uint8_t rts; //right turn signal byte
uint8_t lts; //left turn signal byte
uint8_t horn; //horn

int lts_holder;
int rts_holder;
int horn_holder;



void SystemClock_Config(void);
static void GPIO_Init(void);
static void MX_CAN1_Init(void);


int main(void)
{

  // Reset of all peripherals, Initializes the Flash interface and the Systick.
  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  GPIO_Init();
  MX_CAN1_Init();

  pHeader.DLC=1; //give message size of 1 byte
  pHeader.IDE=CAN_ID_STD; //set identifier to standard
  pHeader.RTR=CAN_RTR_DATA; //set data type to remote transmission request?
  pHeader.StdId=0x244; //define a standard identifier, used for message identification by filters (switch this for the other microcontroller)
  
  //filter one (stack light blink)
  sFilterConfig.FilterFIFOAssignment=CAN_FILTER_FIFO0; //set fifo assignment
  sFilterConfig.FilterIdHigh=0x245<<5; //the ID that the filter looks for (switch this for the other microcontroller)
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
		//these "holders" are used to store a value with the purpose of detecting a change in a switch
    lts_holder = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0);
    rts_holder = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1);
    horn_holder = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2);

    HAL_Delay(200);
		 if(lts_holder != HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0)) {
      //send can message
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);
      a=0x00;
      //MIGHT NEED TO RE-ADD POINTERS
      HAL_CAN_AddTxMessage(&hcan1, &pHeader, &a, &TxMailbox);  //function to add message for transmition
    }
    if(rts_holder != HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1)) {
      //send can message
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);
      a=0x02;
      HAL_CAN_AddTxMessage(&hcan1, &pHeader, &a, &TxMailbox);  //function to add message for transmition
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


//initialize CAN (with HAL)
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

//GPIO initialization function
static void GPIO_Init(void)
{
  //GPIO Ports Clock Enable
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

	//enable PD12-PD15 debugging lights
  GPIOD->MODER=0x55FFFFFF;
	//enable PE0 PD1 PD2 as inputs
	GPIOE->MODER=0xFFFFFFC0;
  //ensure all pulldown resistors are activated
  GPIOE->PUPDR=0x00000000; 

}

//error handler (dont delete)
void Error_Handler(void)
{
  //pass
}


/*****END OF FILE****/
