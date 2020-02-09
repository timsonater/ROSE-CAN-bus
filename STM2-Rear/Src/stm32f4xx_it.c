/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

//---Throttle Watchdog---//
bool throttleFault;

//-----CAN VARIABLES-----//
extern uint32_t TxMailbox;
extern uint8_t message_in;
extern CAN_TxHeaderTypeDef Message_Header;
extern CAN_RxHeaderTypeDef Filter_Header;

//---FLAG DEFINIIONS---///
//these control whether has indicated something should turn on or off
extern bool lts_on_flag;
extern bool lts_off_flag;
extern bool rts_on_flag;
extern bool rts_off_flag;
extern bool haz_on_flag;
extern bool haz_off_flag;

//these indicate whether something is currently activated
extern volatile bool rts_state;
extern volatile bool lts_state;
extern volatile bool haz_state;

//these are for the specific state of the physical light itself
extern volatile bool lts_light_state;
extern volatile bool rts_light_state;
extern volatile bool haz_light_state;
bool fault_light_state;

int count=0;

//DAC structure
extern DAC_HandleTypeDef hdac;

//state 
extern volatile int state;
extern volatile bool fault_flag;
uint8_t message_holder;

//-----CRUISE CONTROL VARIABLES-----//
bool CC_state;  //1 means on, 0 means off
bool CC_enable; //tells whether CC allowed or not (on/off in normal car)
uint8_t throttle_voltage; //voltage put out to throttle
uint8_t cruise_voltage;  //keep track of cruise control voltage
uint8_t cruise_voltage_holder; //holder to prevent underflows


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */
	count++;
	if(count==100){
		//GPIOD->ODR = ~(GPIOD->ODR);
		count=0;
	}
	
  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &Filter_Header, &message_in);
	
	
	
	switch(Filter_Header.StdId){
		//Filter 1, Lights Control
		case 0x240:
			switch(message_in){
				case 0x01:
					rts_on_flag=1;
				  break;
				case 0x02:
					rts_off_flag=1;
				  break;
				case 0x03:
					lts_on_flag=1;
				  break;
				case 0x04:
					lts_off_flag=1;
				  break;
				case 0x05:
				  haz_on_flag=1;
				  break;
				case 0x06:
				  haz_off_flag=1;
				  break;
			}
			break;
		
		//Filter 2: General Operation
		case 0x200:
			switch(message_in){
				
				//CC enable
			  case 0x07:
					//allow CC use
					//CC_enable_disable=1;
				  break;
				
				//CC disable
				case 0x22:
					//disallow CC use
					//CC_enable_disable=0;
				  //CC_on_off=0;
				
				
				//CC inc
				case 0x08:
					//set CC inc flag
				  //CC_inc=1;
					break;
				
				//CC dec/set
				case 0x09:
					//set CC dec/set flag
					//CC_dec_set=1;
					break;
						
				//break lights on
				case 0x12:
					//turn on break lights (PC10)
				  //GPIOC->BSRR=0x00000400;
					//turn off CC
				  //CC_on_off=0;
					break;
				
				//break lights off (and CC disable)
				case 0x13:
					//turn off break lights (PC10)
					//GPIOC->BSRR=0x04000000;
					break;
				
				//TODO: I have no idea if low or high for forward/reverse and power/eco
				//TODO: some of these should start as defaults.
				//power mode (PC7 low)
				case 0x14:
					//GPIOC->BSRR=0x00800000;
				  break;
				
				//eco mode (PC7 high)
				case 0x15:
					//GPIOC->BSRR=0x00000080;
				  break;
				
				//forward (PC5 high)
				case 0x16:
					//GPIOC->BSRR=0x00000020;
				
				//reverse (PC5 low)
				case 0x17:
					//GPIOC->BSRR=0x00200000;
				
				//motor on (PC4 high)
				case 0x18:
					//GPIOC->BSRR=0x00000010;
					break;
				
				//motor off (PC4 low)
				case 0x19:
					//GPIOC->BSRR=0x00100000;
					break;
			  }
			break;
		
		//Filter3: Throttle Position
		//NOTE: this interrupt effectively acts as the timer for the DAC on the rear stm 
		case 0x210:
			
			HAL_DAC_SetValue(&hdac, DAC1_CHANNEL_1, DAC_ALIGN_8B_R, message_in);
			
			//if the message voltage is greater than the cruise voltage just use that no matter what
		  //if(CC_on_off==1 && CC_enable_disable==1 && message_in > cruise_voltage){
			//	HAL_DAC_SetValue(&hdac, DAC1_CHANNEL_1, DAC_ALIGN_8B_R, message_in);
			//	}
			//if CC is both enabled and on, output cruise voltage to DAC
		  //else if(CC_on_off==1 && CC_enable_disable==1){
			//	HAL_DAC_SetValue(&hdac, DAC1_CHANNEL_1, DAC_ALIGN_8B_R, cruise_voltage);
			//	}
			//otherwise just use the message voltage
		  //else{
			//	HAL_DAC_SetValue(&hdac, DAC1_CHANNEL_1, DAC_ALIGN_8B_R, message_in);
				//store the cruise voltage
			//	cruise_voltage = message_in;
			//}
				
		
			//pet the watchdog timer (timer 3)
			//TIM3->CNT=0;
			throttleFault = 0;
		  break;
		
		//Filter4: Regen Position
		case 0x220:
			//output regen position to DAC
		  HAL_DAC_SetValue(&hdac, DAC1_CHANNEL_2, DAC_ALIGN_8B_R, message_in);		
			break;
		
		//Filter4: Error Messages
		case 0x2EE:
			//stop DACs
			HAL_DAC_Stop(&hdac, DAC1_CHANNEL_1);
			HAL_DAC_Stop(&hdac, DAC1_CHANNEL_2);
			//trip fault flag
			fault_flag=1;
			break;
	}    

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
	//Checking throttle position roughly every third of a second
if(throttleFault == 1) 
	{
	HAL_DAC_Stop(&hdac, DAC1_CHANNEL_1);
	HAL_DAC_Stop(&hdac, DAC1_CHANNEL_2);
	}
throttleFault = 1;
  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */
	
	//toggle lts if it's high (and hazards are off)
	if(lts_state==1 && lts_light_state==1 && haz_state==0 && haz_state==0){
		//turn lts off
		lts_light_state=0;
		GPIOC->BSRR=0x01000000;
	  }else if(lts_state==1 && lts_light_state==0){
		//turn lts on
		lts_light_state=1;
		GPIOC->BSRR=0x00000100;
	  }
		
	//toggle rts if it's high (and hazards are off)
	if(rts_state==1 && rts_light_state==1 && haz_state==0){
		//turn rts off
		rts_light_state=0;
		GPIOC->BSRR=0x02000000;
	  }else if(rts_state==1 && rts_light_state==0 && haz_state==0){
		//turn lts on
		rts_light_state=1;
		GPIOC->BSRR=0x00000200;
	  }
		
  //toggle haz if it's high
	if(haz_state==1 && haz_light_state==1){
		//turn haz off
		haz_light_state=0;
		GPIOC->BSRR=0x03000000;
	  }else if(haz_state==1 && haz_light_state==0){
		//turn haz on
		haz_light_state=1;
		GPIOC->BSRR=0x00000300;
	  }
		
		
	//blink fault light if it's high
	if (fault_flag==1 && fault_light_state==0){
	  fault_light_state=1;
		GPIOC->BSRR=0x00000800;
	  }else if (fault_flag==1 && fault_light_state==1){
		fault_light_state=0;
		GPIOC->BSRR=0x08000000;
		}
		
		

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */
  long count;
  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */
	
	count=TIM2->CNT; //get count
	GPIOE->ODR=count;
	
	//TODO: adjust this timer to trigger every second.
	//TODO: actually calibrate this and send out on a CAN message
	
	TIM2->CNT=0; //reset counter
  /* USER CODE END TIM5_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
