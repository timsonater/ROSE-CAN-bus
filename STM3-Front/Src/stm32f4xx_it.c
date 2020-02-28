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
 void msDelay2(uint32_t msTime);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern volatile int state;

//----CAN VARIABLES----//
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

//fault indicators
extern volatile bool fault_flag;
uint32_t bmsFaultHolder = 0;
uint32_t carOffHolder = 0xFFFFFFFF;

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
				  GPIOD->ODR=~(GPIOD->ODR);
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
				//start precharge
				case 0xFE:
					//enter precharge state
					state=0;
					break;
			}

		//Filter 2: General Operation
		case 0x300:
			switch(message_in){
				//horn on (PC10)
			  case 0x10:
					GPIOC->BSRR=0x00000400; 
				  break;
				//horn off (PC10)
				case 0x11:
					GPIOC->BSRR=0x04000000;
				  break;
				//running lights on (PC11)
				case 0x20:
					GPIOC->BSRR=0x00000800;
					break;
				//running lights off (PC11)
				case 0x21:
					GPIOC->BSRR=0x08000000;
					break;
				case 0xF1:
					GPIOC->BSRR=0x00000010;
					break;
					
			}
			break;
			
		//Filter3: Error Messages
		case 0x3EE:
			state=4;
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
	
	
	
	//Shifts in a 1 if bit is high, shift 0 otherwise
	bmsFaultHolder = bmsFaultHolder << 1;
	bmsFaultHolder |= (GPIOE->IDR >> 10) & 0x1; //PE10
	
	carOffHolder = carOffHolder << 1;
	carOffHolder |= (GPIOA->IDR) & 0x1; //PA0
	
	
	//Checks if PE10 (BMS ok) has been high for a 320 ms.
	//Inside if statement is the code for triggering the fault.
	if(bmsFaultHolder == 0xFFFFFFFF){
		
		HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);

		//cut off positive contactor PC11 and PC9
		GPIOC->BSRR = 0x0A000000;
		//cut precharge board off (if its on) PC10
		GPIOC->BSRR = 0x04000000;
	
		fault_flag=1;
		state=4;	
			
	}	
	
	if(carOffHolder == 0){
		//set 24V-12V DC DC low: PD9
		GPIOD->BSRR=0x02000000;				
		//set 110V-12V DC DC low: PE6
		GPIOE->BSRR = 0x00400000;
		//car is now off
	}
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
  /* USER CODE END TIM4_IRQn 1 */
}

/* USER CODE BEGIN 1 */
//function uses clock cycles to create a delay
void msDelay2(uint32_t msTime){
	for(uint32_t i=0; i<msTime*4000;i++);
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
