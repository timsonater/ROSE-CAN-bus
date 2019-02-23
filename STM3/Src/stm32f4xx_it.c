#include "main.h"
#include "stm32f4xx_it.h"
#include "stdbool.h"

//import external CAN variables
extern CAN_HandleTypeDef hcan1;
extern CAN_TxHeaderTypeDef Message_Header;
extern CAN_RxHeaderTypeDef Filter_Header;
extern uint32_t TxMailbox;
extern uint8_t message_out, message_in;

//define local interrupt variables
bool CAN_check_result;

// This function handles Non maskable interrupt.
void NMI_Handler(void){
  //pass
  }

//This function handles Hard fault interrupt.
void HardFault_Handler(void){
  while (1){
    //pass
    }
  }

//This function handles Memory management fault.
void MemManage_Handler(void){
  while (1){
    //pass
    }
  }

//This function handles Pre-fetch fault, memory access fault.
void BusFault_Handler(void){
  while (1){
    //pass
    }
  }

//This function handles Undefined instruction or illegal state.
void UsageFault_Handler(void){
  while (1){
    //pass
   }
  }

//This function handles System service call via SWI instruction.
void SVC_Handler(void){
  //pass
  }

//This function handles Debug monitor.
void DebugMon_Handler(void){
  //pass
  }

//This function handles Pendable request for system service.
void PendSV_Handler(void){
  //pass
  }

//This function handles System tick timer.
void SysTick_Handler(void){
  HAL_IncTick();
  }

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

//This function handles CAN1 RX0 interrupts. (incoming messages)
void CAN1_RX0_IRQHandler(void){
  //unpack the message
	HAL_CAN_IRQHandler(&hcan1);
  HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &Filter_Header, &message_in);
	
	switch(message_in){
		//message: STM check
		case 0xFF:
			CAN_check_result=1;
			break;
    }
	}


//****END OF FILE****//
