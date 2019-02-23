
#include "main.h"
#include "stm32f4xx_it.h"




//import external CAN variables
extern CAN_HandleTypeDef hcan1;
extern CAN_TxHeaderTypeDef Message_Header;
extern CAN_RxHeaderTypeDef Filter_Header;
extern uint32_t TxMailbox;
extern uint8_t message_out, message_in;

//define local interrupt variables
uint8_t rts=0xFF; //right turn signal byte
uint8_t lts=0xFF; //left turn signal byte
uint8_t horn=0xFF; //horn

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

/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
  GPIOD->ODR=0xF000;
	HAL_Delay(1000);
	//unpack can message
  HAL_CAN_IRQHandler(&hcan1);
  HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &Filter_Header, &message_in);
  
  switch(message_in){
    case 0x00:
      rts=0xFF;
		  break;
    case 0x01:
      rts=0x00;
		  break;
    case 0x02:
      lts=0xFF;
		  break;
    case 0x03:
      lts=0x00;
		  break;
    case 0x04:
      horn=0xFF;
		  break;
    case 0x05:
      horn=0x00;
		  break;
		
		//STM check
		case 0xFF:
			//send a message back
		  Message_Header.StdId=0x242;
			message_out=0xFF;
			HAL_CAN_AddTxMessage(&hcan1, &Message_Header, &message_out, &TxMailbox);
			GPIOD->ODR=0xF000;
		  break;
  }   
}
  /* USER CODE END CAN1_RX0_IRQn 1 */


/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
