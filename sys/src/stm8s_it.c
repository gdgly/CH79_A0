/**
  ******************************************************************************
  * @file stm8s_it.c
  * @author STMicroelectronics - MCD Application Team
  * @version V2.0.0
  * @date 15-March-2011
  * @brief Main Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------
*/
#include "stm8s_it.h"
#include "stm8s_tim4.h"
#include "sysctrl.h"
//#include "uart.h"
#include "iostm8s003f3.h" 
#include "user.h"
#include "macro_def.h"

#define   DIS_GIE    asm("sim")
#define   EN_GIE     asm("rim")
 
  
/** @addtogroup TIM2_OC_ActiveMode
  * @{
  */
#ifdef _COSMIC_
/**
  * @brief  Dummy interrupt routine
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(NonHandledInterrupt, 25)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  nop();
}
#endif /*_COSMIC_*/

/**
  * @brief  TRAP interrupt routine
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER_TRAP(TRAP_IRQHandler)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  nop();
}
/**
  * @brief  Top Level Interrupt routine
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(TLI_IRQHandler, 0)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  nop();
}

/**
  * @brief  Auto Wake Up Interrupt routine
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(AWU_IRQHandler, 1)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
   nop();
   AWU->CSR |= 0x00;
}

/**
  * @brief  Clock Controller Interrupt routine
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(CLK_IRQHandler, 2)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  nop();
}

/**
  * @brief  External Interrupt PORTA Interrupt routine
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI_PORTA_IRQHandler, 3)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */ 
    nop();
   
}

/**
  * @brief  External Interrupt PORTB Interrupt routine
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI_PORTB_IRQHandler, 4)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  nop(); 
  //GPIO_Init(GPIOB, GPIO_PIN_5, GPIO_MODE_IN_PU_NO_IT);        //KEY1
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
  * @brief  External Interrupt PORTC Interrupt routine
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI_PORTC_IRQHandler, 5)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  nop();
  
}

/**
  * @brief  External Interrupt PORTD Interrupt routine
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI_PORTD_IRQHandler, 6)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  nop();  
  /**/ 
  //LED1_ON();
  AWU->CSR |= 0x00;
  //LowPower_MCU_Entry_Flag = 0;
  //LowPower_Entry_Delay_t = 0;
}

/**
  * @brief  External Interrupt PORTE Interrupt routine
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI_PORTE_IRQHandler, 7)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  nop();
}
#ifdef STM8S903
/**
  * @brief  External Interrupt PORTF Interrupt routine
  * @param  None
  * @retval None
  */
 INTERRUPT_HANDLER(EXTI_PORTF_IRQHandler, 8)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  nop();
}
#endif /*STM8S903*/

#if defined (STM8S208) || defined (STM8AF52Ax)
/**
  * @brief CAN RX Interrupt routine.
  * @param  None
  * @retval None
  */
 INTERRUPT_HANDLER(CAN_RX_IRQHandler, 8)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

/**
  * @brief  CAN TX Interrupt routine
  * @param  None
  * @retval None
  */
 INTERRUPT_HANDLER(CAN_TX_IRQHandler, 9)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}
#endif /*STM8S208 || STM8AF52Ax */

/**
  * @brief  SPI Interrupt routine
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(SPI_IRQHandler, 10)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  nop();
}

/**
  * @brief  Timer1 Update/Overflow/Trigger/Break Interrupt routine
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(TIM1_UPD_OVF_TRG_BRK_IRQHandler, 11)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  
//FOR DEBUG  
//1MS INTERRUPT FOR USER CODE 
  //TIM1_ClearFlag(TIM1_FLAG_UPDATE);  
  //TimerCnt();  
//FOR DEBUG  
  nop();

  
}

/**
  * @brief  Timer1 Capture/Compare Interrupt routine
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(TIM1_CAP_COM_IRQHandler, 12)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  nop();
}

#ifdef STM8S903
/**
  * @brief  Timer5 Update/Overflow/Break/Trigger Interrupt routine
  * @param  None
  * @retval None
  */
 INTERRUPT_HANDLER(TIM5_UPD_OVF_BRK_TRG_IRQHandler, 13)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  nop();
}
/**
  * @brief  Timer5 Capture/Compare Interrupt routine
  * @param  None
  * @retval None
  */

 INTERRUPT_HANDLER(TIM5_CAP_COM_IRQHandler, 14)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  nop();
}

#else /*STM8S208, STM8S207, STM8S105 or STM8S103 or STM8AF62Ax or STM8AF52Ax or STM8AF626x */
/**
  * @brief  Timer2 Update/Overflow/Break Interrupt routine
  * @param  None
  * @retval None
  */
 INTERRUPT_HANDLER(TIM2_UPD_OVF_BRK_IRQHandler, 13)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */ 
  static uint8_t cntr100ms = 0; 
  static uint16_t Init_Soc_Flag_Delay = 0; 
  nop();
  TIM2_SR1 &= ~0x01;
  if(ChgExchangeMode_Cnt < 255)
  {
    ChgExchangeMode_Cnt += 1;
  }
  if(DisExchangeMode_Cnt < 255)
  {
    DisExchangeMode_Cnt += 1;
  }
  if(IdleExchangeMode_Cnt < 255)
  {
    IdleExchangeMode_Cnt += 1;
  }
  //===================================SOC
  if(Init_Soc_Flag == 0)
  {
    if((Init_Soc_Flag_Delay++) >= 200)
    {
      Init_Soc_Flag = 1;
      Init_Soc_Flag_Delay = 0;
    }
  }
  //if(Init_Soc_Flag ==1)
  {
    SOCAhIntergrate(); 
    if((cntr100ms++) >= 10)
    {
      cntr100ms = 0; 
      if(Init_Soc_Flag == 1)
      {
        SOCCalculate(); // 100ms tick
      }
    }
  } 
  //======================================== 
  if(WorkMode != IDLE_MODE)
  {
    if(LedFlash_t < MAX_UINT16_T_NUM)
    {
      LedFlash_t += 1;
    } 
  }
  else
  {
    LedFlash_t = 0;
  }
  //=======================================
  if(Temp_Volt_Sample_Cnt < MAX_UINT16_T_NUM)
  {  
    Temp_Volt_Sample_Cnt += 1;
  }
  if(Cell_Volt_Sample_Cnt < MAX_UINT16_T_NUM)
  {  
    Cell_Volt_Sample_Cnt += 1;
  }
  if(CC_Volt_Sample_Cnt < MAX_UINT16_T_NUM)
  {  
    CC_Volt_Sample_Cnt += 1;
  } 
      if(Bits_flag.Bit.DisTemp || Bits_flag.Bit.ChgTemp )
      {
        if(Temp_Protect_Delay_t < MAX_UINT16_T_NUM)
        {
          Temp_Protect_Delay_t += 1;
        }
      }
  //=======================================
  if(WorkMode == CHARGE_MODE)
  {
     /*if(ChgTemp_cnt < 1000)
     {
       ChgTemp_cnt += 1; 
     }*/
    if(Chg_Current_Val_Small_Errer_t < MAX_UINT16_T_NUM)
    {
      Chg_Current_Val_Small_Errer_t += 1;
    }
    if(Cell_Balance_Delay_t < MAX_UINT16_T_NUM)
    {
      Cell_Balance_Delay_t += 1;
    }
    if(ChgOv_t < MAX_UINT32_T_NUM)
    {
      ChgOv_t += 1;
    }
    if(ChgCurOv_t < MAX_UINT16_T_NUM)
    {
      ChgCurOv_t += 1;
    }
    if(Bits_flag.Bit.ChgCurOv )
    {
      if(ChgCurOv_Re_t < MAX_UINT16_T_NUM)
      {
        ChgCurOv_Re_t += 1;
      }
    }
    else
    {
      ChgCurOv_Re_t = 0;
    }
  }
  else
  {
    //=======================================
    if(DisOv_t < MAX_UINT16_T_NUM)
    {
      DisOv_t += 1;
    }
    //=======================================
    /*
    if(Bits_flag.Bit.DisOv)
    {
      if( PowerOff_Delay_t < MAX_UINT16_T_NUM)
      {
        PowerOff_Delay_t += 1;
      }
    }
    else
    {
      PowerOff_Delay_t = 0;
    } 
    */ 
    //=======================================
    if(WorkMode == DISCHARGE_MODE)
    { 
     if(DisTemp_cnt < 1000)
     {
       DisTemp_cnt += 1; 
     }
      if(DisTemp_Lock_Cnt < 500)
      {
        DisTemp_Lock_Cnt += 1;
      }
      //==============================
      if(Dis_First_Run_Flag == 0)
      {
        if(Dis_First_Run_t < 200)
        {
          Dis_First_Run_t += 1;
        }
      }
      else
      {
        Dis_First_Run_t = 0;
      }
     /*
      //=======================================
      if(Bits_flag.Bit.DisCurShort)
      {
        if( DisCurShort_Re_t < MAX_UINT16_T_NUM)
        {
          DisCurShort_Re_t += 1;
        }
      }
      else
      {
        DisCurShort_Re_t = 0;
      }
      
      //=======================================
      if(DisCurOv_t < MAX_UINT16_T_NUM)
      {
        DisCurOv_t += 1;
      }
     */
      //=======================================
      if(DisCurOv_t1 < MAX_UINT16_T_NUM)
      {
        DisCurOv_t1 += 1;
      }
      //=======================================
      if(DisCurOv_t2 < MAX_UINT16_T_NUM)
      {
        DisCurOv_t2 += 1;
      }
      //======================================= 
    } 
  }
  //=======================================
  if(Bits_flag.Bit.AfeErr)
  {
    /*if(DEVICE_XREADY_Re_t < MAX_UINT16_T_NUM)
    {
      DEVICE_XREADY_Re_t += 1;
    }*/
    if(AfeErr_t < MAX_UINT16_T_NUM)
    {
      AfeErr_t += 1;
    }
  }
  /*
  else
  {
    DEVICE_XREADY_Re_t = 0; 
  }*/
  //=======================================
  if(WorkMode ==IDLE_MODE)
  {
    if(LowPower_Entry_Delay_t < MAX_UINT16_T_NUM)
    {
      LowPower_Entry_Delay_t += 1;
    }
    if(PowerOff_Delay_t < MAX_UINT16_T_NUM)
    {
      PowerOff_Delay_t += 1;
    }
    if(LedFlash_Off_t < MAX_UINT16_T_NUM)
    {
      LedFlash_Off_t += 1;
    }
    
  } 
}

/**
  * @brief  Timer2 Capture/Compare Interrupt routine
  * @param  None
  * @retval None
  */
 INTERRUPT_HANDLER(TIM2_CAP_COM_IRQHandler, 14)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  nop();
}
#endif /*STM8S903*/

#if defined (STM8S208) || defined(STM8S207) || defined(STM8S105) || defined (STM8AF62Ax) ||\
    defined (STM8AF52Ax) || defined (STM8AF626x)
/**
  * @brief Timer3 Update/Overflow/Break Interrupt routine.
  * @param  None
  * @retval None
  */
 INTERRUPT_HANDLER(TIM3_UPD_OVF_BRK_IRQHandler, 15)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  nop();
}

/**
  * @brief  Timer3 Capture/Compare Interrupt routine
  * @param  None
  * @retval None
  */
 INTERRUPT_HANDLER(TIM3_CAP_COM_IRQHandler, 16)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  nop();
}
#endif /*STM8S208, STM8S207 or STM8S105 or STM8AF62Ax or STM8AF52Ax or STM8AF626x */

#if defined (STM8S208) || defined(STM8S207) || defined(STM8S103) || defined (STM8AF62Ax) ||\
    defined (STM8AF52Ax) || defined (STM8S903)
/**
  * @brief  UART1 TX Interrupt routine
  * @param  None
  * @retval None
  */
 INTERRUPT_HANDLER(UART1_TX_IRQHandler, 17)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  nop();
  //UART1_ClearFlag(UART1_FLAG_RXNE); 
  //if(*str!=0)
  //UART1_SendData8(*srt);
  //srt++;

  
}

/**
  * @brief  UART1 RX Interrupt routine
  * @param  None
  * @retval None
  */
 INTERRUPT_HANDLER(UART1_RX_IRQHandler, 18)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  
  //UART1_ClearFlag(UART1_FLAG_RXNE); 
  //Uart1DataReceived();
  //UART1_SR &=0xD7;
  //UART_Revc = UART1_DR; 
}
#endif /*STM8S105*/

/**
  * @brief  I2C Interrupt routine
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(I2C_IRQHandler, 19)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
  nop();
  I2C->SR1 = 0; 
}

#if defined (STM8S105) || defined (STM8AF626x)
/**
  * @brief  UART2 TX interrupt routine.
  * @param  None
  * @retval None
  */
 INTERRUPT_HANDLER(UART2_TX_IRQHandler, 20)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
  nop();
  }

/**
  * @brief  UART2 RX interrupt routine.
  * @param  None
  * @retval None
  */
 INTERRUPT_HANDLER(UART2_RX_IRQHandler, 21)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
  nop();
  }
#endif /* STM8S105*/

#if defined(STM8S207) || defined(STM8S208) || defined (STM8AF52Ax) || defined (STM8AF62Ax)
/**
  * @brief  UART3 TX interrupt routine.
  * @param  None
  * @retval None
  */
 INTERRUPT_HANDLER(UART3_TX_IRQHandler, 20)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
  nop();
  }

/**
  * @brief  UART3 RX interrupt routine.
  * @param  None
  * @retval None
  */
 INTERRUPT_HANDLER(UART3_RX_IRQHandler, 21)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
  nop();
  }
#endif /*STM8S208 or STM8S207 or STM8AF52Ax or STM8AF62Ax */

#if defined(STM8S207) || defined(STM8S208) || defined (STM8AF52Ax) || defined (STM8AF62Ax)
/**
  * @brief  ADC2 interrupt routine.
  * @param  None
  * @retval None
  */
 INTERRUPT_HANDLER(ADC2_IRQHandler, 22)
{

    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
  nop();
    return;

}
#else /*STM8S105, STM8S103 or STM8S903 or STM8AF626x */
/**
  * @brief  ADC1 interrupt routine.
  * @param  None
  * @retval None
  */
 INTERRUPT_HANDLER(ADC1_IRQHandler, 22)
{

    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
  nop();
    return;

}
#endif /*STM8S208 or STM8S207 or STM8AF52Ax or STM8AF62Ax */

#ifdef STM8S903
/**
  * @brief  Timer6 Update/Overflow/Trigger Interrupt routine
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(TIM6_UPD_OVF_TRG_IRQHandler, 23)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
//200uS INTERRUPT FOR USER CODE 
 TIM6_ClearFlag(TIM6_FLAG_UPDATE);

  nop();
 //TIM6_SR1 = 0x00;
//--------------------test only

 //GPIO_WriteReverse(GPIOD, GPIO_PIN_2 | GPIO_PIN_3 |GPIO_PIN_4 |GPIO_PIN_5);

  
    
  }  
#else /*STM8S208, STM8S207, STM8S105 or STM8S103 or STM8AF62Ax or STM8AF52Ax or STM8AF626x */
/**
  * @brief  Timer4 Update/Overflow Interrupt routine
  * @param  None
  * @retval None
  */
 INTERRUPT_HANDLER(TIM4_UPD_OVF_IRQHandler, 23)
{ 
  
   TIM4->SR1 &= ~0x01;
   //-------------------extern unsigned int DisLedW_t,DisLedW_tt; 
   if(Delay_time_t > 0)
   {
     Delay_time_t -= 1;
   }
   
   //==================================== 
}
#endif /*STM8S903*/

/**
  * @brief  Eeprom EEC Interrupt routine
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EEPROM_EEC_IRQHandler, 24)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
}

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
