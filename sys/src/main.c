//===2016-03-25pm
//===C-version
#include "stm8s.h"  
#include "user.h"  
#include "macro_def.h"  
#include "iostm8s003f3.h"
#include "bq769x0.h" 
void main(void)
{  
  uint8_t i = 0;
  //-----------------------------
  disableInterrupts();
  SysInit(); 
  PortInit(); 
  LED1_ON();  
  I2C_Model_Init();
  //Uart_Model_Init();
  Var_Init();
  VCC1_ON();
  Timer2Init();
  enableInterrupts();  
  Delay_ms(50);
  ClrWdt();      
  Afe_Device_Init();
  Soc_OCV_CorrectEn_Flag = 1; 
  LowPower_MCU_Entry_Flag = 0; 
  LED1_OFF();  
  //Afe_ADC_Disable();  //100+uA
  //Afe_Temp_Disable(); //100+uA
  while(1)
  {
    ClrWdt();   
    if(LowPower_MCU_Entry_Flag == 0)
    { 
      Afe_Get_SysStatus(); 
      
      /* 
      Uart_SendStr("\r\n SYS_STAT= ");  Uart_SendData(SYS_STAT.Byte,16); 
      Uart_SendStr(" SYS_CTRL1 = ");    Uart_SendData(SYS_CTRL1.Byte,16); 
      Uart_SendStr(" SYS_CTRL2 = ");    Uart_SendData(SYS_CTRL2.Byte,16);  
      */
      ModeCheck(); 
      
      ClearStatus();
      
      Afe_Volt_Val_Get();
      
      CurrentCheck();
      
      VoltCheck();
      
      TempCheck();
      
      CellBal_Cntrl();
      
      Afe_FET_ChgDis_Cntrl();
      
      Afe_AbnormalCheck();
      
      LedShow_Cntrl();     //LedShow_WorkMode();   // 
      /*    
      Uart_SendStr(" WorkMode= ");      Uart_SendData(WorkMode,16); 
      Uart_SendStr(" Bits_flag = ");    Uart_SendData(Bits_flag.Byte,16); 
      
      Uart_SendStr(" Current_Val= ");   Uart_SendData((uint16_t)Current_Val,10);
      Uart_SendStr(" Volt_Avg= ");      Uart_SendData(Cell_Volt_Avg,10); 
      Uart_SendStr(" Max= ");           Uart_SendData(Cell_Volt_Max,10);
      Uart_SendStr(" Min= ");           Uart_SendData(Cell_Volt_Min,10);
       
      Uart_SendStr(" soc_rt= ");        Uart_SendData((uint16_t)SocCalc.soc_rt,10); 
      Uart_SendStr(" ah= ");            Uart_SendData((uint16_t)SocReg.ah,10);
      Uart_SendStr(" CellBal= ");       Uart_SendData((uint16_t)CellBalance_Selct,16);  
      
      Uart_SendStr("\r\n");
      for(i =0;i<10;i++)
      { 
        Uart_SendStr(" C_");
        Uart_SendData(i,10);  
        Uart_SendStr("= ");
        Uart_SendData(Cell_Volt[i],10);
      }

      Uart_SendStr("\r\n");*/ 
   
    }
    LowPower_Cntrl();
  }    
 
}
 













