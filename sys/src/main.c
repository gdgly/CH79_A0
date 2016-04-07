//===2016-03-25pm
//===C-version-git-control
#include "stm8s.h"  
#include "user.h"  
#include "macro_def.h"  
#include "iostm8s003f3.h"
#include "bq769x0.h" 

//#define Uart_Model_Enable
void main(void)
{  
  uint8_t i = 0;
  //-----------------------------
  disableInterrupts();        // �ر�MCUȫ�ֱ���
  SysInit();                  // ϵͳ��ʼ����MCUʱ�����á�ADCģ���ʼ��
  PortInit();                 // MCU�ܽ�����
  //LED1_ON();                  // ��λ����LED1�������AFE IC��ʼ��֮��Ϩ��
  I2C_Model_Init();           // ����MCU��IICģ��
#ifdef Uart_Model_Enable
  Uart_Model_Init();          // ����MCU��UARTģ��
#endif
  Var_Init();                 // ��ʼ��������ֵ 
  Timer2Init();               // ����TIMER 2Ϊ��׼ʱ��Ķ�ʱ��
  enableInterrupts();         // ��MCUȫ���ж�
  Delay_ms(50);               // ��ʱ50mS��MCU�ȶ�
  ClrWdt();                   // ���ÿ��Ź�
  Afe_Device_Init();          // ��ʼ��AFE IC����������ADC���������ģ�顢���ù��Ź����о��ѹֵ���ŵ��������·�����Ȳ��� 
  LED1_OFF();                 // �ر�LED1
  SOC_Init();                 // ����EEPROM�б������ݻ�OCV��ʼ��SOC����
  //Soc_OCV_CorrectEn_Flag = 1; // �ϵ�����SOC��OCVУ׼ 
  //Afe_ADC_Disable();        // 100+uA
  //Afe_Temp_Disable();       // 100+uA
  WorkMode = DISCHARGE_MODE;
  Bits_flag.Bit.DisOv = 1;
  while(1)
  {
    //=========
    // ����1���ŵ�״̬�£���о��ѹ���ڹ���ʱ��BQ���Զ��رճ��MOS��;   ����������ŵ�״̬�£���������Ӳ�������ѹΪ4.3V
    // ����2�����״̬�£���о��ѹ���ڹ���ʱ��BQ���Զ��رշŵ�MOS��;   ������������״̬�£���������Ӳ�������ѹΪ1.5V
    // ����3������ŵ缰����ʶ�����⣩Ƿѹ״̬�£����ϳ����ʱ��ʶ���˳����;  
              //��������������Triger�ź�ʱ��Ϊ����ģʽ; Triger�źŷ�Χ��[]ʱ�����⵽�и����źš���ŵ���� >= 10mA ʱ��Ϊ�ŵ�ģʽ������Ϊ���ģʽ.
    // ����4��BQ����MCU��3.3V���ȶ�;
    // ����5����������ʱ��BQ��ʱ����ֵ�������ֵ�����쳣;
    // ����6��
    ClrWdt();                          // ˢ�¿��Ź� 1.02Sδˢ��ϵͳ��λ 
    
    Afe_Get_SysStatus();               // AFE IC ״̬��⣬������ŵ�MOS�ܿ���״̬��������������״̬��AFE IC�����󡢹�������·�����䡢���ţ��쳣״̬
   
#ifdef Uart_Model_Enable
    Uart_SendStr("\r\n");
    Uart_SendStr("\r\n SYS_STAT = ");   Uart_SendData(SYS_STAT.Byte,16); 
    //Uart_SendStr(" ADCGain_Val = ");    Uart_SendData(ADCGain_Val,10); 
    //Uart_SendStr(" ADCOffset_Val = ");  Uart_SendData(ADCOffset_Val,10); 
    //ADCGain_Val = 377 ADCOffset_Val = 47
    // UV_TRIP_Last = (uint8_t)(((uint32_t)1000 * (UV_val - ADCOffset_Val)/ADCGain_Val) >> 4); 
    // UV_TRIP_Last = (uint8_t)(((uint32_t)1000 * (1500 - 47)/377) >> 4);//86 
    // 01 0000,0000 0000 ==4096 
    Uart_SendStr(" SYS_CTRL1 = ");      Uart_SendData(SYS_CTRL1.Byte,16); 
    Uart_SendStr(" SYS_CTRL2 = ");      Uart_SendData(SYS_CTRL2.Byte,16);  
#endif
    
    ModeCheck();         // ��硢�ŵ硢���ع���ģʽ���
    //WorkMode = DISCHARGE_MODE;
    
    ClearStatus();       // ������ģʽ�µı������㴦��
    
    Afe_Volt_Val_Get();  // AFE IC �¶�ֵ����о��ѹֵ������ֵ����
    
    CurrentCheck();      // �����쳣���
    
    VoltCheck();         // ��ѹ�쳣���
    
    TempCheck();         // �¶��쳣���
    
    CellBal_Cntrl();     // ��������ƹ���
    
    ChgDis_AbnormalCheck();         // ��ŵ�״̬�³��ֵ����쳣
     
    Afe_FET_ChgDis_Cntrl();         // ��ŵ�MOS�ܿ��ƹ���
    
    //Afe_AbnormalCheck();          // AFE IC �ڲ��쳣���
    
    LedShow_Cntrl();     // LedShow_WorkMode();   // ��������ָʾ����ʾ
     
#ifdef Uart_Model_Enable  
    
    Uart_SendStr(" Check_Val= ");     Uart_SendData((uint16_t)Check_Val,10);
    Uart_SendStr(" WorkMode= ");      Uart_SendData(WorkMode,16); 
    Uart_SendStr(" Bits_flag = ");    Uart_SendData(Bits_flag.Byte,16); 
    
    //Uart_SendStr(" CC_Val= ");   Uart_SendData((uint16_t)CC_Val,10);
    Uart_SendStr(" Current_Val= ");   Uart_SendData((uint16_t)Current_Val,10);
    Uart_SendStr(" Volt_Avg= ");      Uart_SendData(Cell_Volt_Avg,10);
    Uart_SendStr(" Max= ");           Uart_SendData(Cell_Volt_Max,10);
    Uart_SendStr(" Min= ");           Uart_SendData(Cell_Volt_Min,10);
       
    Uart_SendStr(" soc_rt= ");        Uart_SendData((uint16_t)SocCalc.soc_rt,10); 
    Uart_SendStr(" ah= ");            Uart_SendData((uint16_t)SocReg.ah,10);
    Uart_SendStr(" CellBal = ");      Uart_SendData((uint16_t)CellBalance_Selct,16);  
  /* 
    Uart_SendStr(" Temp_Val = ");     Uart_SendData((uint16_t)Temp_Val,10);  
    Uart_SendStr("\r\n");
    for(i =0;i<10;i++)
    { 
      Uart_SendStr(" C_");
      Uart_SendData(i,10);
      Uart_SendStr("= ");
      Uart_SendData(Cell_Volt[i],10);
    } 
     */
#endif
    
    LowPower_Cntrl();        // �͹��Ŀ��ƹ���
  }    
 
}
 













