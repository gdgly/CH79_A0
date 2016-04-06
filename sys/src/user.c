
#include "stm8s.h" 
#include "user.h"
#include "macro_def.h"
#include "iostm8s003f3.h"
#include "bq769x0.h"
#include "string.h"
//==================================================================
//==================================================================
//==================================================================
#if 1
void PWM2_Init(void)
{ 
    CLK_PCKENR1 |= 0xA0; 
    TIM2_CR1 =0;              //�ر�TIM2
    TIM2_IER = 0; 
    TIM2_PSCR = 0;  
    TIM2_ARRH = 0x07;
    TIM2_ARRL = 0xCF;         //16M/(1999+1)=8KHz   //2MHz/(1999+1) = 1KHz 
    TIM2_CR1 |= 0x80;         //ʹ��ARP,���ض��룬���ϼ���
    TIM2_EGR |= 0x01;         //����TIM1��ʹPSC��Ч
    TIM2_EGR |= 0x20;         //���³�ʼ��TIM1 
    TIM2_CCR1H =0;           
    TIM2_CCR1L =0; 
    TIM2_CCMR1 =0X68;         //����TIM2_CH1ΪPWM1ģʽ��� 
    TIM2_CCER1 =0x01;         //Enable TIM2_CH1 channel 
    TIM2_CR1 |= 0x01;         //ʹ��TIM2 
}
//==================================================================
//==================================================================
void PWM1_Init(void)
{
    CLK_PCKENR1 |= 0x80;
    TIM1_CR1 &= ~0x01;      //�ر�TIM1 
    TIM1_PSCRH = 0;
    TIM1_PSCRL = 0;         //����Ƶ2MHz 
    TIM1_ARRH = 0x07;
    TIM1_ARRL = 0xCF;      //2MHz/(1999+1) = 1KHz 
    TIM1_CR1 |= 0x80;      //ʹ��ARP,���ض��룬���ϼ���
    TIM1_EGR |= 0x01;      //����TIM1��ʹPSC��Ч
    TIM1_EGR |= 0x20;      //���³�ʼ��TIM1 
    
    TIM1_CCR1H = 0;
    TIM1_CCR1L = 0; 
    TIM1_CCMR1 = 0x68;       //����TIM1_CH1ΪPWM1ģʽ���
    TIM1_CCER1 |= 0x01;      //Enable TIM1_CH1 channel
    
    TIM1_CCR2H = 0;
    TIM1_CCR2L = 0;
    TIM1_CCMR2 = 0x68;       //����TIM1_CH2ΪPWM1ģʽ���
    TIM1_CCER1 |= 0x10;      //Enable TIM1_CH2 channel
    
    //TIM1_CCR3H = 0x07;
    //TIM1_CCR3L = 0xE6;       //ռ�ձ�50% 
    //TIM1_CCMR3 = 0x68;       //����TIM1_CH3ΪPWM1ģʽ���
    //TIM1_CCER2 |= 0x01;      //ʹ��TIM1_CH3ͨ��
    
    //TIM1_CCR4H = 0x03;
    //TIM1_CCR4L = 0xE6;       //ռ�ձ�50%
    //TIM1_CCMR4 = 0x68;       //����TIM1_CH4ΪPWM1ģʽ��� 
    //TIM1_CCER2 |= 0x10;      //Enable TIM1_CH4 channel 
    
    TIM1_BKR |= 0x80;          //
    TIM1_CR1 |= 0x01;          //ʹ��TIM1 
}
//==================================================================
//==================================================================
uint16_t ADC(int channel)
{
    unsigned int value =0;
    unsigned char tempH,tempL; 
    //CLK_PCKENR2 |=0x08; 
    ADC_CSR = channel;                // ���EOCת��������־, select channel
    nop();  nop(); nop(); nop(); 
    ADC_CR1 |= 0x01;                  // ��ʼ����ת�� 
    nop(); nop(); nop(); nop(); nop();
    nop(); nop(); nop(); nop(); nop();
    ADC_CR1 |= 0x01;                  // ��ʼ����ת��
    nop(); nop(); nop(); nop();
    nop(); nop(); nop(); nop();
    while(!(ADC_CSR & 0x80))nop();    // �ȴ�����ת����� 
    tempL = ADC_DRL;                  //�� ����8λ
    tempH = ADC_DRH;                  // �ٶ���8λ���������������
    value=(tempH<<8)|tempL;
    ADC_CSR &= ~(0x80);               // ���EOCת��������־  
    ADC_CR1 &= ~(0x01); 
    return(value);  
}
//==================================================================
//==================================================================
uint16_t ADConverse(unsigned char channel)
{ 
    unsigned int value=0;
    unsigned int max_value =0;
    unsigned int min_value =0xEFFF;
    unsigned char i;  
    CLK_PCKENR2 |=0x08;  
    for(i =0;i <100;i++)
    {
      nop();nop();
    }
    for(i=0;i<18;i++)
    {
      Adc_value[i] = ADC(channel); 
    }
    for(i =0;i<18;i++)
    {
      value +=Adc_value[i];
      if(max_value <Adc_value[i])
        max_value =Adc_value[i];
       
      if(min_value >Adc_value[i])
        min_value =Adc_value[i];
    }
    value =value -max_value -min_value;
    value >>= 4; 
    CLK_PCKENR2 &=~(0x08); 
    return value; 
}
//================================================================================
//==================================================================
void SysInit(void)
{ 
    //-------system clock                                           
    CLK_ECKR &= ~0x01;  // external RC Disable                                              
    CLK_ICKR = 0x01;    // internal RC enable
    while(!(CLK_ICKR & 0x02)); 
    CLK_SWCR = 0;       // ��ֹʱ���л�������ж�
    CLK_CKDIVR = 0x02;  // ��ƵΪFmasterΪFhsi��16MHz, FcpuΪ4MHz  0.25us
    //CLK_PCKENR1 = 0;    // ��ֹFmaster ����������
    //CLK_PCKENR2 = 0;
    /* */
    //------------------A/D conversion 
    CLK_PCKENR2 |= 0x08;           //ʹ��Fmaster������ADCģ������ 
    ADC_CR1 = 0x00;               // ADCʱ��=��ʱ��/2=8MHZ,����ת��ģʽTad =1/8,δʹ��ADC
    ADC_CR2 = 0x08;               // A/D ��������Ҷ���
    ADC_CSR = 0x00;               // ��ת��������־λEOC��
    ADC_CR3 = 0x00;
    ADC_TDRH = 0xFF;              //��ֹʩ���ش�������
    ADC_TDRL = 0x00; 
    nop(); nop();
    ADC_TDRL = 0x08;              // AIN3  0b0000 1000 
    CLK_PCKENR2 &=~(0x08);
   
}
//==================================================================
//==================================================================
//---------------WatchDog  LSI 128KHz/2 = 64KHz
/*
    Ԥ��Ƶϵ��    PR[2:0]   ��̳�ʱ(RL[7:0]=0x00)    ���ʱ(RL[7:0]=0xFF)
    /4            0         62.5 ��s                  15.90 ms     
    /8            1         125 ��s                   31.90 ms     
    /16           2         250 ��s                   63.70 ms     
    /32           3         500 ��s                   127 ms     
    /64           4         1.00 ms                   255 ms     
    /128          5         2.00 ms                   510 ms     
    /256          6         4.00 ms                   1.02 s     
*/
/*  */
void ClrWdt(void)
{
    IWDG_KR = 0x55;   //�������
    IWDG_RLR = 0xFF;  //ˢ������
    IWDG_PR = 6; 
    IWDG_KR = 0xAA;   //ˢ�¼��ָ�����
    IWDG_KR = 0xCC;   //�������Ź�����
} 

//==================================================================
//==================================================================
void PortInit(void)
{ 
  GPIO_Init(GPIOA, GPIO_PIN_3, GPIO_MODE_OUT_PP_LOW_FAST);    // undifined
  GPIO_Init(GPIOA, GPIO_PIN_2, GPIO_MODE_OUT_PP_LOW_FAST);    // undifined
  GPIO_Init(GPIOA, GPIO_PIN_1, GPIO_MODE_OUT_PP_LOW_FAST);    // undifined
  
  GPIO_Init(GPIOB, GPIO_PIN_5, GPIO_MODE_OUT_PP_LOW_FAST);           
  GPIO_Init(GPIOB, GPIO_PIN_4, GPIO_MODE_OUT_PP_LOW_FAST);       
 
  GPIO_Init(GPIOC, GPIO_PIN_7, GPIO_MODE_OUT_PP_LOW_FAST);    // ALERT 
  GPIO_Init(GPIOC, GPIO_PIN_6, GPIO_MODE_OUT_PP_LOW_FAST);    // chger fault control 
  GPIO_Init(GPIOC, GPIO_PIN_5, GPIO_MODE_OUT_PP_LOW_FAST);    // LED1
  GPIO_Init(GPIOC, GPIO_PIN_4, GPIO_MODE_OUT_PP_LOW_FAST);    // LED2
  GPIO_Init(GPIOC, GPIO_PIN_3, GPIO_MODE_OUT_PP_LOW_FAST);    // LED3 
   
  GPIO_Init(GPIOD, GPIO_PIN_6, GPIO_MODE_IN_PU_IT);          
  GPIO_Init(GPIOD, GPIO_PIN_5, GPIO_MODE_OUT_PP_HIGH_FAST);          
  GPIO_Init(GPIOD, GPIO_PIN_4, GPIO_MODE_OUT_PP_LOW_FAST);    // undefined
  GPIO_Init(GPIOD, GPIO_PIN_3, GPIO_MODE_IN_FL_NO_IT);        // chger fault input
  GPIO_Init(GPIOD, GPIO_PIN_2, GPIO_MODE_IN_FL_NO_IT);        // Triger Voltage measurement
#if 0
  CPU_CFG_GCR |=0x01;                                         //SWINģʽ�����ã�SWIM���ſɱ�������ͨI/O��
  GPIO_Init(GPIOD, GPIO_PIN_1, GPIO_MODE_OUT_PP_HIGH_FAST);   //MUC_DO3
#else
  GPIO_Init(GPIOD, GPIO_PIN_1, GPIO_MODE_IN_PU_NO_IT);        //SWIN 
#endif
  //EXTI_CR1 |=0x40;//�����ش���//0x80;//�½��ش���
  EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOD, EXTI_SENSITIVITY_FALL_ONLY);//Signal_In�½��ش���
  /*------------------------------------------------------------------------------------
    GPIO_MODE_IN_FL_NO_IT ���жϹ��ܵĸ������롣  // ADC input
    
    GPIO_MODE_IN_PU_NO_IT ���жϹ��ܵ��������롣  // normal digit input
    
    GPIO_MODE_IN_FL_IT ���жϹ��ܵĸ������롣  
    
    GPIO_MODE_IN_PU_IT ���жϹ��ܵ��������롣	
    
    GPIO_MODE_OUT_OD_LOW_FAST ���ٿ�©�͵�ƽ������ɹ�����10MHz��  
    
    GPIO_MODE_OUT_PP_LOW_FAST ��������͵�ƽ������ɹ�����10MHz��  
    
    GPIO_MODE_OUT_OD_LOW_SLOW ���ٿ�©�͵�ƽ������ɹ�����2MHz��	// undefined port
    
    GPIO_MODE_OUT_PP_LOW_SLOW ��������͵�ƽ������ɹ�����2MHz��	
    
    GPIO_MODE_OUT_OD_HIZ_FAST ���ٿ�©����̬������ɹ�����10MHz��  
    
    GPIO_MODE_OUT_PP_HIGH_FAST ��������ߵ�ƽ������ɹ�����10MHz��  
    
    GPIO_MODE_OUT_OD_HIZ_SLOW ���ٿ�©����̬������ɹ�����2MHz��	
    
    GPIO_MODE_OUT_PP_HIGH_SLOW ��������ߵ�ƽ������ɹ�����2MHz��
   ------------------------------------------------------------------------------------*/
} 
//============================================UART����ģ�� begin
/*--------------------------------
  ���� UART1
    - BaudRate = 9600 baud
    - Word Length = 8 Bits
    - One Stop Bit
    - No parity
    - Receive and transmit enabled
    - UART1 Clock disabled
--------------------------------*/
void Uart_Model_Init(void)
{
    //UART1_DeInit(); 
    //UART1_Init((uint32_t)9600, UART1_WORDLENGTH_8D, UART1_STOPBITS_1, UART1_PARITY_NO,UART1_SYNCMODE_CLOCK_DISABLE, UART1_MODE_TX_ENABLE);//UART1_MODE_TXRX_ENABLE
    //================================================================== 
    /*  */
    CLK_PCKENR1 |= 0x0C;         // ʹ��fmaster��UART���� 
    UART1_BRR2  =  0x02;         // ���ò�����9600
    UART1_BRR1  =  0x68;         // 16M/9600 = 0x0682
    UART1_CR1   =  0x00;          //UARTʹ�ܣ�һ����ʼλ��8������λ����ֹ��żУ�飬��ֹ�ж�
    UART1_CR3   =  0x00;          //һ��ֹͣλ    
    UART1_CR2   =  0x08;//  send only //0x0C;           //���ͼ�����ʹ�� 
    //UART1_CR2   |= 0x20;          //�����ж�ʹ��  
} 
//==========================================
void Uart_SendByte(uint8_t DataByte)      
{   
    //===����ͨѶʱ����������ʱ����Ӱ����չܽţ��ʷ�������ʱ�Ƚ�ֹ���ռ����ж�
    //UART1_CR2 &= ~0x04;
    //UART1_CR2 &= ~0x20; 
    while(!UART1_SR_TXE);
    UART1_DR = DataByte;    
    nop(); nop(); nop();
    while(!(UART1_SR &0x40)); 
    //========���ݷ�����ϣ�ʹ�ܽ��ռ����ж�
    UART1_SR &= ~0x48;
    //UART1_CR2 |= 0x24; 
}  
//==========================================
void itoa(char *buf, int i, int base)
{
    #define LEN	20
    char *s;
    int rem;
    static char rev[LEN+1];
  
    rev[LEN] = 0;
    if (i == 0)
    {
      (buf)[0] = '0';
      ++(buf);
      return;
    }
    s = &rev[LEN];
    while (i)
    {
      rem = i % base;
      if (rem < 10)
      {
        *--s = rem + '0';
      }
      else if (base == 16)
      {
        *--s = "abcdef"[rem - 10];
      }
      i /= base;
    }
    while (*s)
    {
      (buf)[0] = *s++;
      ++(buf);
    }
}

//==========================================
void Uart_SendStr(unsigned char *tx_pData) 
{
    unsigned int i, nLen; 
    nLen = strlen(tx_pData);
    ClrWdt();
    for(i=0; i<nLen; i++)
    {
      Uart_SendByte(tx_pData[i]);
    }
}

//==========================================
void Uart_SendData(unsigned int tx_data, int base) 
{
  unsigned char buf[20] = {0};

  itoa((char *)buf, tx_data, base);
  
  if (base == 16)
  {
    Uart_SendStr((unsigned char *)"0x");
  }
  
  Uart_SendStr(buf);
}
//============================================UART����ģ�� end

//==================================================================
void I2C_Model_Init(void)
{ 
  CLK_PCKENR1 |= 0x01;    // Fmaster ������I2C���� 
  I2C_CR1 = 0;
   
  //I2C_ITR   = 0x01;    //ʹ�ܴ����ж�
  I2C_FREQR = 8;//����ʱ��Ϊ8MHz
  
  I2C_TRISER = 9;
   
  //CRR�ļ��㣬ȷ��I2C��ͨ��Ƶ��f_SCL =1/(T_high + T_low), 100KHz -->T_hgih = T_low =5000ns
  //I2C������Ƶ��f_CK = 1/T_CK, CRR = T_hgih /T_CK;
  I2C_CCRL = 80;//40; // �����ؼ��½���ʱ��5000s, 
  I2C_CCRH = 0;  // ��׼ģʽ100KHz,  
     
  I2C_CR1 |= 0x01;//I2C_CR1_PE;
  
  I2C_CR2 |= 0x04;
}
//================================================================== 
//=======================================================================
//======================================================================= 
//======================================================================= 
void Timer2Init(void)
{ 
    CLK_PCKENR1 |= 0x20;    // Fmaster ������TIM2����
    TIM2_CR1 = 0x84;        // TIM2_ARR�Ĵ���ͨ������Ԥװ�أ�ʹ�ܼ�����
    TIM2_IER = 0x00;        // ��ֹ�ж�
    //TIM2_EGR = 0x01;        // ������������¼� 
    TIM2_PSCR = 0x07;       // ������ʱ��=��ʱ��/8=16MHZ/128    8us// �൱�ڼ���������Ϊ 
    TIM2_CNTRH = 0;         //     
    TIM2_CNTRL = 0;         //     
    TIM2_ARRH = 0x04;       //     10ms
    TIM2_ARRL = 0xE2;       //   
    TIM2_IER |= 0x01;       // ʹ�ܸ����жϣ���ֹ�����ж�  
    TIM2_CR1 |= 0x01;        // TIM2_ARR�Ĵ���ͨ������Ԥװ�أ�ʹ�ܼ�����
} 
//=======================================================================
//==================================================================
//==================================================================
//--Fmaster = CK_PSC =16MHz, CK_PSC prescaler(128) to CK_CNT(8us)   TIM4_SR1.UIF�����жϱ�־��
void Timer4_Init_us(void)
{
    CLK_PCKENR1 |= 0x10;    //Fmaster ������TIM4����
    TIM4_IER = 0x00;        // ��ֹ�ж�
    TIM4_EGR = 0x01;        // ������������¼� 
    //TIM4_PSCR = 0x07;       // ������ʱ��=��ʱ��/128=16MHZ/128  // �൱�ڼ���������Ϊ8uS
    TIM4_PSCR = 0x00;       // ������ʱ��=��ʱ��/128=16MHZ/1  // �൱�ڼ���������Ϊ0.0625uS
    TIM4_CR1 = 0x15;        // TIM4_ARR�Ĵ���ͨ������Ԥװ�أ�ʹ�ܼ�����
    TIM4_CNTR = 0;          //     
    TIM4_ARR = 16;           // 0.0625*16 = 1us  // 6*8us = 200us  
    TIM4_IER |= 0x01;       //�����ж�ʹ�ܣ���ֹ�����ж� 
    TIM4_CR1 |= 0x01; 
} 
void Timer4_Init_ms(void)
{
    CLK_PCKENR1 |= 0x10;    //Fmaster ������TIM4����
    TIM4_IER = 0x00;        // ��ֹ�ж�
    TIM4_EGR = 0x01;        // ������������¼� 
    TIM4_PSCR = 0x07;       // ������ʱ��=��ʱ��/128=16MHZ/128  // �൱�ڼ���������Ϊ8uS 
    TIM4_CR1 = 0x15;        // TIM4_ARR�Ĵ���ͨ������Ԥװ�أ�ʹ�ܼ�����
    TIM4_CNTR = 0;          //     
    TIM4_ARR = 125;         // 125*8us = 1ms  
    TIM4_IER |= 0x01;       //�����ж�ʹ�ܣ���ֹ�����ж� 
    TIM4_CR1 |= 0x01; 
} 
void Timer4_Disable(void)
{
    TIM4_CR1 &= ~0x01;
    TIM4_IER &= ~0x01;
    CLK_PCKENR1 &= ~0x10;    //Fmaster ������TIM4�Ͽ����� 
} 
void Delay_us(uint16_t Delay_time)
{
  Delay_time_t = Delay_time;
  Timer4_Init_us();
  while(Delay_time_t > 0);  
  Timer4_Disable();
}
void Delay_ms(uint16_t Delay_time)
{
  Delay_time_t = Delay_time;
  Timer4_Init_ms();
  while(Delay_time_t > 0)
  {
    if(Delay_time_t >= 300)
    {  
      ;//ClrWdt(); 
    } 
  }
  Timer4_Disable();
}
  
//==================================================================
/*
  The following equations show how to use the 14-bit ADC readings in TS1, TS2, and TS3 to determine the
  resistance of the external 103AT thermistor:
    VTSX = (ADC in Decimal) x 382 ��V/LSB                     (4)
    RTS = (10,000 �� VTSX) �� (3.3 �C VTSX)                   (5)
*/
void TempCheck(void)
{ 
    static uint8_t ChgTemp_cnt =0;
    static uint8_t DisTemp_cnt =0;  
     
    Temp_Val = V_TS2_Val;//R_TS2_Val;
    if(WorkMode == CHARGE_MODE)//if(Bits_flag.Bit.Chg)
    {
       if((Temp_Val >ChgTempL_ON) || (Temp_Val < ChgTempH_ON) || (ChgTemp_cnt >= 10))
       {
         if((ChgTemp_cnt ++) >= 10)
         {
           ChgTemp_cnt = 10;
           Bits_flag.Bit.ChgTemp = 1;
         }
       }
       else
       {
         ChgTemp_cnt = 0;
       }
      //==========================����¶ȱ����ָ�
      if(Bits_flag.Bit.ChgTemp && (Temp_Val < ChgTempL_OFF) && (Temp_Val >ChgTempH_OFF))
      {
          ChgTemp_cnt = 0;
          Bits_flag.Bit.ChgTemp = 0;
      } 
    }
    else if(WorkMode == DISCHARGE_MODE)//(Bits_flag.Bit.Dis)
    {
      ChgTemp_cnt = 0;
      if((Temp_Val > DisTempL_ON) || (Temp_Val < DisTempH_ON) || (DisTemp_cnt >= 10)) 
      {
        if((DisTemp_cnt ++) >= 10)
        {
          DisTemp_cnt = 10;
          Bits_flag.Bit.DisTemp = 1;
        }
      }
      else
      {
        DisTemp_cnt = 0;
      }  
      //==========================�ŵ��¶ȱ����ָ�
      if(Bits_flag.Bit.DisTemp && (Temp_Val < DisTempL_OFF) && (Temp_Val > DisTempH_OFF))
      {
        DisTemp_cnt = 0;
        Bits_flag.Bit.DisTemp = 0;
      } 
    }   
    //====================================================
    
  if(SYS_STAT.Bit.DEVICE_XREADY)// || SYS_STAT.Bit.OVRD_ALERT)
  {
    Bits_flag.Bit.AfeErr = 1;
  } 
}
//================================================================== 
//==================================================================
/*      SYS_STAT (0x00)/RESET:0x00
        BIT        7      6           5             4       3   2    1     0
        NAME   CC_READY  RSVD   DEVICE_XREADY   OVRD_ALERT  UV  OV  SCD   OCD
*/
void CurrentCheck(void)
{  
  if(WorkMode == CHARGE_MODE)
  {
    if(Current_Val > ChgCurOv_Val_SET)
    {
      if(ChgCurOv_t >= ChgCurOv_t_SET)
      {
        ChgCurOv_t = ChgCurOv_t_SET;
        Bits_flag.Bit.ChgCurOv = 1;
      }
    }
    else
    {
      ChgCurOv_t = 0;
    } 
    /*
    if(0)//(Bits_flag.Bit.ChgCurOv && ChgCurOv_Re_t >= ChgCurOv_Re_t_SET)
    {
      ChgCurOv_t = 0; 
      ChgCurOv_Re_t = 0;
      Bits_flag.Bit.ChgCurOv = 0;
    } */
  }
  else if(WorkMode == DISCHARGE_MODE)//9A,  15A
  {
    if(SYS_STAT.Bit.OCD || Current_Val > DisCurOv_2_Val_SET || (DisCurOv_t1 >= DisCurOv_t1_SET) || (DisCurOv_t1 >= DisCurOv_t1_SET))
    {
      if(SYS_STAT.Bit.OCD || (DisCurOv_t2 >= DisCurOv_t2_SET) || (DisCurOv_t1 >= DisCurOv_t1_SET))
      {
        DisCurOv_t1 = DisCurOv_t1_SET;
        DisCurOv_t2 = DisCurOv_t2_SET;
        Bits_flag.Bit.DisCurOv = 1;
      } 
    }
    else if(Current_Val > DisCurOv_1_Val_SET)//9A
    { 
      DisCurOv_t2 = 0;
      if(DisCurOv_t1 >= DisCurOv_t1_SET)
      {
        DisCurOv_t1 = DisCurOv_t1_SET; 
        Bits_flag.Bit.DisCurOv = 1;
      }  
    }
    else
    {
      DisCurOv_t1 = 0;
      DisCurOv_t2 = 0;
    }
    /*
    if(Current_Val > DisCurOv_Val_SET || SYS_STAT.Bit.OCD || (DisCurOv_t >= DisCurOv_t_SET))
    {
      if(SYS_STAT.Bit.OCD || (DisCurOv_t >= DisCurOv_t_SET))
      {
        DisCurOv_t = DisCurOv_t_SET; 
        Bits_flag.Bit.DisCurOv = 1;
      } 
    }
    else
    {
      DisCurOv_t = 0;
    }
    */
    /*
    if(0)//(Bits_flag.Bit.DisCurOv)// && DisCurOv_Re_t >= DisCurOv_Re_t_SET)
    {
      //clear the OV bit by writing "1"
      if(SYS_STAT.Bit.OCD)
      {
        SYS_STAT_Last |= 0x01;
        I2C_Write(SYS_STAT_ADDR,SYS_STAT_Last);
        SYS_STAT_Last &= ~0x01;
      }
      DisCurOv_t = 0;
      DisCurOv_Re_t = 0;
      Bits_flag.Bit.DisCurOv = 0;
    }*/
    
    //==========================��·���
    //if(WorkMode != CHARGE_MODE)
    {
      if( SYS_STAT.Bit.SCD )
      {  
         Bits_flag.Bit.DisCurShort = 1; 
      }
      /*
      if(Bits_flag.Bit.DisCurShort && DisCurShort_Re_t >= DisCurShort_Re_t_SET)
      { 
        SYS_STAT_Last |= 0x02;   //clear the SCD bit by writing "1" 
        I2C_Write(SYS_STAT_ADDR,SYS_STAT_Last); 
        SYS_STAT_Last &= ~0x02;
        DisCurOv_Re_t = 0;
        SYS_STAT.Bit.SCD = 0;
        Bits_flag.Bit.DisCurShort =0;
      }
      */
    }
  }
  
  
}
//==================================================================
//==================================================================
/*      SYS_STAT (0x00)/RESET:0x00
        BIT        7      6           5             4       3   2    1     0
        NAME   CC_READY  RSVD   DEVICE_XREADY   OVRD_ALERT  UV  OV  SCD   OCD
*/
void VoltCheck(void)
{  
  if(WorkMode == CHARGE_MODE)
  {
    if(Cell_Volt_Max >= CHG_OV_VAL_SET ||SYS_STAT.Bit.OV)
    { 
      if((ChgOv_t >= ChgOv_t_SET) ||SYS_STAT.Bit.OV)
      {
        ChgOv_t = ChgOv_t_SET;
        Bits_flag.Bit.ChgOv = 1;
      }
    }
    else
    {
      ChgOv_t =0;
    }
    
    if(Bits_flag.Bit.ChgOv && Cell_Volt_Max < CHG_OV_RE_VAL_SET)// Cell_Volt_Avg < CHG_OV_RE_VAL_SET &&
    {
      //clear the OV bit by writing "1"
      if(SYS_STAT.Bit.OV)
      {
        SYS_STAT_Last |= 0x04;
        I2C_Write(SYS_STAT_ADDR,SYS_STAT_Last);
        SYS_STAT_Last &= ~0x04;
      }
      ChgOv_t = 0;
      SYS_STAT.Bit.OV = 0;
      Bits_flag.Bit.ChgOv = 0;
    }
      
  }
  else if(WorkMode == DISCHARGE_MODE)
  {
    //=========���������о��ѹ
    if(Dis_First_Run_Flag ==0 && WorkMode == DISCHARGE_MODE)
    { 
      if((Cell_Volt_Avg < 3300) || (Cell_Volt_Min < 3300) || SYS_STAT.Bit.UV)
      {
        if(Dis_First_Run_t >= 100)
        {
          Dis_First_Run_t = 100;
          Bits_flag.Bit.DisOv = 1;
          Dis_First_Run_Flag  = 1;
        }
      }
      else 
      {
        if(Dis_First_Run_t >= 50)
        {
          Dis_First_Run_t = 100; 
          Bits_flag.Bit.DisOv = 0;
          Dis_First_Run_Flag  = 1;
          DisOv_t = 0;
          //clear the UV bit by writing "1"
          if(SYS_STAT.Bit.UV)
          {
            SYS_STAT_Last |= 0x08;
            I2C_Write(SYS_STAT_ADDR,SYS_STAT_Last);
            SYS_STAT_Last &= ~0x08;
          }
        }
      } 
    } 

    //==== �ŵ�Ƿѹ���
    if((Cell_Volt_Avg < DIS_UV_VAL_SET) ||(Cell_Volt_Min < DIS_UV_MIN_VAL_SET) || SYS_STAT.Bit.UV)// 
    {
      if((DisOv_t >= DisOv_t_SET) || SYS_STAT.Bit.UV)
      {
        DisOv_t = DisOv_t_SET;
        Bits_flag.Bit.DisOv = 1;
      }
    }
    else
    {
      DisOv_t = 0;
    }
    /*
    if(Bits_flag.Bit.DisOv && Cell_Volt_Min >= DIS_UV_RE_VAL_SET)//Cell_Volt_Avg >= DIS_UV_RE_VAL_SET && 
    {
      //clear the UV bit by writing "1"
      if(SYS_STAT.Bit.UV)
      {
        SYS_STAT_Last |= 0x08;
        I2C_Write(SYS_STAT_ADDR,SYS_STAT_Last);
        SYS_STAT_Last &= ~0x08;
      }
      DisOv_t = 0;
      SYS_STAT.Bit.UV = 0;
      Bits_flag.Bit.DisOv = 0;
    }
    */
  }
}   
//==========================================================================
void ModeCheck_Backup(void)
{       
  //����ģʽ�ļ���������ⲿTriger�˵ĵ�ѹ��С���ж�
  Check_Val = ADConverse(3); 
  if((Check_Val >= 130 && Check_Val < 160) || CC_Val >= 10)
  {  
    if(ChgExchangeMode_Cnt >= 30)
    { 
      ChgExchangeMode_Cnt  = 100;
      DisExchangeMode_Cnt  = 0;
      IdleExchangeMode_Cnt = 0;
      WorkMode = CHARGE_MODE;
    }
  }   
  else if((Check_Val >= 220 && Check_Val < 630) || SYS_CTRL1.Bit.LOAD_PRESENT || CC_Val < (-10) )
  {  
    if(DisExchangeMode_Cnt >= 30)
    { 
      IdleExchangeMode_Cnt = 0;
      ChgExchangeMode_Cnt = 0;
      DisExchangeMode_Cnt = 100; 
      WorkMode = DISCHARGE_MODE;
    }
  } 
  else if(Check_Val < 10)
  { 
    if(IdleExchangeMode_Cnt >= 100)
    { 
      IdleExchangeMode_Cnt = 100;
      ChgExchangeMode_Cnt = 0;
      DisExchangeMode_Cnt = 0;
      WorkMode = IDLE_MODE;
    }
  } 
}

//==========================================================================
void ModeCheck(void)
{       
  static uint8_t ModeChange_Lock = 0;
  //����ģʽ�ļ���������ⲿTriger�˵ĵ�ѹ��С���ж�
  
  // ����3������ŵ缰����ʶ�����⣩Ƿѹ״̬�£����ϳ����ʱ��ʶ���˳����;  
          //��������������Triger�ź�ʱ��Ϊ����ģʽ;
                      //Triger�źŷ�Χ��[220,630]ʱ��[350,580]�������⵽�и����źš���ŵ���� >= 10mA ʱ��Ϊ�ŵ�ģʽ������Ϊ���ģʽ.
  
//==========================================================================
//== OV_valΪ���䱣����ѹֵ��mV���� UV_valΪ���ű�����ѹֵ��mV��
//void Afe_OV_UV_Threshold_Set(uint16_t OV_val, uint16_t UV_val)
  
  Check_Val = ADConverse(3); 
  if(Check_Val < 10)// ���ź�
  { 
    if(IdleExchangeMode_Cnt >= 100)
    { 
      IdleExchangeMode_Cnt = 100;
      ChgExchangeMode_Cnt = 0;
      DisExchangeMode_Cnt = 0;
      WorkMode = IDLE_MODE;
      if(ModeChange_Lock != 0)
      {
        ModeChange_Lock = 0;
        Afe_OV_UV_Threshold_Set(OV_THREHOLD_VAL_SET, UV_THREHOLD_VAL_SET); 
      }
    }
  } 
  else if((Check_Val >= 220 && Check_Val < 630) || SYS_CTRL1.Bit.LOAD_PRESENT || CC_Val < (-10) )
  {  
    if(DisExchangeMode_Cnt >= 30)
    { 
      IdleExchangeMode_Cnt = 0;
      ChgExchangeMode_Cnt = 0;
      DisExchangeMode_Cnt = 100; 
      WorkMode = DISCHARGE_MODE;
      if(ModeChange_Lock != 2)
      {
        ModeChange_Lock = 2;
        // ����1���ŵ�״̬�£���о��ѹ���ڹ���ʱ��BQ���Զ��رճ��MOS��;   ����������ŵ�״̬�£���������Ӳ�������ѹΪ4.3V
        Afe_OV_UV_Threshold_Set(4300, UV_THREHOLD_VAL_SET); 
      }
    }
  }  
  else// if((Check_Val >= 130 && Check_Val < 160) || CC_Val >= 10)
  {  
    if(ChgExchangeMode_Cnt >= 30)
    { 
      ChgExchangeMode_Cnt  = 100;
      DisExchangeMode_Cnt  = 0;
      IdleExchangeMode_Cnt = 0;
      WorkMode = CHARGE_MODE;
      if(ModeChange_Lock != 1)
      {
        ModeChange_Lock = 1;
        // ����2�����״̬�£���о��ѹ���ڹ���ʱ��BQ���Զ��رշŵ�MOS��;   ������������״̬�£���������Ӳ�������ѹΪ1.0V
        Afe_OV_UV_Threshold_Set(OV_THREHOLD_VAL_SET, 1500);
      }
    }
  }   
}

  //===========================================
void ClearStatus(void)
{
  if(WorkMode == CHARGE_MODE)
  {  
      LowPower_MCU_Entry_Flag = 0;
      Dis_First_Run_Flag = 0;
      Dis_First_Run_t = 0;
      DisOv_t = 0;  
      //DisCurOv_t = 0;  
      //DisCurOv_Re_t = 0; 
      PowerOff_Delay_t = 0;
       
      Bits_flag.Bit.DisTemp = 0;
      //Bits_flag.Bit.DisCurShort = 0; 
      
      if(Bits_flag.Bit.DisOv || SYS_STAT.Bit.UV)
      { 
        if(SYS_STAT.Bit.UV)  //clear the UV bit by writing "1"
        {
          SYS_STAT_Last |= 0x08;
          I2C_Write(SYS_STAT_ADDR,SYS_STAT_Last);
          SYS_STAT_Last &= ~0x08;
        }  
        Bits_flag.Bit.DisOv = 0;
      }
     /* if(Bits_flag.Bit.DisCurOv || SYS_STAT.Bit.OCD) 
      { 
        if(SYS_STAT.Bit.OCD)  //clear the OV bit by writing "1"
        {
          SYS_STAT_Last |= 0x01;
          I2C_Write(SYS_STAT_ADDR,SYS_STAT_Last);
          SYS_STAT_Last &= ~0x01;
        }
        Bits_flag.Bit.DisCurOv = 0;
      } */
  }/*
  else if(WorkMode == DISCHARGE_MODE)
  {
      ChgOv_t = 0; 
      ChgCurOv_t = 0; 
      ChgCurOv_Re_t = 0; 
       
      Bits_flag.Bit.ChgCurOv = 0;
      Bits_flag.Bit.ChgTemp = 0;
      if(Bits_flag.Bit.ChgOv || SYS_STAT.Bit.OV)
      { 
        if(SYS_STAT.Bit.OV)  //clear the UV bit by writing "1"
        {
          SYS_STAT_Last |= 0x04;
          I2C_Write(SYS_STAT_ADDR,SYS_STAT_Last);
          SYS_STAT_Last &= ~0x04;
        }  
        Bits_flag.Bit.ChgOv = 0;
      }
  }*/
  else if(WorkMode == IDLE_MODE)
  {   
    Dis_First_Run_Flag = 0;
    Dis_First_Run_t = 0;
    //if(Bits_flag.Bit.ChgCurOv)// && ChgCurOv_Re_t >= ChgCurOv_Re_t_SET)
    {
      ChgCurOv_t = 0; 
      DisCurOv_t1 = 0;
      DisCurOv_t2 = 0;
      ChgCurOv_Re_t = 0;
      Bits_flag.Bit.ChgCurOv = 0;
    } 
    //=========================================================
    DisCurShort_Re_t = 0;
    if(Bits_flag.Bit.DisCurShort  || SYS_STAT.Bit.SCD) 
    { 
      SYS_STAT_Last |= 0x02;
      I2C_Write(SYS_STAT_ADDR,SYS_STAT_Last); //clear the SCD bit by writing "1" 
      SYS_STAT_Last &= ~0x02;
      Afe_SCD_Set(SCD_THREHOLD_VAL_SET, SCD_DELAY_SET);
      DisCurOv_Re_t = 0;
      SYS_STAT.Bit.SCD = 0;
      Bits_flag.Bit.DisCurShort = 0;
    }
    //=========================================================
    if(Bits_flag.Bit.DisCurOv || SYS_STAT.Bit.OCD) 
    { 
      if(SYS_STAT.Bit.OCD)  //clear the OV bit by writing "1"
      {
        SYS_STAT_Last |= 0x01;
        I2C_Write(SYS_STAT_ADDR,SYS_STAT_Last);
        SYS_STAT_Last &= ~0x01;
        Afe_OCD_Set(OCD_THREHOLD_VAL_SET, OCD_DELAY_SET);
      }
      Bits_flag.Bit.DisCurOv = 0;
    } 
    //=========================================================
    DisCurOv_t = 0;
    DisCurOv_Re_t = 0;
    Bits_flag.Bit.ChgOv = 0;
    Bits_flag.Bit.ChgCurOv = 0;
    Bits_flag.Bit.ChgTemp = 0;
     
  }
  else
  { 
    LowPower_MCU_Entry_Flag = 0; 
  }
}  
//==========================================================================
  /*The ADC transfer function is a linear equation defined as follows:
    V(cell) = GAIN x ADC(cell) + OFFSET
    If the DEVICE_XREADY is set, the voltage register values from the device should not be used. If
  */ 
void Afe_Volt_Val_Get(void)
{ 
  uint8_t i = 0;
  uint16_t volt_sum = 0;
  uint16_t volt_max = 0;
  uint16_t volt_min = 0xFFFF;
  //====================================250mS
  if(Cell_Volt_Sample_Cnt >= 26)
  {
    Cell_Volt_Sample_Cnt = 0;
    for(i =0;i< 20;i += 2)
    {  
      Adc_value[i/2] = Afe_Get_Adc((VC1_HI_ADDR + i)) & 0x3FFF; //VC1_HI_ADDR =0x0C
      Cell_Volt[i/2] = (uint16_t)((uint32_t)ADCGain_Val *Adc_value[i/2]/1000) + ADCOffset_Val;
      volt_sum += Cell_Volt[i/2];
      if(Cell_Volt[i/2] < volt_min)
      {
         volt_min = Cell_Volt[i/2];
      }
      if(Cell_Volt[i/2] > volt_max)
      {
         volt_max = Cell_Volt[i/2];
      }
    } 
    Cell_Volt_Tol = volt_sum; 
    Cell_Volt_Avg = Cell_Volt_Tol/10; 
    Cell_Volt_Max = volt_max; 
    Cell_Volt_Min = volt_min; 
      
  }
  //====================================250mS
  /*  Once converted to digital form, each cell voltage is added up and the summation result stored in the BAT
      registers. This 16-bit value has a nominal LSB of 1.532 mV. */
  //Pack_Volt = (uint16_t)((uint32_t)4 *ADCGain_Val *Afe_Get_Adc(BAT_HI_ADDR)/1000+(10 *ADCOffset_Val));
  //Pack_Volt = (uint16_t)((uint32_t)1532 *Afe_Get_Adc(BAT_HI_ADDR)/1000);
  
  //====================================250mS
  //Positive values are charge current; negative values are discharge current.
  //an average over the 250-ms integration period.
  /*Why does the CC value read not change when I set the CC_ONESHOT bit?
        The CC_READY bit must be cleared before the CC_ONESHOT bit is set. Note also that the CC_EN bit
        should be cleared or a CC conversion may already be in progress.
    The full scale range of the CC is �� 270 mV, with a max recommended input range of �� 200 mV, thus yielding an LSB of approximately 8.44 ��V.
    The following equation shows how to convert the 16-bit CC reading into an analog voltage if no boardlevel calibration is performed:
    CC Reading (in ��V) = [16-bit 2��s Complement Value] �� (8.44 ��V/LSB) 
  */
  if( SYS_STAT.Bit.CC_READY) //CC_Volt_Sample_Cnt >= 23 &&
  {
    CC_Volt_Sample_Cnt = 0;
    SYS_STAT.Bit.CC_READY = 0;
    CC_AD = Afe_Get_Adc(CC_HI_ADDR);  
    //CC_Val = (int32_t)820 * CC_AD /100; //mA (int32_t)
    CC_Val = (int32_t)820 * CC_AD /500; //mA (int32_t)
     
    Afe_CC_1Shot_Set();
    if(CC_Val < -320000)
    {
      CC_Val = -320000;
    }
    else if(CC_Val > 320000)
    {
      CC_Val = 320000;
    }
    Current_Val = CC_Val;
    if(CC_Val < 0)
    {
      Current_Val = -CC_Val;
    }
  }
  //====================================2S 
  if(Temp_Volt_Sample_Cnt >= 210)
  {
    //V_TS2_Val = (uint16_t)((uint32_t)(382 * ((uint16_t)adcval <<8 | tmpval))/1000);//mV
    V_TS2_Val = Afe_Get_Adc(TS2_HI_ADDR) &0x3FFF ;
    V_TS2_Val = (uint16_t)(((uint32_t)382 * V_TS2_Val /1000));  //mV
    V_TS2_Val = (uint16_t)(((uint32_t)1024 * V_TS2_Val /3300));  //mV
    //R_TS2_Val = (uint16_t)((uint32_t)10000 * V_TS2_Val)/(3300 - V_TS2_Val); 
  }
  //=====================================================================================
  
}
 
//==========================================================================
/*
  turn on condition  :    >4.0V && deltaVolt >200mV
  turn off condition :   over charge assert the charging protection
*/ 
void CellBal_Cntrl(void)
{
  uint8_t i = 0;
  if(WorkMode == CHARGE_MODE)
  {
    if(Cell_Balance_Delay_t >= 100)
    {
      Cell_Balance_Delay_t = 0;
      if((Cell_Volt_Max >= CELLBALANCE_BEGIN_VAL) && ((Cell_Volt_Max - Cell_Volt_Min) >= CELLBALANCE_DELTA_VAL) && !Bits_flag.Bit.ChgOv && !Bits_flag.Bit.ChgTemp && !Bits_flag.Bit.ChgCurOv)
      {
        CellBal_Cntrl_Lock = 1;
        for(i =0; i <10; i++)
        {
          if(Cell_Volt[i] > CELLBALANCE_BEGIN_VAL && ((Cell_Volt[i] - Cell_Volt_Min) >= CELLBALANCE_DELTA_VAL))
          {   
            CellBalance_Cur_Selct |= (0x0001 << i); 
          }
          else
          {
            CellBalance_Cur_Selct &= ~(0x0001 << i);
          }
        }  
        CellBalance_Selct = 0x0000;
        //===================================
        for(i =0; i <10; i++ )
        {
          if((CellBalance_Cur_Selct &(0x0001 <<i)))
          {
            CellBalance_Selct |=(0x0001 <<i);
            if(4 != i)
            {
              i +=1; 
            }
          }  
        } 
        Afe_CellBalance_Enable(CellBalance_Selct);
      } 
      else if(CellBal_Cntrl_Lock ==1)
      { 
        Afe_CellBalance_Disable();
      } 
    }
  }
  else if(CellBal_Cntrl_Lock ==1)
  {
      Afe_CellBalance_Disable();
  }
}
 
//==========================================================================
/*
1�����ģʽ�����쳣ʱ�������쳣��LED1��LED2��˸������Ϊ1�룩���¶��쳣��LED1��LED2��LED3��˸������Ϊ1�룩
             ���쳣ʱ���ͽڵ���LED�������߽ڵ�����LED��˸��T = 1s
                0%----50%     LED1��˸
                50%---70%     LED1������LED2��˸
                70%---99%     LED1��LED2������LED3��˸
                ����          LEDȫ��
             
2���ŵ�ģʽ�����쳣ʱ�������쳣��LED1��LED2��˸������Ϊ1�룩���¶��쳣��LED1��LED2��LED3��˸������Ϊ1�룩
             ���쳣ʱ����ʾ��ǰ����
                10%           LED1��˸ 0.5Hz
                10%---30%     LED1����
                30%---50%     LED1��LED2����
                50%           LEDȫ��
*/  
void LedShow_Cntrl(void)
{
  static uint8_t FlowLedCnt = 0;
  static uint8_t FlowLed_Finish_Flag = 0;
  // soc ��ʾ
  // �쳣�󣬱���5s��Ϩ��
  if(WorkMode == IDLE_MODE)// ==��ˮ��ʾ��ʽ
  {  
    FlowLedCnt = 0;
    FlowLed_Finish_Flag = 0; 
    if(LedFlash_Off_t < 100)
    {
      LED1_ON();
      LED2_ON();
      LED3_ON();  
    }  
    else if(LedFlash_Off_t < 200)
    {
      LED1_ON();
      LED2_ON();
      LED3_OFF();  
    }  
    else if(LedFlash_Off_t < 300)
    {
      LED1_ON();
      LED2_OFF();
      LED3_OFF();  
    }  
    else  
    {
      LED1_OFF();
      LED2_OFF();
      LED3_OFF();  
    }   
  }
  else
  { 
    LedFlash_Off_t = 0;
    if(FlowLed_Finish_Flag ==0)
    {
      if(FlowLedCnt ==0)
      {
        LED1_ON();
        LED2_OFF();
        LED3_OFF(); 
        if(LedFlash_t >= 50)
        {
          LedFlash_t = 0;
          FlowLedCnt = 1;
        }  
      }
      else if(FlowLedCnt ==1)
      {
        LED1_OFF();
        LED2_ON();
        LED3_OFF(); 
        if(LedFlash_t >= 50)
        {
          LedFlash_t = 0;
          FlowLedCnt = 2;
        }  
      }
      else if(FlowLedCnt ==2)
      {
        LED1_OFF();
        LED2_ON();
        LED3_OFF(); 
        if(LedFlash_t >= 50)
        {
          LedFlash_t = 0;
          FlowLedCnt = 3;
          FlowLed_Finish_Flag = 1;
        }  
      }
    }
    else if(Bits_flag.Bit.AfeErr)
    {
      if(FlowLedCnt ==0)
      {
        LED1_ON();
        LED2_OFF();
        LED3_OFF(); 
        if(LedFlash_t >= 50)
        {
          LedFlash_t = 0;
          FlowLedCnt = 1;
        }  
      }
      else if(FlowLedCnt ==1)
      {
        LED1_OFF();
        LED2_ON();
        LED3_OFF(); 
        if(LedFlash_t >= 50)
        {
          LedFlash_t = 0;
          FlowLedCnt = 2;
        }  
      }
      else if(FlowLedCnt ==2)
      {
        LED1_OFF();
        LED2_ON();
        LED3_OFF();
        if(LedFlash_t >= 50)
        {
          LedFlash_t = 0;
          FlowLedCnt = 0;
        }  
      }
    }
    else if(WorkMode == CHARGE_MODE)
    {
      /*
      if(Bits_flag.Bit.ChgCurOv)
      {
        LED1_OFF();
        if(LedFlash_t >= 50)
        {
          LED2_XOR();
          LedFlash_t = 0;
        } 
        LED3_OFF();  
      }
      else if(Bits_flag.Bit.ChgTemp)
      {
        LED1_OFF();
        LED2_OFF();  
        if(LedFlash_t >= 50)
        {
          LED3_XOR();
          LedFlash_t = 0;
        } 
      }
      else */
      if(Bits_flag.Bit.ChgOv || Cell_Volt_Max >= 4100) 
      {
        LED1_OFF();
        LED2_OFF(); 
        LED3_ON(); 
      } 
      else if(SocCalc.soc_rt >= 70)  // 65%---90%     LED1��LED2������LED3��˸  
      {
        LED1_OFF();
        if(LedFlash_t < 50)
        {
          LED2_ON();
          LED3_OFF(); 
        } 
        else if(LedFlash_t < 100)
        {
          LED2_OFF();
          LED3_ON();
        }
        else
        {
          LedFlash_t = 0;
        } 
      }
      else if(SocCalc.soc_rt >= 50 )//30%---65%     LED1������LED2��˸
      {
        if(LedFlash_t < 50)
        {
          LED1_ON();
          LED2_OFF(); 
        } 
        else if(LedFlash_t < 100)
        {
          LED1_OFF();
          LED2_ON();
        }
        else
        {
          LedFlash_t = 0;
        } 
        LED3_OFF();
      }
      else //0%----30%     LED1��˸
      { 
        if(LedFlash_t >= 50)
        {
          LedFlash_t = 0;
          LED1_XOR();
        } 
        LED2_OFF();
        LED3_OFF();
      } 
    } 
    else if(WorkMode == DISCHARGE_MODE)
    { 
      /*
      if(Bits_flag.Bit.DisCurOv  || Bits_flag.Bit.DisCurShort)
      {
        LED1_OFF();
        if(LedFlash_t >= 50)
        {
          LED2_XOR();
          LedFlash_t = 0;
        } 
        LED3_OFF();  
      }
      else if(Bits_flag.Bit.DisTemp)
      {
        LED1_OFF();
        LED2_OFF();  
        if(LedFlash_t >= 50)
        {
          LED3_XOR();
          LedFlash_t = 0;
        } 
      }
      else */
      if(Bits_flag.Bit.DisOv)
      {
        LED1_OFF();
        LED2_OFF(); 
        LED3_OFF(); 
      }
      else if(SocCalc.soc_rt >= 50)  //50%           LEDȫ��
      {
        LED1_OFF();  LED2_OFF(); LED3_ON();
        //LED3_OFF();  LED2_OFF(); LED1_ON();
      }
      else if(SocCalc.soc_rt >= 30) //30%---50%     LED1��LED2����
      {
        LED1_OFF();
        LED2_ON();
        LED3_OFF();
      }
      else if(SocCalc.soc_rt >= 10) //10%---30%     LED1����
      {
        LED1_ON();
        LED2_OFF();
        LED3_OFF();
      }
      else                             //10%           LED1��˸ 1Hz
      {
        if(LedFlash_t >= 100)
        {
          LedFlash_t = 0;
          LED1_XOR();
        }
        LED2_OFF();
        LED3_OFF();
      } 
    }
  }
}

void LedShow_WorkMode(void)
{
  if(WorkMode == IDLE_MODE)
  {
    //LED1_XOR();
    LED1_ON();
    LED2_OFF(); 
    LED3_OFF(); 
  }
  else if(WorkMode == CHARGE_MODE)
  {
    LED1_OFF();
    //LED2_XOR(); 
    LED2_ON(); 
    LED3_OFF(); 
  }
  else if(WorkMode == DISCHARGE_MODE)
  {
    LED1_OFF();
    LED2_OFF(); 
    LED3_ON(); 
    //LED3_XOR(); 
  }
}
//==========================================================================
void LowPower_Entry_MCU_Set(void)
{   
  static uint8_t LowPower_Entry_Exit_Cnt = 0;
  if(WorkMode == IDLE_MODE && (LowPower_Entry_Delay_t >= 500 || LowPower_MCU_Entry_Flag == 1))
  { 
    //if(LowPower_MCU_Entry_Flag == 0)
    {
       Afe_ADC_Disable(); Afe_Temp_Disable();  
    }
    CLK_PCKENR2 = CLK_PCKENR2_AWU;
    AWU_Init(AWU_TIMEBASE_1S); 
    CLK->ICKR |= CLK_ICKR_SWUAH;
    FLASH->CR1 |= 0x04;	  
    EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOD, EXTI_SENSITIVITY_FALL_ONLY);//Signal_In�½��ش���
    ClrWdt();  
    LowPower_MCU_Entry_Flag = 1; 
    halt(); 
    ClrWdt();  
    Delay_ms(5); 
    //======================================
    if(LowPower_MCU_Entry_Flag == 1)
    {
      if((LowPower_Entry_Exit_Cnt ++) >= 100) 
      {
        LowPower_MCU_Entry_Flag = 0; 
        LowPower_Entry_Exit_Cnt = 0; 
      }
    }
    else
    {
      LowPower_Entry_Exit_Cnt = 0; 
      CLK_PCKENR2 &= ~CLK_PCKENR2_AWU; 
      CLK->ICKR &= ~CLK_ICKR_SWUAH; 
      Afe_ADC_Enable(); Afe_Temp_Enable();Delay_ms(10); 
    } 
    //=============================================
    LowPower_Entry_Delay_t = 0; 
  }
} 
//========================================================================== 
void LowPower_Powerdown_Enter(void)
{
  uint8_t i = 0;
  if(AfeErr_t >= 2000 || (LedFlash_Off_t >= 450) || Bits_flag.Bit.DisOv)//PowerOff_Delay_t >= PowerOff_Delay_t_SET && 
  {
    SOC_SavedtoEEPROM();
    Delay_ms(10);
    while(1)
    {   
      Afe_EnterShipMode(); 
      if((i ++) < 100)
      { 
        ClrWdt();
        Delay_ms(50);
      }
      else
      {
        Delay_ms(1000);
      }
    } 
  }  
}
//==============================================================================
void LowPower_Cntrl(void)
{  
  if(WorkMode == IDLE_MODE || AfeErr_t >= 2000 || Bits_flag.Bit.DisOv)
  {
    if(!SYS_CTRL2.Bit.DSG_ON && !SYS_CTRL2.Bit.CHG_ON )  
    {
      LowPower_Powerdown_Enter();  
    } 
  }
  else
  {
    PowerOff_Delay_t = 0;
  }
}
//==============================================================================
void LowPower_Cntrl_1(void)
{ 
  if(LowPower_MCU_Entry_Flag ==0)//(0)//
  {
    if(!SYS_CTRL1.Bit.ADC_EN) 
    { 
      Afe_ADC_Enable(); 
    }
    if(!SYS_CTRL1.Bit.TEMP_SEL )
    {
      Afe_Temp_Enable();
    }
  }
  if(WorkMode == CHARGE_MODE)
  {
    return;
  }
  if(Bits_flag.Bit.DisOv)  
  {
    if(!SYS_CTRL2.Bit.DSG_ON && !SYS_CTRL2.Bit.CHG_ON )  
    {
      LowPower_Powerdown_Enter();  
    }
  }
  else
  {
    LowPower_Entry_MCU_Set();  
  }
}
//==============================================================================
void LowPower_Cntrl_Backup(void)
{ 
  if(LowPower_MCU_Entry_Flag ==0)//(0)//
  {
    if(!SYS_CTRL1.Bit.ADC_EN) 
    { 
      Afe_ADC_Enable(); 
    }
    if(!SYS_CTRL1.Bit.TEMP_SEL )
    {
      Afe_Temp_Enable();
    }
  }
  if(WorkMode == CHARGE_MODE)
  {
    return;
  }
  if(Bits_flag.Bit.DisOv)  
  {
    if(!SYS_CTRL2.Bit.DSG_ON && !SYS_CTRL2.Bit.CHG_ON )  
    {
      LowPower_Powerdown_Enter();  
    }
  }
  else
  {
    LowPower_Entry_MCU_Set();  
  }
}

uint8_t CRC8_Caculate(uint8_t *ptr,uint8_t len)
{
  uint8_t i;
	
  uint8_t crc = 0;

  uint8_t key = 0x07; //_Poly_07
  while((len--) != 0)
	
  {
		
    for(i = 0x80; i != 0; i /= 2)
		
    {
			
      if((crc & 0x80) != 0)
			
      {
				
        crc *= 2;
				
        crc ^= key;
			
      }
			
      else
				
      {
        crc *= 2;

      }
			
      if((*ptr & i) != 0)
				
      {
        crc ^= key;
      }
		
    }
		
    ptr++;
	
  }
  return (crc);

} 


//==========================================================================
/*
In a single-byte read transaction, the CRC is calculated after the second start and uses the slave address and data byte.
*/
void I2C_Read(uint8_t addr,uint8_t *data)//slave device address 0x18
{      
  uint16_t OverTimeDelay = OverTimeDelay_SET;
  uint8_t CRC_ReData,Retry_Cnt = 5; 
  Buf[0] = SLAVE_ADDR; 
  while(Retry_Cnt > 0)
  {
    //���¼�stm8s���������ֲ�P251��ͼ96���豸����ģʽ��������ͼ��  
    disableInterrupts(); 
    I2C->CR2 |= 0x04; //ackʹ�� 
    //=============================������ʼλ I2C_Start() 
    while(I2C->SR3 & 0x02){if((OverTimeDelay --) == 0){OverTimeDelay = OverTimeDelay_SET;I2C_COM_ERROR_Flag = 1;break;}}    // �ȴ����߿���   ���i2c-SR3 busyλ        
    I2C->CR2 |= 0x01;        // ������ʼλ     CR2 startλ  
    
    //=============================����������ַ I2C_SendDAdr(0xD0)
    //EV5��SB=1����SR1 Ȼ�󽫵�ַд��DR�Ĵ���������ñ�־��  
    while(!(I2C->SR1 & 0x01));  //�ȴ�START������ E5  
    I2C->DR = (SLAVE_ADDR << 1) + 0x00;  //���� ������ַ(���һλ��0,��ʾ����) 
    
    while(!(I2C->SR1 & 0x02 == 0x02)){if((OverTimeDelay --) == 0){OverTimeDelay = OverTimeDelay_SET;break;}}// I2C_COM_ERROR_Flag = 1; //����7λ������ַ�����겢���յ�ack,ADDR��1    
    //while(!(I2C->SR1 & 0x02));  //����7λ������ַ�����겢���յ�ack,ADDR��1    
    //EV6:ADDR �������ȡSR1�󣬶�SR3�Ĵ��������� �������λ 
    I2C->SR1; //��P251 ��SR1 (ʵ��֤�����Բ�Ҫ)  
    I2C->SR3; //Ȼ���SR3 ��  ADDR 
    
    if(I2C_COM_ERROR_Flag == 0)
    {
      //=============================DATA ���ͼĴ�����ַ I2C_SendDat()  
      I2C->DR = (u8)(addr);      
      //EV8_2 TxE=1 ��BTF=1������ֹͣ����ʱ��Ӳ�������   
      while(!(I2C->SR1 & 0x84));  //���SR1: TXE1(���ݼĴ���Ϊ��"1") BTF(�ֽڷ��ͽ���"1")λ��(ֻ�е�stm8�յ�ack,TxE�Ż���1����ʵ����൱���ж��յ�ackû�У�)
      //�ڷ��͵�ַ�����ADDR ֮��I2C�ӿڽ������豸����ģʽ�����¼�stm8s���������ֲ�P252��ͼ97���豸����ģʽ��������ͼ��  
      
      //=============================I2C_Start()        
      I2C->CR2 |= 0x01;  //�����ظ���ʼλ   
      //EV5��SB=1����SR1 Ȼ�󽫵�ַд��DR�Ĵ���������ñ�־��  
      while(!(I2C->SR1 & 0x01));  //�ȴ�START������     
      //ADDRESS (����)   
      I2C->DR = (SLAVE_ADDR <<1) + 0x01;  //����MLX90615������ַ(���һλ��1,��ʾ����)����������Զ�����ack(��ǰ��CR2 ackλʹ��)      
      //EV6:ADDR �������ȡSR1�󣬶�SR3�Ĵ��������� �������λ   
      
      while(!(I2C->SR1 & 0x02 == 0x02)){if((OverTimeDelay --) == 0){OverTimeDelay = OverTimeDelay_SET;break;}} // I2C_COM_ERROR_Flag = 1;//����7λ������ַ�����겢���յ�ack,ADDR��1    
      //while(!(I2C->SR1 & 0x02));  //����7λ������ַ�����겢���յ�ack,ADDR��1  
      I2C->SR1; //��P251 ��SR1 (ʵ��֤�����Բ�Ҫ)  
      I2C->SR3; //Ȼ���SR3 ��  ADDR   
         Buf[0] = 0x31;  Buf[1] = 0;     Buf[2] = 0;           
      //����EV7 RxNE=1���յ�һ���ֽں�RxNE��1�� ���ж�DR�Ĵ���������    
      while(!(I2C->SR1 & 0x40));                                        
      Buf[1] = I2C->DR;//�ڽ���ģʽ�£��յ������ֽں�,�Զ�����ack(��ǰ��CR2 ackλʹ�ܣ�����Ҫר��CR2 ackλ��1)                                               
       
     
      
      //����EV7 RxNE=1���յ�һ���ֽں�RxNE��1�� ���ж�DR�Ĵ���������    
      while(!(I2C->SR1 & 0x40));                                        
      Buf[2] = I2C->DR;//�ڽ���ģʽ�£��յ������ֽں�,�Զ�����ack(��ǰ��CR2 ackλʹ�ܣ�����Ҫר��CR2 ackλ��1)  
        //EV7_1 ��RxNE=1 ����DR�Ĵ�������ñ�־������ACK=0��STOP ����(�ڽ������һ���ֽ�ǰ)     
      I2C->SR3;
      I2C->DR; 
      I2C->CR2 &= ~0x04; //ackʹ��  
      I2C->CR2 |= 0x02;  //ֹͣλ����stop      
       
      enableInterrupts(); 
      //while((I2C->CR2 & I2C_CR2_STOP));
      CRC_ReData = CRC8_Caculate(Buf,2);
      /*Uart_SendByte(0x11);
      Uart_SendByte(Buf[0]);
      Uart_SendByte(Buf[1]);
      Uart_SendByte(Buf[2]);*/
      if(CRC_ReData == Buf[2])
      {
        *data = Buf[1];
        OverTimeDelay = 200; while((OverTimeDelay --) >0);
        return;
      }
    }
    I2C_COM_ERROR_Flag = 0;
    Retry_Cnt -= 1;
  }
  enableInterrupts(); 
  //Uart_SendByte(0xDD);
  //Uart_SendByte(0xDD);
}

//==========================================================================
/*
In a single-byte write transaction, the CRC is calculated over the slave address, register address, and data.
*/
void I2C_Write(uint8_t addr,uint8_t data )
{     
  uint8_t Write_Retry_Cnt = 5,CRC_data = 0;
  uint16_t OverTimeDelay = OverTimeDelay_SET;
  Buf[0] = SLAVE_ADDR <<1 ;
  Buf[1] = addr;
  Buf[2] = data;
  CRC_data = CRC8_Caculate(Buf,3);//slave_addr + reg_addr + data + CRC
  I2C_COM_ERROR_Flag =0;
  
  while(Write_Retry_Cnt > 0)
  { 
    disableInterrupts();
    I2C_CR2 |= 0x04;
    //���¼�stm8s���������ֲ�P251��ͼ96���豸����ģʽ��������ͼ�� 
    //=============================������ʼλ I2C_Start() 
    while(I2C->SR3 & 0x02) {if((OverTimeDelay --) == 0){OverTimeDelay = 500;I2C_COM_ERROR_Flag = 1;break;}}//   // �ȴ����߿���   ���i2c-SR3 busyλ
    //I2C_CR2 &= ~0x04;        
    I2C->CR2 |= 0x01;        // ������ʼλ     CR2 startλ    
    //=============================����������ַ I2C_SendDAdr(0xD0)
    //EV5��SB=1����SR1 Ȼ�󽫵�ַд��DR�Ĵ���������ñ�־��  
    while(!(I2C->SR1 & 0x01));// {if((OverTimeDelay --) == 0){OverTimeDelay = 50;I2C_COM_ERROR_Flag = 1;break;}} //�ȴ�START������ E5  
    
    I2C->DR = (SLAVE_ADDR <<1) + 0x00;  //���� ������ַ(���һλ��0,��ʾ����) 
    
    OverTimeDelay = 100; while((OverTimeDelay --) >0);OverTimeDelay = OverTimeDelay_SET;
    //while(!(I2C->SR1 & 0x02 == 0x02)) {if((OverTimeDelay --) == 0){OverTimeDelay = OverTimeDelay_SET;I2C_COM_ERROR_Flag = 0;break;}} // //����7λ������ַ�����겢���յ�ack,ADDR��1    
    //EV6:ADDR �������ȡSR1�󣬶�SR3�Ĵ��������� �������λ 
    I2C->SR1;   
    I2C->SR3; //Ȼ���SR3 ��  ADDR 
    if(I2C_COM_ERROR_Flag == 0)
    {
      //I2C_CR2 |= 0x04;
      //=============================DATA ���ͼĴ�����ַ I2C_SendDat()  
      I2C->DR = (u8)(addr);      
      //EV8_2 TxE=1 ��BTF=1������ֹͣ����ʱ��Ӳ�������   
      while(!(I2C->SR1 & 0x84));//84//0x80  //���SR1: TXE1(���ݼĴ���Ϊ��"1") BTF(�ֽڷ��ͽ���"1")λ��(ֻ�е�stm8�յ�ack,TxE�Ż���1����ʵ����൱���ж��յ�ackû�У�)
       
      //============================= ��������  
      I2C->DR = data;  //����MLX90615������ַ(���һλ��1,��ʾ����)����������Զ�����ack(��ǰ��CR2 ackλʹ��)    
      while(!(I2C->SR1 & 0x84));  //���SR1: TXE1(���ݼĴ���Ϊ��"1") BTF(�ֽڷ��ͽ���"1")λ��(ֻ�е�stm8�յ�ack,TxE�Ż���1����ʵ����൱���ж��յ�ackû�У�)
      //EV8_2��TxE=1��BTF=1������ֹͣ����ʱ��Ӳ�������  
      
      //============================= ��������  
      I2C->DR = CRC_data;    
      while(!(I2C->SR1 & 0x84));  //���SR1: TXE1(���ݼĴ���Ϊ��"1") BTF(�ֽڷ��ͽ���"1")λ��(ֻ�е�stm8�յ�ack,TxE�Ż���1����ʵ����൱���ж��յ�ackû�У�)
      //EV8_2��TxE=1��BTF=1������ֹͣ����ʱ��Ӳ�������  
      
      I2C->DR;  //���I2C_SR2��BTFλ
      I2C->CR2 |= 0x02;  //ֹͣλ����stop  //*ע�⣺ ����ֹͣλʱ�����������I2C_SR2��BTFλ  
      
      enableInterrupts();
      OverTimeDelay = 200; while((OverTimeDelay --) >0);
      if(I2C_COM_ERROR_Flag == 0)
      {  
        return;
      }
    }
    I2C_COM_ERROR_Flag = 0;
    Write_Retry_Cnt -= 1; 
  } 
  enableInterrupts();
  //Uart_SendByte(0xCC);
  //Uart_SendByte(0xCC);
}
 
void I2C_Read_Backup(uint8_t addr,uint8_t *data)//slave device address 0x18
{      
  uint16_t OverTimeDelay = 50;
  uint8_t CRC_ReData,Retry_Cnt = 5; 
  Buf[0] = SLAVE_ADDR; 
  while(Retry_Cnt > 0)
  {
    //���¼�stm8s���������ֲ�P251��ͼ96���豸����ģʽ��������ͼ�� 
    
    I2C->CR2 |= 0x04; //ackʹ�� 
    //=============================������ʼλ I2C_Start() 
    while(I2C->SR3 & 0x02);//{if((OverTimeDelay --) == 0){OverTimeDelay = 50;break;}}    // �ȴ����߿���   ���i2c-SR3 busyλ        
    I2C->CR2 |= 0x01;        // ������ʼλ     CR2 startλ  
    
    //=============================����������ַ I2C_SendDAdr(0xD0)
    //EV5��SB=1����SR1 Ȼ�󽫵�ַд��DR�Ĵ���������ñ�־��  
    while(!(I2C->SR1 & 0x01));  //�ȴ�START������ E5  
    I2C->DR = (SLAVE_ADDR << 1) + 0x00;  //���� ������ַ(���һλ��0,��ʾ����) 
    
    while(!(I2C->SR1 & 0x02 == 0x02)){if((OverTimeDelay --) == 0){OverTimeDelay = 50;break;}}  //����7λ������ַ�����겢���յ�ack,ADDR��1    
    //while(!(I2C->SR1 & 0x02));  //����7λ������ַ�����겢���յ�ack,ADDR��1    
    //EV6:ADDR �������ȡSR1�󣬶�SR3�Ĵ��������� �������λ 
    I2C->SR1; //��P251 ��SR1 (ʵ��֤�����Բ�Ҫ)  
    I2C->SR3; //Ȼ���SR3 ��  ADDR 
    
    //=============================DATA ���ͼĴ�����ַ I2C_SendDat()  
    I2C->DR = (u8)(addr);      
    //EV8_2 TxE=1 ��BTF=1������ֹͣ����ʱ��Ӳ�������   
    while(!(I2C->SR1 & 0x84));  //���SR1: TXE1(���ݼĴ���Ϊ��"1") BTF(�ֽڷ��ͽ���"1")λ��(ֻ�е�stm8�յ�ack,TxE�Ż���1����ʵ����൱���ж��յ�ackû�У�)
    //�ڷ��͵�ַ�����ADDR ֮��I2C�ӿڽ������豸����ģʽ�����¼�stm8s���������ֲ�P252��ͼ97���豸����ģʽ��������ͼ��  
    
    //=============================I2C_Start()        
    I2C->CR2 |= 0x01;  //�����ظ���ʼλ   
    //EV5��SB=1����SR1 Ȼ�󽫵�ַд��DR�Ĵ���������ñ�־��  
    while(!(I2C->SR1 & 0x01));  //�ȴ�START������     
    //ADDRESS (����)   
    I2C->DR = (SLAVE_ADDR <<1) + 0x01;  //����MLX90615������ַ(���һλ��1,��ʾ����)����������Զ�����ack(��ǰ��CR2 ackλʹ��)      
    //EV6:ADDR �������ȡSR1�󣬶�SR3�Ĵ��������� �������λ   
    
  while(!(I2C->SR1 & 0x02 == 0x02)){if((OverTimeDelay --) == 0){OverTimeDelay = 50;break;}}  //����7λ������ַ�����겢���յ�ack,ADDR��1    
  //while(!(I2C->SR1 & 0x02));  //����7λ������ַ�����겢���յ�ack,ADDR��1  
    I2C->SR1; //��P251 ��SR1 (ʵ��֤�����Բ�Ҫ)  
    I2C->SR3; //Ȼ���SR3 ��  ADDR   
       Buf[0] = 0x31;  Buf[1] = 0;     Buf[2] = 0;           
    //����EV7 RxNE=1���յ�һ���ֽں�RxNE��1�� ���ж�DR�Ĵ���������    
    while(!(I2C->SR1 & 0x40));                                        
    Buf[1] = I2C->DR;//�ڽ���ģʽ�£��յ������ֽں�,�Զ�����ack(��ǰ��CR2 ackλʹ�ܣ�����Ҫר��CR2 ackλ��1)                                               
     
   
    
    //����EV7 RxNE=1���յ�һ���ֽں�RxNE��1�� ���ж�DR�Ĵ���������    
    while(!(I2C->SR1 & 0x40));                                        
    Buf[2] = I2C->DR;//�ڽ���ģʽ�£��յ������ֽں�,�Զ�����ack(��ǰ��CR2 ackλʹ�ܣ�����Ҫר��CR2 ackλ��1)  
      //EV7_1 ��RxNE=1 ����DR�Ĵ�������ñ�־������ACK=0��STOP ����(�ڽ������һ���ֽ�ǰ)     
    I2C->SR3;
    I2C->DR; 
    I2C->CR2 &= ~0x04; //ackʹ��  
    I2C->CR2 |= 0x02;  //ֹͣλ����stop      
     
    //while((I2C->CR2 & I2C_CR2_STOP));
    CRC_ReData = CRC8_Caculate(Buf,2);
    /*Uart_SendByte(0x11);
    Uart_SendByte(Buf[0]);
    Uart_SendByte(Buf[1]);
    Uart_SendByte(Buf[2]);*/
    if(CRC_ReData == Buf[2])
    {
      *data = Buf[1];
      OverTimeDelay =100; while((OverTimeDelay --) >0);
      return;
    }
    Retry_Cnt -= 1;
  } 
  //Uart_SendByte(0xAA);
}

void I2C_Write_Backup(uint8_t addr,uint8_t data )
{     
  uint8_t Write_Retry_Cnt = 5,CRC_data = 0;
  uint16_t OverTimeDelay = 50;
  Buf[0] = SLAVE_ADDR <<1 ;
  Buf[1] = addr;
  Buf[2] = data;
  CRC_data = CRC8_Caculate(Buf,3);//slave_addr + reg_addr + data + CRC
  
  while(Write_Retry_Cnt > 0)
  {
    I2C_CR2 |= 0x04;
    //���¼�stm8s���������ֲ�P251��ͼ96���豸����ģʽ��������ͼ�� 
    //=============================������ʼλ I2C_Start() 
    while(I2C->SR3 & 0x02);// {if((OverTimeDelay --) == 0){OverTimeDelay = 50;I2C_COM_ERROR_Flag = 1;break;}}   // �ȴ����߿���   ���i2c-SR3 busyλ
    //I2C_CR2 &= ~0x04;        
    I2C->CR2 |= 0x01;        // ������ʼλ     CR2 startλ    
    //=============================����������ַ I2C_SendDAdr(0xD0)
    //EV5��SB=1����SR1 Ȼ�󽫵�ַд��DR�Ĵ���������ñ�־��  
    while(!(I2C->SR1 & 0x01));// {if((OverTimeDelay --) == 0){OverTimeDelay = 50;I2C_COM_ERROR_Flag = 1;break;}} //�ȴ�START������ E5  
    
    I2C->DR = (SLAVE_ADDR <<1) + 0x00;  //���� ������ַ(���һλ��0,��ʾ����) 
    while(!(I2C->SR1 & 0x02 == 0x02)) {if((OverTimeDelay --) == 0){OverTimeDelay = 50;I2C_COM_ERROR_Flag = 1;break;}}  //����7λ������ַ�����겢���յ�ack,ADDR��1    
    //EV6:ADDR �������ȡSR1�󣬶�SR3�Ĵ��������� �������λ 
    I2C->SR1;   
    I2C->SR3; //Ȼ���SR3 ��  ADDR 
    
    //I2C_CR2 |= 0x04;
    //=============================DATA ���ͼĴ�����ַ I2C_SendDat()  
    I2C->DR = (u8)(addr);      
    //EV8_2 TxE=1 ��BTF=1������ֹͣ����ʱ��Ӳ�������   
    while(!(I2C->SR1 & 0x84));//84//0x80  //���SR1: TXE1(���ݼĴ���Ϊ��"1") BTF(�ֽڷ��ͽ���"1")λ��(ֻ�е�stm8�յ�ack,TxE�Ż���1����ʵ����൱���ж��յ�ackû�У�)
     
    //============================= ��������  
    I2C->DR = data;  //����MLX90615������ַ(���һλ��1,��ʾ����)����������Զ�����ack(��ǰ��CR2 ackλʹ��)    
    while(!(I2C->SR1 & 0x84));  //���SR1: TXE1(���ݼĴ���Ϊ��"1") BTF(�ֽڷ��ͽ���"1")λ��(ֻ�е�stm8�յ�ack,TxE�Ż���1����ʵ����൱���ж��յ�ackû�У�)
    //EV8_2��TxE=1��BTF=1������ֹͣ����ʱ��Ӳ�������  
    
    //============================= ��������  
    I2C->DR = CRC_data;    
    while(!(I2C->SR1 & 0x84));  //���SR1: TXE1(���ݼĴ���Ϊ��"1") BTF(�ֽڷ��ͽ���"1")λ��(ֻ�е�stm8�յ�ack,TxE�Ż���1����ʵ����൱���ж��յ�ackû�У�)
    //EV8_2��TxE=1��BTF=1������ֹͣ����ʱ��Ӳ�������  
    
    I2C->DR;  //���I2C_SR2��BTFλ
    I2C->CR2 |= 0x02;  //ֹͣλ����stop  //*ע�⣺ ����ֹͣλʱ�����������I2C_SR2��BTFλ  
    
    OverTimeDelay =100; while((OverTimeDelay --) >0);
    if(I2C_COM_ERROR_Flag == 1)
    {
      return;
    }
    I2C_COM_ERROR_Flag = 0;
    Write_Retry_Cnt -= 1;
  }  
}
//===========================================================
void SOC_Init(void)
{
  uint8_t result = 0;
  uint8_t Soc_Tmp = 0;
  result = FLASH_ReadByte(ADJUST_ADDR);
  
  if(result == 0xAA)//�ж�У���ַ�������Ƿ�Ϊ0xAA, Y: EEPROM �б�����SOC����;  N: EEPROM �б�����SOC����
  {
    FLASH_Unlock(FLASH_MEMTYPE_DATA);       // ����EEPROM
    FLASH_ProgramByte(ADJUST_ADDR,0x00);    // ���У���ַ������ 
    FLASH_Lock(FLASH_MEMTYPE_DATA);         // ����EEPROM
    Soc_Tmp = FLASH_ReadByte(SOC_ADDR);     // ��ȡSOC����
     
    SocReg.soc = Soc_Tmp; //SocReg.ah = SocCalc.curAh; // ����SOC�� 
    SocCalc.curAh = ((uint32_t)SocReg.rated_cap * Soc_Tmp) / 100;
    SocReg.ah = SocCalc.curAh;
    SocCalc.soc_rt = SocReg.soc;
    Soc_OCV_CorrectEn_Flag = 0;  // �ϵ�����SOC��OCVУ׼  
  }
  else
  {
    Soc_OCV_CorrectEn_Flag = 1;  // �ϵ�����SOC��OCVУ׼
  } 
}
void SOC_SavedtoEEPROM(void)
{   
  FLASH_Unlock(FLASH_MEMTYPE_DATA);               // ����EEPROM
  FLASH_ProgramByte(SOC_ADDR,SocCalc.soc_rt);     // ����SOC��EEPROM 
  FLASH_Lock(FLASH_MEMTYPE_DATA);                 // ����EEPROM
  if(SocCalc.soc_rt == FLASH_ReadByte(SOC_ADDR))  // ��ȡ���� ?= д������
  {
    FLASH_Unlock(FLASH_MEMTYPE_DATA);             // ����EEPROM
    FLASH_ProgramByte(ADJUST_ADDR,0xAA);          // д�뱣��SOC�ɹ���־��
    FLASH_Lock(FLASH_MEMTYPE_DATA);               // ����EEPROM
  } 
}
//======================================
void Var_Init(void)
{ 
  uint8_t i = 0;
   
  LowPower_MCU_Entry_Flag = 0; // MCU�����ڵ͹���״̬��ʶ��
  LedFlash_Off_t = 0;
  ChgExchangeMode_Cnt  = 100;
  DisExchangeMode_Cnt  = 0;
  IdleExchangeMode_Cnt = 0;
  for(i =0; i <10; i++)
  {
    Adc_value[i] = 0;
    Cell_Volt[i] = 0;
  }
  Temp_Volt_Sample_Cnt = 0;
  Cell_Volt_Sample_Cnt = 0;
  I2C_COM_ERROR_Flag   = 0;
  Current_Val   = 0;
  ADCOffset_Val = 0;
  CC_Val = 0;
  Cell_Volt_Tol = 0;
  Cell_Volt_Avg = 0;
  Cell_Volt_Max = 0;
  Cell_Volt_Min = 0; 
  ADCGain_Val = 0; 
  Pack_Volt = 0;
  Temp_Val = 0;
  V_TS2_Val = 0; 
  R_TS2_Val = 0; 
  
  SYS_STAT_Last = 0;
  CELLBAL1_Last = 0;
  CELLBAL2_Last = 0;
  SYS_CTRL1_Last = 0;
  SYS_CTRL2_Last = 0; 
  PROTECT1_Last = 0;
  PROTECT2_Last = 0;
  PROTECT3_Last = 0;
  OV_TRIP_Last = 0;
  UV_TRIP_Last = 0;
  CC_CFG_Last = 0;
  
  
  ChgOv_t = 0;
  ChgCurOv_t = 0;
  ChgCurOv_Re_t = 0;  
  DisOv_t = 0;
  DisCurOv_t = 0;
  DisCurOv_t1 = 0;
  DisCurOv_t2 = 0;
  DisCurOv_Re_t = 0;
  DisCurShort_Re_t = 0;
  
  DEVICE_XREADY_Re_t = 0;
  
  CellBalance_Cur_Selct = 0;  
  LedFlash_t = 0;
  PowerOff_Delay_t = 0;
  Delay_time_t = 0;
  
  WorkMode = IDLE_MODE;
  
  SYS_STAT.Byte = 0;
  
  RevcComData.uintdata = 0;
  Bits_flag.Byte = 0;
  
  SocCalc.inAh = 0;
  SocCalc.inAh_bak = 0;
  SocCalc.totalInAh = 0;
  SocCalc.totalInAh_bak = 0;
  SocCalc.outAh = 0;
  SocCalc.outAh_bak = 0;
  SocCalc.totalOutAh = 0;
  SocCalc.totalOutAh_bak = 0;
  SocCalc.curAh = 0;
  SocCalc.ov_cnt = 0;
  SocCalc.uv_cnt = 0;
  SocCalc.stb_cnt = 0;
  SocCalc.soc_rt = 0; 
  
}

#endif