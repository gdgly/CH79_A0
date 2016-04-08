/*
 *****************************************************************************
 *					PACE ELECTRONICS CO.,LTD
 *	Copyright 2012, PACE ELECTRONICS CO.,LTD  Shenzhen, China
 *					All rights reserved.
 *
 * Filename:			bat.h
 * Description:			bat head file
 *
 * Change History:
 *			Goldjun	- 09/13'2012 - Ver 0.1
 *					- created
 *			xxxxx	- xx/xx/20xx - Verx.x
 *					- change code
 ******************************************************************************
 */ 

//#include "macro_def.h"
 //#include "bq769x0.h"

#ifndef	__USER_H__
#define __USER_H__
  

  //================================================================== 
  uint16_t ADC(int channel);
  uint16_t ADConverse( unsigned char channel);
  uint16_t Afe_Get_Adc(uint8_t addr);
  void Var_Init(void);
  void PWM2_Init(void);
  void PWM1_Init(void);
  void SysInit(void);
  void Uart_Model_Init(void);
  void Uart_SendByte(uint8_t DataByte);
  void Uart_SendStr(uint8_t *DataStr);
  
  void itoa(char *buf, int i, int base); 
  void Uart_SendData(unsigned int tx_data, int base) ;
  void I2C_Model_Init(void);
  void ClrWdt(void);
  void PortInit(void);
  void VarInit(void); 
  void Timer2Init(void);
  void Timer4_Init_us(void);
  void Timer4_Init_ms(void);
  void Timer4_Disable(void);
  void Delay_us(uint16_t Delay_time);
  void Delay_ms(uint16_t Delay_time);
  void ModeCheck(void);
  void ClearStatus(void);
  void ModeCheck_Backup(void);
  void CurrentCheck(void);
  void VoltCheck(void);
  void TempCheck(void);
  void CellBal_Cntrl(void);
  void LedShow_Cntrl(void);
  void LedShow_WorkMode(void);
  void LowPower_Entry_MCU_Set(void);
  void LowPower_Exit_MCU_Set(void);
void LowPower_Entry_MCU_Set_Backup(void);
  void LowPower_Cntrl(void);
void LowPower_Powerdown_Enter(void);
  
  void Afe_CellBalance_Enable(uint16_t selct);
  void Afe_CellBalance_Disable(void);
  void Afe_FET_ChgDis_Cntrl(void);
  void Afe_AbnormalCheck(void); 
  void Afe_OV_UV_Delay_Set(uint8_t OV_delay, uint8_t UV_delay); 
  void Afe_OV_UV_Threshold_Set(uint16_t OV_val, uint16_t UV_val);
  void Afe_SCD_Set(uint16_t SCD_val, uint16_t SCD_delay);
  void Afe_OCD_Set(uint16_t OCD_val, uint16_t OCD_delay);
  void Afe_EnterShipMode(void);
  void Afe_Device_Init(void);
  void Afe_ADC_Enable(void);
  void Afe_ADC_Disable(void);
  void Afe_Temp_Disable(void);
  void Afe_Temp_Enable(void);
  void Afe_Get_GainOffset(void);
  void Afe_CC_Disable(void);
  void Afe_CC_1Shot_Set(void); 
  void Afe_CC_AlwaysOn_Set(void);
  void Afe_Volt_Val_Get(void);
  
  void Afe_Get_SysStatus(void);
  void Afe_FET_ChgOn_DisOn(void);
  void Afe_FET_ChgOn_DisOff(void);
  void Afe_FET_ChgOff_DisOn(void);
  void Afe_FET_ChgOff_DisOff(void); 
  
  uint8_t CRC8_Caculate(uint8_t *ptr,uint8_t len);
  void I2C_Read(uint8_t addr,uint8_t *data);
  void I2C_Write(uint8_t addr,uint8_t data );


  void SOCAhIntergrate(void);
  void SOCCalculate(void);
  void SOCCorrectOCV(void);
  void SOCCorrectTemp(void);
  void SOCSmooth(void);

  void SOC_Init(void); 
  void SOC_SavedtoEEPROM(void);
  void ChgDis_AbnormalCheck(void);



  extern unsigned int ChgTemp_cnt;
  extern unsigned int DisTemp_cnt; 
  extern unsigned int DisTemp_Lock_Cnt;
  extern unsigned int Temp_Protect_Delay_t;
  extern unsigned char Dis_First_Run_Flag;
  extern unsigned char Dis_First_Run_t;
  extern unsigned char ChgExchangeMode_Cnt;
  extern unsigned char DisExchangeMode_Cnt;
  extern unsigned char IdleExchangeMode_Cnt;
  extern unsigned char I2C_COM_ERROR_Flag;
  extern signed char  ADCOffset_Val;
  extern unsigned long Current_Val;
  extern signed long  CC_Val;
  extern signed int CC_AD; 
  //extern unsigned int CC_Val;
  //extern unsigned int Current_Val;
  
  extern unsigned int Check_Val;
  extern unsigned char Buf[3];
  extern unsigned int Adc_value[10];
  extern unsigned int Cell_Volt[10];
  extern unsigned int Cell_Volt_Tol;
  extern unsigned int Cell_Volt_Avg;
  extern unsigned int Cell_Volt_Max;
  extern unsigned int Cell_Volt_Min; 
  extern unsigned int ADCGain_Val; 
  extern unsigned int Pack_Volt;
  extern unsigned int Temp_Val;
  extern unsigned int V_TS2_Val; 
  extern unsigned int R_TS2_Val; 
  
  extern unsigned char     SYS_STAT_Last;
  extern unsigned char     CELLBAL1_Last;
  extern unsigned char     CELLBAL2_Last;
  extern unsigned char     SYS_CTRL1_Last;
  extern unsigned char     SYS_CTRL2_Last; 
  extern unsigned char     PROTECT1_Last;
  extern unsigned char     PROTECT2_Last;
  extern unsigned char     PROTECT3_Last;
  extern unsigned char     PROTECT1_Last_Copy;
  extern unsigned char     PROTECT2_Last_Copy;
  extern unsigned char     PROTECT3_Last_Copy;
  extern unsigned char     OV_TRIP_Last; 
  extern unsigned char     UV_TRIP_Last;
  extern unsigned char     CC_CFG_Last;
  
  extern unsigned int Temp_Volt_Sample_Cnt;
  extern unsigned int Cell_Volt_Sample_Cnt;
  extern unsigned int CC_Volt_Sample_Cnt;
  extern unsigned int ChgOv_t;
  extern unsigned int ChgCurOv_t;
  extern unsigned int ChgCurOv_Re_t;   
  extern unsigned int DisOv_t;  
  extern unsigned int DisCurOv_t; 
  extern unsigned int DisCurOv_t1; 
  extern unsigned int DisCurOv_t2; 
  extern unsigned int DisCurOv_Re_t; 
  extern unsigned int DisCurShort_Re_t; 

  //extern unsigned int DEVICE_XREADY_Re_t; 
  extern unsigned int AfeErr_t; 

  extern unsigned int CellBalance_Cur_Selct; 
  extern unsigned int LedFlash_t; 
  extern unsigned int LedFlash_Off_t; 
  extern unsigned int PowerOff_Delay_t;
  extern unsigned int Delay_time_t;
  
  extern unsigned char Soc_OCV_CorrectEn_Flag ;
  extern unsigned char CellBal_Cntrl_Lock ;
  extern unsigned int CellBalance_Selct ;
  
  extern unsigned char Init_Soc_Flag;
  extern unsigned char LowPower_MCU_Entry_Flag ;
  extern unsigned int LowPower_Entry_Delay_t ;
  extern unsigned int Cell_Balance_Delay_t;
  //============================================
  extern enum  em_workmode
  {
    IDLE_MODE 	    = 0x00,
    CHARGE_MODE     = 0x01,
    DISCHARGE_MODE  = 0x02
  }WorkMode;
  extern union STATUS_FLAG
  {
    unsigned char Byte;
    struct FLAG_BITS
    { 
      unsigned char ChgOv           : 1;
      unsigned char ChgCurOv        : 1;
      unsigned char ChgTemp         : 1;  
      unsigned char DisOv           : 1;
      unsigned char DisTemp         : 1;  
      unsigned char DisCurOv        : 1; 
      unsigned char DisCurShort     : 1;   
      //unsigned char OpenDetect      : 1; 
      unsigned char AfeErr          : 1; 
    }Bit;
  } Bits_flag;
  extern union  SYS_STATUS_ST//#define   SYS_STAT  0x00      __IO
  {
    uint8_t Byte;  
    struct
    {
      uint8_t OCD           :1; 
      uint8_t SCD    	    :1;  
      uint8_t OV    	    :1; 
      uint8_t UV    	    :1;		 
      uint8_t OVRD_ALERT    :1;	 
      uint8_t DEVICE_XREADY :1;		 
      uint8_t RSVD 	    :1;	
      uint8_t CC_READY 	    :1;	
    }Bit;
  }SYS_STAT;  
  
  
  extern union  SYS_CTRL1_ST 
  {
    uint8_t Byte;  
    struct
    {
      uint8_t SHUT_B 	    :1;		 
      uint8_t SHUT_A 	    :1;		 
      uint8_t RSVD          :1;		 
      uint8_t TEMP_SEL	    :1;
      uint8_t ADC_EN        :1;	 
      uint8_t RSVD1    	    :1; 
      uint8_t RSVD2    	    :1; 
      uint8_t LOAD_PRESENT  :1; 
    }Bit;
  }SYS_CTRL1; 
  
  extern union  SYS_CTRL2_ST      
  {
    uint8_t Byte;  
    struct
    {
      uint8_t CHG_ON 	    :1;		 
      uint8_t DSG_ON 	    :1;		 
      uint8_t RSVD          :1;		 
      uint8_t RSVD1	    :1;
      uint8_t RSVD2    	    :1;	 
      uint8_t CC_ONESHOT    :1; 
      uint8_t CC_EN    	    :1; 
      uint8_t DELAY_DIS     :1; 
    }Bit;
  }SYS_CTRL2; 
        
  extern union UINT_UCHAR
  {
    unsigned int uintdata;
    unsigned char uchardata[2];
  }RevcComData;
  

extern struct X_SOC
{
    uint8_t 	soc;                	        // soc
    uint32_t 	ah;               	 	// 剩余容量
         //uint16_t 	vlt_pack;          		// 实时总电压mV
    uint16_t 	curr;              		// 实时电流mA
    //uint8_t 	curr_dir;          		// 电流方向0为放电1为充电。
    uint16_t 	min_cell_vlt;      		// 单体最低/高电压mV
    uint16_t 	max_cell_vlt;
    int8_t 	min_cell_temp;      	        // 电池最低/高温度drg
    //int8_t 	max_cell_temp;
    uint16_t 	min_cell_temp_vlt;
    uint16_t	min_cell_dchg_vlt; 		// 最低放电电压
    uint16_t 	max_cell_chg_vlt;  		// 最高充电电压
    uint16_t 	rated_cap;     			// 额定容量
    uint8_t 	temp_corr;     		 	// 执行温度修正标志，1执行，0不执行。
}SocReg;

extern struct X_SOC_CALC
{
    uint32_t inAh;     		// 充入的电量
    uint32_t inAh_bak;
    uint32_t totalInAh;
    uint32_t totalInAh_bak;
    uint32_t outAh;        	// 放出的电量
    uint32_t outAh_bak;
    uint32_t totalOutAh;
    uint32_t totalOutAh_bak;
    uint32_t curAh;
    uint8_t ov_cnt;    		// 过冲计时
    uint8_t uv_cnt;    		// 过放计时
    uint32_t stb_cnt; 	        // 静置计时
    uint8_t  soc_rt;
}SocCalc;


 /**/
#endif










