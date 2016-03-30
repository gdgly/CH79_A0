//#include "stdio.h"
#include "macro_def.h"
#include "bq769x0.h"
 
unsigned char Buf[3] ={0,0,0};


unsigned char Dis_First_Run_Flag = 0;
unsigned char Dis_First_Run_t = 0;
    
unsigned char ChgExchangeMode_Cnt;
unsigned char DisExchangeMode_Cnt;
unsigned char IdleExchangeMode_Cnt;
unsigned char I2C_COM_ERROR_Flag;
unsigned long Current_Val;
signed long  CC_Val;
signed int int16_CC_AD;
signed int CC_AD;

//unsigned int CC_Val;
//unsigned int Current_Val;
signed char  ADCOffset_Val;

unsigned int Check_Val;
unsigned int Adc_value[10];
unsigned int Cell_Volt[10];
unsigned int Cell_Volt_Tol;
unsigned int Cell_Volt_Avg;
unsigned int Cell_Volt_Max;
unsigned int Cell_Volt_Min; 
unsigned int ADCGain_Val; 
unsigned int Pack_Volt;
unsigned int Temp_Val;
unsigned int V_TS2_Val; 
unsigned int R_TS2_Val;  

unsigned char     SYS_STAT_Last;
unsigned char     CELLBAL1_Last;
unsigned char     CELLBAL2_Last;
unsigned char     SYS_CTRL1_Last;
unsigned char     SYS_CTRL2_Last; 
unsigned char     PROTECT1_Last;
unsigned char     PROTECT2_Last;
unsigned char     PROTECT3_Last;
unsigned char     PROTECT1_Last_Copy;
unsigned char     PROTECT2_Last_Copy;
unsigned char     PROTECT3_Last_Copy;
unsigned char     OV_TRIP_Last; 
unsigned char     UV_TRIP_Last;
unsigned char     CC_CFG_Last;

unsigned int Temp_Volt_Sample_Cnt;
unsigned int Cell_Volt_Sample_Cnt;
unsigned int CC_Volt_Sample_Cnt;
unsigned int ChgOv_t;
unsigned int ChgCurOv_t;
unsigned int ChgCurOv_Re_t;   
unsigned int DisOv_t;  
unsigned int DisCurOv_t; 
unsigned int DisCurOv_t1; 
unsigned int DisCurOv_t2; 
unsigned int DisCurOv_Re_t; 
unsigned int DisCurShort_Re_t; 

unsigned int DEVICE_XREADY_Re_t; 

unsigned int CellBalance_Cur_Selct;  
unsigned int LedFlash_t; 
unsigned int PowerOff_Delay_t;
unsigned int Delay_time_t;

unsigned char Soc_OCV_CorrectEn_Flag = 0;
unsigned char CellBal_Cntrl_Lock = 0;
unsigned int CellBalance_Selct = 0;
unsigned char Init_Soc_Flag =0;
unsigned char LowPower_MCU_Entry_Flag = 0;
unsigned int LowPower_Entry_Delay_t = 0;
unsigned int Cell_Balance_Delay_t = 0;
enum  em_workmode
{
   IDLE_MODE 	    =0x00,
   CHARGE_MODE 	    =0x01,
   DISCHARGE_MODE   =0x02
}WorkMode;
 
 union  SYS_STATUS_ST//#define   SYS_STAT  0x00__IO     
{
  uint8_t Byte;  
  struct
  {
    uint8_t OCD           :1; 
    uint8_t SCD    	  :1;  
    uint8_t OV    	  :1; 
    uint8_t UV    	  :1;		 
    uint8_t OVRD_ALERT	  :1;	 
    uint8_t DEVICE_XREADY :1;		 
    uint8_t RSVD 	  :1;	
    uint8_t CC_READY 	  :1;	
  }Bit;
}SYS_STAT; 

union  SYS_CTRL1_ST 
{
  uint8_t Byte;  
  struct
  {
    uint8_t SHUT_B 	  :1;		 
    uint8_t SHUT_A 	  :1;		 
    uint8_t RSVD          :1;		 
    uint8_t TEMP_SEL	  :1;
    uint8_t ADC_EN    	  :1;	 
    uint8_t RSVD1    	  :1; 
    uint8_t RSVD2    	  :1; 
    uint8_t LOAD_PRESENT  :1; 
  }Bit;
}SYS_CTRL1; 
 
union  SYS_CTRL2_ST      
{
  uint8_t Byte;  
  struct
  {
    uint8_t CHG_ON 	  :1;		 
    uint8_t DSG_ON 	  :1;		 
    uint8_t RSVD          :1;		 
    uint8_t RSVD1	  :1;
    uint8_t RSVD2    	  :1;	 
    uint8_t CC_ONESHOT    :1; 
    uint8_t CC_EN    	  :1; 
    uint8_t DELAY_DIS     :1; 
  }Bit;
}SYS_CTRL2; 
         
union UINT_UCHAR
{
    unsigned int uintdata;
    unsigned char uchardata[2];
} RevcComData;

//------------------------------------------------------------------
union STATUS_FLAG
{
  unsigned char Byte;
  struct FLAG_BITS
  {
    //unsigned char Chg             : 1;
    unsigned char ChgOv           : 1;
    unsigned char ChgCurOv        : 1;
    unsigned char ChgTemp         : 1; 
    //unsigned char Dis             : 1;
    unsigned char DisOv           : 1;
    unsigned char DisTemp         : 1;  
    unsigned char DisCurOv        : 1; 
    unsigned char DisCurShort     : 1;  
    //unsigned char KEY_on          : 1; 
    unsigned char AfeErr          : 1; 
  }Bit;
} Bits_flag;
 
struct X_SOC
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

struct X_SOC_CALC
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
    uint8_t  ov_cnt;    		// 过冲计时
    uint8_t  uv_cnt;    		// 过放计时
    uint32_t stb_cnt; 	                // 静置计时
    uint8_t  soc_rt;
}SocCalc;

//SocCalc.soc_rt











