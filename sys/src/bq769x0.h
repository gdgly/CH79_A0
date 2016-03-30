  

#include "type.h"

#define BQ76930_DEVICE

/*
typedef union  //#define   SYS_STAT  0x00     
{
  uint8_t Byte;  
  struct
  {
    uint8_t CC_READY 	  :1;		 
    uint8_t RSVD 	  :1;		 
    uint8_t DEVICE_XREADY :1;		 
    uint8_t OVRD_ALERT	  :1;
    uint8_t UV    	  :1;	 
    uint8_t OV    	  :1; 
    uint8_t SCD    	  :1; 
    uint8_t OCD           :1; 
  }Bit;
}SYS_STATUS_ST; */


typedef union //#define   SYS_CTRL1  0x04     
{
  uint8_t Byte;  
  struct
  {
    uint8_t LOAD_PRESENT  :1;		 
    uint8_t RSVD1 	  :2;		 
    uint8_t ADC_EN        :1;		 
    uint8_t TEMP_SEL	  :1;
    uint8_t RSVD2    	  :1;	 
    uint8_t SYUT_A    	  :1; 
    uint8_t SHUT_B    	  :1;  
  }Bit;
}SYS_CTRL1_ST; 


typedef union //#define   SYS_CTRL2  0x05     
{
  uint8_t Byte;  
  struct
  {
    uint8_t DELAY_DIS     :1;		 
    uint8_t CC_EN 	  :1;		 
    uint8_t CC_ONESHOT    :1;		 
    uint8_t RSVD1	  :1;
    uint8_t RSVD2    	  :1;	 
    uint8_t RSVD3    	  :1;	
    uint8_t DSG_ON    	  :1; 
    uint8_t SHG_ON    	  :1;  
  }Bit;
}SYS_CTRL2_ST; 

typedef union //#define   PROTECT1  0x06     
{
  uint8_t Byte;  
  struct
  {
    uint8_t RSNS          :1;		 
    uint8_t RSVD 	  :2;		 
    uint8_t SCD_D1        :1;		 
    uint8_t SCD_D0	  :1;
    uint8_t SCD_T2    	  :1;	 
    uint8_t SCD_T1    	  :1;	
    uint8_t SCD_T0    	  :1;   
  }Bit;
}PROTECT1_ST; 

typedef union //#define   PROTECT2  0x07     
{
  uint8_t Byte;  
  struct
  {
    uint8_t RSVD          :1;		 
    uint8_t OCD_D2 	  :2;		 
    uint8_t OCD_D1        :1;		 
    uint8_t OCD_D0	  :1;
    uint8_t OCD_T2    	  :1;	 
    uint8_t OCD_T1    	  :1;	
    uint8_t OCD_T0    	  :1;   
  }Bit;
}PROTECT2_ST; 

typedef union //#define   PROTECT3  0x08    
{
  uint8_t Byte;  
  struct
  {
    uint8_t UV_D1         :1;		 
    uint8_t UV_D0 	  :1;		 
    uint8_t OV_D1         :1;		 
    uint8_t OV_D0	  :1;
    uint8_t RSVD    	  :4;	  
  }Bit;
}PROTECT3_ST; 

typedef union //#define   OV_TRIP  0x09    
{
  uint8_t Byte;  
  struct
  {
    uint8_t OV_T7         :1;		 
    uint8_t OV_T6 	  :1;		 
    uint8_t OV_T5         :1;		 
    uint8_t OV_T4	  :1;	 
    uint8_t OV_T3	  :1;	 
    uint8_t OV_T2	  :1;	 
    uint8_t OV_T1	  :1;	 
    uint8_t OV_T0	  :1;  
  }Bit;
}OV_TRIP_ST; 

typedef union //#define   UV_TRIP  0x0A    
{
  uint8_t Byte;  
  struct
  {
    uint8_t UV_T7         :1;		 
    uint8_t UV_T6 	  :1;		 
    uint8_t UV_T5         :1;		 
    uint8_t UV_T4	  :1;	 
    uint8_t UV_T3	  :1;	 
    uint8_t UV_T2	  :1;	 
    uint8_t UV_T1	  :1;	 
    uint8_t UV_T0	  :1;  
  }Bit;
}UV_TRIP_ST; 

typedef union //#define   CC_CFG  0x0B    
{
  uint8_t Byte;  
  struct
  {
    uint8_t RSVD          :2;		 
    uint8_t CC_CFG5 	  :1;		 
    uint8_t CC_CFG4       :1;		 
    uint8_t CC_CFG3	  :1;	 
    uint8_t CC_CFG2	  :1;	 
    uint8_t CC_CFG1	  :1;	 
    uint8_t CC_CFG0	  :1;	  
  }Bit;
}CC_CFG_ST; 








  #define     SYS_STAT_ADDR         0x00 
  #define     CELLBAL1_ADDR         0x01 

#if defined (BQ76940_DEVICE) || defined (BQ76930_DEVICE)
  #define     CELLBAL2_ADDR         0x02 
#endif

#ifdef BQ76940_DEVICE
  #define     CELLBAL3_ADDR         0x03 
#endif

  #define     SYS_CTRL1_ADDR        0x04 
  #define     SYS_CTRL2_ADDR        0x05 
  #define     PROTECT1_ADDR         0x06 
  #define     PROTECT2_ADDR         0x07 
  #define     PROTECT3_ADDR         0x08 
  #define     OV_TRIP_ADDR          0x09 
  #define     UV_TRIP_ADDR          0x0A 
  #define     CC_CFG_ADDR           0x0B 
  #define     VC1_HI_ADDR           0x0C 
  #define     VC1_LO_ADDR           0x0D 
  #define     VC2_HI_ADDR           0x0E 
  #define     VC2_LO_ADDR           0x0F 
  #define     VC3_HI_ADDR           0x10 
  #define     VC3_LO_ADDR           0x11 
  #define     VC4_HI_ADDR           0x12 
  #define     VC4_LO_ADDR           0x13 
  #define     VC5_HI_ADDR           0x14 
  #define     VC5_LO_ADDR           0x15 

#if defined (BQ76940_DEVICE) || defined (BQ76930_DEVICE)
  #define     VC6_HI_ADDR           0x16 
  #define     VC6_LO_ADDR           0x17 
  #define     VC7_HI_ADDR           0x18 
  #define     VC7_LO_ADDR           0x19 
  #define     VC8_HI_ADDR           0x1A 
  #define     VC8_LO_ADDR           0x1B 
  #define     VC9_HI_ADDR           0x1C 
  #define     VC9_LO_ADDR           0x1D 
  #define     VC10_HI_ADDR          0x1E 
  #define     VC10_LO_ADDR          0x1F 
#endif

#ifdef BQ76940_DEVICE
  #define     VC11_HI_ADDR          0x20 
  #define     VC11_LO_ADDR          0x21 
  #define     VC12_HI_ADDR          0x22 
  #define     VC12_LO_ADDR          0x23 
  #define     VC13_HI_ADDR          0x24 
  #define     VC13_LO_ADDR          0x25 
  #define     VC14_HI_ADDR          0x26 
  #define     VC14_LO_ADDR          0x27 
  #define     VC15_HI_ADDR          0x28 
  #define     VC15_LO_ADDR          0x29 
#endif

  #define     BAT_HI_ADDR           0x2A 
  #define     BAT_LO_ADDR           0x2B 
  #define     TS1_HI_ADDR           0x2C 
  #define     TS1_LO_ADDR           0x2D 

#if defined (BQ76940_DEVICE) || defined (BQ76930_DEVICE)
  #define     TS2_HI_ADDR           0x2E 
  #define     TS2_LO_ADDR           0x2F 
#endif

#ifdef BQ76940_DEVICE
  #define     TS3_HI_ADDR           0x30 
  #define     TS3_LO_ADDR           0x31 
#endif

  #define     CC_HI_ADDR            0x32 
  #define     CC_LO_ADDR            0x33 
  #define     ADCGAIN1_ADDR         0x50 
  #define     ADCOFFSET_ADDR        0x51 
  #define     ADCGAIN2_ADDR         0x59 
  
/*

  #define     SYS_STAT         0x00 
  #define     CELLBAL1         0x01 

#ifdef (BQ76940_DEVICE || BQ76930_DEVICE)
  #define     CELLBAL2(1)      0x02 
#endif

#ifdef BQ76940_DEVICE
  #define     CELLBAL3(2)      0x03 
#endif

  #define     SYS_CTRL1        0x04 
  #define     SYS_CTRL2        0x05 
  #define     PROTECT1         0x06 
  #define     PROTECT2         0x07 
  #define     PROTECT3         0x08 
  #define     OV_TRIP          0x09 
  #define     UV_TRIP          0x0A 
  #define     CC_CFG           0x0B 
  #define     VC1_HI           0x0C 
  #define     VC1_LO           0x0D 
  #define     VC2_HI           0x0E 
  #define     VC2_LO           0x0F 
  #define     VC3_HI           0x10 
  #define     VC3_LO           0x11 
  #define     VC4_HI           0x12 
  #define     VC4_LO           0x13 
  #define     VC5_HI           0x14 
  #define     VC5_LO           0x15 

#ifdef (BQ76940_DEVICE || BQ76930_DEVICE)
  #define     VC6_HI(1)        0x16 
  #define     VC6_LO(1)        0x17 
  #define     VC7_HI(1)        0x18 
  #define     VC7_LO(1)        0x19 
  #define     VC8_HI(1)        0x1A 
  #define     VC8_LO(1)        0x1B 
  #define     VC9_HI(1)        0x1C 
  #define     VC9_LO(1)        0x1D 
  #define     VC10_HI(1)       0x1E 
  #define     VC10_LO(1)       0x1F 
#endif

#ifdef BQ76940_DEVICE
  #define     VC11_HI(2)       0x20 
  #define     VC11_LO(2)       0x21 
  #define     VC12_HI(2)       0x22 
  #define     VC12_LO(2)       0x23 
  #define     VC13_HI(2)       0x24 
  #define     VC13_LO(2)       0x25 
  #define     VC14_HI(2)       0x26 
  #define     VC14_LO(2)       0x27 
  #define     VC15_HI(2)       0x28 
  #define     VC15_LO(2)       0x29 
#endif

  #define     BAT_HI           0x2A 
  #define     BAT_LO           0x2B 
  #define     TS1_HI           0x2C 
  #define     TS1_LO           0x2D 

#ifdef (BQ76940_DEVICE || BQ76930_DEVICE)
  #define     TS2_HI(1)        0x2E 
  #define     TS2_LO(1)        0x2F 
#endif

#ifdef BQ76940_DEVICE
  #define     TS3_HI(2)        0x30 
  #define     TS3_LO(2)        0x31 
#endif

  #define     CC_HI            0x32 
  #define     CC_LO            0x33 
  #define     ADCGAIN1         0x50 
  #define     ADCOFFSET        0x51 
  #define     ADCGAIN2         0x59 

#ifdef (BQ76940_DEVICE || BQ76930_DEVICE)
  #define     VC7_HI(1)        0x18 
  #define     VC7_LO(1)        0x19 
  #define     VC8_HI(1)        0x1A 
  #define     VC8_LO(1)        0x1B 
  #define     VC9_HI(1)        0x1C 
  #define     VC9_LO(1)        0x1D 
  #define     VC10_HI(1)       0x1E 
  #define     VC10_LO(1)       0x1F 
#endif

#ifdef BQ76940_DEVICE
  #define     VC11_HI(2)       0x20 
  #define     VC11_LO(2)       0x21 
  #define     VC12_HI(2)       0x22 
  #define     VC12_LO(2)       0x23 
  #define     VC13_HI(2)       0x24 
  #define     VC13_LO(2)       0x25 
  #define     VC14_HI(2)       0x26 
  #define     VC14_LO(2)       0x27 
  #define     VC15_HI(2)       0x28 
  #define     VC15_LO(2)       0x29 
#endif

  #define     BAT_HI           0x2A 
  #define     BAT_LO           0x2B 
  #define     TS1_HI           0x2C 
  #define     TS1_LO           0x2D 

#ifdef (BQ76940_DEVICE || BQ76930_DEVICE)
  #define     TS2_HI(1)        0x2E 
  #define     TS2_LO(1)        0x2F 
#endif

#ifdef BQ76940_DEVICE
  #define     TS3_HI(2)        0x30 
  #define     TS3_LO(2)        0x31 
#endif

  #define     CC_HI            0x32 
  #define     CC_LO            0x33 
  #define     ADCGAIN1         0x50 
  #define     ADCOFFSET        0x51 
  #define     ADCGAIN2         0x59 
*/