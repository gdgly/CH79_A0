 
#include "type.h"
//#define ClrWdt()   nop()
#define   PB4_XOR()    //PB_ODR_ODR4 = ~PB_ODR_ODR4
#define   PB5_XOR()    //PB_ODR_ODR5 = ~PB_ODR_ODR5
//==================================
#define   SLAVE_ADDR            0x18
#define   CC_CFG_INIT_VAL       0x19
//==================================
#define   MAX_UINT16_T_NUM      65535
#define   MAX_UINT8_T_NUM       255

//==================================
#define   E2PROM_ADDR_BASE      0x004000
#define   E2PROM_ADDR_END       0x00407F
#define   ADJUST_ADDR           0x004000
#define   SOC_ADDR              0x004004  
//========================================= 
#define   OV_DELAY_SET          1                   //  单位：S, AFE（硬件）过充保护延时时间, 有效范围:[1,2,4,8]
#define   UV_DELAY_SET          1                   //  单位：S, AFE（硬件）过放保护延时时间, 有效范围:[1,4,8,16]
#define   OCD_DELAY_SET         640                 //  单位：mS,AFE（硬件）放电过流保护延时时间, 有效范围:[8,20,40,80,160,320,640,1280]
#define   SCD_DELAY_SET         400                 //  单位：uS,AFE（硬件）短路过流保护延时时间, 有效范围:[70,100,200,400]
#define   OV_THREHOLD_VAL_SET   4150                //  单位：mV,AFE（硬件）过充保护电压值, 有效范围:[2993,4482][3150,4700]
#define   UV_THREHOLD_VAL_SET   3000                //  单位：mV,AFE（硬件）过充保护电压值, 有效范围:[1495,2984][1580,3100]
#define   OCD_THREHOLD_VAL_SET  11000               //  单位：mA,AFE（硬件）放电保护电流值, 有效范围:[800,10000]     
#define   SCD_THREHOLD_VAL_SET  44000               //  单位：mA,AFE（硬件）短路保护电流值, 有效范围:[22000,200000]
//==========================================
#define   CHG_OV_VAL_SET        4150               //  单位：mV, MCU（软件）过充保护电压值
#define   DIS_UV_VAL_SET        3000               //  单位：mV, MCU（软件）过过放保护电压值（平均电芯电压值）
#define   DIS_UV_MIN_VAL_SET    3000               //  单位：mV, MCU（软件）过过放保护电压值（最小电芯电压值）
#define   CHG_OV_RE_VAL_SET     4000               //  单位：mV, MCU（软件）过过充保护释放电压值
#define   DIS_UV_RE_VAL_SET     3300               //  单位：mV, MCU（软件）过过放保护释放电压值

#define   ChgCurOv_Val_SET      3000               //  单位：mA, MCU（软件）充电保护电流值
#define   DisCurOv_Val_SET      9000//11000              //  单位：mA, MCU（软件）
#define   DisCurOv_1_Val_SET    9000//11000              //  单位：mA, MCU（软件）放电保护电流值
#define   DisCurOv_2_Val_SET    15000//11000              //  单位：mA, MCU（软件）放电保护电流值
//================================== 
#define   ChgOv_t_SET           100                 //  单位：mS, MCU（软件）过充保护延时时间
#define   DisOv_t_SET           100                 //  单位：mS, MCU（软件）过放保护延时时间
#define   ChgCurOv_t_SET        100                 //  单位：mS, MCU（软件）充电过流保护延时时间
#define   DisCurOv_t_SET        80                  //  单位：mS, MCU（软件）放电过流保护延时时间
#define   DisCurOv_t1_SET       80                  //  单位：mS, MCU（软件）放电过流保护延时时间
#define   DisCurOv_t2_SET       10                  //  单位：mS, MCU（软件）放电过流保护延时时间
#define   ChgCurOv_Re_t_SET     2000                //  单位：mS, MCU（软件）充电过流保护释放延时时间
#define   DisCurOv_Re_t_SET     2000                //  单位：mS, MCU（软件）放电过流保护释放延时时间
#define   DisCurShort_Re_t_SET  2000                //  单位：mS, MCU（软件）过充保护延时时间
//===================================
#define   PowerOff_Delay_t_SET  200                 //  单位：mS
#define   CELLBALANCE_BEGIN_VAL 4000                //  单位：mV
#define   CELLBALANCE_DELTA_VAL 50                 //  单位：mV


#define   DEVICE_XREADY_Re_SET  300                 //  单位：mS


#define   OverTimeDelay_SET     50                //
//================================== 
//#define   IS_SIGNAL_IN()        PD_IDR_IDR3 == 0
#define   IS_FAULT_ON()         PD_IDR_IDR3 == 1
#define   IS_FAULT_OFF()        PD_IDR_IDR3 == 0
#define   IS_CHG_DETECT()       PD_IDR_IDR2 == 0
//#define   IS_LOAD_DETECT()    PA_IDR_IDR3 == 0
//#define   IS_ALERT()          PC_IDR_IDR7 == 1
#define   ALERT_PIN_HIGH()      PC_ODR_ODR7 = 1
#define   ALERT_PIN_LOW()       PC_ODR_ODR7 = 0

#define   VCC1_ON()             PA_ODR_ODR1 = 1

#define   WAKEUP_AFE_ON()       PD_ODR_ODR4 = 1
#define   WAKEUP_AFE_OFF()      PD_ODR_ODR4 = 0

#define   FAULT_DETECT_CTRL_ON()   PC_ODR_ODR6 = 1
#define   FAULT_DETECT_CTRL_OFF()  PC_ODR_ODR6 = 0

#define   LED1_ON()             PC_ODR_ODR3 = 1
#define   LED1_OFF()            PC_ODR_ODR3 = 0
#define   LED2_ON()             PC_ODR_ODR4 = 1
#define   LED2_OFF()            PC_ODR_ODR4 = 0
#define   LED3_ON()             PC_ODR_ODR5 = 1
#define   LED3_OFF()            PC_ODR_ODR5 = 0
#define   LED1_XOR()            PC_ODR_ODR3 = ~PC_ODR_ODR3
#define   LED2_XOR()            PC_ODR_ODR4 = ~PC_ODR_ODR4
#define   LED3_XOR()            PC_ODR_ODR5 = ~PC_ODR_ODR5
 
 
#define   ChgTempH_ON            264     //P55C
#define   ChgTempH_OFF           334     //P45C
#define   ChgTempL_ON            800     //N5C
#define   ChgTempL_OFF           711     //P5C

#define   DisTempH_ON            207     //P65C
#define   DisTempH_OFF           264     //P55C
#define   DisTempL_ON            873     //N15C
#define   DisTempL_OFF           800     //N5C

 
 