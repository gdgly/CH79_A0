//===2016-03-25pm
//===2016-04-09pm
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
  disableInterrupts();        // 关闭MCU全局变量
  SysInit();                  // 系统初始化，MCU时钟配置、ADC模块初始化
  PortInit();                 // MCU管脚配置
  ALERT_PIN_HIGH();
  VCC1_ON();
  //LED2_ON();                // 复位后亮LED1，在完成AFE IC初始化之后，熄灭
  I2C_Model_Init();           // 启用MCU的IIC模块
#ifdef Uart_Model_Enable
  Uart_Model_Init();          // 启用MCU的UART模块
#endif
  Var_Init();                 // 初始化各变量值 
  Timer2Init();               // 设置TIMER 2为基准时间的定时器
  enableInterrupts();         // 打开MCU全局中断
  Delay_ms(20);               // 延时50mS，MCU稳定
  ClrWdt();                   // 启用看门狗
  Afe_Device_Init();          // 初始化AFE IC，包括开启ADC、电流检测模块、设置过放过充电芯电压值、放电过流、短路保护等参数 
  LED2_OFF();                 // 关闭LED1
  SOC_Init();                 // 利用EEPROM中保存数据或OCV初始化SOC数据
  //Soc_OCV_CorrectEn_Flag = 1; // 上电允许SOC的OCV校准 
  //Afe_ADC_Disable();        // 100+uA
  //Afe_Temp_Disable();       // 100+uA
  
  WorkMode = DISCHARGE_MODE;  
  DisExchangeMode_Cnt  = 100;  
  IdleExchangeMode_Cnt = 0;
  ChgExchangeMode_Cnt  = 50; 
  Bits_flag.Bit.DisOv  = 1;    
  
  DisCurOv_t1 = 0;
  DisCurOv_t2 = 0;
  CC_Volt_Sample_Cnt = 0;
  
  //Bits_flag.Bit.DisTemp = 1; 
  DisTemp_cnt = 0;
  ALERT_PIN_LOW();
       
  
  while(1)
  {
    //=========
    // 问题1：放电状态下，电芯电压处于过充时，BQ会自动关闭充电MOS管;   解决方案：放电状态下，重新设置硬件过充电压为4.3V
    // 问题2：充电状态下，电芯电压处于过放时，BQ会自动关闭放电MOS管;   解决方案：充电状态下，重新设置硬件过充电压为1.5V
    // 问题3：（充放电及空载识别问题）欠压状态下，插上充电器时，识别不了充电器;  
              //解决方案：检测无Triger信号时，为空载模式; Triger信号范围在[]时，或检测到有负载信号、或放电电流 >= 10mA 时，为放电模式，否则为充电模式.
    // 问题4：BQ供给MCU的3.3V不稳定;    解决方案：由于BQ的电流采样（电量库仑计）的启用，会存在+-80mV的浮动；
    // 问题5：拨出负载时，BQ有时会出现电流采样值变大的异常;    解决方案：MCU读取BQ内电流值寄存器时，添加个每隔250mS的限时读操作；
    // 问题6：充电状态下，出现异常，需要MCU掉电休眠时，MCU会被未移除的充电器电压自动激活，从而形成: 异常---休眠--激活--异常--休眠--激活的循环;  
              // 解决方案：模式检测时所用的Triger信号AD采样的MOS管驱动，有之前直接VCC控制，改为MCU的IO口控制，该驱动脚信号上电时开始为高电平，进入休眠之前为低电平信号；
    // 问题7：在BQ硬件过流保护时，出现自动保护解除异常;  原因：由于电流启用单次采样时，误把BQ中过流标志符清除;
    // 问题8：
    ClrWdt();                          // 刷新看门狗 1.02S未刷新系统复位 
    
    Afe_Get_SysStatus();               // AFE IC 状态检测，包括充放电MOS管开关状态、电流采样结束状态、AFE IC（错误、过流、短路、过充、过放）异常状态
   
#ifdef Uart_Model_Enable
    Uart_SendStr("\r\n");
    Uart_SendStr("\r\n SYS_STAT = ");   Uart_SendData(SYS_STAT.Byte,16);  
    Uart_SendStr(" SYS_CTRL1 = ");      Uart_SendData(SYS_CTRL1.Byte,16); 
    Uart_SendStr(" SYS_CTRL2 = ");      Uart_SendData(SYS_CTRL2.Byte,16);  
#endif
    
    ModeCheck();         // 充电、放电、空载工作模式检测 
    
    ClearStatus();       // 各工作模式下的变量清零处理
    
    Afe_Volt_Val_Get();  // AFE IC 温度值、电芯电压值、电流值采样
    
    CurrentCheck();      // 电流异常检测
    
    VoltCheck();         // 电压异常检测
    
    TempCheck();         // 温度异常检测
    
    CellBal_Cntrl();     // 充电均衡控制管理
    
    ChgDis_AbnormalCheck();         // 充放电状态下出现电流异常
     
    Afe_FET_ChgDis_Cntrl();         // 充放电MOS管控制管理 
    
    LedShow_Cntrl();     // LedShow_WorkMode();   // 工作电量指示灯显示
     
#ifdef Uart_Model_Enable  
    
    Uart_SendStr(" Check_Val= ");     Uart_SendData((uint16_t)Check_Val,10);
    Uart_SendStr(" WorkMode= ");      Uart_SendData(WorkMode,16); 
    Uart_SendStr(" Bits_flag = ");    Uart_SendData(Bits_flag.Byte,16); 
     
    Uart_SendStr(" Current_Val= ");   Uart_SendData((uint16_t)Current_Val,10);
    Uart_SendStr(" Volt_Avg= ");      Uart_SendData(Cell_Volt_Avg,10);
    Uart_SendStr(" Max= ");           Uart_SendData(Cell_Volt_Max,10);
    Uart_SendStr(" Min= ");           Uart_SendData(Cell_Volt_Min,10);
       
    Uart_SendStr(" soc_rt= ");        Uart_SendData((uint16_t)SocCalc.soc_rt,10); 
    Uart_SendStr(" ah= ");            Uart_SendData((uint16_t)SocReg.ah,10);
    Uart_SendStr(" CellBal = ");      Uart_SendData((uint16_t)CellBalance_Selct,16);  
    Uart_SendStr(" Temp_Val = ");     Uart_SendData((uint16_t)Temp_Val,10); 
    /*  
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
    LowPower_Cntrl();        // 低功耗控制管理
  }    
 
}
 













