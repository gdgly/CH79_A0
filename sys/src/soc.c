
#include "stm8s.h" 
#include"user.h"

 
typedef struct
{
    uint8_t soc;   // 对应的soc。
    uint16_t volt;  // 开路电压
} X_SOC_2_OCV;

typedef struct
{
    int8_t temp;     // 温度
    uint16_t rated_cap; // 对应的额定容量
} X_TEMP_2_CAP;

/*
const X_SOC_2_OCV Soc2OcvTbl[11] =  // 根据实际修改
{// soc, volt,
    {0, 3440}, {10, 3523}, {20, 3585}, {30, 3615}, {40, 3643}, {50, 3690}, // 润丰电池
    {60, 3755}, {70, 3850}, {80, 3930}, {90, 4040}, {100, 4190},
    //{0, 3435}, {10, 3510}, {20, 3572}, {30, 3610}, {40, 3635}, {50, 3675}, // 天能电池
    //{60, 3755}, {70, 3890}, {80, 3935}, {90, 4041}, {100, 4188},
};

const X_TEMP_2_CAP Temp2CapacityTbl[10] =   // 根据实际修改
{// temp, ratedcap
    // -20drg -> 50%, -10drg -> 75%, 0drg -> 90%, 10drg -> 95%, 25drg -> 100%
    { -20, 11000}, { -15, 14000},
    { -10, 16500}, { -5, 17800},
    {0, 19800}, {5, 19950},
    {10, 21000}, {15, 21333},
    {20, 21666}, {25, 22000},
};
*/
///const 
X_SOC_2_OCV Soc2OcvTbl[11] =  // 根据实际修改
{
    //{0, 3440}, {10, 3523}, {20, 3585}, {30, 3615}, {40, 3643}, {50, 3690}, {60, 3755}, {70, 3850}, {80, 3930}, {90, 4040}, {100, 4190},
    //{0, 3335}, {10, 3510}, {20, 3572}, {30, 3610}, {40, 3635}, {50, 3675},{60, 3755}, {70, 3890}, {80, 3935}, {90, 4041}, {100, 4188},
    {0, 3035}, {10, 3210}, {20, 3472}, {30, 3610}, {40, 3635}, {50, 3675},{60, 3755}, {70, 3890}, {80, 3935}, {90, 4041}, {100, 4188},
};
/*const X_SOC_2_OCV Soc2OcvTbl_10deg[11] = // 10'C
{
    {0, 3440}, {10, 3512}, {20, 3571}, {30, 3605}, {40, 3633}, {50, 3674},
    {60, 3749}, {70, 3836}, {80, 3931}, {90, 4038}, {100, 4191},
};*/
const X_SOC_2_OCV Soc2OcvTbl_0deg[11] = // 0'C - 20160131
{
    {0, 3339}, {10, 3500}, {20, 3555}, {30, 3594}, {40, 3629}, {50, 3671}, {60, 3736}, {70, 3830}, {80, 3932}, {90, 4040}, {100, 4196},
};
const X_SOC_2_OCV Soc2OcvTbl_N10deg[11] = // -10'C
{
    {0, 3303}, {10, 3385}, {20, 3531}, {30, 3574}, {40, 3611}, {50, 3656}, {60, 3720}, {70, 3811}, {80, 3918}, {90, 4029}, {100, 4184},
};

const X_TEMP_2_CAP Temp2CapacityTbl[10] =  // 根据实际修改
{
    // -20drg -> 50%, -10drg -> 75%, 0drg -> 90%, 10drg -> 95%, 25drg -> 100%
    //{ -20, 11000}, { -15, 14000}, { -10, 16500}, { -5, 17800}, {0, 19800}, {5, 19950}, {10, 21000}, {15, 21333}, {20, 21666}, {25, 22000}, 
    // -20drg -> 83%, -10drg -> 87%, 0drg -> 92%, 10drg -> 98%, 25drg -> 100%
    { -20, 18260}, { -15, 18700}, { -10, 19140}, { -5, 19800}, {0, 20020}, {5, 20900}, {10, 21560}, {15, 21780}, {20, 21890}, {25, 22000},
};

  
 

/*  计算电流积分，每10ms 调用一次。
*
*   输入参数:
*  SocReg.curr                  实时电流
*  SocReg.curr_dir              电流方向
*
*   输出:   用于计算的中间值。
*
*/
void SOCAhIntergrate(void)
{
    uint32_t tmpah;
    tmpah = Current_Val;    // xmA 10ms
     
    //if(WorkMode == CHARGE_MODE)
    if(CC_Val > 0)
    {
      SocCalc.inAh_bak += tmpah;

      if(SocCalc.inAh_bak > 360000)
      {
        SocCalc.totalInAh += SocCalc.inAh_bak / 360000;
        SocCalc.inAh_bak = SocCalc.inAh_bak % 360000; // 充入1Ah。
      }
    }
    //else if(WorkMode == DISCHARGE_MODE)
    else// if(WorkMode == CHARGE_MODE)
    {
      
      //tmpah = Current_Val;    // xmA 10ms
      //if(WorkMode == DISCHARGE_MODE && SocCalc.totalOutAh > SocCalc.totalOutAh_bak) // 放出。
      //deltAh = (uint16_t)(SocCalc.totalOutAh - SocCalc.totalOutAh_bak);
      //SocCalc.totalOutAh_bak = SocCalc.totalOutAh;
      SocCalc.outAh_bak += tmpah;

      if(SocCalc.outAh_bak > 360000)
      {
        SocCalc.totalOutAh += SocCalc.outAh_bak / 360000;
        SocCalc.outAh_bak = SocCalc.outAh_bak % 360000; // 放出1Ah。
      }
    }/*
    else if(WorkMode == IDLE_MODE)//
    {
      //if((SocCalc.stb_cnt ++) >= 120000)//静置20分钟后，允许进行OCV校准
      {
        SocCalc.stb_cnt = 120000;
        //Soc_OCV_CorrectEn_Flag = 1;
      }
    }*/
}
//================= 将soc 范围 10~90 转换为 soc_rt 所需的 0~100。
void SOCConvert(void)
{
  if(SocReg.soc > 90)
  {
    SocCalc.soc_rt = 100;
  }
  else if(SocReg.soc < 10)
  {
    SocCalc.soc_rt = 0;
  }
  else
  {
    SocCalc.soc_rt = (SocReg.soc -10)*5/4;
  }
}
/*  计算SOC，每100ms 调用一次。
*
*   输入参数:
*  SocReg.ah                当前剩余容量
*  SocReg.max_cell_vlt      最高单节电压
*  SocReg.max_cell_chg_vlt  过冲电压
*  SocReg.min_cell_vlt      最低单节电压
*  SocReg.min_cell_dchg_vlt 过放电压
*  SocReg.rated_cap         额定容量
*
*   输出:
*  SocReg.ah            新的容量
*  SocReg.soc           新的SOC
*
*/
void SOCCalculate(void)
{
    uint32_t deltAh;
    SocCalc.curAh = SocReg.ah; 
    SocReg.rated_cap = 2050;
    SocReg.min_cell_temp_vlt = V_TS2_Val; 
    SocReg.min_cell_vlt = Cell_Volt_Min;
    SOCCorrectOCV();      // OCV 校准
    //SOCCorrectTemp();   // 温度校准 
    
    if( SocCalc.totalInAh > SocCalc.totalInAh_bak)   // 充入。
    {
      deltAh = (uint16_t)(SocCalc.totalInAh - SocCalc.totalInAh_bak);
      SocCalc.totalInAh_bak = SocCalc.totalInAh;
      SocCalc.curAh += deltAh; 

      if(SocCalc.curAh > SocReg.rated_cap)    // 防止大于额定容量。
      {
        SocCalc.curAh = SocReg.rated_cap; 
      } 
    }

    if( SocCalc.totalOutAh > SocCalc.totalOutAh_bak) // 放出。
    {
      deltAh = (uint16_t)(SocCalc.totalOutAh - SocCalc.totalOutAh_bak);
      SocCalc.totalOutAh_bak = SocCalc.totalOutAh;
 
      if(SocCalc.curAh < deltAh)  // 防止小于0。
      {
        SocCalc.curAh = 0;
      }
      else
      { 
        SocCalc.curAh -= deltAh; 
      } 
    } 
    if(WorkMode == CHARGE_MODE  )   // 充入。
    {  
      if(Bits_flag.Bit.ChgOv)   // 过充 
      {
        if((SocCalc.ov_cnt ++) >= 30 && (Cell_Volt_Max >= 4100 && Cell_Volt_Avg >= 4100))
        {
          SocCalc.ov_cnt = 30;
          SocCalc.curAh = SocReg.rated_cap;
          SocCalc.totalInAh = 0;
          SocCalc.totalInAh_bak = 0;
        } 
      }
      else
      {
        SocCalc.ov_cnt = 0;
      }
    }
    else //(WorkMode == DISCHARGE_MODE ) // 放出。 
    {  
      if(Bits_flag.Bit.DisOv )//&& Cell_Volt_Avg < 2900)  // 有任一电池过放了。
      {
        if(SocReg.min_cell_vlt > 0)
        {
         // if((SocCalc.uv_cnt++) >= 5)
          {
            SocCalc.uv_cnt = 5;
            SocCalc.curAh = 0;
            SocCalc.totalOutAh = 0;
            SocCalc.totalOutAh_bak = 0; 
          } 
        }
      }
      else
      {
        SocCalc.uv_cnt = 0;
      }

    } 
   
    SocReg.ah = SocCalc.curAh; // 计算SOC。  
    SocReg.soc = (uint8_t)(SocCalc.curAh * 100 / SocReg.rated_cap);
    SocCalc.soc_rt = SocReg.soc;
    
     /*
    Soc_Tmp = FLASH_ReadByte(SOC_ADDR);     // 读取SOC数据
    SocReg.soc = Soc_Tmp; //SocReg.ah = SocCalc.curAh; // 计算SOC。 
    SocCalc.curAh = ((uint32_t)SocReg.rated_cap * Soc_Tmp) / 100;
    SocReg.ah = SocCalc.curAh;
    SocCalc.soc_rt = SocReg.soc;
    
    
    SocReg.ah = SocCalc.curAh; // 计算SOC。  
    SocReg.soc = (uint8_t)((uint32_t)SocCalc.curAh * 100 / SocReg.rated_cap);
    SocCalc.soc_rt = SocReg.soc;*/
}
void SOCCalculate1(void)
{
    uint32_t deltAh;
    SocCalc.curAh = SocReg.ah; 
    SocReg.rated_cap = 2050;
    SocReg.min_cell_temp_vlt = V_TS2_Val; 
    SocReg.min_cell_vlt = Cell_Volt_Min;
    SOCCorrectOCV();      // OCV 校准
    //SOCCorrectTemp();   // 温度校准 
    
    if(WorkMode == CHARGE_MODE && SocCalc.totalInAh > SocCalc.totalInAh_bak)   // 充入。
    {
      deltAh = (uint16_t)(SocCalc.totalInAh - SocCalc.totalInAh_bak);
      SocCalc.totalInAh_bak = SocCalc.totalInAh;
      SocCalc.curAh += deltAh; 

      if(SocCalc.curAh > SocReg.rated_cap)    // 防止大于额定容量。
      {
        SocCalc.curAh = SocReg.rated_cap;
      }
 
      if(Bits_flag.Bit.ChgOv)   // 过充 
      {
        if((SocCalc.ov_cnt ++) >= 30)
        {
          SocCalc.ov_cnt = 30;
          SocCalc.curAh = SocReg.rated_cap;
        } 
      }
      else
      {
        SocCalc.ov_cnt = 0;
      }
    }

    if(WorkMode == DISCHARGE_MODE && SocCalc.totalOutAh > SocCalc.totalOutAh_bak) // 放出。
    {
      deltAh = (uint16_t)(SocCalc.totalOutAh - SocCalc.totalOutAh_bak);
      SocCalc.totalOutAh_bak = SocCalc.totalOutAh;
 
      if(SocCalc.curAh < deltAh)  // 防止小于0。
      {
        SocCalc.curAh = 0;
      }
      else
      { 
        SocCalc.curAh -= deltAh; 
      }
 
      if(Bits_flag.Bit.DisOv)  // 有任一电池过放了。
      {
        if(SocReg.min_cell_vlt > 0)
        {
          if((SocCalc.uv_cnt++) >= 30)
          {
            SocCalc.uv_cnt = 30;
            SocCalc.curAh = 0;
          } 
        }
      }
      else
      {
        SocCalc.uv_cnt = 0;
      }

    }
/*
    
    SocReg.soc = soc;   
    SocCalc.curAh = (uint32_t)(SocReg.rated_cap * soc) / 100;
    SocReg.ah = SocCalc.curAh;
    */
   
    SocReg.ah = SocCalc.curAh; // 计算SOC。  
    SocReg.soc = (uint8_t)(SocCalc.curAh * 100 / SocReg.rated_cap);
    SocCalc.soc_rt = SocReg.soc;
    
    
    //SOCSmooth();
    //SOCConvert();//BMSRegs.soc_rt = SocReg.soc;
    //BMSReflashFromSOC();
}
/*  静置后进行OCV 修正。
*
*   需要输入的参数:
*
*  1. SocReg.curr           实时电流
*  2. SocReg.min_cell_vlt   最低单节电压(关键参数)
*
*   输出:
*
*  1. SocReg.soc
*  2. SocCalc.curAh
*/
void SOCCorrectOCV(void)
{
  uint8_t i, soc;
  uint32_t tmpah;
  uint16_t deltV;
  tmpah = SocReg.curr;    // xmA 10ms 
  if(WorkMode != IDLE_MODE && Soc_OCV_CorrectEn_Flag == 1) 
  { 
    Soc_OCV_CorrectEn_Flag = 0; 
    /*
    //if(SocReg.min_cell_temp >= 0) // 20160131 - add for OCV tbl under different temp.
    if(SocReg.min_cell_temp_vlt < 757)  // 20160224 - add for OCV tbl under different temp.
    {
      for(i = 0; i < 11; i++)
      { 
        Soc2OcvTbl[i].volt = Soc2OcvTbl_0deg[i].volt; 
      }
    }
    else
    {
      for(i = 0; i < 11; i++)
      { 
        Soc2OcvTbl[i].volt = Soc2OcvTbl_N10deg[i].volt; 
      }
    }
    */
    if(SocReg.min_cell_vlt <= Soc2OcvTbl[0].volt)   // 防止超范围
    {
      soc = 0;
    }
    else if(SocReg.min_cell_vlt >= Soc2OcvTbl[10].volt)
    {
      soc = 100;
    }
    else
    {
      for(i = 1; i < 11; i++)     // 查表根据OCV 表取soc。
      {
        if(SocReg.min_cell_vlt == Soc2OcvTbl[i].volt)   // NOTE: 采用最低电压的一节电池来比较。
        {
          soc = Soc2OcvTbl[i].soc;
          break;
        }
        else if(SocReg.min_cell_vlt > Soc2OcvTbl[i-1].volt && SocReg.min_cell_vlt < Soc2OcvTbl[i].volt)
        {
          deltV = (uint16_t)(Soc2OcvTbl[i].soc - Soc2OcvTbl[i - 1].soc) * 1000 / (Soc2OcvTbl[i].volt - Soc2OcvTbl[i - 1].volt);
          soc = Soc2OcvTbl[i - 1].soc + (uint8_t)((SocReg.min_cell_vlt - Soc2OcvTbl[i - 1].volt) * deltV / 1000);
          break;
        } 
      }
      /*
      for(i = 1; i < 11; i++)     // 查表根据OCV 表取soc。
      {
        if(SocReg.min_cell_vlt == Soc2OcvTbl[i].volt)   // NOTE: 采用最低电压的一节电池来比较。
        {
          soc = Soc2OcvTbl[i].soc;
          break;
        }
        else if(SocReg.min_cell_vlt < Soc2OcvTbl[i].volt)
        {
          deltV = (Soc2OcvTbl[i].soc - Soc2OcvTbl[i - 1].soc) * 1000 / (Soc2OcvTbl[i].volt - Soc2OcvTbl[i - 1].volt);
          soc = Soc2OcvTbl[i - 1].soc + (SocReg.min_cell_vlt - Soc2OcvTbl[i - 1].volt) * deltV / 1000;
          break;
        }
      }*/
    }

    SocReg.soc = soc; //SocReg.ah = SocCalc.curAh; // 计算SOC。 
    SocCalc.curAh = ((uint32_t)SocReg.rated_cap * soc) / 100;
    SocReg.ah = SocCalc.curAh;
    SocCalc.soc_rt = SocReg.soc;
  }
}

/*  进行温度和额定容量的校正。
*
*   需要输入的参数:
*
*  1. SocReg.min_cell_temp  最低单节温度
*  2. SocReg.temp_corr      校正使能标志位
*
*   输出:
*
*  1. SocReg.rated_cap      修正后的额定容量
*/
void SOCCorrectTemp(void)
{
    int8_t minTemp;
    uint16_t deltT;
    uint8_t i;
    minTemp = SocReg.min_cell_temp;

    if(SocReg.temp_corr == 1)
    {
        SocReg.temp_corr = 0;

        if(minTemp <= Temp2CapacityTbl[0].temp)
        {SocReg.rated_cap = Temp2CapacityTbl[0].rated_cap;}
        else if(minTemp >= Temp2CapacityTbl[9].temp)
        {SocReg.rated_cap = Temp2CapacityTbl[9].rated_cap;}
        else
        {
            for(i = 1; i < 10; i++)
            {
                if(minTemp == Temp2CapacityTbl[i].temp)
                {
                    SocReg.rated_cap = Temp2CapacityTbl[i].rated_cap;
                    break;
                }
                else if(minTemp < Temp2CapacityTbl[i].temp)
                {
                    // 线性化再等分。
                    deltT = (Temp2CapacityTbl[i].rated_cap - Temp2CapacityTbl[i - 1].rated_cap)
                            / (Temp2CapacityTbl[i].temp - Temp2CapacityTbl[i - 1].temp);
                    SocReg.rated_cap = Temp2CapacityTbl[i - 1].rated_cap
                                       + (SocReg.min_cell_temp - Temp2CapacityTbl[i - 1].temp) * deltT;
                    break;
                }
            }
        }
    }
}

/************************************************************************/
/************************************************************************/
/************************************************************************/
/************************************************************************/
/************************************************************************/
/************************************************************************/

uint8_t smooth_cnt;
//sword smooth_delt_v;
uint32_t smooth_delt_v;
uint16_t old_v = 0;
uint16_t new_v = 0;

/*  10 秒内将SocReg.soc  值平滑变化到BMSRegs.soc_rt  值。
*/
void SOCSmooth(void)
{
    new_v = SocReg.soc * 10;

    //if((old_v != new_v)&&(abs(old_v-new_v)<10))
    if(old_v != new_v)
    {
        smooth_cnt++;

        if(smooth_cnt > 5) // 100ms=1 x 100ms
        {
            smooth_cnt = 0;

            if(old_v > new_v)
            {
                smooth_delt_v = (old_v - new_v) / 10;
                old_v -= smooth_delt_v;
            }
            else
            {
                smooth_delt_v = (new_v - old_v) / 10;
                old_v += smooth_delt_v;
            }

            if(smooth_delt_v == 0)
            { old_v = new_v; }
        }
    }
    else
    { smooth_cnt = 0; }

    SocCalc.soc_rt = (uint8_t)(old_v / 10);
}


