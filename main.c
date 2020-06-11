#include "iostm8s103f3.h"

/* 按鈕定義 */
#define SW_Set  PC_IDR_IDR3          
#define SW_inc  PC_IDR_IDR4
#define SW_dec  PC_IDR_IDR5
/* 七段顯示器定義 */
#define DEG_A   PD_ODR_ODR5
#define DEG_B   PA_ODR_ODR2
#define DEG_C   PC_ODR_ODR7
#define DEG_D   PD_ODR_ODR3
#define DEG_E   PD_ODR_ODR1
#define DEG_F   PA_ODR_ODR1
#define DEG_G   PC_ODR_ODR6
#define DEG_DP  PD_ODR_ODR2
#define DIG_1   PB_ODR_ODR4
#define DIG_2   PB_ODR_ODR5
#define DIG_3   PD_ODR_ODR4
/* 七段顯示器程式定義*/
#define SSD_SEG_A_BIT       0x20
#define SSD_SEG_B_BIT       0x04
#define SSD_SEG_C_BIT       0x80
#define SSD_SEG_D_BIT       0x08
#define SSD_SEG_E_BIT       0x02
#define SSD_SEG_F_BIT       0x02
#define SSD_SEG_G_BIT       0x40
#define SSD_SEG_P_BIT       0x04
/* ADC定義 */
#define ADC_RAW_TABLE_SIZE      sizeof rawAdc / sizeof rawAdc[0]    // 矩陣大小
#define ADC_RAW_TABLE_BASE_TEMP -100                                // 溫度偏移
//#define ADC_RAW_TABLE_BASE_TEMP -520                              // 溫度偏移
#define temp_offset 0                                               // 溫度補償
/* 資料型態定義 */
typedef unsigned char uint8_t;
typedef unsigned int  uint16_t;
typedef unsigned long uint32_t;
#define bool _Bool
#define true 1
#define false 0
#define u8  uint8_t
#define u16 uint16_t
#define u32 uint32_t
/* EEPROM位址定義 */
#define EEPROM_BASE_ADDR        0x4000
#define EEPROM_PARAMS_OFFSET    100
/* Relay定義 */
#define Relay   PA_ODR_ODR3

u8 i,j;

/* TIM4變數 */
u16 TIM4_2ms = 0;                // 2ms 
u8 TIM4_1sec = 0;                // 1sec
u8 TIM4_1min = 0;                // 1min
u8 TIM4_1hour = 0;               // 1hour
u8 TIM4_1day = 0;                // 1day
u16 Hot_OK_5min_timer = 0;        // 溫控完成，累加七段顯示器顯示時間
u16 Hot_OK_5min_counter = 0;     // 溫控完成，累加七段顯示器計數器

/*SW 變數 */
u8 debounce_timer = 20;           // 防彈跳時間，2ms*20=40ms
u8 SW_Set_debounce_timer = 0;    // Set防彈跳時間
u8 SW_inc_debounce_timer = 0;    // inc防彈跳時間
u8 SW_dec_debounce_timer = 0;    // dec防彈跳時間
u8 SW_inc_press_1timer = 0;      // inc按壓時間1秒  
u8 SW_inc_press_5timer = 0;      // 無按壓時間5秒
u16 SW_inc_press_1counter = 0;    // 按鈕按壓1秒計數器,2ms
u16 SW_inc_press_5counter = 0;    // 按鈕無按壓5秒計數器,2ms
u8 SW_flag = 0;    // 0x01=set,0x02=inc,0x04=dec,0x08=set_temp_time,0x10=溫度異常指標,0x20=按鈕釋放後才能再進入ModeId=1,0x40=按鈕釋放後再按按鈕才進入ModeId=0

/*ADC 變數 */
u16 ADC_result = 0;          // ADC 結果10bit
u32 ADC_averaged = 0;        // ADC 平均
int Temp_val;                // 真實溫度
const u16 rawAdc[] = {
    685, 675, 664, 654, 644, 633, 623, 612, 601, 591,
    580, 570, 559, 549, 538, 528, 518, 507, 497, 487, 
    477, 467, 457, 448, 438, 429, 419, 410, 401, 392, 
    383, 375, 366, 358, 349, 341, 333, 326, 318, 310, 
    303, 296, 289, 282, 275, 269, 262, 256, 250, 244, 
    238, 232, 226, 221, 215, 210, 205, 200, 195, 191, 
    186, 181, 177, 173, 169, 165, 161, 157, 153, 149, 
    146, 142, 139, 136, 132, 129, 126, 123, 120, 117, 
    115, 112, 109, 107, 104, 102, 100, 97, 95, 93, 
    91, 89, 87, 85, 83, 81, 79, 78, 76, 74, 
    73, 71, 69, 68, 67, 65, 64, 62, 61, 60, 
    58, 57, 56, 55, 54, 53, 52, 51, 49, 48, 
    47,
};    // -10~110度

/*七段顯示器變數 */
u8 activeDigitId=0;        // 三個七段顯示器ID
u8 displayAC[3];           
u8 displayD[3];
bool displayOff;
u8 stringBuffer[7];

/*Relay變數 */
bool Relay_state;

/*參數變數 */
u8 modeId = 0;       // =0(待機),=1(Power),=2(溫度控制),=3(溫控完成),=4(高低溫異常),=5(加熱異常),=6(初始與設定溫度異常)
u8 tempId = 0;       // 溫度/時間參數指標
int set_temp;        // 設定溫度
u16 set_time;        // 設定時間
u8 temp_Hys = 10;        // 遲滯溫度,單位0.1度
u16 temp_high = 1100;   // 高溫異常
int temp_low = -100;    // 低溫異常
const int tempCache[] = {750,850,990,360,500,600,700,800,900,1000};      // 溫度矩陣,單位0.1度
const int timeCache[] = {60,45,30,15,14,15,16,17,18,19};                 // 時間矩陣,單位1秒
int init_temp = 0;                                                       // 溫度控制啟動初始溫度
u16 check_sec_counter = 0;                                               // 溫度異常時間計數器
u16 check_sec_timer = 0;                                                  // 溫度異常時間
u16 check_5_sec_timer = 300;    // 設定X分鐘溫度上升檢查時間,單位1秒
u8 check_5_min_temp = 50;     // 設定X分鐘溫度上升檢查X溫度,單位0.1度
u16 check_hot_ok_timer = 0;    // 溫度加熱的一半時間

/*測試變數*/
u8 Test_Set=0,Test_inc=0,Test_dec=0;

/* 副程式 */
void GPIO_init(void);
void CLK_init(void);
void EEPROM_init(void);
void EEPROM_lock(void);
void EEPROM_write(u8 addr, u8 dat);
u8 EEPROM_read(u8 addr);
void Init_Timer4(void);
void ADC_Init(void);
void getTemperature();
void setDigit (u8 id, u8 val, bool dot);
void enableDigit (u8 id);
void setDisplayStr (u8* val);
void setDisplayOff (bool val);
void itofpa (int val, u8* str, u8 pointPosition);
void refreshDisplay();
void initRelay();
void setRelay (bool on);
void refreshRelay();

void delay(u16 count);

#pragma vector=0x07        // PC中斷向量
__interrupt void EXTI2_PC(void)
{
    SW_inc_press_1counter = 0;        // 按鈕按下，清除按下計數
    SW_inc_press_5counter = 0;        // 按鈕按下，清除按下計數
    SW_inc_press_1timer = 0;         // 按鈕按下，清除按下1 sec時間
    SW_inc_press_5timer = 0;         // 按鈕按下，清除按下5 sec時間
    if (SW_Set == 0)    // 按鈕按下?
    {
       SW_Set_debounce_timer = debounce_timer;   // 設定防彈跳時間
       if ( (SW_flag & 0x01 ) == 0)              // 判斷按鈕旗標(無按下旗標)
       {
           SW_flag |= 0x01;                      // 設定按鈕旗標
           Test_Set++; 
       }
    }
    if (SW_inc == 0)
    {
       SW_inc_debounce_timer = debounce_timer;
       if ( (SW_flag & 0x02 ) == 0)
       {
           SW_flag |= 0x02;
           if ((modeId == 3) || (modeId == 4) || (modeId == 5) || (modeId == 6)) modeId = 0;    //=3(溫控完成),=4(高低溫異常),=5(加熱異常),=6(初始與設定溫度異常)，按按鈕確認後回待機
           Test_inc++; 
       }
    }

    if (SW_dec == 0)
    {  
       SW_dec_debounce_timer = debounce_timer;
       if ( (SW_flag & 0x04 ) == 0)
       {
           SW_flag |= 0x04;
           if (modeId == 1)                        // Power模式才可調參數
           {
             if (tempId < 3) tempId ++;            // 參數累加
             else tempId = 0; 
             set_temp = tempCache[tempId];         // 設定溫度
             set_time = timeCache[tempId];         // 設定時間
             check_hot_ok_timer = ((set_time *60 ) >> 1);   // 設定時間一半為確認時間
           }
           if (modeId == 2)                        // 溫度控制模式選擇要顯示偵測溫度或剩餘加熱時間
           {
              if (SW_flag & 0x08) SW_flag &= 0xF7;
              else SW_flag |= 0x08;
           }
           Test_dec++; 
       }
    }
}

#pragma vector = 0x18    //0x18
__interrupt void ADC1_EOC_IRQHandler(void)
{
  ADC_result = ADC_DRH << 2;
  ADC_result |= ADC_DRL;
  ADC_CSR &= 0x7F;     // reset EOC
  ADC_averaged = ADC_result << 4;
//  if (ADC_averaged == 0) ADC_averaged = ADC_result << 4;
//  else ADC_averaged += ADC_result - (ADC_averaged >> 4);
}

#pragma vector = 0x19    //0x19
__interrupt void TIM4_OVR_IRQHandler(void)
{
    TIM4_SR=0x00;
    TIM4_2ms++;
    if(TIM4_2ms>=500)    //2ms*500=1s
    {
      TIM4_2ms = 0;
      if (modeId == 2)                         // 溫度控制模式再更新秒/分/時/日計數
      {  
        TIM4_1sec++;
        if (TIM4_1sec>=60)  //1min=60sec
        {
          TIM4_1sec = 0;
          TIM4_1min++;

          if (set_time > 1) set_time--;          // 加熱時間倒數，時間到進入溫控完成
          else
          {
            modeId = 3;
            SW_flag &= 0xF7;                     // 溫度偵測模式
            Hot_OK_5min_counter = 0;      // 清除5分鐘計數器
            Hot_OK_5min_timer = 0;         // 清除5分鐘時間
          }
          
        }
/* 
        if (TIM4_1min>=60)  //1hour=60min
        {
          TIM4_1min = 0;
          TIM4_1hour++;
        }
        if (TIM4_1hour>=24) //1day=24hour
        {
          TIM4_1hour = 0;
          TIM4_1day++;
        }
*/
      }
    }
      
    if ((TIM4_2ms&0xF)==1) ;
    else if ((TIM4_2ms&0xF)==2) ADC_CR1 |= 0x01;     // 2ms*16=32ms，ADC轉換 
    else if ((TIM4_2ms&0xF)==3) refreshRelay();      // 2ms*16=32ms，Relay狀態更新
    else if ((TIM4_2ms&0xFF)==4) getTemperature();   // 溫度轉換(數位對應溫度)
    refreshDisplay();                                // 2ms更新7短顯示器

// 防彈跳時間成立才判斷按鈕是否釋放    
    if (SW_Set_debounce_timer> 0)                           // 是否還在防彈跳時間
    {
        if ( SW_Set == 1)                                   // 按鈕是否釋放
        {
           SW_Set_debounce_timer--;                         // 防彈跳時間-1
           if (SW_Set_debounce_timer == 0) SW_flag &= 0xFE; // 防彈跳時間按鈕都為釋放
        }
    }
    if (SW_inc_debounce_timer> 0)             // 按鈕須釋放才能清除按鈕記憶旗標
    {
        if ( SW_inc == 1)
        {
           SW_inc_debounce_timer--;
           if (SW_inc_debounce_timer == 0) 
           {
               SW_flag &= 0xFD;                    
               SW_flag &= 0xDF;                     //  清除Power與溫控模式按鈕取消旗標,避免按鈕長按時待機與Power重複執行(進入)
               SW_flag &= 0xBF;                     //  清除按鈕記憶旗標，避免按鈕常按時從待機到Power又到待機
           }
        }
    }
    if (SW_dec_debounce_timer> 0)
    {
        if ( SW_dec == 1)
        {
           SW_dec_debounce_timer--;
           if (SW_dec_debounce_timer == 0) SW_flag &= 0xFB;
        }
    }

    if ( modeId == 0)          // 待機模式，確認按鈕按下是否持續1 sec，是的話進入Power模式，否則清除計數器與時間
    {
      if ( SW_inc == 0)
      {
           if ((SW_flag & 0x20) == 0)           // 確認設定按紐需釋放後才能再進入Power模式
           {
         
                SW_inc_press_1counter++;
                if (SW_inc_press_1counter >= 500)
                {
                   SW_inc_press_1counter = 0;
                   SW_inc_press_1timer = 1;
                   modeId = 1;
                   SW_flag |= 0x40;                 // 由待機模式常按按鈕進入Power模式，設置旗標，須按鈕釋放才能清除
                }

           }
      }
      else 
      {
         SW_inc_press_1counter = 0;
         SW_inc_press_1timer = 0;      
      }
    }
    if ( modeId == 1)          // Power模式，按鈕無按壓時間計數器與時間是否5 sec，是的話進入溫度控制模式，將溫度計數累加時間歸零，讀取初始溫度，溫度確認時間歸零
    {
        SW_inc_press_5counter++;
        if (SW_inc_press_5counter >= 500)
        {
           SW_inc_press_5counter = 0;
           SW_inc_press_5timer ++;
           if (SW_inc_press_5timer >= 5)
           {
 //             SW_inc_press_5timer = 5 ;
              TIM4_1sec = 0;
              TIM4_1min = 0;
              TIM4_1hour = 0;
              TIM4_1day = 0;
              getTemperature();                                // 溫度轉換(數位對應溫度)
              init_temp = Temp_val;
              check_sec_counter = 0;
              check_sec_timer = 0;
              if (init_temp >= set_temp) modeId = 6;       // 初始溫度大於設定溫度，進入異常
              else modeId = 2;
           }
        } 
        if ((SW_inc == 0) || (SW_dec == 0))                // 按鈕都釋放才開始計數5 sec
        {
            SW_inc_press_5counter = 0;
            SW_inc_press_5timer = 0;
        }
    }
    if (( modeId == 1) || ( modeId == 2))                      // Power或溫度控制模式偵測按鈕是否按下1 Sec，是的話進入待機模式
    {
//      if ( SW_inc == 0) 
      if (( SW_inc == 0) && ((SW_flag & 0x40) == 0))      //  按鈕持續按下1 sec 且按鈕前一狀態須曾經釋放過,避免待機進入Power後再進入待機
      {
        SW_inc_press_1counter++;
        if (SW_inc_press_1counter >= 500)
        {
           modeId = 0;
           SW_flag  |= 0x20;                                           // 設定Power與溫控模式按鈕取消旗標
        }
      }
    }
    if ( modeId == 2)                                                   //  溫控模式,第一階段確認是否5分鐘內溫升5度,第二階段確認設定加熱時間一半到達設定溫度
    {
      check_sec_counter++;
      if (check_sec_counter >= 500)
      {
        check_sec_counter = 0;
        check_sec_timer ++;
      }
/* 判斷五分鐘內有無升5度 */      
      if ((check_sec_timer <= check_5_sec_timer) && ((SW_flag & 0x10) == 0))
      {
//        if ((init_temp + check_5_min_temp) < Temp_val) SW_flag |= 0x10;             // 5分鐘內有增加5度或到達設定溫度,設定旗標
       if (((init_temp + check_5_min_temp) < Temp_val) || (set_temp <= Temp_val))  SW_flag |= 0x10;             // 5分鐘內有增加5度或到達設定溫度,設定旗標
      }            
      if ((check_sec_timer == check_5_sec_timer) && ((SW_flag & 0x10) == 0)) modeId = 5;
/* 清除5分鐘內到達5度旗標，再進行一半時間有無到達設定溫度判別 */
      if (check_sec_timer == (check_5_sec_timer +1)) SW_flag &= 0xEF;
/* 判斷一半時間內有無到達設定溫度 */
      if (((check_sec_timer) <= check_hot_ok_timer) && ((SW_flag & 0x10) == 0))
      {
        if (set_temp <= Temp_val) SW_flag |= 0x10;                                                 // 一半時間內到達設定溫度,設定旗標
      }            
      if (((check_sec_timer) == check_hot_ok_timer) && ((SW_flag & 0x10) == 0)) modeId = 5;
      
    }

    if (modeId == 3)                        // 溫控完成,5分鐘顯示FFF,時間到進入待機模式
    {
       Hot_OK_5min_counter++;
       if (Hot_OK_5min_counter >= 500)
       {
          Hot_OK_5min_counter = 0;
          Hot_OK_5min_timer ++;
          if (Hot_OK_5min_timer >= 300)
          {
             Hot_OK_5min_timer = 300;
             modeId = 0;
          }
       }
     }

    
}

int main( void )
{
  asm("sim");     //關閉全局中斷
  GPIO_init();  
  CLK_init();
  Init_Timer4();
  asm("rim");     //開啟全局中斷
  ADC_Init();
  initRelay();
  
  /*
  EEPROM_init();
  EEPROM_write(0x00,0x55);
  EEPROM_write(0x00,0xAA);
  EEPROM_write(0x01,0x33);
  EEPROM_lock();
  EEPROM_write(0x00,0x77);
  
  EE_Data[0] = EEPROM_read(0x00);
  EE_Data[1] = EEPROM_read(0x01);
*/
 
   while(1)
  {
     if (modeId == 0)                 // 待機模式
     {
       setDisplayOff(1);    // 電源上電，LED OFF
       tempId = 0;
       set_temp = tempCache[tempId];
       set_time = timeCache[tempId];
       SW_flag &= 0xF7;                                           //  設定初始顯示溫度
       SW_flag &= 0xEF;                                          //  清除溫度異常旗標
       check_hot_ok_timer = ((set_time *60 ) >> 1);   // 設定時間一半為確認時間
     }
     else if (modeId == 1)      // Power模式
     {
       itofpa (tempCache[tempId],(u8*) stringBuffer, 0);   // 溫度顯示三位數,最後一位為小數點
       setDisplayStr ( (u8*) stringBuffer);
       setDisplayOff(0);
     }
     else if (modeId == 2)     // 溫度控制模式 
     {
//        getTemperature();                                // 溫度轉換(數位對應溫度)
        if (Temp_val <= temp_low )              //  偵測溫度異常,到達高低溫限制值
        {
          setDisplayStr ("LLL");
          modeId = 4;
        } 
        else if (Temp_val >= temp_high ) 
        {
          setDisplayStr ("HHH");
          modeId = 4;
        }
        else                                                    // 顯示偵測溫度或剩餘時間
        {
          if ((SW_flag & 0x08) == 0)
          {
             itofpa (Temp_val, (u8*) stringBuffer, 0);   
             setDisplayStr ( (u8*) stringBuffer);
          }
          else 
          {
             itofpa (set_time, (u8*) stringBuffer, 6);   
             setDisplayStr ( (u8*) stringBuffer);
          }
        }
     }
     else if (modeId == 3)                   // 溫控完成,顯示FFF
     {
        setDisplayStr ("FFF");
     }
     else if (modeId == 5)                  //  加熱異常,顯示EEE
     {
        setDisplayStr ("EEE");
     }
     else if (modeId == 6)                  // 初始與設定溫度異常,顯示UUU
     {
        setDisplayStr ("UUU");
     }
  }
}

void GPIO_init(void)
{
  PD_DDR_DDR5 = 1;       //PD5輸出，DEG_A  
  PD_CR1_C15 = 1;        //PD5推挽
  PA_DDR_DDR2 = 1;       //PA2輸出，DEG_B
  PA_CR1_C12 = 1;        //PA2推挽
  PC_DDR_DDR7 = 1;       //PC7輸出，DEG_C
  PC_CR1_C17 = 1;        //PC7推挽
  PD_DDR_DDR3 = 1;       //PD3輸出，DEG_D
  PD_CR1_C13 = 1;        //PD3推挽
  PD_DDR_DDR1 = 1;       //PD1輸出，DEG_E
  PD_CR1_C11 = 1;        //PD1推挽
  PA_DDR_DDR1 = 1;       //PA1輸出，DEG_F
  PA_CR1_C11 = 1;        //PA1推挽
  PC_DDR_DDR6 = 1;       //PC6輸出，DEG_G
  PC_CR1_C16 = 1;        //PC6推挽
  PD_DDR_DDR2 = 1;       //PD2輸出，DEG_DP
  PD_CR1_C12 = 1;        //PD2推挽
  
  PD_DDR_DDR4 = 1;       //PD4輸出，DIG_1
  PD_CR1_C14 = 1;        //PD4推挽
  PB_DDR_DDR5 = 1;       //PB5輸出，DIG_2
  PB_CR1_C15 = 1;        //PB5推挽
  PB_DDR_DDR4 = 1;       //PB4輸出，DIG_3
  PB_CR1_C14 = 1;        //PB4推挽 

  PC_DDR_DDR3 = 0;    //PC3輸入
  PC_DDR_DDR4 = 0;    //PC4輸入
  PC_DDR_DDR5 = 0;    //PC5輸入
  
  PC_CR1_C13 = 1;     //PC3上拉
  PC_CR1_C14 = 1;     //PC4上拉
  PC_CR1_C15 = 1;     //PC5上拉
    
  PC_CR2_C23 = 1;     //PC3中斷
  PC_CR2_C24 = 1;     //PC4中斷
  PC_CR2_C25 = 1;     //PC5中斷

  EXTI_CR1_PCIS = 2;    //PC負源觸發
}


void EEPROM_init(void)
{
    FLASH_CR1=0x00;
    FLASH_CR2=0x00;
}

void EEPROM_lock(void) 
{
    FLASH_IAPSR=(u8)(~0x08);            //重新上鎖
}

void EEPROM_write(u8 addr, u8 dat)      //先解鎖EEPROM再寫入EEPROM資料
{
   do 
  {
      FLASH_DUKR = 0xAE;	// 輸入第一密碼
      FLASH_DUKR = 0x56;	// 輸入第二密碼
  } while((FLASH_IAPSR & 0x08) == 0);	// 若解鎖未成功，重新再鎖
  u8  *p;                                                           //位址旗標
  p=(u8  *)(EEPROM_BASE_ADDR + addr);      //位址定義
  *p=dat;                                                         //資料寫入EEPROM
  while(!(FLASH_IAPSR&0x04));                 //等待寫入完成
}

u8 EEPROM_read(u8 addr)              //讀取
{
  u8 *p;
  p=( u8  *)(EEPROM_BASE_ADDR + addr); 
  return *p; 
}

void CLK_init(void)
{
  CLK_CKDIVR = 0x00;  //內震16M，內頻16M，0x00=16M，0x08=8M，0x10=4M，0x18=2M
}

void Init_Timer4(void)
{
  //128分頻 250計數，在16MHz下是2ms一次中斷！16M/(128*(249+1))
  TIM4_CR1=0x00;        //關閉計數器
  TIM4_IER=0x01;        //更新中斷使能
  TIM4_EGR=0x01;
  TIM4_CNTR=249;      //計數器值
  TIM4_ARR=249;        //自動重裝的值
  TIM4_PSCR=0x07;    //分頻值 
  TIM4_CR1=0x01;      //使能計數器
}

void ADC_Init(void)
{
  /*PD6*/
  PD_DDR &=0xBF;                 // PD6輸入模式
  PD_CR1 &=0xBF;                 // PD6浮空輸入
  PD_CR2 &=0xBF;                 // PD6禁止外部中斷
  CLK_PCKENR2 |= 0x08;           // ADC時鐘，默認開啟，可省略
  ADC_CR1 = 0x70;                // ADC時鐘16MHz/18，單次轉換，關閉ADC 
  ADC_CR2 = 0x00;                // 數據左對齊，禁止外部触發，禁止掃描模式 
  ADC_CSR = 0x26;                // 選擇通道6 AIN6   默?如此
  ADC_CR1 |= 0x01;               // Power up ADC
}

u16 Get_ADC_Result(void)
{
  ADC_CR1 |= 0x01;                                   // 啟動ADC ，啟動ADC電源
  delay(10);                                         // 延遲1ms 等待ADC啟動
  ADC_CR1 |= 0x01;                                   // 啟動ADC轉換
  while((ADC_CSR&0x80) == 0);                        // 等待ADC轉換結束
  ADC_result = ADC_DRH << 2;
  ADC_result |= ADC_DRL;
  if (ADC_averaged == 0) ADC_averaged = ADC_result << 4;
  else ADC_averaged += ADC_result - (ADC_averaged >> 4);
  return ADC_averaged;                                   // 返回10位ADCr結果
}

void getTemperature()   // 使用二分法搜尋最接近的高低數值，並做小數點計算，將整數位乘10做小數點位移。
{
      u16 val = ADC_averaged >> 4;
      u8 rightBound = ADC_RAW_TABLE_SIZE;
      u8 leftBound = 0;
    // search through the rawAdc lookup table
    while ( (rightBound - leftBound) > 1) {
        u8 midId = (leftBound + rightBound) >> 1;

        if (val > rawAdc[midId]) {
            rightBound = midId;
        } else {
            leftBound = midId;
        }
    }
    // reusing the "val" for storing an intermediate result
    if (val >= rawAdc[leftBound]) {
        val = leftBound * 10;
    } else {
        val = (rightBound * 10) - ( (val - rawAdc[rightBound]) * 10)
              / (rawAdc[leftBound] - rawAdc[rightBound]);
    }
//    Temp_val = val + temp_offset;
    Temp_val = ADC_RAW_TABLE_BASE_TEMP + val + temp_offset;
    // Final calculation and correction
}

void setDigit (u8 id, u8 val, bool dot)    //  將三位數值填入三個七段顯示器對應LED矩陣
{
  if (id > 2) return;

  switch (val) {
  case '-':
        displayAC[id] = SSD_SEG_G_BIT;
        displayD[id] = 0;
      break;

    case ' ':
        displayAC[id] = 0;
        displayD[id] = 0;
      break;

    case '0':
        displayAC[id] = SSD_SEG_B_BIT | SSD_SEG_F_BIT | SSD_SEG_C_BIT;
        displayD[id] = SSD_SEG_A_BIT | SSD_SEG_D_BIT | SSD_SEG_E_BIT;
      break;

    case '1':
        displayAC[id] = SSD_SEG_B_BIT | SSD_SEG_C_BIT;
        displayD[id] = 0;
        break;

    case '2':
        displayAC[id] = SSD_SEG_B_BIT | SSD_SEG_G_BIT;
        displayD[id] = SSD_SEG_A_BIT | SSD_SEG_D_BIT | SSD_SEG_E_BIT;
        break;

    case '3':
        displayAC[id] = SSD_SEG_B_BIT | SSD_SEG_C_BIT | SSD_SEG_G_BIT;
        displayD[id] = SSD_SEG_A_BIT | SSD_SEG_D_BIT;
        break;

    case '4':
        displayAC[id] = SSD_SEG_B_BIT | SSD_SEG_C_BIT | SSD_SEG_F_BIT | SSD_SEG_G_BIT;
        displayD[id] = 0;
        break;

    case '5':
        displayAC[id] = SSD_SEG_C_BIT | SSD_SEG_F_BIT | SSD_SEG_G_BIT;
        displayD[id] = SSD_SEG_A_BIT | SSD_SEG_D_BIT;
        break;

    case '6':
        displayAC[id] = SSD_SEG_C_BIT | SSD_SEG_F_BIT | SSD_SEG_G_BIT;
        displayD[id] = SSD_SEG_A_BIT | SSD_SEG_D_BIT | SSD_SEG_E_BIT;
        break;

    case '7':
        displayAC[id] = SSD_SEG_B_BIT | SSD_SEG_C_BIT;
        displayD[id] = SSD_SEG_A_BIT;
        break;

    case '8':
        displayAC[id] = SSD_SEG_B_BIT | SSD_SEG_C_BIT | SSD_SEG_F_BIT | SSD_SEG_G_BIT;
        displayD[id] = SSD_SEG_A_BIT | SSD_SEG_D_BIT | SSD_SEG_E_BIT;
        break;

    case '9':
        displayAC[id] = SSD_SEG_B_BIT | SSD_SEG_C_BIT | SSD_SEG_F_BIT | SSD_SEG_G_BIT;
        displayD[id] = SSD_SEG_A_BIT | SSD_SEG_D_BIT;
        break;

    case 'A':
        displayAC[id] = SSD_SEG_B_BIT | SSD_SEG_C_BIT | SSD_SEG_F_BIT | SSD_SEG_G_BIT;
        displayD[id] = SSD_SEG_A_BIT | SSD_SEG_E_BIT;
        break;

    case 'B':
        displayAC[id] = SSD_SEG_C_BIT | SSD_SEG_F_BIT | SSD_SEG_G_BIT;
        displayD[id] = SSD_SEG_D_BIT | SSD_SEG_E_BIT;
        break;

    case 'C':
        displayAC[id] = SSD_SEG_F_BIT;
        displayD[id] = SSD_SEG_A_BIT | SSD_SEG_D_BIT | SSD_SEG_E_BIT;
        break;

    case 'D':
        displayAC[id] = SSD_SEG_B_BIT | SSD_SEG_C_BIT | SSD_SEG_G_BIT;
        displayD[id] = SSD_SEG_D_BIT | SSD_SEG_E_BIT;
        break;

    case 'E':
        displayAC[id] = SSD_SEG_F_BIT | SSD_SEG_G_BIT;
        displayD[id] = SSD_SEG_A_BIT | SSD_SEG_D_BIT | SSD_SEG_E_BIT;
        break;

    case 'F':
        displayAC[id] = SSD_SEG_F_BIT | SSD_SEG_G_BIT;
        displayD[id] = SSD_SEG_A_BIT | SSD_SEG_E_BIT;
        break;

    case 'H':
        displayAC[id] = SSD_SEG_B_BIT | SSD_SEG_C_BIT | SSD_SEG_F_BIT | SSD_SEG_G_BIT;
        displayD[id] = SSD_SEG_E_BIT;
        break;

    case 'L':
        displayAC[id] = SSD_SEG_F_BIT;
        displayD[id] = SSD_SEG_D_BIT | SSD_SEG_E_BIT;
        break;

    case 'N':
        displayAC[id] = SSD_SEG_B_BIT | SSD_SEG_F_BIT | SSD_SEG_C_BIT;
        displayD[id] = SSD_SEG_A_BIT | SSD_SEG_E_BIT;
        break;

    case 'O':
        displayAC[id] = SSD_SEG_B_BIT | SSD_SEG_F_BIT | SSD_SEG_C_BIT;
        displayD[id] = SSD_SEG_A_BIT | SSD_SEG_D_BIT | SSD_SEG_E_BIT;
        break;

    case 'P':
        displayAC[id] = SSD_SEG_B_BIT | SSD_SEG_F_BIT | SSD_SEG_G_BIT;
        displayD[id] = SSD_SEG_A_BIT | SSD_SEG_E_BIT;
        break;

    case 'R':
        displayAC[id] = SSD_SEG_F_BIT;
        displayD[id] = SSD_SEG_A_BIT | SSD_SEG_E_BIT;
        break;

    case 'U':
        displayAC[id] = SSD_SEG_B_BIT | SSD_SEG_F_BIT | SSD_SEG_C_BIT;
        displayD[id] = SSD_SEG_D_BIT | SSD_SEG_E_BIT;
        break;
        
    default:
        displayAC[id] = 0;
        displayD[id] = SSD_SEG_D_BIT;
    }

    if (dot) {
        displayD[id] |= SSD_SEG_P_BIT;
    } else {
        displayD[id] &= ~SSD_SEG_P_BIT;
    }
  return;
}

void enableDigit (u8 id)      //  決定哪一個七段顯示器顯示
{
    switch (id) {
    case 0:
        DIG_1 = 0;
        DIG_2 = 1;
        DIG_3 = 1;
        break;

    case 1:
        DIG_1 = 1;
        DIG_2 = 0;
        DIG_3 = 1;
        break;

    case 2:
        DIG_1 = 1;
        DIG_2 = 1;
        DIG_3 = 0;
        break;

    default:
        DIG_1 = 1;
        DIG_2 = 1;
        DIG_3 = 1;
        break;
    }
}

void setDisplayStr (u8* val)      //  決定小數點在哪一個七段顯示器，與轉製矩陣
{
    u8 i, d;
    // get number of display digit(s) required to show given string.
    for (i = 0, d = 0; * (val + i) != 0; i++, d++) {
        if (* (val + i) == '.' && i > 0 && * (val + i - 1) != '.') d--;
    }

    // at this point d = required digits
    // but SSD have 3 digits only. So rest is doesn't matters.
    if (d > 3) {
        d = 3;
    }

    // disable the digit if it is not needed.
    for (i = 3 - d; i > 0; i--) {
        setDigit (3 - i, ' ', false);
    }

    // set values for digits.
    for (i = 0; d != 0 && *val + i != 0; i++, d--) {
        if (* (val + i + 1) == '.') {
            setDigit (d - 1, * (val + i), true);
            i++;
        } else {
            setDigit (d - 1, * (val + i), false);
        }
    }
  
}
 
void itofpa (int val, u8* str, u8 pointPosition)   // 將數值轉換為ASC2碼
{
    u8 i, l,buffer[] = {0, 0, 0, 0, 0, 0};
    bool minus = false;

    // No calculation is required for zero value
    if (val == 0) {
        ( (u8*) str) [0] = '0';
        ( (u8*) str) [1] = 0;
        return;
    }

    // Correction for processing of negative value
    if (val < 0) {
        minus = true;
        val = -val;
    }

    // Forming the reverse string
    for (i = 0; val != 0; i++) {
        buffer[i] = '0' + (val % 10);

        if (i == pointPosition) {
            i++;
            buffer[i] = '.';
        }

        val /= 10;
    }

    // Add leading '0' in case of ".x" result
    if (buffer[i - 1] == '.') {
        buffer[i] = '0';
        i++;
    }

    // Add '-' sign for negative values
    if (minus) {
        buffer[i] = '-';
        i++;
    }

    // Reversing to get the result string
    for (l = i; i > 0; i--) {
        ( (u8*) str) [l - i] = buffer[i - 1];
    }

    // Put null at the end of string
    ( (u8*) str) [l] = 0;

}

void setDisplayOff (bool val)
{
    displayOff = val;
}

void refreshDisplay()
{
    enableDigit (3);

    if (displayOff) {
        return;
    }

    PA_ODR &= ~0x06;
    PA_ODR |= displayAC[activeDigitId] & 0x06;
    PC_ODR &= ~0xC0;
    PC_ODR |= displayAC[activeDigitId] & 0xC0;
    PD_ODR &= ~0x2E;
    PD_ODR |= displayD[activeDigitId];
    enableDigit (activeDigitId); 

    if (activeDigitId > 1) {
        activeDigitId = 0;
    } else {
        activeDigitId++;
    }
}

void initRelay()
{
    PA_DDR_DDR3 = 1;       //PA3輸出，Relay
    PA_CR1_C13 = 1;            //PA3推挽
    Relay_state = false;
}

void setRelay (bool on)
{
    if (on) Relay = 1;
    else Relay = 0;
}

void refreshRelay()
{
    if ( modeId == 2 )
    {
       if (Relay_state)
       {
         if ( Temp_val > (tempCache[tempId] + temp_Hys ))
         {
            Relay_state = 0;
            setRelay(Relay_state);
         }
       }
       else
       {
         if ( Temp_val < (tempCache[tempId] - temp_Hys ))
         {
            Relay_state = 1;
            setRelay(Relay_state);
         }
       }
    }
    else 
    {
        Relay_state = 0;
        setRelay(Relay_state);
    }
}
/*
void delay(u16 count)
{
  while(count--)
  {
    for (i=0;i<50;i++)
      for (j=0;j<20;j++);
  }
}
*/