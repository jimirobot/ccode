#include "iostm8s103f3.h"

/* ���s�w�q */
#define SW_Set  PC_IDR_IDR3          
#define SW_inc  PC_IDR_IDR4
#define SW_dec  PC_IDR_IDR5
/* �C�q��ܾ��w�q */
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
/* �C�q��ܾ��{���w�q*/
#define SSD_SEG_A_BIT       0x20
#define SSD_SEG_B_BIT       0x04
#define SSD_SEG_C_BIT       0x80
#define SSD_SEG_D_BIT       0x08
#define SSD_SEG_E_BIT       0x02
#define SSD_SEG_F_BIT       0x02
#define SSD_SEG_G_BIT       0x40
#define SSD_SEG_P_BIT       0x04
/* ADC�w�q */
#define ADC_RAW_TABLE_SIZE      sizeof rawAdc / sizeof rawAdc[0]    // �x�}�j�p
#define ADC_RAW_TABLE_BASE_TEMP -100                                // �ūװ���
//#define ADC_RAW_TABLE_BASE_TEMP -520                              // �ūװ���
#define temp_offset 0                                               // �ū׸��v
/* ��ƫ��A�w�q */
typedef unsigned char uint8_t;
typedef unsigned int  uint16_t;
typedef unsigned long uint32_t;
#define bool _Bool
#define true 1
#define false 0
#define u8  uint8_t
#define u16 uint16_t
#define u32 uint32_t
/* EEPROM��}�w�q */
#define EEPROM_BASE_ADDR        0x4000
#define EEPROM_PARAMS_OFFSET    100
/* Relay�w�q */
#define Relay   PA_ODR_ODR3

u8 i,j;

/* TIM4�ܼ� */
u16 TIM4_2ms = 0;                // 2ms 
u8 TIM4_1sec = 0;                // 1sec
u8 TIM4_1min = 0;                // 1min
u8 TIM4_1hour = 0;               // 1hour
u8 TIM4_1day = 0;                // 1day
u16 Hot_OK_5min_timer = 0;        // �ű������A�֥[�C�q��ܾ���ܮɶ�
u16 Hot_OK_5min_counter = 0;     // �ű������A�֥[�C�q��ܾ��p�ƾ�

/*SW �ܼ� */
u8 debounce_timer = 20;           // ���u���ɶ��A2ms*20=40ms
u8 SW_Set_debounce_timer = 0;    // Set���u���ɶ�
u8 SW_inc_debounce_timer = 0;    // inc���u���ɶ�
u8 SW_dec_debounce_timer = 0;    // dec���u���ɶ�
u8 SW_inc_press_1timer = 0;      // inc�����ɶ�1��  
u8 SW_inc_press_5timer = 0;      // �L�����ɶ�5��
u16 SW_inc_press_1counter = 0;    // ���s����1��p�ƾ�,2ms
u16 SW_inc_press_5counter = 0;    // ���s�L����5��p�ƾ�,2ms
u8 SW_flag = 0;    // 0x01=set,0x02=inc,0x04=dec,0x08=set_temp_time,0x10=�ūײ��`����,0x20=���s�����~��A�i�JModeId=1,0x40=���s�����A�����s�~�i�JModeId=0

/*ADC �ܼ� */
u16 ADC_result = 0;          // ADC ���G10bit
u32 ADC_averaged = 0;        // ADC ����
int Temp_val;                // �u��ū�
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
};    // -10~110��

/*�C�q��ܾ��ܼ� */
u8 activeDigitId=0;        // �T�ӤC�q��ܾ�ID
u8 displayAC[3];           
u8 displayD[3];
bool displayOff;
u8 stringBuffer[7];

/*Relay�ܼ� */
bool Relay_state;

/*�Ѽ��ܼ� */
u8 modeId = 0;       // =0(�ݾ�),=1(Power),=2(�ūױ���),=3(�ű�����),=4(���C�Ų��`),=5(�[�����`),=6(��l�P�]�w�ūײ��`)
u8 tempId = 0;       // �ū�/�ɶ��Ѽƫ���
int set_temp;        // �]�w�ū�
u16 set_time;        // �]�w�ɶ�
u8 temp_Hys = 10;        // �𺢷ū�,���0.1��
u16 temp_high = 1100;   // ���Ų��`
int temp_low = -100;    // �C�Ų��`
const int tempCache[] = {750,850,990,360,500,600,700,800,900,1000};      // �ūׯx�},���0.1��
const int timeCache[] = {60,45,30,15,14,15,16,17,18,19};                 // �ɶ��x�},���1��
int init_temp = 0;                                                       // �ūױ���Ұʪ�l�ū�
u16 check_sec_counter = 0;                                               // �ūײ��`�ɶ��p�ƾ�
u16 check_sec_timer = 0;                                                  // �ūײ��`�ɶ�
u16 check_5_sec_timer = 300;    // �]�wX�����ūפW���ˬd�ɶ�,���1��
u8 check_5_min_temp = 50;     // �]�wX�����ūפW���ˬdX�ū�,���0.1��
u16 check_hot_ok_timer = 0;    // �ūץ[�����@�b�ɶ�

/*�����ܼ�*/
u8 Test_Set=0,Test_inc=0,Test_dec=0;

/* �Ƶ{�� */
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

#pragma vector=0x07        // PC���_�V�q
__interrupt void EXTI2_PC(void)
{
    SW_inc_press_1counter = 0;        // ���s���U�A�M�����U�p��
    SW_inc_press_5counter = 0;        // ���s���U�A�M�����U�p��
    SW_inc_press_1timer = 0;         // ���s���U�A�M�����U1 sec�ɶ�
    SW_inc_press_5timer = 0;         // ���s���U�A�M�����U5 sec�ɶ�
    if (SW_Set == 0)    // ���s���U?
    {
       SW_Set_debounce_timer = debounce_timer;   // �]�w���u���ɶ�
       if ( (SW_flag & 0x01 ) == 0)              // �P�_���s�X��(�L���U�X��)
       {
           SW_flag |= 0x01;                      // �]�w���s�X��
           Test_Set++; 
       }
    }
    if (SW_inc == 0)
    {
       SW_inc_debounce_timer = debounce_timer;
       if ( (SW_flag & 0x02 ) == 0)
       {
           SW_flag |= 0x02;
           if ((modeId == 3) || (modeId == 4) || (modeId == 5) || (modeId == 6)) modeId = 0;    //=3(�ű�����),=4(���C�Ų��`),=5(�[�����`),=6(��l�P�]�w�ūײ��`)�A�����s�T�{��^�ݾ�
           Test_inc++; 
       }
    }

    if (SW_dec == 0)
    {  
       SW_dec_debounce_timer = debounce_timer;
       if ( (SW_flag & 0x04 ) == 0)
       {
           SW_flag |= 0x04;
           if (modeId == 1)                        // Power�Ҧ��~�i�հѼ�
           {
             if (tempId < 3) tempId ++;            // �ѼƲ֥[
             else tempId = 0; 
             set_temp = tempCache[tempId];         // �]�w�ū�
             set_time = timeCache[tempId];         // �]�w�ɶ�
             check_hot_ok_timer = ((set_time *60 ) >> 1);   // �]�w�ɶ��@�b���T�{�ɶ�
           }
           if (modeId == 2)                        // �ūױ���Ҧ���ܭn��ܰ����ūשγѾl�[���ɶ�
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
      if (modeId == 2)                         // �ūױ���Ҧ��A��s��/��/��/��p��
      {  
        TIM4_1sec++;
        if (TIM4_1sec>=60)  //1min=60sec
        {
          TIM4_1sec = 0;
          TIM4_1min++;

          if (set_time > 1) set_time--;          // �[���ɶ��˼ơA�ɶ���i�J�ű�����
          else
          {
            modeId = 3;
            SW_flag &= 0xF7;                     // �ūװ����Ҧ�
            Hot_OK_5min_counter = 0;      // �M��5�����p�ƾ�
            Hot_OK_5min_timer = 0;         // �M��5�����ɶ�
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
    else if ((TIM4_2ms&0xF)==2) ADC_CR1 |= 0x01;     // 2ms*16=32ms�AADC�ഫ 
    else if ((TIM4_2ms&0xF)==3) refreshRelay();      // 2ms*16=32ms�ARelay���A��s
    else if ((TIM4_2ms&0xFF)==4) getTemperature();   // �ū��ഫ(�Ʀ�����ū�)
    refreshDisplay();                                // 2ms��s7�u��ܾ�

// ���u���ɶ����ߤ~�P�_���s�O�_����    
    if (SW_Set_debounce_timer> 0)                           // �O�_�٦b���u���ɶ�
    {
        if ( SW_Set == 1)                                   // ���s�O�_����
        {
           SW_Set_debounce_timer--;                         // ���u���ɶ�-1
           if (SW_Set_debounce_timer == 0) SW_flag &= 0xFE; // ���u���ɶ����s��������
        }
    }
    if (SW_inc_debounce_timer> 0)             // ���s������~��M�����s�O�кX��
    {
        if ( SW_inc == 1)
        {
           SW_inc_debounce_timer--;
           if (SW_inc_debounce_timer == 0) 
           {
               SW_flag &= 0xFD;                    
               SW_flag &= 0xDF;                     //  �M��Power�P�ű��Ҧ����s�����X��,�קK���s�����ɫݾ��PPower���ư���(�i�J)
               SW_flag &= 0xBF;                     //  �M�����s�O�кX�СA�קK���s�`���ɱq�ݾ���Power�S��ݾ�
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

    if ( modeId == 0)          // �ݾ��Ҧ��A�T�{���s���U�O�_����1 sec�A�O���ܶi�JPower�Ҧ��A�_�h�M���p�ƾ��P�ɶ�
    {
      if ( SW_inc == 0)
      {
           if ((SW_flag & 0x20) == 0)           // �T�{�]�w���û������~��A�i�JPower�Ҧ�
           {
         
                SW_inc_press_1counter++;
                if (SW_inc_press_1counter >= 500)
                {
                   SW_inc_press_1counter = 0;
                   SW_inc_press_1timer = 1;
                   modeId = 1;
                   SW_flag |= 0x40;                 // �ѫݾ��Ҧ��`�����s�i�JPower�Ҧ��A�]�m�X�СA�����s����~��M��
                }

           }
      }
      else 
      {
         SW_inc_press_1counter = 0;
         SW_inc_press_1timer = 0;      
      }
    }
    if ( modeId == 1)          // Power�Ҧ��A���s�L�����ɶ��p�ƾ��P�ɶ��O�_5 sec�A�O���ܶi�J�ūױ���Ҧ��A�N�ū׭p�Ʋ֥[�ɶ��k�s�AŪ����l�ūסA�ū׽T�{�ɶ��k�s
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
              getTemperature();                                // �ū��ഫ(�Ʀ�����ū�)
              init_temp = Temp_val;
              check_sec_counter = 0;
              check_sec_timer = 0;
              if (init_temp >= set_temp) modeId = 6;       // ��l�ūפj��]�w�ūסA�i�J���`
              else modeId = 2;
           }
        } 
        if ((SW_inc == 0) || (SW_dec == 0))                // ���s������~�}�l�p��5 sec
        {
            SW_inc_press_5counter = 0;
            SW_inc_press_5timer = 0;
        }
    }
    if (( modeId == 1) || ( modeId == 2))                      // Power�ηūױ���Ҧ��������s�O�_���U1 Sec�A�O���ܶi�J�ݾ��Ҧ�
    {
//      if ( SW_inc == 0) 
      if (( SW_inc == 0) && ((SW_flag & 0x40) == 0))      //  ���s������U1 sec �B���s�e�@���A�����g����L,�קK�ݾ��i�JPower��A�i�J�ݾ�
      {
        SW_inc_press_1counter++;
        if (SW_inc_press_1counter >= 500)
        {
           modeId = 0;
           SW_flag  |= 0x20;                                           // �]�wPower�P�ű��Ҧ����s�����X��
        }
      }
    }
    if ( modeId == 2)                                                   //  �ű��Ҧ�,�Ĥ@���q�T�{�O�_5�������Ť�5��,�ĤG���q�T�{�]�w�[���ɶ��@�b��F�]�w�ū�
    {
      check_sec_counter++;
      if (check_sec_counter >= 500)
      {
        check_sec_counter = 0;
        check_sec_timer ++;
      }
/* �P�_�����������L��5�� */      
      if ((check_sec_timer <= check_5_sec_timer) && ((SW_flag & 0x10) == 0))
      {
//        if ((init_temp + check_5_min_temp) < Temp_val) SW_flag |= 0x10;             // 5���������W�[5�שΨ�F�]�w�ū�,�]�w�X��
       if (((init_temp + check_5_min_temp) < Temp_val) || (set_temp <= Temp_val))  SW_flag |= 0x10;             // 5���������W�[5�שΨ�F�]�w�ū�,�]�w�X��
      }            
      if ((check_sec_timer == check_5_sec_timer) && ((SW_flag & 0x10) == 0)) modeId = 5;
/* �M��5��������F5�׺X�СA�A�i��@�b�ɶ����L��F�]�w�ūקP�O */
      if (check_sec_timer == (check_5_sec_timer +1)) SW_flag &= 0xEF;
/* �P�_�@�b�ɶ������L��F�]�w�ū� */
      if (((check_sec_timer) <= check_hot_ok_timer) && ((SW_flag & 0x10) == 0))
      {
        if (set_temp <= Temp_val) SW_flag |= 0x10;                                                 // �@�b�ɶ�����F�]�w�ū�,�]�w�X��
      }            
      if (((check_sec_timer) == check_hot_ok_timer) && ((SW_flag & 0x10) == 0)) modeId = 5;
      
    }

    if (modeId == 3)                        // �ű�����,5�������FFF,�ɶ���i�J�ݾ��Ҧ�
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
  asm("sim");     //�����������_
  GPIO_init();  
  CLK_init();
  Init_Timer4();
  asm("rim");     //�}�ҥ������_
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
     if (modeId == 0)                 // �ݾ��Ҧ�
     {
       setDisplayOff(1);    // �q���W�q�ALED OFF
       tempId = 0;
       set_temp = tempCache[tempId];
       set_time = timeCache[tempId];
       SW_flag &= 0xF7;                                           //  �]�w��l��ܷū�
       SW_flag &= 0xEF;                                          //  �M���ūײ��`�X��
       check_hot_ok_timer = ((set_time *60 ) >> 1);   // �]�w�ɶ��@�b���T�{�ɶ�
     }
     else if (modeId == 1)      // Power�Ҧ�
     {
       itofpa (tempCache[tempId],(u8*) stringBuffer, 0);   // �ū���ܤT���,�̫�@�쬰�p���I
       setDisplayStr ( (u8*) stringBuffer);
       setDisplayOff(0);
     }
     else if (modeId == 2)     // �ūױ���Ҧ� 
     {
//        getTemperature();                                // �ū��ഫ(�Ʀ�����ū�)
        if (Temp_val <= temp_low )              //  �����ūײ��`,��F���C�ŭ����
        {
          setDisplayStr ("LLL");
          modeId = 4;
        } 
        else if (Temp_val >= temp_high ) 
        {
          setDisplayStr ("HHH");
          modeId = 4;
        }
        else                                                    // ��ܰ����ūשγѾl�ɶ�
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
     else if (modeId == 3)                   // �ű�����,���FFF
     {
        setDisplayStr ("FFF");
     }
     else if (modeId == 5)                  //  �[�����`,���EEE
     {
        setDisplayStr ("EEE");
     }
     else if (modeId == 6)                  // ��l�P�]�w�ūײ��`,���UUU
     {
        setDisplayStr ("UUU");
     }
  }
}

void GPIO_init(void)
{
  PD_DDR_DDR5 = 1;       //PD5��X�ADEG_A  
  PD_CR1_C15 = 1;        //PD5����
  PA_DDR_DDR2 = 1;       //PA2��X�ADEG_B
  PA_CR1_C12 = 1;        //PA2����
  PC_DDR_DDR7 = 1;       //PC7��X�ADEG_C
  PC_CR1_C17 = 1;        //PC7����
  PD_DDR_DDR3 = 1;       //PD3��X�ADEG_D
  PD_CR1_C13 = 1;        //PD3����
  PD_DDR_DDR1 = 1;       //PD1��X�ADEG_E
  PD_CR1_C11 = 1;        //PD1����
  PA_DDR_DDR1 = 1;       //PA1��X�ADEG_F
  PA_CR1_C11 = 1;        //PA1����
  PC_DDR_DDR6 = 1;       //PC6��X�ADEG_G
  PC_CR1_C16 = 1;        //PC6����
  PD_DDR_DDR2 = 1;       //PD2��X�ADEG_DP
  PD_CR1_C12 = 1;        //PD2����
  
  PD_DDR_DDR4 = 1;       //PD4��X�ADIG_1
  PD_CR1_C14 = 1;        //PD4����
  PB_DDR_DDR5 = 1;       //PB5��X�ADIG_2
  PB_CR1_C15 = 1;        //PB5����
  PB_DDR_DDR4 = 1;       //PB4��X�ADIG_3
  PB_CR1_C14 = 1;        //PB4���� 

  PC_DDR_DDR3 = 0;    //PC3��J
  PC_DDR_DDR4 = 0;    //PC4��J
  PC_DDR_DDR5 = 0;    //PC5��J
  
  PC_CR1_C13 = 1;     //PC3�W��
  PC_CR1_C14 = 1;     //PC4�W��
  PC_CR1_C15 = 1;     //PC5�W��
    
  PC_CR2_C23 = 1;     //PC3���_
  PC_CR2_C24 = 1;     //PC4���_
  PC_CR2_C25 = 1;     //PC5���_

  EXTI_CR1_PCIS = 2;    //PC�t��Ĳ�o
}


void EEPROM_init(void)
{
    FLASH_CR1=0x00;
    FLASH_CR2=0x00;
}

void EEPROM_lock(void) 
{
    FLASH_IAPSR=(u8)(~0x08);            //���s�W��
}

void EEPROM_write(u8 addr, u8 dat)      //������EEPROM�A�g�JEEPROM���
{
   do 
  {
      FLASH_DUKR = 0xAE;	// ��J�Ĥ@�K�X
      FLASH_DUKR = 0x56;	// ��J�ĤG�K�X
  } while((FLASH_IAPSR & 0x08) == 0);	// �Y���ꥼ���\�A���s�A��
  u8  *p;                                                           //��}�X��
  p=(u8  *)(EEPROM_BASE_ADDR + addr);      //��}�w�q
  *p=dat;                                                         //��Ƽg�JEEPROM
  while(!(FLASH_IAPSR&0x04));                 //���ݼg�J����
}

u8 EEPROM_read(u8 addr)              //Ū��
{
  u8 *p;
  p=( u8  *)(EEPROM_BASE_ADDR + addr); 
  return *p; 
}

void CLK_init(void)
{
  CLK_CKDIVR = 0x00;  //���_16M�A���W16M�A0x00=16M�A0x08=8M�A0x10=4M�A0x18=2M
}

void Init_Timer4(void)
{
  //128���W 250�p�ơA�b16MHz�U�O2ms�@�����_�I16M/(128*(249+1))
  TIM4_CR1=0x00;        //�����p�ƾ�
  TIM4_IER=0x01;        //��s���_�ϯ�
  TIM4_EGR=0x01;
  TIM4_CNTR=249;      //�p�ƾ���
  TIM4_ARR=249;        //�۰ʭ��˪���
  TIM4_PSCR=0x07;    //���W�� 
  TIM4_CR1=0x01;      //�ϯ�p�ƾ�
}

void ADC_Init(void)
{
  /*PD6*/
  PD_DDR &=0xBF;                 // PD6��J�Ҧ�
  PD_CR1 &=0xBF;                 // PD6�B�ſ�J
  PD_CR2 &=0xBF;                 // PD6�T��~�����_
  CLK_PCKENR2 |= 0x08;           // ADC�����A�q�{�}�ҡA�i�ٲ�
  ADC_CR1 = 0x70;                // ADC����16MHz/18�A�榸�ഫ�A����ADC 
  ADC_CR2 = 0x00;                // �ƾڥ�����A�T��~���D�o�A�T��y�Ҧ� 
  ADC_CSR = 0x26;                // ��ܳq�D6 AIN6   �q?�p��
  ADC_CR1 |= 0x01;               // Power up ADC
}

u16 Get_ADC_Result(void)
{
  ADC_CR1 |= 0x01;                                   // �Ұ�ADC �A�Ұ�ADC�q��
  delay(10);                                         // ����1ms ����ADC�Ұ�
  ADC_CR1 |= 0x01;                                   // �Ұ�ADC�ഫ
  while((ADC_CSR&0x80) == 0);                        // ����ADC�ഫ����
  ADC_result = ADC_DRH << 2;
  ADC_result |= ADC_DRL;
  if (ADC_averaged == 0) ADC_averaged = ADC_result << 4;
  else ADC_averaged += ADC_result - (ADC_averaged >> 4);
  return ADC_averaged;                                   // ��^10��ADCr���G
}

void getTemperature()   // �ϥΤG���k�j�M�̱��񪺰��C�ƭȡA�ð��p���I�p��A�N��Ʀ쭼10���p���I�첾�C
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

void setDigit (u8 id, u8 val, bool dot)    //  �N�T��ƭȶ�J�T�ӤC�q��ܾ�����LED�x�}
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

void enableDigit (u8 id)      //  �M�w���@�ӤC�q��ܾ����
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

void setDisplayStr (u8* val)      //  �M�w�p���I�b���@�ӤC�q��ܾ��A�P��s�x�}
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
 
void itofpa (int val, u8* str, u8 pointPosition)   // �N�ƭ��ഫ��ASC2�X
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
    PA_DDR_DDR3 = 1;       //PA3��X�ARelay
    PA_CR1_C13 = 1;            //PA3����
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