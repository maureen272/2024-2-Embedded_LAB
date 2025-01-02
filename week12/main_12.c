
#include "stm32f10x_adc.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "misc.h"
#include "lcd.h"
#include "touch.h"

#define LCD_TEAM_NAME_X 40
#define LCD_TEAM_NAME_Y 50


void Init(void);
void RccInit(void);
void GpioInit(void);
void AdcInit(void);
void DMA_Configure(void);

const int color[12] = {WHITE,CYAN,BLUE,RED,MAGENTA,LGRAY,GREEN,YELLOW,BROWN,BRRED,GRAY};

volatile uint32_t ADC_Value = 10;

void Delay(void) {
  int i;
  for (i = 0; i < 1000000; i++);
}

int main(){
    Init();
    LCD_Clear(GRAY);
    
    int flag = 0;
    LCD_ShowString(LCD_TEAM_NAME_X+50, LCD_TEAM_NAME_Y, "WED_Team02", WHITE, GRAY);
    LCD_ShowNum(LCD_TEAM_NAME_X+50, LCD_TEAM_NAME_Y+50, ADC_Value, 4, WHITE, GRAY);

    while(1){
       if(ADC_Value <= 3400){
         if(flag == 1){
           LCD_Clear(GRAY);
           flag = 0;
         }
         LCD_ShowString(LCD_TEAM_NAME_X+50, LCD_TEAM_NAME_Y, "WED_Team02", WHITE, GRAY);
         LCD_ShowNum(LCD_TEAM_NAME_X+50, LCD_TEAM_NAME_Y+50, ADC_Value, 4, WHITE, GRAY);
       }

       else{
         if(flag == 0){
           LCD_Clear(WHITE);
           flag = 1;
         }
         LCD_ShowString(LCD_TEAM_NAME_X+50, LCD_TEAM_NAME_Y, "WED_Team02", GRAY, WHITE);
         LCD_ShowNum(LCD_TEAM_NAME_X+50, LCD_TEAM_NAME_Y+50, ADC_Value, 4, GRAY, WHITE);       
       }

       Delay();
    }
        
}

void Init(void) {
   SystemInit();
   RccInit();
   GpioInit();
   AdcInit();
  DMA_Configure();

   LCD_Init();
   Touch_Configuration();
   Touch_Adjust();
}

void RccInit(void) {
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
}

void GpioInit(void) {
   GPIO_InitTypeDef GPIO_InitStructure;
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
   GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void AdcInit(void) {
    ADC_InitTypeDef ADC_InitStructure;

    //TODO: ADC1 Configuration
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_NbrOfChannel = 1;           // Use 1 channel.
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_Init(ADC1, &ADC_InitStructure);  
    
    

    ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_239Cycles5);
    ADC_DMACmd(ADC1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);
    ADC_ResetCalibration(ADC1);

    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);

    while(ADC_GetCalibrationStatus(ADC1));
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

//TODO
void DMA_Configure(void) {
   DMA_InitTypeDef DMA_Instructure;
      
        DMA_DeInit(DMA1_Channel1);
        DMA_StructInit(&DMA_Instructure);

        DMA_Instructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;

        DMA_Instructure.DMA_MemoryBaseAddr = (uint32_t)&ADC_Value;

        DMA_Instructure.DMA_BufferSize = 1;

        DMA_Instructure.DMA_MemoryInc = DMA_MemoryInc_Disable;

        DMA_Instructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;

        DMA_Instructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;

        DMA_Instructure.DMA_Mode = DMA_Mode_Circular;

        DMA_Instructure.DMA_Priority = DMA_Priority_High;

        DMA_Init(DMA1_Channel1, &DMA_Instructure);

   DMA_Cmd(DMA1_Channel1, ENABLE);
}
