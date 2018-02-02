#include "beep.h"

#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"

// Beeper is on PB8 = TIM4.CH3 and PB9 = TIM4.CH4

void Beep_Configuration(void)
{
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  TIM_TimeBaseInitTypeDef SetupTimer;
  SetupTimer.TIM_ClockDivision = TIM_CKD_DIV1;       // 1/2/4 ?
  SetupTimer.TIM_CounterMode = TIM_CounterMode_Up;
  SetupTimer.TIM_Prescaler = 2-1;                    // lowest tone about 500 Hz.
  SetupTimer.TIM_Period    = 60000-1;
  SetupTimer.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM4, &SetupTimer);
  TIM_Cmd(TIM4, ENABLE);

  TIM_OCInitTypeDef TIM_OCStruct;
  TIM_OCStruct.TIM_OCMode      = TIM_OCMode_PWM2;
  TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCStruct.TIM_OCPolarity  = TIM_OCPolarity_Low;
  TIM_OCStruct.TIM_Pulse       = 0;                            // 0% duty cycle
  TIM_OC3Init(TIM4, &TIM_OCStruct);                            // TIM4.CH3
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

  TIM_OC4Init(TIM4, &TIM_OCStruct);                            // TIM4.CH4
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

  // GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);

  RCC_APB2PeriphClockCmd (RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

  GPIO_InitTypeDef  GPIO_InitStruct;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;                      // PB8/9
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStruct);

}

void Beep(uint16_t Period, uint8_t Duty, uint8_t DoubleAmpl)          // beep frequency = CPUclk/2/Period = 30000000/Period
{
  uint32_t Pulse = ((uint32_t)Period * (uint32_t)Duty + 0x0080) >> 8; // PWM pulse length
  if(Pulse) TIM_OC4PolarityConfig(TIM4, TIM_OCPolarity_High);
       else TIM_OC4PolarityConfig(TIM4, TIM_OCPolarity_Low);
  TIM_SetCompare3(TIM4, (uint16_t)Pulse);                    // TIM4->CCR3 = Pulse;
  if(DoubleAmpl) TIM_SetCompare4(TIM4, (uint16_t)Pulse);     // TIM4->CCR4 = Pulse;
            else TIM_SetCompare4(TIM4, (uint16_t)0);
  TIM_SetAutoreload(TIM4, Period);                           // TIM4->ARR = Period;
}

// Period for notes of the lowest octave: C,     C#,    D,     D#,    E,     F,     F#,    G,     G#,    A,     A#,    B
//                                        0,     1,     2,     3,     4,     5,     6,     7,     8,     9,     A,     B
static const uint16_t NotePeriod[12] = { 57334, 54116, 51079, 48212, 45506, 42952, 40541, 38266, 36118, 34091, 32178, 30372 } ;

void Beep_Note(uint8_t Note) // Note = VVOONNNN: VV = Volume, OO=Octave, NNNN=Note
{ uint8_t Volume =  Note>>6;
  uint8_t Octave = (Note>>4)&0x03;
  Note &= 0x0F; if(Note>=12) { Note-=12; Octave+=1; }
  uint8_t Duty = 0; uint8_t DoubleAmpl=0;
  if(Volume) { Duty=0x10; Duty<<=Volume; } // Duty = 0x00, 0x20, 0x40, 0x80
  if(Volume>2) { DoubleAmpl=1; }           // DoubleAmpl = 0, 0, 1, 1
  uint16_t Period = NotePeriod[Note];
  if(Octave) { Period += 1<<(Octave-1); Period >>= Octave; }
  Beep(Period, Duty, DoubleAmpl); }
