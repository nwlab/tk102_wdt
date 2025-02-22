/**
 ******************************************************************************
 * @file     Project/STM8L10x_StdPeriph_Templates/main.c
 * @author   MCD Application Team
 * @version V1.2.1
 * @date    30-September-2014
 * @brief    This file contains the firmware main function.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
 *
 * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *        http://www.st.com/software_license_agreement_liberty_v2
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <stm8l10x.h>

/** @addtogroup STM8L10x_StdPeriph_Templates
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#if defined(_SDCC_)
#define ASM __asm__
#else
#define ASM asm
#endif

/* This delay should be added just after reset to have access to SWIM pin
 and to be able to reprogram the device after power on (otherwise the
 device will be locked) */
#define STARTUP_SWIM_DELAY_5S \
  {                           \
    ASM(" PUSHW X \n"         \
        " PUSH A \n"          \
        " LDW X, #0xFFFF \n"  \
        "loop1: LD A, #50 \n" \
                              \
        "loop2: DEC A \n"     \
        " JRNE loop2 \n"      \
                              \
        " DECW X \n"          \
        " JRNE loop1 \n"      \
                              \
        " POP A \n"           \
        " POPW X ");          \
  }
/* not connected pins as output low state (the best EMC immunity)
(PA1, PA3, PA5, PB0, PB1, PB2, PB4, PC5, PC6, PD1, PD2, PD3, PD4, PD5,
 PD6, PD7)*/
#define CONFIG_UNUSED_PINS_STM8L001                                   \
  {                                                                   \
    GPIOA->DDR |= GPIO_Pin_1 | GPIO_Pin_3 | GPIO_Pin_5;               \
    GPIOB->DDR |= GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_4;  \
    GPIOC->DDR |= GPIO_Pin_5 | GPIO_Pin_6;                            \
    GPIOD->DDR |= GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | \
                  GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;               \
  }
  
/* Private macro -------------------------------------------------------------*/
#define WDT_TIMEOUT (60000)
#define RESET_PERIOD (500)

/* Private variables ---------------------------------------------------------*/
volatile uint32_t time_keeper = 0;
volatile uint32_t tick_count = 0;
volatile uint32_t watchdog_time = 0;

/* Private function prototypes -----------------------------------------------*/
void delay_using_timer4_init(void);
void delay_ms(uint32_t time);
uint32_t jsom_time_delta(uint32_t base_time, uint32_t now_time);
void TIM2_Config(void);
void enter_sleep_mode(void);
void EXTI_Setup(void);

/* Private functions ---------------------------------------------------------*/

void main(void)
{
  /* -------------STM8L001 startup-------------- */
  /* configure unbonded pins */
  CONFIG_UNUSED_PINS_STM8L001;
  /* delay for SWIM connection: ~5seconds */
  STARTUP_SWIM_DELAY_5S;
  /* ------------------------------------------- */

  /* configure all STM8L001 pins as input with pull up */
  // GPIO_Init(GPIOA, GPIO_Pin_0, GPIO_Mode_In_PU_No_IT); // pin 1
  GPIO_Init(GPIOA, GPIO_Pin_2, GPIO_Mode_In_PU_No_IT); // pin 2
  GPIO_Init(GPIOD, GPIO_Pin_0, GPIO_Mode_In_PU_No_IT); // pin 5
  GPIO_Init(GPIOB, GPIO_Pin_6, GPIO_Mode_In_PU_No_IT); // pin 6
  GPIO_Init(GPIOB, GPIO_Pin_7, GPIO_Mode_In_PU_No_IT); // pin 7
  GPIO_Init(GPIOC, GPIO_Pin_2, GPIO_Mode_In_PU_No_IT); // pin 8

  /* Initialize I/Os in Output Mode */
  // GPIO_Init(GPIOC, GPIO_Pin_2, GPIO_Mode_Out_OD_HiZ_Slow);
  GPIO_Init(GPIOC, GPIO_Pin_2, GPIO_Mode_Out_PP_High_Slow); // pin 8
  GPIO_SetBits(GPIOC, GPIO_Pin_2);

  /* Configure PD0 as input with pull-up and external interrupt */
  GPIO_Init(GPIOD, GPIO_Pin_0, GPIO_Mode_In_PU_IT); // pin 5

  /* TIM2 configuration ------------------------------------------------------*/
  TIM2_Config(); // Configure TIM2 for 1-millisecond wake-up
  /* TIM4 configuration ------------------------------------------------------*/
  delay_using_timer4_init();
  /* Configure external interrupt */
  EXTI_Setup();

  enableInterrupts();

  /* Reset time */
  watchdog_time = tick_count;

  /* Infinite loop */
  while (1)
  {
    enter_sleep_mode();
    /* Execution resumes here after wake-up */
    uint32_t now = tick_count;
    if (jsom_time_delta(watchdog_time, now) > WDT_TIMEOUT)
    {
      watchdog_time = now;
      /* Toggle GPIOC PIN2 */
      GPIO_ResetBits(GPIOC, GPIO_Pin_2);
      delay_ms(RESET_PERIOD);
      GPIO_SetBits(GPIOC, GPIO_Pin_2);
    }
  }
}

void EXTI_Setup(void)
{
  disableInterrupts();
  EXTI_DeInit();
  EXTI_ClearITPendingBit(EXTI_IT_Pin0); 
  /* Configure EXTI for falling edge (PD0) */
  EXTI_SetPinSensitivity(EXTI_Pin_0, EXTI_Trigger_Falling);
}

void extpin0_isr(void)
{
  if (EXTI_GetITStatus(EXTI_IT_Pin0))
  {
    /* Clear EXTI flag */
    EXTI_ClearITPendingBit(EXTI_IT_Pin0);
    /* Update watchdog flag */
    watchdog_time = tick_count;
    /* Testing */
    // GPIO_ToggleBits(GPIOC, GPIO_Pin_2);
  }
}

/**
 * @brief  Configure TIM2 peripheral to generate an interrupt each 128?s
 * @param  None
 * @retval None
 */
void TIM2_Config(void)
{
  /* Enable TIM2 CLK */
  CLK_PeripheralClockConfig(CLK_Peripheral_TIM2, ENABLE);

  TIM2_DeInit();

  /* Time base configuration */
  TIM2_TimeBaseInit(TIM2_Prescaler_16, TIM2_CounterMode_Up, (2000 / 16) - 1); // 1ms if Assuming 2 MHz clock
  TIM2_ITConfig(TIM2_IT_Update, ENABLE);

  /* Enable TIM2 */
  TIM2_Cmd(ENABLE);
}

void timecounter_isr(void)
{
  tick_count++;
  TIM2_ClearITPendingBit(TIM2_IT_Update);
}

uint32_t jsom_time_delta(uint32_t base_time, uint32_t now_time)
{
  uint32_t delta;

  if (now_time >= base_time)
  {
    delta = now_time - base_time;
  }
  else
  {
    delta = U32_MAX - (base_time - now_time);
  }

  return delta;
}

void delay_using_timer4_init(void)
{
  /* Enable TIM4 CLK */
  CLK_PeripheralClockConfig(CLK_Peripheral_TIM4, ENABLE);

  TIM4_DeInit();

  /* Time base configuration */
  TIM4_TimeBaseInit(TIM4_Prescaler_16, (2000 / 16) - 1); // 1ms if fMaster=2Mhz
  TIM4_ClearFlag(TIM4_FLAG_Update);
  TIM4_ITConfig(TIM4_IT_Update, ENABLE);

  /* Disable TIM4 */
  TIM4_Cmd(DISABLE);
}

/* @Brief	: 	Timer 4 Interrupt Service Rountie for Delay function
 * @Para	:	None
 * @Return	:	None
 * @Note	:   User must be implement function delay_isr()
 * 				at INTERRUPT_HANDLER(TIM4_UPD_OVF_IRQHandler, 23) vector
 */
void delay_isr(void)
{
  if (TIM4_GetITStatus(TIM4_IT_Update) == SET)
  {
    if (time_keeper != 0)
    {
      time_keeper--;
    }
    else
    {
      /* Disable Timer to reduce power consumption */
      TIM4->CR1 &= (uint8_t)(~TIM4_CR1_CEN);
    }

    TIM4_ClearITPendingBit(TIM4_IT_Update);
  }
}

/* @Brief	: 	Delay function
 * @Para	:	Time to delay (millis seconds)
 * @Return	:	None
 * @Note	:   None
 */
void delay_ms(uint32_t time)
{
  time_keeper = time;

  /* Reset Counter Register value */
  TIM4->CNTR = (uint8_t)(0);

  /* Enable Timer */
  TIM4->CR1 |= TIM4_CR1_CEN;

  while (time_keeper)
  {
    enter_sleep_mode();
  };
}

void enter_sleep_mode(void)
{
  /* Enable sleep mode */
  wfi(); // Wait-for-interrupt (CPU sleeps until TIM2 wakes it up)
}

#ifdef USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *   where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval : None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/