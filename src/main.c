/*
 * This file is part of the µOS++ distribution.
 *   (https://github.com/micro-os-plus)
 * Copyright (c) 2014 Liviu Ionescu.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"

#include "Timer.h"

#include "main.h"

// ----------------------------------------------------------------------------
//
// Standalone STM32F0 led blink sample (trace via DEBUG).
//
// In debug configurations, demonstrate how to print a greeting message
// on the trace device. In release configurations the message is
// simply discarded.
//
// Then demonstrates how to blink a led with 1 Hz, using a
// continuous loop and SysTick delays.
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the DEBUG output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//
// The external clock frequency is specified as a preprocessor definition
// passed to the compiler via a command line option (see the 'C/C++ General' ->
// 'Paths and Symbols' -> the 'Symbols' tab, if you want to change it).
// The value selected during project creation was HSE_VALUE=8000000.
//
// Note: The default clock settings take the user defined HSE_VALUE and try
// to reach the maximum possible system clock. For the default 8 MHz input
// the result is guaranteed, but for other values it might not be possible,
// so please adjust the PLL settings in system/src/cmsis/system_stm32f0xx.c
//

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);


// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


static inline void mainInit( void );
static inline void sysClockInit(void);
static inline void pwrInit( void );

int main(int argc, char* argv[])
{
  // Send a greeting to the trace device (skipped on Release).
  trace_puts("Hello ARM World!");

  // At this stage the system clock should have already been configured
  // at high speed.
  trace_printf("System clock: %u Hz\n", SystemCoreClock);

  sysClockInit();
  pwrInit();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_ADC_Init();
  MX_I2C1_Init();

  // Infinite loop
  while (1){
    }
  // Infinite loop, never return.
}

static inline void mainInit( void ){
  // Power ON
  RCC->APB1ENR |= (RCC_APB1ENR_PWREN);
}


static inline void sysClockInit(void){

  // MSI range 4194 kHz
  RCC->ICSCR = (RCC->ICSCR & ~RCC_ICSCR_MSIRANGE) | RCC_ICSCR_MSIRANGE_2 | RCC_ICSCR_MSIRANGE_1;

  SysTick_Config( 4194 );
  // SysTick_IRQn interrupt configuration
  NVIC_SetPriority(SysTick_IRQn, 0);

}

static inline void pwrInit( void ){
  // Power range 3
  PWR->CR |= PWR_CR_VOS;
  //------------------- Stop mode config -------------------------
  // Stop mode
  PWR->CR &= ~PWR_CR_PDDS;
  // Clear PWR_CSR_WUF
  PWR->CR |= PWR_CR_CWUF;
  // MSI clock wakeup enable
  RCC->CFGR &= RCC_CFGR_STOPWUCK;
  // Stop mode enable, Interrupt-Wakeup, SleepOnExit enable
  SCB->SCR = (SCB->SCR & ~SCB_SCR_SEVONPEND_Msk) | SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk;
}



#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
