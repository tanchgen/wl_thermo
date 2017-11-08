/*
 * process.c
 *
 *  Created on: 7 нояб. 2017 г.
 *      Author: GennadyTanchin <g.tanchin@yandex.ru>
 */


#include "stm32l0xx.h"

#include "main.h"
#include "my_time.h"
#include "thermo.h"
#include "process.h"

void mesureStart(){
  batStart();
  thermoStart();
  // Устанавливаем время, через которое надо проснутся и запускаем
  wutSet( 23 * (THERMO_ACCUR - 8) );
}

/* Установка и запуск wakeup-таймера
 * mc - время в мс.
 */
inline void wutSet( uint16_t ms ){

  // Вычисляем значение таймера: wukt = (ms * (RTCCLOCK / 2) + 500)/1000
  uint16_t  wukt = (ms * (RTCCLOCK / 2) + 500)/1000;

  RTC->WPR = 0xCA;
  RTC->WPR = 0x53;
  RTC->CR &=~ RTC_CR_WUTE;
  while((RTC->ISR & RTC_ISR_WUTWF) != RTC_ISR_WUTWF)
  {}
  RTC->WUTR = wukt;
  RTC->CR |= RTC_CR_WUTE;
  RTC->WPR = 0xFE;
  RTC->WPR = 0x64;
}

inline void wutIrqHandler( void ){
  // По какому поводу был включен WUT? - состояние машины
  switch( state ){
    case STAT_T_MESUR:
      // Пора читать измеренную температуру из датчика
      thermoRead();
      flags.thermCplt = TRUE;
      state = STAT_T_READ;
      // Не пара ли передавать данные серверу?
      dataSendTry();
      break;
    default:
      break;
  }
}

inline void deepSleepOn( void ){
  // STOP mode (SLEEPDEEP) enable
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
}

inline void deepSleepOff( void ){
  // STOP mode (SLEEPDEEP) disable
  SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
}

