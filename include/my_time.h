/*
 * my_time.h
 *
 *  Created on: 31 окт. 2017 г.
 *      Author: GennadyTanchin <g.tanchin@yandex.ru>
 */

#ifndef MY_TIME_H_
#define MY_TIME_H_

#include <stdint.h>
#include "stm32l0xx_ll_rtc.h"
#include "stm32l0xx.h"

#define ALARM_UPDATE_TOUT   100


#define ALRM_A      0   // НЕ МЕНЯТЬ !!!!
#define ALRM_B      1   // НЕ МЕНЯТЬ !!!!

typedef struct {
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  uint8_t ss;     // Доля секунды = (PREDIV_S - SS) / (PREDIV_S + 1)
} tTime;

typedef struct {
  uint8_t year;
  uint8_t month;
  uint8_t date;
  uint8_t wday;
} tDate;

typedef struct {
  uint8_t year;
  uint8_t month;
  uint8_t date;
  uint8_t wday;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  uint8_t ss;     // Доля секунды = (PREDIV_S - SS) / (PREDIV_S + 1)
} tRtc;

#define TIMEZONE_MSK      (+3)

  // DEF: standard signed format
  // UNDEF: non-standard unsigned format
  #define _XT_SIGNED

#ifdef  _XT_SIGNED
  typedef int32_t                           tXtime;
#else
  typedef uint32                          tXtime;
#endif

extern volatile tXtime uxTime;

extern __IO uint32_t myTick;

// *********** Инициализация структуры ВРЕМЯ (сейчас - системное ) ************
void timeInit( void );
// Получение системного мремени
uint32_t getTick( void );
tXtime xtmtot( tDate *mdate, tTime *mtime );
void xttotm( tDate * mdate, tTime *mtime, tXtime secsarg);
void setRtcTime( tXtime xtime );
tXtime getRtcTime( void );
void timersProcess( void );

void timersHandler( void );
void myDelay( uint32_t del );

#endif /* UNIX_TIME_H_ */
