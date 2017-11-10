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
#include "bat.h"
#include "rfm69.h"
#include "process.h"

volatile tUxTime sendTryStopTime;

void mesureStart( void ){
  // Запускаем измерение напряжения батареи
  batStart();
  // Запускаем измерение температуры
  thermoStart();
  // Устанавливаем время, через которое надо проснутся и запускаем
  wutSet( 23 * (THERMO_ACCUR - 8) );
}

void wutIrqHandler( void ){
  // По какому поводу был включен WUT? - состояние машины
  switch( state ){
    case STAT_T_MESUR:
      // Пора читать измеренную температуру из датчика
      thermoRead();
      flags.thermCplt = TRUE;
      state = STAT_T_READ;
    case STAT_T_READ:
      // Не пара ли передавать данные серверу?
      dataSendTry();
      break;
    default:
      break;
  }
}

int8_t dataSendTry( void ){
  int8_t rc = 0;
  int16_t tmp;
  uint8_t tmrf;

  // ------ Надо ли отправлять ? ------------
  if( flags.batCplt && flags.thermCplt ){
    if( ((tmrf = rtc.min % 6) == 0 ) || // Время передачи наступило
         (((tmp = sensData.temp - sensData.tempPrev) > 0.5) || (tmp < -0.5)) || // За 1 мин температура изменилась более, чем 0.5 гр.С
         (((tmp = sensData.temp - sensData.tempPrev6) > 1.5) || (tmp < -1.5)) ){ // С предыдущей ОЧЕРЕДНОЙ отправки температура изменилась более, чем 1 гр.С
      if(tmrf == 0){
        sensData.tempPrev6 = sensData.temp;
      }
      // Можно отправлять по радиоканалу
      // Запоминаем время остановки попыток отправки - пробуем не более 1-2 секунды
      sendTryStopTime = getRtcTime() + 1;
      csmaRun();
    }
    // Сохраняем нынешнюю температуру, как предыдущую
    sensData.tempPrev = sensData.temp;
  }

  return rc;
}

void csmaRun( void ){
  state = STAT_RF_CSMA_START;
  rfmSetMode_s( REG_OPMODE_RX );
  // Будем слушать эфир в течение 20 мс
  setWut( 20 );
}
