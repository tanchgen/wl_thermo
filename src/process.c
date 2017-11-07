/*
 * process.c
 *
 *  Created on: 7 нояб. 2017 г.
 *      Author: GennadyTanchin <g.tanchin@yandex.ru>
 */


#include "stm32l0xx.h"

#include "process.h"

void mesureStart(){
  thermoStart();
  batStart();

  // Устанавливаем время, через которое надо проснутся

}
