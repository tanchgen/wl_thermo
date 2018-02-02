/*
 * i2c.h
 *
 *  Created on: 3 нояб. 2017 г.
 *      Author: GennadyTanchin <g.tanchin@yandex.ru>
 */

#ifndef THERMO_H_
#define THERMO_H_

#include "stm32l0xx.h"


#define TMP75_ADDR     0x48      // I2C-Адрес термодатчика с учетом сдвига под R/NW-бит
#define R_NW            0x01      // R/NW-бит

#define TMP75_REG_CR   0x01      //Адрес регистра конфигурации
#define TMP75_REG_T    0x00      //Адрес регистра температуры

#define TMP75_SD       (uint8_t)0x01      // Маска бита ShutDown Mode (SD)
#define TMP75_TM       (uint8_t)0x02      // Маска бита Thermostat Mode (TM)
#define TMP75_R09      (uint8_t)0x00      // Маска точности 0.5 гр.Ц (9 бит)
#define TMP75_R10      (uint8_t)0x20      // Маска точности 0.25 гр.Ц (10 бит)
#define TMP75_R11      (uint8_t)0x40      // Маска точности 0.125 гр.Ц (11 бит)
#define TMP75_R12      (uint8_t)0x60      // Маска точности 0.0625 гр.Ц (12 бит)
#define TMP75_OS       (uint8_t)0x80      // Маска бита START One-Shot

#define TMP75_ACCUR       9		// Точность измерения (здесь допустимо 9 - 0.5гр.Ц и 10 - 0.25гр.Ц)
#define TMP75_REG_ACCUR   (((TMP75_ACCUR) - 9) << 5)
#define TO_MESUR_DELAY   (28 * (1 << (TMP75_ACCUR - 9)))

//#define TMP75_START      0x02      // Значение CR - проснутся
//#define TMP75_STOP      0x03      // Значение CR - уснуть

#define I2C_TOUT        20

typedef enum {
  TMP75_REG_TO,
  TMP75_REG_CFG,
  TMP75_REG_HI,
  TMP75_REG_LOW,
} eTmp75Reg;


void tmp75Init( void );

// Считывание регистра из TMP75
uint8_t tmp75RegRead( uint8_t regAddr );
// Останов TMP75
void tmp75Stop( void );
// Запуск конвертирования температуры
void thermoStart( void );
// Считывание измеренной температуры
int16_t tmp75ToRead( void );

uint8_t thermoRead( void );

void thermoIrqHandler( void );

#endif /* THERMO_H_ */
