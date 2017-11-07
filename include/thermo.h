/*
 * i2c.h
 *
 *  Created on: 3 нояб. 2017 г.
 *      Author: GennadyTanchin <g.tanchin@yandex.ru>
 */

#ifndef THERMO_H_
#define THERMO_H_

#define THERMO_ADDR     0x48      // I2C-Адрес термодатчика с учетом сдвига под R/NW-бит
#define R_NW            0x01      // R/NW-бит

#define THERMO_REG_CR   0x01      //Адрес регистра конфигурации
#define THERMO_REG_T    0x00      //Адрес регистра температуры

#define THERMO_START      0x02      // Значение CR - проснутся
#define THERMO_STOP      0x03      // Значение CR - уснуть

#define I2C_TOUT        20

void i2cInit( void );

// Запуск конвертирования температуры
void thermoStart( void );
// Считывание измеренной температуры
uint16_t stlm75Read( void );
void i2cSend( uint8_t * buf, uint8_t len );

inline void thermoIrqHandler( void );

#endif /* THERMO_H_ */
