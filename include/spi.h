/*
 * spi.h
 *
 *  Created on: 7 нояб. 2017 г.
 *      Author: GennadyTanchin <g.tanchin@yandex.ru>
 */

#ifndef SPI_H_
#define SPI_H_

void spiInit(void);
// Передача по SPI в блокирующем режима
int8_t spiTrans_s( uint8_t *buf, len );
// Прием по SPI в блокирующем режима
int8_t spiRecv_s( uint8_t *buf, len );
// Передача с одновременным приемом по SPI в блокирующем режима
int8_t spiTransRecv_s( uint8_t *txBuf, *rxBuf, len );

#endif /* SPI_H_ */
