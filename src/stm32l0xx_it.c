/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx.h"

#include "thermo.h"
#include "main.h"
#include "process.h"
#include "rfm69.h"
#include "stm32l0xx_it.h"

/* External variables --------------------------------------------------------*/

uint8_t rssi;
uint8_t txCpltCount = 0;        // Счетчик действительно отправленных сообщений
extern volatile uint8_t csmaCount;

void extiPdTest( void ){
  if(EXTI->PR != 0){
    uint32_t tmp = EXTI->PR;
    EXTI->PR = tmp;
    NVIC->ICPR[0] = NVIC->ISPR[0];
  }
}

/******************************************************************************/
/*            Cortex-M0+ Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

void NMI_Handler(void){
}

void HardFault_Handler(void){
  while (1)
  {}
}

void SVC_Handler(void){
}

void PendSV_Handler(void){
}

void SysTick_Handler(void) {
//  mTick++;
}
/**
* RTC global interrupt through EXTI lines 17, 19 and 20.
*/
void RTC_IRQHandler(void){
// Восстанавливаем настройки портов

  restoreContext();
  // Отмечаем запуск MCU
#if DEBUG_TIME
	dbgTime.mcuStart = mTick;
#endif // DEBUG_TIME

  if( RTC->ISR & RTC_ISR_WUTF ){
    // Wake-Up timer interrupt
  	wutStop();
  	wutIrqHandler();
  }
  if( RTC->ISR & RTC_ISR_ALRAF ){
    // Alarm A interrupt
    //Clear ALRAF
    RTC->ISR &= ~RTC_ISR_ALRAF;
    uxTime = getRtcTime();
    // Тест - отправляет каждые 10 секунд
    if((uxTime % 10) == 0) {
      if(state == STAT_READY){
        mesureStart();
      }
    }
    // Стираем флаг прерывания EXTI
    EXTI->PR &= EXTI_PR_PR17;
  }

  // Отмечаем Останов MCU
#if DEBUG_TIME
	dbgTime.mcuEnd = mTick;
#endif // DEBUG_TIME

  // Стираем PWR_CR_WUF
  PWR->CR |= PWR_CR_CWUF;
  while( (PWR->CSR & PWR_CSR_WUF) != 0)
  {}
	// Сохраняем настройки портов
	saveContext();
	// Проверяем на наличие прерывания EXTI
	extiPdTest();
}

/**
* Главное прерывание от RFM - DIO0
*/
void EXTI0_1_IRQHandler(void)
{
	// Восстанавливаем настройки портов
  restoreContext();

  // Стираем флаг прерывания EXTI
  EXTI->PR &= DIO0_PIN;
  if( rfm.mode == MODE_RX ){
    // Если что-то и приняли, то случайно
    // Опустошаем FIFO
    while( dioRead(DIO_RX_FIFONE) == SET ){
      rfmRegRead( REG_FIFO );
    }
  }
  else if( rfm.mode == MODE_TX ) {
    // Отправили пакет с температурой
    txCpltCount++;
  	wutStop();
  	txEnd();
  }
  // Отмечаем останов RFM_TX
#if DEBUG_TIME
	dbgTime.rfmTxEnd = mTick;
#endif // DEBUG_TIME

  // Выключаем RFM69
  rfmSetMode_s( REG_OPMODE_SLEEP );

	// Сохраняем настройки портов
	saveContext();
	// Проверяем на наличие прерывания EXTI
	extiPdTest();
}

// Прерывание по PA3 - DIO3 RSSI
// Канал кем-то занят
void EXTI2_3_IRQHandler( void ){
  tUxTime timeNow;

  // Восстанавливаем настройки портов
  restoreContext();
  wutStop();

  // Выключаем прерывание от DIO3 (RSSI)
  EXTI->PR &= DIO3_PIN;
  EXTI->IMR &= ~(DIO3_PIN);

  rssi = rfmRegRead( REG_RSSI_VAL );
  rfmSetMode_s( REG_OPMODE_SLEEP );

  // Отмечаем останов RFM_RX
#if DEBUG_TIME
	dbgTime.rfmRxEnd = mTick;
#endif // DEBUG_TIME

  // Канал занят - Выжидаем паузу 30мс + x * 20мс
  timeNow = getRtcTime();
  if( (csmaCount >= CSMA_COUNT_MAX) || (timeNow > sendTryStopTime) ){
    // Количество попыток и время на попытки отправить данные вышло - все бросаем до следующего раза
  	csmaCount = 0;
    txEnd();
  }
  else {
  	// Можно еще попытатся - выждем паузу
    csmaPause();
  }

	// Сохраняем настройки портов
	saveContext();
	// Проверяем на наличие прерывания EXTI
  extiPdTest();
  return;
}

#if 0
/**
* @brief This function handles I2C1 event global interrupt / I2C1 wake-up interrupt through EXTI line 23.
*/
void I2C1_IRQHandler(void) {
  // Обработка прерывания I2C: работа с термодатчиком
  thermoIrqHandler();
}
#endif
