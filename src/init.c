/*
 * init.c
 *
 *  Created on: 30 окт. 2017 г.
 *      Author: GennadyTanchin <g.tanchin@yandex.ru>
 */

#include "stm32l0xx.h"
#include "stm32l0xx_ll_adc.h"

#include "main.h"

#include <init.h>



/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RFM_RST_GPIO_Port, RFM_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Dio0_Pin */
  GPIO_InitStruct.Pin = Dio0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Dio0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Dio1_Pin Dio3_Pin Dio3A3_Pin Dio4_Pin */
  GPIO_InitStruct.Pin = Dio1_Pin|Dio3_Pin|Dio3A3_Pin|Dio4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Dio5_Pin */
  GPIO_InitStruct.Pin = Dio5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Dio5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RFM_RST_Pin */
  GPIO_InitStruct.Pin = RFM_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RFM_RST_GPIO_Port, &GPIO_InitStruct);

}
