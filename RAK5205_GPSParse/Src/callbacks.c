/*
 * callbacks.c
 *
 *  Created on: Aug 26, 2018
 *      Author: uli
 */

#include "stm32l1xx_hal.h"
#include <stdbool.h>
#include "main.h"
#include "fifo.h"
extern Fifo_t rxFifo;
extern bool endOfNmeaMessage;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == gps_1PPS_Pin) {
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		endOfNmeaMessage = true;
	}
}

UartContext_t UartContext;

void HAL_UART_RxCpltCallback( UART_HandleTypeDef *handle )
{
	uint8_t c;
	if (handle-> Instance != USART3)
		return;

    if( !IsFifoFull( &rxFifo )) {
      // Read one byte from the receive data register
    	c = (uint8_t)handle->Instance->DR;
      FifoPush( &rxFifo, (uint8_t) c );
    }
    else {
    	printf("Fifo is full\r\n");
    	return;
    }

    HAL_UART_Receive_IT( handle, &UartContext.RxData, 1 );
//	HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
}

