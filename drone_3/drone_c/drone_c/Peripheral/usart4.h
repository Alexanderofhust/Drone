#ifndef __UART4_H
#define __UART4_H

#define JudgeBufBiggestSize 256

void UART4_Configuration(void);
void HAL_UART_IDLECallback(UART_HandleTypeDef *huart);
void MX_UART4_Init(void);

#endif
