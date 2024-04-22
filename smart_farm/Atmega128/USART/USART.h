/*
 * USART.h
 *
 * Created: 2021-04-01 오후 2:47:46
 *  Author: IOT
 */ 
#include <avr/io.h>
#include <stdio.h>

#ifndef USART_H_
#define USART_H_

void Usart_Init(void);
void Usart_TX(uint8_t data);
void Uart(int num);
void Data_send(int data);
//uint8_t Usart_RX(void);
void Print_String(const char str[]);
//void Read_String(char str[], uint8_t length);

#endif /* USART_H_ */