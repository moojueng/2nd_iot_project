/*
 * GccApplication3.c
 *
 * Created: 2021-03-23 오전 10:54:42
 * Author : IOT
 */ 
#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include "ADC/adc.h"
#include "DHT/DHT.h"
#include "USART/USART.h"
#include "RFID/spi.h"
#include "RFID/mfrc522.h"
#include "FUNC/FUNC.h"
#include "Sens/Sens.h"

int main(void)
{
	unsigned int tem=0, humid=0, soil=0, water=0, cds_res=0, cds_new = 0, cds_old=0; // 센서 값 절대 값으로 선언
	float humid_value = 0, tem_value = 0, soil_value = 0, water_value = 0, cds_value = 0; // 센서 전송값 실수로 선언
	uint8_t byte;
	
	// 초기화
	spi_init();
	mfrc522_init();
	Usart_Init();
	adc_init();
	PWM_init();
	PIN_init();
	
	// RFID 관련 레지스터 변수 초기화, 레지스터는 RFID 헤더 파일 참조
	byte = mfrc522_read(ComIEnReg);
	mfrc522_write(ComIEnReg,byte|0x20);
	byte = mfrc522_read(DivIEnReg);
	mfrc522_write(DivIEnReg,byte|0x80);		
	
	//문 닫음
	PORTA |= (1<<PORTA3);
	_delay_ms(20);
	close;
	_delay_ms(1200);
	PORTA &= ~(1<<PORTA3);
	
	while(1)
	{			
		// 온습도 센서 		
		humid = get_HUM();	// 습도 데이터 입력
		humid_value = humid;
		tem = get_TEM();	// 온도 데이터 입력
		tem_value = tem;
		
		//환풍기 조정
		Venti_con(humid);
		
		// 토양 센서
		soil = adc_read(SOIL);	// 토양센서 ADC 2번 입력
		soil_value = soil;
		
		//물펌프
		W_Pump(soil);
				
		// 조도 센서
		cds_new = adc_read(CDS);	// 조도 센서 ADC 1번 입력
		cds_res = (cds_old * 0.9) + (cds_new * 0.1);
		cds_old = cds_res;
		cds_value = cds_res;
		
		//LED 조정
		Led_Con(cds_res);		
		
		//물높이 센서
		water = adc_read(WATER);		//수위 센서 ADC 0번 입력
		water_value = water;		
		
		// uart로 데이터값 전송
		// 각 데이터의 머릿글자를 메이터 식별 및 통신 시작 신호로 사용하기 위하여 전송
		//  / 를 각 데이터의 끝을 알리기 위하여 전송
		
		// 조도센서
		Usart_TX('-');
		Data_send(cds_value);
		Usart_TX('/');
		
		// 토양센서
		Data_send(soil_value);
		Usart_TX('/');
		
		//온도
		Data_send(tem_value);
		Usart_TX('/');
		
		//습도
		Data_send(humid_value);
		Usart_TX('/');
		
		//수위
		Data_send(water_value);
		Usart_TX('_');
		
		RFID();	// RFID 동작 함수			
	}
	return 0;
}

