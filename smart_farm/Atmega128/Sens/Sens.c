/*
 * CFile1.c
 *
 * Created: 2021-07-27 오후 2:00:01
 *  Author: ohh
 */ 


#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>

#include "Sens.h"
#include "../FUNC/FUNC.h"
#include "../RFID/spi.h"
#include "../RFID/mfrc522.h"
void PIN_init(void) // 입출력 핀설정 여서 할 것
{
	// 환풍기 A 4
	// 워터펌프 A 3,4
	// 토양센서 ac0
	// 조도센서 ac1
	// 수위 ac2
	// 온습도 ac7
	DDRB |= (1<<DDB5) | (1<<DDB6);
	DDRE |= (1<<DDE3) | (1<<DDE4);
	DDRA |= (1<<DDA3) | (1<<DDA4) | (1<<DDA6);
}

void PWM_init(void) // PWM A,B  PINB5,6  15,16
{
	// FAST PWM MODE ICR 제어	
	// COM1An, COM1Bn <- PWM 출력 핀 설정
	// WGM1n 출력 모드 설정
	// CS1n 분주비 설정
	TCCR1B |= (1 << WGM12) | (1<<WGM13) | (1 << CS11);
	TCCR1A |= (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
	ICR1 = 40000;
	
	//FAST PWM MODE 8-bit	
	TCCR3A |= (1 << WGM30) | (1 << COM3A1) | (1 << COM3B1);
	TCCR3B |= (1<<WGM32) | (1<<CS31) | (1<<CS30);	
}

void RFID(void)
{
	static uint8_t str[MAX_LEN], pin_num[MAX_LEN]; // 카드 아이디 저장 변수
	static int old_stat = 1, new_stat = 1, cnt = 3, cmd = 0;	// 버튼 값 저장 변수
	static bool pin_stat = false;	// 버튼 상태 변수
	static uint8_t New_state, Old_state, state;  // RFID 상태 변수

	new_stat = (PINA & (1 << PINA1));	// RFID 마스터 모드 동작 스위치 상태 입력
	New_state =  mfrc522_request(PICC_REQALL,str);	// RFID 카드 입력 상태 입력
	
	if(old_stat != 0 && new_stat == 0)	// 버튼의 이전 상태와 현재 상태를 비교하여 버튼 상태 변경
	{
		pin_stat = !pin_stat;
	}
	
	old_stat = new_stat;	// 버튼 상태 저장
	
	if(New_state == ERROR) cnt++;	// RFID에 카드가 없으면 카운터를 증가
	else cnt = 0;					// 이외의 경우에 카운터 초기화
	
	if(cnt >= 3)					// 카운터가 3보다 같거나 크면
	{
		state = ERROR;				//RFID의 상태를 카드 미입력으로 설정하고
		cnt = 3;					//카운터를 현재 상태 유지
	}
	else state = CARD_FOUND;		//이 외의 경우에 카드 입력 
	
	if(state == CARD_FOUND) // 카드가 입력되면 
	{
		mfrc522_get_card_serial(str); // 카드의 아이디를 받음			
		
		if(pin_stat == true) // 마스터 모드 버튼이 활성화 되어 있으면
		{
			for(int i = 0;i<MAX_LEN;i++) // 입력된 카드의 아이디를 저장함
			{
				pin_num[i] = str[i];
			}
			
			pin_stat = false;	// 마스터 모드 해제
			_delay_ms(5);
		}
		
		else
		{		
			if(Arry_Compare(pin_num,str,5)) // 카드의 아이디와 저장된 아이디를 비교하여 문을 염
			{
				cmd = 1;		// 문 개방 명령
			}
				
			else // 아이디를 비교하여 불일치시 문을 닫음
			{
				if(cmd==1)cmd = 0;		// 문 폐쇄 명령
			}	
		}			
	}
	else
	{
		cmd = 0;	// 카드가 입력되지 않는 상태에서 문 폐쇄
	}
		
	if(Old_state != state)	// RFID의 상태가 변동되면
	{
		if(cmd == 1)		// 문 개방
		{
			PORTA |= (1<<PORTA3);
			_delay_ms(20);
			open;
			_delay_ms(1200);
		}
		
		else				// 문 폐쇄
		{
			PORTA |= (1<<PORTA3);
			_delay_ms(20);
			close;
			_delay_ms(1200);
		}
	}
	
	Old_state = state;			// RFID의 상태 저장
	PORTA &= ~(1<<PORTA3);		// 서보모터 전원 공급 중단
}

void Venti_con(int handle) // 환풍기 컨트롤 4
{
	if(handle > 60)	// 습도가 70%를 넘으면
	{		
		PORTA |= (1<<PORTA4); // 환풍기 동작
	}				// 이외의 경우에
	else if(handle < 58)
	{
		PORTA &= ~(1<<PORTA4);	// 환풍기 정지
	}
}

void W_Pump(int handle) // 물 펌프 컨트롤 3,4
{
	if(handle > 420)		// 토양이 건조한 상태
	{
		PORTA |= (1<<PORTA6); // 펌프 동작
	}
	else if(handle < 370)	// 토양에 물이 충분히 공급되면
	{
		PORTA &= ~(1<<PORTA6);	// 펌프 정지
	}
}

void Led_Con(int con)  // LED 컨트롤 PWM 3a,3b
{
	static int pos, t_pos;
	pos =  255 - con/4;	// 센싱값의 범위가 넓어 4배 줄여 사용
	
	if(pos<=60)		// 제어 값이 일정 이하면 LED OFF
	{
		t_pos = 0;
	}
	else if(pos>60 && pos <=170)		// 각 구간별 센싱값의 반영 비율을 조정
	{
		t_pos = map(pos , 60, 170, 60, 110);
	}
	else if(pos>170)
	{
		t_pos = map(pos , 170, 255, 110, 255);
	}
	
	OCR3A = t_pos;	// LED의 PWM 제어를 조도센서 값으로 제어함
	OCR3B = t_pos;	
}