/*
 * USART.c
 *
 * Created: 2021-04-01 오후 2:46:32
 *  Author: IOT
 */                   
#define F_CPU 16000000

#include "USART.h"
#include <avr/io.h>
#include <stdio.h>

#define Usart_BaudRate 9600
#define UBRR_Value (((F_CPU / (Usart_BaudRate * 16UL))) - 1)

/*  Usart 레지스터 핀 설명 

	UCSR n A
	
	7번 => RXCn (Receive Complete): 수신 버퍼에 읽히지 않는 수신 문자가 있으면 1, 얽는 경우 0으로 Clear되고
								    1로 세트되는 경우 수신완료 인터럽트 요청
					 
	6번 => TXCn (Transmit Complete) : 송신 버퍼에 아직 새로운 송신문자가 기록되지 않는 경우 1로 세트 
	                                  이 경우 송신 완료 인터럽트 요청
										 
	5번 => UDREn (Usart Data Register Empty) : UDRn의 송신버퍼가 비어있어 받을 준비가 되면 1로 세트
											   이 경우에 준비완료 인터럽트 요청한다.
											   
	4번 => FEn (Frame Error) : UDR에 있는 문자를 수신하는 동안에 첫번째 스톱 비트가 0으로 검출되는 프레임 에러
	                           가 발생함을 나타내며, 플랙 비트이다.
							   
	3번 => DORn (Data Overrun Error) : 수신 동작에서 읽지 않는 문자가 있는데 다음 수신이 이루어지면 알려주는 플랙 비트
	
	2번 => UPEn (Usart Parity Error) : 수신 버퍼의 패리티에 에러가 있다는 것을 알려주는 플랙비트
	
	1번 => U2Xn (Usart 2 Transmission Speed) : 비동기에서만 유효하며, 1로 세트하면 분주를 반으로 낮춰 2배로 가속
	
	0번 => MPCMn (Multi-processor Communication Mode) : 1로 세트하면 멀티 프로세서 통신모드로 설정
	
	UCSR n B
	
	7번 => RXCIE (RX Complete Interrupt Enable) : 수신완료 인터럽트 허용
	
	6번 => TXCIEn (TX Complete Interrupt Enable) : 송신완료 인터럽트 허용
	
	5번 => UDRIE (UDR empty Interrupt Enable) : 송신 데이터 레지스터 준비완료 인터럽트 허용
	
	4번 => RXENn (RX ENable) : 1로 세트하면 Usart용 수신단자를 허용
	
	3번 => TXENn (TX ENable) : 1로 세트하면 Usart용 송신단자를 허용. 클리어하더라도 완료될때까지 송신단자로 동작
	
	2번 => UCSZn2 (Usart Character Size) : 전송 문자의 데이터 비트수 설정 ex) UCSZn2 ~ UCSZn0까지 3자리 비트로 설정
	
	1번 => RXB8n (RX data Bit 8) : 전송문자가 9비트로 설정된 경우 수신된 문자의 9번쨰 비트를 저장
	
	0번 => TXB8n (TX data Bit 8) : 전송문자가 9비트로 설정된 경우 송신할 문자의 9번째 비트를 저장
	
	UCSR n C
	
	6번 => UMSELn (Usart Mode Select) : 1이면 동기모드, 0이면 비동기 모드로 설정
	
	5번 ~ 4번 => UPMn1 .. 0 (Usart Parity Mode) : 패리티 모드를 성정
	
	3번 => USBSn (Usart Stop Bit Select) : 1로 세트하면 스톱비트 2개, 0이면 한개로 설정
	
	2번 ~ 1번 => UCSZn1 .. 0 (Usart Character Size) : 전송 문자의 데이터 비트수 설정, 위의 UCSRnB의 UCSZn2비트와
	                                                  같이 쓰인다.
													  
	0번 => UCPOLn (Usart Clock POLarity) : 동기 모드의 슬레이브만 유효
	                                       1로 설정 - Falling Edge 에서 출력, Rising Edge 에서 획득
										   0로  설정 - Falling Edge 에서 획득, Rising Edge 에서 출력
										   
	
	UBBR n (Usart Baud Rate Regiseter)
	
	UBRRnH = 11번 비트 ~ 8번 비트
	
	UBRRnL = 7번 비트 ~ 0번 비트 
	
	Baud Rate = 2 ^(U2Xn) x F_CPU  / 16 (UBRRn + 1)
*/

void Usart_Init(void)
{
	UBRR1H = (uint8_t)(UBRR_Value >> 8);
	UBRR1L = (uint8_t) UBRR_Value;
	
	UCSR1C |= (1 << UCSZ10) | (1 << UCSZ11);
	UCSR1C &= ~(1 << USBS1);
	UCSR1C &= ~((1 << UPM11) | (1 << UPM10));
	
	UCSR1B = (1 << RXEN1) | (1 << TXEN1);
	/*UCSR1B = 0b00011000; //TXENn : 송신허가, RXENn : 수신 허가 설정
	UCSR1C = 0b10000110; // 데이터 : 8비트, 비동기, 패리티: 없음, 정지비트 1비트
	UBRR1H = 0;
	UBRR1L = 103; //9600bps 속도*/
}

// uart 통신 데이터 송신
void Usart_TX(uint8_t data)
{
	while(!(UCSR1A & (1 << UDRE1))); // 통신이 준비되면
	
	UDR1 = data; // 데이터 송신
}


uint8_t Usart_RX(void)
{
	while(!(UCSR1A & (1 << RXC1)));
	
	return UDR1;
}

void Print_String(const char str[]) // 배열 송신
{
	uint8_t i = 0;
	while(str[i])
	{
		Usart_TX(str[i]);
		i++;
	}
	
}

void Uart(int num)  // 유아트 통신 스트링으로 전송
{
	static char s[10];
	itoa(num, s, 10); // 정수를 문자열로 변환     정수 num을 문자열 s로 변환
	Print_String((char*)s); // 유아트 통신 값 전송
}

void Data_send(int data)
{
	Uart(data);
}


/*
void Read_String(char str[], uint8_t length)
{
	char response;
	uint8_t i = 0;
	
	while ( i < length - 1)
	{
		response = Usart_RX();
		Usart_TX(response);
		if (response == '\n')
		{
			break;
		}
		else
		{
			str[i] = response;
			i++;
		}
	}
	str[i] = 0;
}
*/