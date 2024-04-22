#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include "dht.h"

char DHT11_data = 0,hum_int,hum_dec,tem_int,tem_dec,parity;

void get_dht()
{
	request();						//온습도 센서 호출
	if(response() != 1)				//온습도 센서 응답 대기
	{
		hum_int = receive_data();	//습도 소수점
		hum_dec = receive_data();	//습도 상수
		tem_int = receive_data();	//온도 소수점
		tem_dec = receive_data();	//온도 상수
		parity = receive_data();	//parity
	}	
}

// 습도 데이터 출력 상수 자리만 출력
int get_HUM(int data)
{
	get_dht();
	data = hum_int;
	return data;
}

// 온도 데이터 출력 상수 자리만 출력
int get_TEM(int data)
{
	get_dht();
	data = tem_int;
	return data;
}

//DHT 모듈에게 데이터 요청
void request()
{
	sbi(DDRF,PIN_NUM);		//레지스터 1 세팅 출력
	cbi(PORTF,PIN_NUM);		//레지스터 0 세팅 입력
	_delay_ms(20);		//more than 18ms
	sbi(PORTF,PIN_NUM);		//레지스터 1 세팅 출력
}

//DHT 모듈 응답 확인 1이 리턴되지 않으면 모듈 준비
int response(void)
{
	cbi(DDRF,PIN_NUM);			//input
	_delay_us(39);			//pulled wait 20~40us
	if((PINF & (1<<PIN_NUM)))		//response signal check
	return 1;
	_delay_us(80);			//wait 80us
	if(!(PINF & (1<<PIN_NUM)))	//pulled ready output check
	return 1;
	_delay_us(80);			//wait 80us
}

//모듈의 데이터 받음
char receive_data()
{
	for (int q=0; q<8; q++)
	{
		while((PINF & (1<<PIN_NUM)) == 0); // 데이터가 입력되기까지 대기
		_delay_us(30);
		if(PINF & (1<<7))			//greater than 30ms -> HIGH
		DHT11_data = (DHT11_data<<1)|(0x01);
		else						//less than 30ms -> LOW
		DHT11_data = (DHT11_data<<1);
		while(PINF & (1<<PIN_NUM)); // 데이터입력 끝날 때까지 대기
	}
	return DHT11_data;
}

