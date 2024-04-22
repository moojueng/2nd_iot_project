#include <avr/io.h>
#include "adc.h"

/* ADC 레지스터 설명

	ADMUX (ADC Multiplexer Selection Register)
	
	7번 ~ 6번 => REFS1 ~ 0 : ADS모듈에서 사용하는 기준전압을 설정
	
	5번 => ADLAR : 1번으로 설정하면 변환 결과 ADC 데이터 레지스터에 저장될때 좌특으로 끝을 맞추어 저장
	
	4 ~ 0번 => MUX4 ~ 0 : ADC모듈의 아날로그 입력채널을 설정
	
	
	ADCSRA (ADC Control and Status Register A) 
	
	7번 => ADEN (ADC Enable) : 1로 설정시 ADC 모듈 Enable
	
	6번 => ADSC (ADC Start Conversion) : A/D 변환 시작 (1로 설정시 변환이 시작)
	                                     ADEN이 1로 설정된 후, 첫번째 변환에 25개의 ADC 클록 주기가 필요 
										 다음 변환부터 13클록이 요구
										 AD 변환 종료시, 자동으로 0으로 설정
										 
	5번 => ADFR (ADC Free Running Select) : 1로 설정시 자동으로 계속해서 A/D 변환, 0일때는 한번만 A/D 변환
	
	4번 => ADIF (ADC Interrupt Flag) : AD 변환 후, ADC Data Register값이 업데이트되면 이 비트가 1로 세트
									   AD변환 완료 인터럽트 요청 이때, ADIE = 1로 설정, SREG 레지스터의 비트가
									   1로 설정되어 있으면 이 인터럽트가 발생되어 처리됨.
									   
	3번 => ADIE (ADC Interrupt Enable) : A/D 인터럽트 허용, A/D변환 완료 인터럽트를 개별적 설정
	                                     SREG의 비트가 1로 설정되어야 함.
										 
	2번 ~ 0번 => ADPS2 ~ 0 : A/D 컨버터 클록 분주비 선택
	
	

	ADCH, ADCL => 변환된 값이 저장되는 레지스터 ADMUX의 ADLAR 비트의 값에 따라 저장 방식이 달라짐.
	
*/


/* adc 함수 초기 설정 */
void adc_init() 
{
	ADCSRA |= ((1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)); //16Mhz/128 = 125Khz	
	ADCSRA |= (1<<ADEN);      //ADC enable
}

/* 아날로그 값 -> 디지털 값 변화 함수
ex) 조도, 토양 센서
channel => ADC 핀 넘버 */

uint16_t adc_read(uint8_t channel) {
	ADMUX &= 0xF0;
	ADMUX |= channel;
	
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	
	return ADCW; // 레지스터로 값 반환받기
}
