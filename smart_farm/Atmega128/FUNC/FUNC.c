/*
 * CFile1.c
 *
 * Created: 2021-07-27 오후 2:04:05
 *  Author: ohh
 */ 


// 데이터의 값 범위 조정 함수
#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>

#include "FUNC.h"

// 값의 범위를 변경
long map(long x, long in_min, long in_max, long out_min, long out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


int Aver_fiter(int arry[])  // 평균 필터
{
	static int temp = 0, arr_size = 0, result = 0;
	arr_size = sizeof(*arry)/sizeof(arry[0]);	// 배열의 크기를 구함
	
	for(int i = arr_size-1 ; i > 0 ; i--)	// 배열을 쉬프트 시킴
	{
		arry[i] = arry[i-1];
	}
	
	temp = arry[0];
	for(int i = 1;i < arr_size; i++) temp = temp + arry[i]; // 배열의 모든 값을 더하여 
	
	return result = temp/arr_size;	// 나눈 값을 출력
}

// 배열 비교 함수
int Arry_Compare(uint8_t str1[], uint8_t str2[], int len)	
{
	static int cond;
	for(int i = 0; i <= len; i++)	// 입력된 두 배열을 크기만큼 비교
	{
		if(str1[i]==str2[i])	// 두 원소의 값이 같다면 1을 출력
		{
			cond = 1;
		}
		else				// 값이 다르면 0을 출력하고 계산 종료
		{
			cond = 0;
			break;
		}
	}
	return cond;
}