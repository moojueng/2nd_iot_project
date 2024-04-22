/*
 * IncFile1.h
 *
 * Created: 2021-07-27 오후 2:00:10
 *  Author: ohh
 */ 


#ifndef Sens_H_
#define Sens_H_

#define open OCR1A = 3000, OCR1B = 1350			// 서보모터 동작 변수
#define close OCR1A = 1000, OCR1B = 3650

#define SOIL 2
#define CDS 1
#define WATER 0

void PIN_init(void);
void PWM_init(void);
void RFID(void);
void Venti_con(int handle);
void W_Pump(int handle);
void Led_Con(int con);
int Servo_Con(int con);

#endif /* Sens_H_ */