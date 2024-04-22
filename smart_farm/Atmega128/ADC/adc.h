/*
 * adc.h
 *
 * Created: 2021-03-23 오후 12:34:09
 *  Author: IOT
 */ 


#ifndef ADC_H_
#define ADC_H_

uint8_t channel;

void adc_init(void);
uint16_t adc_read(uint8_t channel);


#endif /* ADC_H_ */