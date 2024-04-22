
#ifndef DHT_H_
#define DHT_H_

#define DHT_ERROR 255

#define TEM_INT 0x01
#define TEM_DEC 0x02
#define HUM_INT 0x03
#define HUM_DEC 0x04

#define PIN_NUM 7
#define sbi(PORTX,bitX) PORTX|=(1<<bitX)
#define cbi(PORTX,bitX) PORTX&=~(1<<bitX)
#define tbi(PORTX,bitX) PORTX^=(1<<bitX)

void get_dht();
int get_HUM();
int get_TEM();
void request();
int response();
char receive_data();


#endif /* DHT11_H_ */