#ifndef MyMotor_H
#define MyMotor_H

void Motor_Init(void);
void Right_Motor(int speed);
void Left_Motor(int speed);
void Back_N_Foward(int speed, int deadzone);
void Left_N_Right(int speed, int deadzone);

#endif
