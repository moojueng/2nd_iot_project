import RPi.GPIO as GPIO
from time import sleep
import cv2
import tensorflow.keras
import numpy as np
from smbus import SMBus
import time
import Camera
import sql

addr = 0x8 # blet address
addr1 = 0x9 # RobotArm address
bus = SMBus(1) # indicates /dev/ic2-1

servo = 7
mag = 11
servo1 = 13
servo2 = 12
servo3 = 16

GPIO.setmode(GPIO.BOARD)
GPIO.setup(servo, GPIO.OUT)
GPIO.setup(servo1, GPIO.OUT)
GPIO.setup(servo2, GPIO.OUT)
GPIO.setup(servo3, GPIO.OUT)
GPIO.setup(mag,GPIO.OUT)

pwm = GPIO.PWM(servo,50)
pwm1 = GPIO.PWM(servo1,50)
pwm_l = GPIO.PWM(servo3,50)
pwm_r = GPIO.PWM(servo2,50)

model_filename = 'keras_model.h5'
model = tensorflow.keras.models.load_model(model_filename)

pwm.start(2.1)
pwm1.start(3)
pwm_l.start(8)
pwm_r.start(8)

sql.Start_SQL()

def moter_stop():
    pwm_r.ChangeDutyCycle(0)
    sleep(0.1)
    pwm1.ChangeDutyCycle(0)
    sleep(0.1)
    pwm_l.ChangeDutyCycle(0)
    sleep(0.1)
    pwm.ChangeDutyCycle(0)
    sleep(0.1)

def Cam_Point(c):
    
    if c == 1:
        Arm_1=2.5;Arm_2=10.3;Arm_3=8.7
    elif c == 2:
        Arm_1=3.5;Arm_2=10.3;Arm_3=8.7
    elif c == 3:
        Arm_1=4.7;Arm_2=10.4;Arm_3=8.7
    elif c == 4:
        Arm_1=2.7;Arm_2=10.8;Arm_3=9.3
    elif c == 5:
        Arm_1=3.5;Arm_2=10.7;Arm_3=9.3
    elif c == 6:
        Arm_1=4.7;Arm_2=10.7;Arm_3=9.3
    
    pwm_l.ChangeDutyCycle(Arm_3)
    sleep(1)
    pwm_r.ChangeDutyCycle(Arm_2)
    sleep(1)
    pwm1.ChangeDutyCycle(Arm_1)
    sleep(1)
    pwm.ChangeDutyCycle(12.5)
    sleep(1)
    
def Magnet_Point(m):
    if m == 1:
        Arm_1=2;Arm_2=10.6;Arm_3=8
    elif m == 2:
        Arm_1=3;Arm_2=10.6;Arm_3=8
    elif m == 3:
        Arm_1=4.2;Arm_2=10.6;Arm_3=8
    elif m == 4:
        Arm_1=2.3;Arm_2=11;Arm_3=8.8
    elif m == 5:
        Arm_1=3.1;Arm_2=11;Arm_3=8.8
    elif m == 6:
        Arm_1=4.2;Arm_2=11;Arm_3=8.8
        
    pwm.ChangeDutyCycle(2.3)
    sleep(1)
    pwm1.ChangeDutyCycle(Arm_1)
    sleep(1)
    pwm_l.ChangeDutyCycle(Arm_3)
    sleep(1)
    pwm_r.ChangeDutyCycle(Arm_2)
    sleep(1)
    GPIO.output(mag,True)
    sleep(2)
    pwm_l.ChangeDutyCycle(Arm_3+0.2)
    sleep(1)
    pwm_r.ChangeDutyCycle(10)
    sleep(0.2)
    pwm_r.ChangeDutyCycle(9)
    sleep(0.2)
    pwm_r.ChangeDutyCycle(8)
    sleep(0.2)
    
try:
    while(1):
        for i in range(1,7):
            Cam_Point(i)
            moter_stop()
            #check=test(0,0)
            camera = Camera.check(0,0,model)
            check=camera.cam_check()
            Magnet_Point(i)
            
            if check == 1:
                pwm_l.ChangeDutyCycle(8)
                sleep(1)
                pwm1.ChangeDutyCycle(12.5)
                sleep(3)
                moter_stop()
                GPIO.output(mag,False)
                sleep(2)
                pwm_l.ChangeDutyCycle(8.5)
                sleep(1)
                bus.write_byte(addr, 0x1)
                sleep(2)
                color_check=bus.read_byte(addr)
                print(color_check)
                sql.Color_SQL(color_check)
                bus.write_byte(addr1, a)
            else:
                pwm1.ChangeDutyCycle(6.5)
                sleep(1)
                GPIO.output(mag,False)
            
            #pwm_l.ChangeDutyCycle(8)
            #sleep(1)
            pwm_r.ChangeDutyCycle(8)
            sleep(1)
            pwm1.ChangeDutyCycle(3.5)
            sleep(1)
        
        
except KeyboardInterrupt:
    pwm.stop()
    pwm1.stop()
    pwm_l.stop()
    pwm_r.stop()
finally:
    GPIO.cleanup()
    sql.Finish_SQL()
    #capture.release()
    #cv2.destroyAllWindows()