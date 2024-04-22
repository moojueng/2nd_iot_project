import serial
from time import sleep

ser = serial.Serial ("/dev/ttyAMA1", 9600)
sleep(0.03)
cds = 2000
def ser_work():
    
    if ser.readable():
        t_data = ser.read_until(b'_')
        print(t_data)
        if (t_data[0] == 45) :
            global cds, soil, tem, hum, war
            
            data = t_data.decode('utf-8')            
            data = data.strip('-_')
            data = data.split('/')
            
            print(data[0])
            print(data[1])
            print(data[2])
            print(data[3])
            print(data[4])
            cds = int(data[0])
            soil = int(data[1])
            hum = int(data[2])
            tem = int(data[3])
            war = int(data[4])
            
            
            
            


       



    

