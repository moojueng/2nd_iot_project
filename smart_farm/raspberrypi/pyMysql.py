import pymysql
import Serial
import time
# STEP 2: MySQL Connection 연결
con = pymysql.connect(host='localhost', user='farm_admin', password='1234', db='smart_farm', charset='utf8')
 
# STEP 3: Connection 으로부터 Cursor 생성
cur = con.cursor()
state = 0
old_hour = 0;
while True:
    
    Serial.ser_work()
    date = time.strftime('%m/%d', time.localtime(time.time()))
    hour = time.strftime('%H', time.localtime(time.time()))
    
    if (hour != old_hour):
        state = 1
    else:
        state = 0        
        
    if (state == 1):
        if (Serial.cds != 2000):
            sql = "INSERT INTO data VALUES (%s, %s, %s, %s, %s, %s)"
            val = (Serial.cds, Serial.soil, Serial.hum, Serial.tem, Serial.war, date)
            cur.execute(sql, val)
            con.commit()
              
    old_hour = hour            

