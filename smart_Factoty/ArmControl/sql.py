import pymysql
import datetime

db = pymysql.connect(user='root',password='1234', host='localhost', db='smart_factory', charset = 'utf8')

cursor = db.cursor()

#table = "CREATE TABLE storage(ID int,RED int(100) ,GREEN int(100),BLUE int(100),TOTAL int(100),DATE date)" #table create
#cursor.execute(table) #table create

def Color_SQL(rgb):
    if rgb==1:
        color="RED"
        
    elif rgb==2:
        color="GREEN"

    elif rgb==3:
        color="BLUE"
        
    cursor.execute("select EA from RGBdata where Name=%s",(color))
    row = cursor.fetchone()
    cursor.execute("update RGBdata SET EA=%s where Name=%s",(row[0]+1,color))
      
    cursor.execute("select EA from RGBdata")
    row = cursor.fetchall()
    rows = row[0]+row[1]+row[2]
    rows = list(rows)
    cursor.execute("update RGBdata set EA=%s where Name=%s",(rows[0]+rows[1]+rows[2],"TOTAL"))
    
    db.commit()
    
def Start_SQL():
    start_data = [[0,"RED"],[0,"GREEN"],[0,"BLUE"],[0,"TOTAL"]]
    start_sql = "update RGBdata set EA=%s where Name=%s"
    cursor.executemany(start_sql,start_data)
    db.commit()
    
def Finish_SQL():
    day=datetime.date.today()
    cursor.execute("select EA from RGBdata")
    row = cursor.fetchall()
    cursor.execute("insert into storage(RED,GREEN,BLUE,TOTAL,DATE) VALUES(%s,%s,%s,%s,%s)",(row[0],row[1],row[2],row[3],day))
    db.commit()
    