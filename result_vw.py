import time
from tkinter import *
import numpy as np
import math
import vw
import setting

#vw.solution_list

tk = Tk()
tk.title(u"vw_result")
tk.geometry("1000x500")
canvas= Canvas(tk,width=900,height=500,bg="white", highlightthickness=0)
canvas.pack()


##壁生成部分
wall1=canvas.create_rectangle(0,0,430,160,fill='green')##左上壁 左上のx座標,y座標,右下のx座標,y座標
wall2=canvas.create_rectangle(0,0,270,230,fill='green')
wall3=canvas.create_rectangle(470,0,900,160,fill='green')##右上壁
wall4=canvas.create_rectangle(630,0,900,230,fill='green')
wall5=canvas.create_rectangle(0,350,430,500,fill='green')##左下壁
wall6=canvas.create_rectangle(0,270,270,500,fill='green')
wall7=canvas.create_rectangle(470,350,900,500,fill='green')##右下壁
wall8=canvas.create_rectangle(630,270,900,500,fill='green')



class CarAgent():
    def __init__(self, type, start_x, start_y):
        #self.type = type
        self.start = canvas.moveto(type, start_x, start_y)#初期位置設定
        self.x = self.dx = start_x
        self.y = self.dy = start_y
        self.speed = setting.speed #根拠のある数値にする
        self.car_width = setting.car_width #根拠のある数値にする
        self.car_length = setting.car_length #根拠のある数値にする
        self.car_root = []
        

    def move(self,dx,dy):

        rad = np.arctan(abs(dy - self.y)/abs(dx - self.x))
        dig = math.degrees(rad)
        if dx == self.x and dy == self.y:
            self.x += 0
            self.y += 0
        else:
            print("testttt")
            if  dx > self.x and dy > self.y:
                self.x += (math.cos(math.radians(dig))*self.speed)
                self.y += (math.sin(math.radians(dig))*self.speed)
            elif dx < self.x and dy > self.y:
                self.x -= (math.cos(math.radians(dig))*self.speed)
                self.y += (math.sin(math.radians(dig))*self.speed) 
            elif dx > self.x and dy < self.y:
                self.x += (math.cos(math.radians(dig))*self.speed)
                self.y -= (math.sin(math.radians(dig))*self.speed)  
            elif dx < self.dx and dy < self.dy:
                self.x -= (math.cos(math.radians(dig))*self.speed)
                self.y -= (math.sin(math.radians(dig))*self.speed)
        return [self.x,self.y]

##車生成部分
car1=canvas.create_rectangle(0,0, 25, 10,fill='red')##左車
car2=canvas.create_rectangle(0,0, 10, 25,fill='blue')##上車
car3=canvas.create_rectangle(0,0, 25, 10,fill='yellow')##右車
car4=canvas.create_rectangle(0,0, 10, 25,fill='black')##下車

CarAgent_1 = CarAgent(car1,0,240)
CarAgent_2 = CarAgent(car2,440,0)
CarAgent_3 = CarAgent(car3,860,240)
CarAgent_4 = CarAgent(car4,440,460)

#y=300-dy#ボールの初期y座標
seta=0#ボールを投げる角度
V=0.5#ボールの初速度
Vx=V*np.cos(seta)#x軸方向の速度
# Vy=-V*np.sin(seta)#y軸方向の速度
dt=0.5 #時間刻み
start=time.time() #開始時間
stop_time=10000 #目標終了時間


#計算部分
while True:

    car_move = CarAgent_1.move(100,240)#学習のプログラムで求めたダイクストラの経路をそのまま入れる形？？
    #car_move[0] * dt

    #car1_x+=V*dt #微小時間dtでx方向に動く量
    # y+=Vy*dt #微小時間dtでy方向に動く量
    tk.update() #ウインド画面を更新
    
    stop_time -= 1
    if stop_time == 0:
        break

tk.mainloop()

