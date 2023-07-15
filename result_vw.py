import time
from tkinter import *
import numpy as np

import vw
import setting

#vw.solution_list





tk = Tk()
tk.title(u"vw_result")
tk.geometry("1000x500")
canvas= Canvas(tk,width=900,height=500,bg="white", highlightthickness=0)
canvas.pack()


##壁生成部分
wall1=canvas.create_rectangle(0,0,430,180,fill='green')##左上壁 左上のx座標,y座標,右下のx座標,y座標
wall2=canvas.create_rectangle(0,0,330,230,fill='green')
wall3=canvas.create_rectangle(470,0,900,180,fill='green')##右上壁
wall4=canvas.create_rectangle(570,0,900,230,fill='green')
wall5=canvas.create_rectangle(0,320,430,500,fill='green')##左下壁
wall6=canvas.create_rectangle(0,270,330,500,fill='green')
wall7=canvas.create_rectangle(470,320,900,500,fill='green')##右下壁
wall8=canvas.create_rectangle(570,270,900,500,fill='green')

##車生成部分
car1=canvas.create_rectangle(0,0, 40, 20,fill='red')##左車
car2=canvas.create_rectangle(0,0, 20, 40,fill='blue')##上車
car3=canvas.create_rectangle(0,0, 40, 20,fill='yellow')##右車
car4=canvas.create_rectangle(0,0, 20, 40,fill='black')##下車

#動かすためのパラメータ
#車の初期座標
car1_x=0 
car1_y=240

car2_x=440
car2_y=0

car3_x=860
car3_y=240

car4_x=440
car4_y=460



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
    #車を指定の座標に移動させる
    canvas.moveto(car1, car1_x, car1_y)
    canvas.moveto(car2, car2_x, car2_y)
    canvas.moveto(car3, car3_x, car3_y)
    canvas.moveto(car4, car4_x, car4_y)


    car1_x+=V*dt #微小時間dtでx方向に動く量
    # y+=Vy*dt #微小時間dtでy方向に動く量
    tk.update() #ウインド画面を更新
    
    stop_time -= 1
    if stop_time == 0:
        break

tk.mainloop()

