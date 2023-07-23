import time
from tkinter import *
import numpy as np
import math
from geneticalgorithm2 import geneticalgorithm2 as ga

import vw
import setting




def main():
    solution  = vw.main()
    solution_list = solution
    vw_list = np.array(solution_list).reshape(4,setting.VWnum**2).tolist()
    #print(vw_list)

    tk = Tk()
    tk.title(u"vw_result")
    tk.geometry("1000x500")
    canvas= Canvas(tk,width=900,height=500,bg="white", highlightthickness=0)
    canvas.pack()

    class CarAgent():
        def __init__(self, type, start, goal):
            self.start = start
            self.goal = goal
            self.x = start[0]
            self.y = start[1]
            self.goal_x = goal[0]
            self.goal_y = goal[1]
            self.speed = setting.speed #根拠のある数値にする
            self.car_width = setting.car_width #根拠のある数値にする
            self.car_length = setting.car_length #根拠のある数値にする
            self.car_root = []
            self.start_point = canvas.moveto(type, self.x, self.y)#初期位置設定

        def move(self, dx, dy):
            rad = np.arctan(abs(self.dy - self.y)/abs(self.dx - self.x))
            dig = math.degrees(rad)

            if self.dx == self.x and self.dy == self.y:
                self.x += 0
                self.y += 0
            else:
                #print("testttt")
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

    CarAgent_1 = CarAgent(car1, setting.car1_start, setting.car1_goal)
    CarAgent_2 = CarAgent(car2, setting.car2_start, setting.car2_goal)
    CarAgent_3 = CarAgent(car3, setting.car3_start, setting.car3_goal)
    CarAgent_4 = CarAgent(car4, setting.car4_start, setting.car4_goal)


    # def set_transparent_color(canvas, item_id, color, alpha):
    #     color_with_alpha = tuple(int(c * 255) for c in (*color, alpha))
    #     canvas.itemconfig(item_id, fill=f"#{color_with_alpha[0]:02x}{color_with_alpha[1]:02x}{color_with_alpha[2]:02x}")

##vw設置
    for i in range(4):
        car_vw = np.array(vw_list[i]).reshape(setting.VWnum,setting.VWnum).tolist()
        vw_point_x1 = 380
        vw_point_y1 = 200
        list1 = []
        list2 = []
        list3 = []
        list4 = []

        for j in range(setting.VWnum):
            for k in range(setting.VWnum):
                if car_vw[j][k]>=1:
                    if i==0:
                        car1_vw = canvas.create_rectangle(vw_point_x1, vw_point_y1, vw_point_x1+25, vw_point_y1+25,fill='red')
                        
                    elif i==1:
                        car2_vw = canvas.create_rectangle(vw_point_x1, vw_point_y1, vw_point_x1+25, vw_point_y1+25,fill='blue')
                        
                    elif i==2:
                        canvas.create_rectangle(vw_point_x1, vw_point_y1, vw_point_x1+25, vw_point_y1+25,fill='yellow')
                    
                    elif i==3:
                        canvas.create_rectangle(vw_point_x1, vw_point_y1, vw_point_x1+25, vw_point_y1+25,fill='black')
                    
                    list1.extend([setting.car1_start, setting.car1_goal, [vw_point_x1, vw_point_y1], [vw_point_x1, vw_point_y1+25], [vw_point_x1+25, vw_point_y1], [vw_point_x1+25, vw_point_y1+25]])##始点,終点と頂点リスト
                    list2.extend([setting.car2_start, setting.car2_goal, [vw_point_x1,vw_point_y1], [vw_point_x1, vw_point_y1+25], [vw_point_x1+25, vw_point_y1], [vw_point_x1+25, vw_point_y1+25]])
                    list3.extend([setting.car3_start, setting.car3_goal, [vw_point_x1,vw_point_y1], [vw_point_x1, vw_point_y1+25], [vw_point_x1+25, vw_point_y1], [vw_point_x1+25, vw_point_y1+25]])
                    list4.extend([setting.car4_start, setting.car4_goal, [vw_point_x1,vw_point_y1], [vw_point_x1, vw_point_y1+25], [vw_point_x1+25, vw_point_y1], [vw_point_x1+25, vw_point_y1+25]])

                vw_point_x1 += 25
            vw_point_x1 = 380
            vw_point_y1 += 25

    print("-----------------")
    
    for i in range(setting.VWnum):
        vertex = vw.Environment.set_vertex_list(vw_list[0],CarAgent_1)
        print(vertex)
    visibility =  vw.Execution.visibility_graph(list1, list2) ##始まりの点から終わりの点と重さsetが入ったlistが返って来る [(はじ、おわ、重さ)・・・・・]
    #print(visibility)
    #for i in range(len(visibility)):
        #Canvas.create_line(x0,y0,x1,y1)
        #print(visibility[i][0])
    dijkstra_path, dijkstra_length= vw.Execution.dijkstra(visibility) ##最短経路のpath(list)と最短距離(整数型or浮動小数点型)が返って来る [0,35,32,・・・・・ ,1] vartex_list[35],（ラベルの座標））
    #print(dijkstra_path)



    ##壁生成部分
    wall1=canvas.create_rectangle(0,0,430,140,fill='green')##左上壁 左上のx座標,y座標,右下のx座標,y座標
    wall2=canvas.create_rectangle(0,0,270,230,fill='green')
    wall3=canvas.create_rectangle(470,0,900,140,fill='green')##右上壁
    wall4=canvas.create_rectangle(630,0,900,230,fill='green')
    wall5=canvas.create_rectangle(0,350,430,500,fill='green')##左下壁
    wall6=canvas.create_rectangle(0,270,270,500,fill='green')
    wall7=canvas.create_rectangle(470,350,900,500,fill='green')##右下壁
    wall8=canvas.create_rectangle(630,270,900,500,fill='green')
 


    ##VW生成部分
    #Car1_vw = VW(350,195,fill='red')#350,195,370,215
    
    #car2_vw =  canvas.create_rectangle(0,0,430,160,fill='blue')
    
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

        car_move = CarAgent_1.move(100,240)
        #car_move[0] * dt

        #car1_x+=V*dt #微小時間dtでx方向に動く量
        # y+=Vy*dt #微小時間dtでy方向に動く量
        tk.update() #ウインド画面を更新
        
        stop_time -= 1
        if stop_time == 0:
            break

    tk.mainloop()
            


if __name__ == '__main__':
    main()

