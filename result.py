import time
from tkinter import *
import numpy as np
import networkx as nx
import math
from geneticalgorithm2 import geneticalgorithm2 as ga

import vw
import setting


def move(x, y, dx, dy):
            rad = np.arctan(abs(dy - y)/abs(dx - x))
            dig = math.degrees(rad)

            if dx == x and dy == y:
                x += 0
                y += 0
            else:
                #print("testttt")
                if  dx > x and dy > y:
                    x += (math.cos(math.radians(dig))*setting.speed)
                    y += (math.sin(math.radians(dig))*setting.speed)
                elif dx < x and dy > y:
                    x -= (math.cos(math.radians(dig))*setting.speed)
                    y += (math.sin(math.radians(dig))*setting.speed) 
                elif dx > x and dy < y:
                    x += (math.cos(math.radians(dig))*setting.speed)
                    y -= (math.sin(math.radians(dig))*setting.speed)  
                elif dx < x and dy < y:
                    x -= (math.cos(math.radians(dig))*setting.speed)
                    y -= (math.sin(math.radians(dig))*setting.speed)
            return [x,y]



def set_virtual_wall(GA_list):
        """
        遺伝的アルゴリズムの結果に対応したVWを設置する関数,VWの4つの頂点のlistと障害物の線分のlistを返す
        """
        size = setting.VWsize
        field_x = setting.VWfield_x
        field_y = setting.VWfield_y
        obstacles_vertex_list = []
        obstacles_line_list = []
        total_num_obstacles = 0
        
        ga_list = [GA_list]
        # for i in GA_list:
        #     print(GA_list[i])
        #     if GA_list[i] >=1.0:
        #         ga_list.append(1)
        #     elif GA_list[i]<1:
        #         ga_list.append(0)
        # print("galist"+str(ga_list))

        #indexにインデックスをdeploy_checkには値(0,1)が入る.
        for index, oneDivisionList in enumerate(ga_list):
            for twoDivisionIndex, deploy_check in enumerate(oneDivisionList):
                if deploy_check >= 1:
                    total_num_obstacles += 1
                    #VWの左上, 左下, 右上, 右下を設定
                    VW_LeftUp = [(field_x + (size) * twoDivisionIndex), (field_y + (size * index))]
                    VW_LeftDown = [VW_LeftUp[0], VW_LeftUp[1] + size]
                    VW_RightUp = [VW_LeftUp[0] + size, VW_LeftUp[1]]
                    VW_RightDown = [VW_LeftUp[0] + size, VW_LeftUp[1] + size]
                    
                    obstacles_vertex_list.extend([VW_LeftUp, VW_LeftDown, VW_RightUp, VW_RightDown])
                    obstacles_line_list.extend([[VW_LeftUp, VW_LeftDown], [VW_LeftUp, VW_RightUp], [VW_RightUp, VW_RightDown], [VW_RightUp, VW_LeftDown]])
        return obstacles_vertex_list, obstacles_line_list




def set_vertex_list(obstacle_list, car_start_goal_list):
        """
        頂点のリストを作成し返す関数
        """
        vertex_list = car_start_goal_list
        vertex_list.extend(obstacle_list)
        return vertex_list

def visibility_graph(vertex_list, obstacle_line_list):
        """
    
        O(n^3)の素朴な可視グラフ法を実行し可視グラフのリストを返す関数
        """
        visibility_graph_list = []

        #vertex_listが2次元ではなく1次元になっているので確認＆修正
        #print(vertex_list)
        for index, vertex_u in enumerate(vertex_list):  # 頂点とインデックスのペアを取得 O(n)
            for goal_index, vertex_v in enumerate(vertex_list[index + 1:], index + 1):  # インデックスの次の頂点から順番に取り出す O(n)
                #vertex_u, vertex_vをつなぐ線分をLineとし
                Line = [index,goal_index]

                #障害物との交差しているかのFlagを立てる
                cross = False

                #全ての障害物の各辺に対し
                for obstacle_Line in obstacle_line_list: #O(n)
                    #print(obstacle_Line)
                    #print(vertex_v[0])
                    #障害物の各辺に対し衝突を判定する
                    #外積による線分交差判定
                    s = (vertex_v[0] - vertex_u[0])*(obstacle_Line[0][1] - vertex_u[1]) - (obstacle_Line[0][0] - vertex_u[0]) * (vertex_v[1] - vertex_u[1])#外積の計算
                    t = (vertex_v[0] - vertex_u[0])*(obstacle_Line[1][1] - vertex_u[1]) - (obstacle_Line[1][0] - vertex_u[0]) * (vertex_v[1] - vertex_u[1])
                    
                    if s * t < 0:
                        #障害物との衝突が検出された時点で障害物と衝突判定のfor文を抜ける
                        cross = True
                        break
                
                if cross == False:
                    #衝突が発生しなかった場合、経路長を計算し追加
                    Line.append(np.sqrt(((vertex_v[0] - vertex_u[0])**2 + (vertex_v[1] - vertex_u[1])**2)))
            
                    visibility_graph_list.append(tuple(Line))
        
        return visibility_graph_list


def dijkstra(visibility_graph_list):
        """
        
        ダイクストラ法を実行し最短経路のリストを返す関数
        """
        nx_Graph = nx.Graph()
        nx_Graph.add_weighted_edges_from(visibility_graph_list)

        #始点, 終点となるノードを指定
        origin_node = 0
        destination_node = 1

        #最短経路と距離をダイクストラ法により求める
        shortest_path = nx.dijkstra_path(nx_Graph,origin_node,destination_node)
        shortest_length = nx.dijkstra_path_length(nx_Graph,origin_node,destination_node)

        return shortest_path, shortest_length

def main():
    #solution_list = vw.main()
    solution_list = [0.15180780696053775, 0.8073581697399996, 1.477140413972299, 1.7997473606463872, 1.0130161896814958, 0.26786757982117493, 1.0266994861594445, 0.4157493750389958, 0.010426577803933412, 0.9765359409995915, 1.8540325152413883, 1.1274910728838243, 1.319590569693116, 1.449654406400599, 0.7196989355074781, 0.40116572861447586, 1.103637222919178, 0.6316405381188614, 1.8954056909891641, 1.4433933049373924, 1.9603248808320768, 0.437992781451644, 1.346275285338597, 1.690349870159493, 1.7552874033168995, 1.9486448531247733, 0.8688180227629614, 0.7212801312284212, 1.741517473148413, 0.8485090716355492, 0.3994523653006583, 1.6528081659718425, 1.1258270971857633, 1.8550045699408813, 1.462493894194725, 1.9709985177161566, 0.9894612303823174, 1.6528036258281, 0.40434071749076805, 0.08032099437243967, 1.277660414263412, 0.6809461778289496, 0.6245100542117898, 1.9633590339998621, 1.2463694298511374, 1.5782856369275582, 1.7220368477728998, 1.7435997734660043, 1.0716963642320276, 0.18906432709643428, 1.9294901555002868, 1.6542194682350937, 1.6763364360574062, 0.5223517545529688, 0.16544317976261103, 1.3738313562569902, 0.6211227771028829, 1.0955529700729336, 0.6052226082282239, 0.17161299756871373, 1.2232498457569274, 0.2584960988345433, 1.4834255322762293, 0.7920366501625498]
    vw_list = np.array(solution_list).reshape(4,setting.VWnum**2).tolist()
    
    
    # class CarAgent():
    #     def __init__(self, type, start, goal):
    #         self.start = start
    #         self.goal = goal
    #         x = start[0]
    #         y = start[1]
    #         self.goal_x = goal[0]
    #         self.goal_y = goal[1]
    #         setting.speed = setting.speed #根拠のある数値にする
    #         self.car_width = setting.car_width #根拠のある数値にする
    #         self.car_length = setting.car_length #根拠のある数値にする
    #         self.car_root = []
    #         self.start_point = canvas.moveto(type, x, y)#初期位置設定

        # def move(self, dx, dy):
        #     rad = np.arctan(abs(dy - y)/abs(dx - x))
        #     dig = math.degrees(rad)

        #     if dx == x and dy == y:
        #         x += 0
        #         y += 0
        #     else:
        #         #print("testttt")
        #         if  dx > x and dy > y:
        #             x += (math.cos(math.radians(dig))*setting.speed)
        #             y += (math.sin(math.radians(dig))*setting.speed)
        #         elif dx < x and dy > y:
        #             x -= (math.cos(math.radians(dig))*setting.speed)
        #             y += (math.sin(math.radians(dig))*setting.speed) 
        #         elif dx > x and dy < y:
        #             x += (math.cos(math.radians(dig))*setting.speed)
        #             y -= (math.sin(math.radians(dig))*setting.speed)  
        #         elif dx < self.dx and dy < self.dy:
        #             x -= (math.cos(math.radians(dig))*setting.speed)
        #             y -= (math.sin(math.radians(dig))*setting.speed)
        #     return [x,y]
    
    tk = Tk()
    tk.title(u"vw_result")
    tk.geometry("1000x500")
    canvas= Canvas(tk,width=900,height=500,bg="white", highlightthickness=0)
    canvas.pack()
    
    ##壁生成部分
    wall1=canvas.create_rectangle(0,0,430,140,fill='green')##左上壁 左上のx座標,y座標,右下のx座標,y座標
    wall2=canvas.create_rectangle(0,0,270,230,fill='green')
    wall3=canvas.create_rectangle(470,0,900,140,fill='green')##右上壁
    wall4=canvas.create_rectangle(630,0,900,230,fill='green')
    wall5=canvas.create_rectangle(0,350,430,500,fill='green')##左下壁
    wall6=canvas.create_rectangle(0,270,270,500,fill='green')
    wall7=canvas.create_rectangle(470,350,900,500,fill='green')##右下壁
    wall8=canvas.create_rectangle(630,270,900,500,fill='green')
    ##車生成部分
    car1=canvas.create_rectangle(0,0, 25, 10,fill='red')##左車
    car2=canvas.create_rectangle(0,0, 10, 25,fill='blue')##上車
    car3=canvas.create_rectangle(0,0, 25, 10,fill='yellow')##右車
    car4=canvas.create_rectangle(0,0, 10, 25,fill='black')##下車

    # CarAgent_1 = CarAgent(car1, setting.car1_start, setting.car1_goal)
    # CarAgent_2 = CarAgent(car2, setting.car2_start, setting.car2_goal)
    # CarAgent_3 = CarAgent(car3, setting.car3_start, setting.car3_goal)
    # CarAgent_4 = CarAgent(car4, setting.car4_start, setting.car4_goal)

    car1_x0 = setting.car1_STARTtoGOAL[0][0]
    car1_y0 = setting.car1_STARTtoGOAL[0][1]
    car2_x0 = setting.car2_STARTtoGOAL[0][0]
    car2_y0 = setting.car2_STARTtoGOAL[0][1]
    car3_x0 = setting.car3_STARTtoGOAL[0][0]
    car3_y0 = setting.car3_STARTtoGOAL[0][1]
    car4_x0 = setting.car4_STARTtoGOAL[0][0]
    car4_y0 = setting.car4_STARTtoGOAL[0][1]
        
    for i in range(4):
        car_vw = np.array(vw_list[i]).reshape(setting.VWnum,setting.VWnum).tolist()
        vw_point_x = setting.VWfield_x
        vw_point_y = setting.VWfield_y
        list1 = []
        list2 = []
        list3 = []
        list4 = []

        for j in range(setting.VWnum):
            for k in range(setting.VWnum):
                if car_vw[j][k]>=1:
                    if i==0:
                        canvas.create_rectangle(vw_point_x, vw_point_y, vw_point_x+setting.VWsize, vw_point_y+setting.VWsize,fill='red')
                        
                    elif i==1:
                        canvas.create_rectangle(vw_point_x, vw_point_y, vw_point_x+setting.VWsize, vw_point_y+setting.VWsize,fill='blue')
                        
                    elif i==2:
                        canvas.create_rectangle(vw_point_x, vw_point_y, vw_point_x+setting.VWsize, vw_point_y+setting.VWsize,fill='yellow')
                    
                    elif i==3:
                        canvas.create_rectangle(vw_point_x, vw_point_y, vw_point_x+setting.VWsize, vw_point_y+setting.VWsize,fill='black')
                    
                vw_point_x += setting.VWsize
            vw_point_x = setting.VWfield_x
            vw_point_y += setting.VWsize

    print("testttt")
    car1_VW_list, car1_vw_line_list = set_virtual_wall(vw_list[0])
    car2_VW_list, car2_vw_line_list = set_virtual_wall(vw_list[1])
    car3_VW_list, car3_vw_line_list = set_virtual_wall(vw_list[2])
    car4_VW_list, car4_vw_line_list = set_virtual_wall(vw_list[3])

    car1_start_goal_list = setting.car1_STARTtoGOAL
    car2_start_goal_list = setting.car2_STARTtoGOAL
    car3_start_goal_list = setting.car3_STARTtoGOAL
    car4_start_goal_list = setting.car4_STARTtoGOAL

    car1_vertex_list = set_vertex_list(car1_VW_list, car1_start_goal_list)
    car2_vertex_list = set_vertex_list(car2_VW_list, car2_start_goal_list)
    car3_vertex_list = set_vertex_list(car3_VW_list, car3_start_goal_list)
    car4_vertex_list = set_vertex_list(car4_VW_list, car4_start_goal_list)

    car1_vertex_list = set_vertex_list(car1_VW_list, car1_start_goal_list)
    car2_vertex_list = set_vertex_list(car2_VW_list, car2_start_goal_list)
    car3_vertex_list = set_vertex_list(car3_VW_list, car3_start_goal_list)
    car4_vertex_list = set_vertex_list(car4_VW_list, car4_start_goal_list)


    car1_vis_graph = visibility_graph(car1_vertex_list, car1_vw_line_list)
    car2_vis_graph = visibility_graph(car2_vertex_list, car2_vw_line_list)
    car3_vis_graph = visibility_graph(car3_vertex_list, car3_vw_line_list)
    car4_vis_graph = visibility_graph(car4_vertex_list, car4_vw_line_list)
    print(car1_vis_graph)

    car1_shortest_path, car1_shortest_length = dijkstra(car1_vis_graph)
    car2_shortest_path, car2_shortest_length = dijkstra(car2_vis_graph)
    car3_shortest_path, car3_shortest_length = dijkstra(car3_vis_graph)
    car4_shortest_path, car4_shortest_length = dijkstra(car4_vis_graph)
    print(car1_shortest_path)
    print("testttt2222")

    #car1の最短経路描画
    for i in range(len(car1_shortest_path)-1):
         canvas.create_line(car1_vertex_list[car1_shortest_path[i]][0],car1_vertex_list[car1_shortest_path[i]][1], car1_vertex_list[car1_shortest_path[i+1]][0],car1_vertex_list[car1_shortest_path[i+1]][1], fill = "red", width = 5)
    #canvas.create_line(car1_vertex_list,car1_vertex_list, car1_vertex_list, car1_vertex_list, fill = "red", width = 5)
    #canvas.create_line(car1_vertex_list, car1_vertex_list, 860, 240, fill = "red", width = 5)

    x_size=25
    y_size=10

    #計算部分
    while True:
        #CarAgent_1.move(270,0)
        canvas.coords(car1, car1_x0, car1_y0, car1_x0+x_size, car1_y0+y_size)
        canvas.coords(car2, car2_x0, car2_y0, car2_x0+y_size, car2_y0+x_size)
        canvas.coords(car3, car3_x0, car3_y0, car3_x0+x_size, car3_y0+y_size)
        canvas.coords(car4, car4_x0, car4_y0, car4_x0+y_size, car4_y0+x_size)
        car1_x0+=5
        car2_y0+=5
        car3_x0-=5
        car4_y0-=5
        time.sleep(0.02)
        tk.update() #ウインド画面を更新
        
        if car1_x0 >= 500:
            break
        

    tk.mainloop()

if __name__ == '__main__':
    main()