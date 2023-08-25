import time
from tkinter import *
import numpy as np
import networkx as nx
import math
from geneticalgorithm2 import geneticalgorithm2 as ga

import vw
import setting


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
    
    #ga_list = [GA_list]

    #indexにインデックスをdeploy_checkには値(0,1)が入る.
    for index, oneDivisionList in enumerate(GA_list):
        for twoDivisionIndex, deploy_check in enumerate(oneDivisionList):
            if deploy_check >= 1:
                total_num_obstacles += 1
                #VWの左上, 左下, 右上, 右下を設定
                VW_LeftUp = [(field_x + (size * twoDivisionIndex)), (field_y + (size * index))]
                VW_LeftDown = [VW_LeftUp[0], VW_LeftUp[1] + size]
                VW_RightUp = [VW_LeftUp[0] + size, VW_LeftUp[1]]
                VW_RightDown = [VW_LeftUp[0] + size, VW_LeftUp[1] + size]
                
                obstacles_vertex_list.extend([VW_LeftUp, VW_LeftDown, VW_RightUp, VW_RightDown])
                obstacles_line_list.extend([[VW_LeftUp, VW_LeftDown], [VW_LeftUp, VW_RightUp], [VW_RightUp, VW_RightDown], [VW_RightDown, VW_LeftDown]])
    return obstacles_vertex_list, obstacles_line_list




def set_vertex_list(obstacle_list, carAgent, wall_edge):
        """
        頂点のリストを作成し返す関数
        """
        start = carAgent[0].copy()
        goal = carAgent[1].copy()
        vertex_list = [start, goal]

        vertex_list.extend(obstacle_list)
        vertex_list.extend(wall_edge)
        #print("ver"+str(vertex_list))
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
                
                if s * t < 0.0:
                    #障害物との衝突が検出された時点で障害物と衝突判定のfor文を抜ける
                    cross = True
                    continue
            
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


def set_wall():
        wall_edge_list = []
        #設置する壁の考慮すべきエッジをlistにまとめる
        wall_edge_list = setting.wall_edge

        wall_line_list = setting.wall_line
        return wall_edge_list, wall_line_list


def collision_CarToCar(car1_vertex_list, car1_shortest_path, car2_vertex_list, car2_shortest_path, car3_vertex_list, car3_shortest_path, car4_vertex_list, car4_shortest_path):
    """
    
    車同士の衝突判定
    """
    collision = 0

    #同じ速度で目標地点に動いた場合の予測地点をlistにまとめる

    car1_node_move_list = []
    for index in range(0, len(car1_shortest_path)-1):
        start = car1_vertex_list[car1_shortest_path[index]]
        node = car1_vertex_list[car1_shortest_path[index + 1]]
        car_position = start
        while car_position != node:
            if node[0] == start[0]:
                if  start[0] > node[0] and start[1] > node[1]:
                    car_position[1] -= (setting.speed)
                elif start[1] > node[1]:
                    car_position[1] -= (setting.speed) 
                elif start[0] > node[0]:
                    car_position[1] += (setting.speed)  
                else:
                    car_position[1] += (setting.speed)
            else:
                #移動角度の計算
                rad = np.arctan(abs(node[1] - start[1])/abs(node[0] - start[0]))
                if  start[0] > node[0] and start[1] > node[1]:    
                    car_position[0] -= np.cos(rad) * setting.speed
                    car_position[1] -= np.sin(rad) * setting.speed
                elif start[1] > node[1]:
                    car_position[0] += np.cos(rad) * setting.speed
                    car_position[1] -= np.sin(rad) * setting.speed
                elif start[0] > node[0]:
                    car_position[0] -= np.cos(rad) * setting.speed
                    car_position[1] += np.sin(rad) * setting.speed
                else:
                    car_position[0] += np.cos(rad) * setting.speed
                    car_position[1] += np.sin(rad) * setting.speed
            if car_position[0] >= node[0]:
                car_position[0] = node[0]
            if car_position[1] >= node[1]:
                car_position[1] = node[1]
            car1_node_move_list.append([int(car_position[0]),int(car_position[1])])

    car2_node_move_list = []
    for index in range(0, len(car2_shortest_path)-1):
        start = car2_vertex_list[car2_shortest_path[index]]
        node = car2_vertex_list[car2_shortest_path[index + 1]]
        car_position = start
        while car_position != node:
            if node[0] == start[0]:
                if  start[0] > node[0] and start[1] > node[1]:
                    car_position[1] -= (setting.speed)
                elif start[1] > node[1]:
                    car_position[1] -= (setting.speed) 
                elif start[0] > node[0]:
                    car_position[1] += (setting.speed)  
                else:
                    car_position[1] += (setting.speed)
            else:
                #移動角度の計算
                rad = np.arctan(abs(node[1] - start[1])/abs(node[0] - start[0])) 
                if  start[0] > node[0] and start[1] > node[1]:    
                    car_position[0] -= np.cos(rad) * setting.speed
                    car_position[1] -= np.sin(rad) * setting.speed
                elif start[1] > node[1]:
                    car_position[0] += np.cos(rad) * setting.speed
                    car_position[1] -= np.sin(rad) * setting.speed
                elif start[0] > node[0]:
                    car_position[0] -= np.cos(rad) * setting.speed
                    car_position[1] += np.sin(rad) * setting.speed
                else:
                    car_position[0] += np.cos(rad) * setting.speed
                    car_position[1] += np.sin(rad) * setting.speed
            if car_position[0] >= node[0]:
                car_position[0] = node[0]
            if car_position[1] >= node[1]:
                car_position[1] = node[1]
            car2_node_move_list.append([int(car_position[0]),int(car_position[1])])
    
    car3_node_move_list = []
    for index in range(0, len(car3_shortest_path)-1):
        start = car3_vertex_list[car3_shortest_path[index]]
        node = car3_vertex_list[car3_shortest_path[index + 1]]
        car_position = start
        while car_position != node:
            if node[0] == start[0]:
                if  start[0] > node[0] and start[1] > node[1]:
                    car_position[1] -= (setting.speed)
                elif start[1] > node[1]:
                    car_position[1] -= (setting.speed) 
                elif start[0] > node[0]:
                    car_position[1] += (setting.speed)  
                else:
                    car_position[1] += (setting.speed)
            else:
                #移動角度の計算
                rad = np.arctan(abs(node[1] - start[1])/abs(node[0] - start[0]))
                if  start[0] > node[0] and start[1] > node[1]:    
                    car_position[0] -= np.cos(rad) * setting.speed
                    car_position[1] -= np.sin(rad) * setting.speed
                elif start[1] > node[1]:
                    car_position[0] += np.cos(rad) * setting.speed
                    car_position[1] -= np.sin(rad) * setting.speed
                elif start[0] > node[0]:
                    car_position[0] -= np.cos(rad) * setting.speed
                    car_position[1] += np.sin(rad) * setting.speed
                else:
                    car_position[0] += np.cos(rad) * setting.speed
                    car_position[1] += np.sin(rad) * setting.speed
            if car_position[0] >= node[0]:
                car_position[0] = node[0]
            if car_position[1] >= node[1]:
                car_position[1] = node[1]
            car3_node_move_list.append([int(car_position[0]),int(car_position[1])])
    
    car4_node_move_list = []
    for index in range(0, len(car4_shortest_path)-1):
        start = car4_vertex_list[car4_shortest_path[index]]
        node = car4_vertex_list[car4_shortest_path[index + 1]]
        car_position = start
        while car_position != node:
            if node[0] == start[0]:
                if  start[0] > node[0] and start[1] > node[1]:
                    car_position[1] -= (setting.speed)
                elif start[1] > node[1]:
                    car_position[1] -= (setting.speed) 
                elif start[0] > node[0]:
                    car_position[1] += (setting.speed)  
                else:
                    car_position[1] += (setting.speed)
            else:
                #移動角度の計算
                rad = np.arctan(abs(node[1] - start[1])/abs(node[0] - start[0]))
                if  start[0] > node[0] and start[1] > node[1]:    
                    car_position[0] -= np.cos(rad) * setting.speed
                    car_position[1] -= np.sin(rad) * setting.speed
                elif start[1] > node[1]:
                    car_position[0] += np.cos(rad) * setting.speed
                    car_position[1] -= np.sin(rad) * setting.speed
                elif start[0] > node[0]:
                    car_position[0] -= np.cos(rad) * setting.speed
                    car_position[1] += np.sin(rad) * setting.speed
                else:
                    car_position[0] += np.cos(rad) * setting.speed
                    car_position[1] += np.sin(rad) * setting.speed
            if car_position[0] >= node[0]:
                car_position[0] = node[0]
            if car_position[1] >= node[1]:
                car_position[1] = node[1]
            car4_node_move_list.append([int(car_position[0]),int(car_position[1])])

    #同じ速度で動いた場合の予測地点のlistが存在する場合、同じindexで車同士の距離が閾値以下になった時、衝突したといえる

    for index, move_pos in enumerate(car1_node_move_list):
        if index <= len(car2_node_move_list)-1: 
            carTocar_distance = np.sqrt(((car2_node_move_list[index][0] - move_pos[0])**2) + ((car2_node_move_list[index][1] - move_pos[1])**2))
            if carTocar_distance <= 27:
                collision += 1
        
        if index <= len(car3_node_move_list)-1: 
            carTocar_distance = np.sqrt(((car3_node_move_list[index][0] - move_pos[0])**2) + ((car3_node_move_list[index][1] - move_pos[1])**2))
            if carTocar_distance <= 27:
                collision += 1
        
        if index <= len(car4_node_move_list)-1: 
            carTocar_distance = np.sqrt(((car4_node_move_list[index][0] - move_pos[0])**2) + ((car4_node_move_list[index][1] - move_pos[1])**2))
            if carTocar_distance <= 27:
                collision += 1
    
    for index, move_pos in enumerate(car2_node_move_list):
        if index <= len(car3_node_move_list)-1: 
            carTocar_distance = np.sqrt(((car3_node_move_list[index][0] - move_pos[0])**2) + ((car3_node_move_list[index][1] - move_pos[1])**2))
            if carTocar_distance <= 27:
                collision += 1
        
        if index <= len(car4_node_move_list)-1: 
            carTocar_distance = np.sqrt(((car4_node_move_list[index][0] - move_pos[0])**2) + ((car4_node_move_list[index][1] - move_pos[1])**2))
            if carTocar_distance <= 27:
                collision += 1
    
    for index, move_pos in enumerate(car3_node_move_list):
        if index <= len(car4_node_move_list)-1: 
            carTocar_distance = np.sqrt(((car4_node_move_list[index][0] - move_pos[0])**2) + ((car4_node_move_list[index][1] - move_pos[1])**2))
            if carTocar_distance <= 27:
                collision += 1
        
    return collision




def main():
    #solution_list = vw.main()
    solution_list = [0.9954823030028079, 0.832286318735352, 0.3641213972722781, 0.8647723318860676, 0.7979267000200745, 0.45658060701060643, 0.9394462620977604, 1.7931876118387886, 0.6986748957850217, 0.7587682173217307, 0.7972479881704786, 0.5525695347563149, 0.2817552245806145, 1.931992619456184, 0.574021212542051, 0.5770309784401376, 1.4528326504049842, 0.5792397391638529, 0.6266923225726821, 0.39779341084143116, 0.9257062579739788, 0.5720807479594718, 0.480349620638995, 0.3296449678855504, 1.3786686302763864, 0.4088429436619103, 0.33511683388365676, 0.6178251459873814, 0.6603345914982242, 1.252975224059661, 0.6655806667228041, 0.8974334422774133, 0.4592140520185599, 0.49859466688964793, 1.0856580477727933, 0.40778713316268034, 1.621819583786396, 0.8216769581416217, 0.8838159066081368, 0.8722908988945908, 0.09786653613521493, 0.49056246255551317, 0.42811948296923247, 0.6720378336341677, 0.46089904553772365, 1.1487598817034301, 0.7817966728011436, 0.5430969388502669, 0.4241381309736889, 0.9840257733972383, 0.7552422614200132, 0.5871473946067254, 0.5711417651409059, 0.26390258318671156, 0.522910946849841, 1.4039757838986457, 0.9983043363134751, 0.5517164685006619, 0.6007241252072055, 0.010634808939372409, 0.7006192938211628, 0.2472207188103599, 1.106666826125733, 0.8533603421327431, 0.2580861024180425, 0.38290647759780194, 1.0375531238924702, 0.25100238238491945, 0.8603423897862392, 1.6521807206156565, 0.571760707830099, 0.9658456237699324, 1.6903022833833852, 0.2895911728076035, 0.9860166668457369, 0.12276944794117783, 0.847165069802164, 0.260133317164569, 0.6595248555159996, 0.42347590606738694, 0.11618717240065904, 0.3714022537007877, 0.8073173167897405, 1.0556488492082003, 0.3039812481881108, 0.6479035749057656, 1.3751196721744163, 0.687651645045099, 0.4008302907609378, 0.12243240485747497, 0.535357908255567, 0.20323612395483942, 0.6534294831171206, 0.06049749368756174, 1.2401150037042297, 0.6227517431230052, 0.9316661810591109, 1.0610985355591176, 0.03314360884395251, 0.9660677111566573]
    #vw_list = np.array(solution_list)
    #vw_list = np.array(solution_list).reshape(4,setting.VWnum**2).tolist()
    vw_list = np.array(solution_list).reshape(4,5,5).tolist()
    print(vw_list)

    f = open('data.txt', 'a', encoding='UTF-8')
    f.writelines(str(vw_list))
    f.writelines('\n')

    
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
    
    # tk = Tk()
    # tk.title(u"vw_result")
    # tk.geometry("1000x500")
    # canvas= Canvas(tk,width=900,height=500,bg="white", highlightthickness=0)
    # canvas.pack()
    
    # ##壁生成部分
    # wall1=canvas.create_rectangle(0,0,430,140,fill='green')##左上壁 左上のx座標,y座標,右下のx座標,y座標
    # wall2=canvas.create_rectangle(0,0,270,230,fill='green')
    # wall3=canvas.create_rectangle(470,0,900,140,fill='green')##右上壁
    # wall4=canvas.create_rectangle(630,0,900,230,fill='green')
    # wall5=canvas.create_rectangle(0,350,430,500,fill='green')##左下壁
    # wall6=canvas.create_rectangle(0,270,270,500,fill='green')
    # wall7=canvas.create_rectangle(470,350,900,500,fill='green')##右下壁
    # wall8=canvas.create_rectangle(630,270,900,500,fill='green')
    # ##車生成部分
    # car1=canvas.create_rectangle(0,0, 25, 10,fill='red')##左車
    # car2=canvas.create_rectangle(0,0, 10, 25,fill='blue')##上車
    # car3=canvas.create_rectangle(0,0, 25, 10,fill='yellow')##右車
    # car4=canvas.create_rectangle(0,0, 10, 25,fill='black')##下車

    # # CarAgent_1 = CarAgent(car1, setting.car1_start, setting.car1_goal)
    # # CarAgent_2 = CarAgent(car2, setting.car2_start, setting.car2_goal)
    # # CarAgent_3 = CarAgent(car3, setting.car3_start, setting.car3_goal)
    # # CarAgent_4 = CarAgent(car4, setting.car4_start, setting.car4_goal)

    # car1_x0 = setting.car1_STARTtoGOAL[0][0]
    # car1_y0 = setting.car1_STARTtoGOAL[0][1]
    # car2_x0 = setting.car2_STARTtoGOAL[0][0]
    # car2_y0 = setting.car2_STARTtoGOAL[0][1]
    # car3_x0 = setting.car3_STARTtoGOAL[0][0]
    # car3_y0 = setting.car3_STARTtoGOAL[0][1]
    # car4_x0 = setting.car4_STARTtoGOAL[0][0]
    # car4_y0 = setting.car4_STARTtoGOAL[0][1]
        
    # ##4vw用
    # for i in range(4):
    #     car_vw = np.array(vw_list[i]).reshape(setting.VWnum,setting.VWnum).tolist()
    #     vw_point_x = setting.VWfield_x
    #     vw_point_y = setting.VWfield_y
    #     list1 = []
    #     list2 = []
    #     list3 = []
    #     list4 = []

    #     for j in range(setting.VWnum):
    #         for k in range(setting.VWnum):
    #             if car_vw[j][k]>=1:
    #                 if i==0:
    #                     canvas.create_rectangle(vw_point_x, vw_point_y, vw_point_x+setting.VWsize, vw_point_y+setting.VWsize,fill='red')
                        
    #                 elif i==1:
    #                     canvas.create_rectangle(vw_point_x, vw_point_y, vw_point_x+setting.VWsize, vw_point_y+setting.VWsize,fill='blue')
                        
    #                 elif i==2:
    #                     canvas.create_rectangle(vw_point_x, vw_point_y, vw_point_x+setting.VWsize, vw_point_y+setting.VWsize,fill='yellow')
                    
    #                 elif i==3:
    #                     canvas.create_rectangle(vw_point_x, vw_point_y, vw_point_x+setting.VWsize, vw_point_y+setting.VWsize,fill='black')
                    
    #             vw_point_x += setting.VWsize
    #         vw_point_x = setting.VWfield_x
    #         vw_point_y += setting.VWsize

    # print("testttt")
    # #4vw用
    car1_VW_list, car1_vw_line_list = set_virtual_wall(vw_list[0])
    car2_VW_list, car2_vw_line_list = set_virtual_wall(vw_list[1])
    car3_VW_list, car3_vw_line_list = set_virtual_wall(vw_list[2])
    car4_VW_list, car4_vw_line_list = set_virtual_wall(vw_list[3])
    
    
    vw_num = len(car1_VW_list)/4 + len(car2_VW_list)/4 + len(car3_VW_list)/4 + len(car4_VW_list)/4
    f.writelines("vw_num::"+str(vw_num))
    f.writelines('\n')
    # print("car1_vw:"+str(car1_VW_list))
    # print("car2_vw:"+str(car2_VW_list))
    # print("car3_vw:"+str(car3_VW_list))
    # print("car4_vw:"+str(car4_VW_list))

    # car1_start_goal_list = setting.car1_STARTtoGOAL
    # car2_start_goal_list = setting.car2_STARTtoGOAL
    # car3_start_goal_list = setting.car3_STARTtoGOAL
    # car4_start_goal_list = setting.car4_STARTtoGOAL

    cars_tuple = ((setting.car1_STARTtoGOAL[0],setting.car1_STARTtoGOAL[1]), (setting.car2_STARTtoGOAL[0],setting.car2_STARTtoGOAL[1]), (setting.car3_STARTtoGOAL[0],setting.car3_STARTtoGOAL[1]), (setting.car4_STARTtoGOAL[0],setting.car4_STARTtoGOAL[1]))



    wall_edge, wall_line = set_wall()

    car1_vertex_list = set_vertex_list(car1_VW_list, cars_tuple[0], wall_edge)
    car2_vertex_list = set_vertex_list(car2_VW_list, cars_tuple[1], wall_edge)
    car3_vertex_list = set_vertex_list(car3_VW_list, cars_tuple[2], wall_edge)
    car4_vertex_list = set_vertex_list(car4_VW_list, cars_tuple[3], wall_edge)


    car1_vis_graph = visibility_graph(car1_vertex_list, car1_vw_line_list)
    car2_vis_graph = visibility_graph(car2_vertex_list, car2_vw_line_list)
    car3_vis_graph = visibility_graph(car3_vertex_list, car3_vw_line_list)
    car4_vis_graph = visibility_graph(car4_vertex_list, car4_vw_line_list)

    car1_shortest_path, car1_shortest_length = dijkstra(car1_vis_graph)
    car2_shortest_path, car2_shortest_length = dijkstra(car2_vis_graph)
    car3_shortest_path, car3_shortest_length = dijkstra(car3_vis_graph)
    car4_shortest_path, car4_shortest_length = dijkstra(car4_vis_graph)
    
    # f.writelines("car1_vw:"+str(car1_VW_list))
    # f.writelines('\n')
    # f.writelines("car2_vw:"+str(car2_VW_list))
    # f.writelines('\n')
    # f.writelines("car3_vw:"+str(car3_VW_list))
    # f.writelines('\n')
    # f.writelines("car4_vw:"+str(car4_VW_list))
    # f.writelines('\n')
    # f.writelines('\n')


    # for i in range(len(car1_shortest_path)):
    #     print("car1_path::"+"x:"+str(car1_vertex_list[car1_shortest_path[i]][0])+"y:"+str(car1_vertex_list[car1_shortest_path[i]][1]))
    #     f.writelines("car1_path::"+"x:"+str(car1_vertex_list[car1_shortest_path[i]][0])+"y:"+str(car1_vertex_list[car1_shortest_path[i]][1]))
    #     f.writelines('\n')
        
    # for i in range(len(car2_shortest_path)):
    #     print("car2_path::"+"x:"+str(car2_vertex_list[car2_shortest_path[i]][0])+"y:"+str(car2_vertex_list[car2_shortest_path[i]][1]))
    #     f.writelines("car2_path::"+"x:"+str(car2_vertex_list[car2_shortest_path[i]][0])+"y:"+str(car2_vertex_list[car2_shortest_path[i]][1]))
    #     f.writelines('\n')
    
    # for i in range(len(car3_shortest_path)):
    #     print("car3_path::"+"x:"+str(car3_vertex_list[car3_shortest_path[i]][0])+"y:"+str(car3_vertex_list[car3_shortest_path[i]][1]))
    #     f.writelines("car3_path::"+"x:"+str(car3_vertex_list[car3_shortest_path[i]][0])+"y:"+str(car3_vertex_list[car3_shortest_path[i]][1]))
    #     f.writelines('\n')

    # for i in range(len(car4_shortest_path)):
    #     print("car4_path::"+"x:"+str(car4_vertex_list[car4_shortest_path[i]][0])+"y:"+str(car4_vertex_list[car4_shortest_path[i]][1]))
    #     f.writelines("car4_path::"+"x:"+str(car4_vertex_list[car4_shortest_path[i]][0])+"y:"+str(car4_vertex_list[car4_shortest_path[i]][1]))
    all_len = car1_shortest_length + car2_shortest_length + car3_shortest_length + car4_shortest_length
    f.writelines("all_len::"+str(all_len))
    f.writelines('\n')

    collision = collision_CarToCar(car1_vertex_list, car1_shortest_path, car2_vertex_list, car2_shortest_path, car3_vertex_list, car3_shortest_path, car4_vertex_list, car4_shortest_path)
    f.writelines("collision::"+str(collision))
    f.writelines('\n')


    f.close()

    
    # print(car1_shortest_path)
    # print("testttt2222")

    # #car1の最短経路描画
    # for i in range(len(car1_shortest_path)-1):
    #      canvas.create_line(car1_vertex_list[car1_shortest_path[i]][0],car1_vertex_list[car1_shortest_path[i]][1], car1_vertex_list[car1_shortest_path[i+1]][0],car1_vertex_list[car1_shortest_path[i+1]][1], fill = "red", width = 5)
    # #canvas.create_line(car1_vertex_list,car1_vertex_list, car1_vertex_list, car1_vertex_list, fill = "red", width = 5)
    # #canvas.create_line(car1_vertex_list, car1_vertex_list, 860, 240, fill = "red", width = 5)
    # for i in range(len(car2_shortest_path)-1):
    #      canvas.create_line(car2_vertex_list[car2_shortest_path[i]][0],car2_vertex_list[car2_shortest_path[i]][1], car2_vertex_list[car2_shortest_path[i+1]][0],car2_vertex_list[car2_shortest_path[i+1]][1], fill = "blue", width = 5)
    
    # for i in range(len(car3_shortest_path)-1):
    #      canvas.create_line(car3_vertex_list[car3_shortest_path[i]][0],car3_vertex_list[car3_shortest_path[i]][1], car3_vertex_list[car3_shortest_path[i+1]][0],car3_vertex_list[car3_shortest_path[i+1]][1], fill = "yellow", width = 5)         
    
    # for i in range(len(car4_shortest_path)-1):
    #      canvas.create_line(car4_vertex_list[car4_shortest_path[i]][0],car4_vertex_list[car4_shortest_path[i]][1], car4_vertex_list[car4_shortest_path[i+1]][0],car4_vertex_list[car4_shortest_path[i+1]][1], fill = "black", width = 5)
         
    
    # x_size=25
    # y_size=10

    # #計算部分
    # while True:
    #     #CarAgent_1.move(270,0)
    #     canvas.coords(car1, car1_x0, car1_y0, car1_x0+x_size, car1_y0+y_size)
    #     canvas.coords(car2, car2_x0, car2_y0, car2_x0+y_size, car2_y0+x_size)
    #     canvas.coords(car3, car3_x0, car3_y0, car3_x0+x_size, car3_y0+y_size)
    #     canvas.coords(car4, car4_x0, car4_y0, car4_x0+y_size, car4_y0+x_size)
    #     car1_x0+=5
    #     car2_y0+=5
    #     car3_x0-=5
    #     car4_y0-=5
    #     time.sleep(0.02)
    #     tk.update() #ウインド画面を更新
        
    #     if car1_x0 >= 630:
    #         break
        

    # tk.mainloop()

if __name__ == '__main__':
    main()