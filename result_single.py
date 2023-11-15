import time
from tkinter import *
import numpy as np
import networkx as nx
import math
from geneticalgorithm2 import geneticalgorithm2 as ga

import vw
import setting


<<<<<<< HEAD
# def move(x, y, dx, dy):
#             rad = np.arctan(abs(dy - y)/abs(dx - x))
#             dig = math.degrees(rad)

#             if dx == x and dy == y:
#                 x += 0
#                 y += 0
#             else:
#                 #print("testttt")
#                 if  dx > x and dy > y:
#                     x += (math.cos(math.radians(dig))*setting.speed)
#                     y += (math.sin(math.radians(dig))*setting.speed)
#                 elif dx < x and dy > y:
#                     x -= (math.cos(math.radians(dig))*setting.speed)
#                     y += (math.sin(math.radians(dig))*setting.speed) 
#                 elif dx > x and dy < y:
#                     x += (math.cos(math.radians(dig))*setting.speed)
#                     y -= (math.sin(math.radians(dig))*setting.speed)  
#                 elif dx < x and dy < y:
#                     x -= (math.cos(math.radians(dig))*setting.speed)
#                     y -= (math.sin(math.radians(dig))*setting.speed)
#             return [x,y]



=======
>>>>>>> 16980ca4676e1340b9194fde184c3d491faf5fa8
def set_virtual_wall(GA_list):
    """
<<<<<<< HEAD
    遺伝的アルゴリズムの結果に対応したVWを設置する関数,VWの4つの頂点のlistと障害物の線分のlistを返す
    """
=======

        遺伝的アルゴリズムの結果に対応したVWを設置する関数,VWの4つの頂点のlistと障害物の線分のlistを返す
        """
>>>>>>> e0e05d466632c38eef4f25cfb2a451b9c8935c95
    size = setting.VWsize
    field_x = setting.VWfield_x
    field_y = setting.VWfield_y
    obstacles_vertex_list = []
    obstacles_line_list = []
    total_num_obstacles = 0
<<<<<<< HEAD
    
    #ga_list = [GA_list]

    #indexにインデックスをdeploy_checkには値(0,1)が入る.
    for index, oneDivisionList in enumerate(GA_list):
        for twoDivisionIndex, deploy_check in enumerate(oneDivisionList):
            if deploy_check >= 1:
=======

    #indexにインデックスをdeploy_checkには値(0,1)が入る.
    for index, oneDivisionList in enumerate(GA_list):
        #print(index)
        #print(oneDivisionList)
        for twoDivisionIndex, deploy_check in enumerate(oneDivisionList):
            if deploy_check >= 1.0:
>>>>>>> e0e05d466632c38eef4f25cfb2a451b9c8935c95
                total_num_obstacles += 1
                #VWの左上, 左下, 右上, 右下を設定
                VW_LeftUp = [(field_x + (size * twoDivisionIndex)), (field_y + (size * index))]
                VW_LeftDown = [VW_LeftUp[0], VW_LeftUp[1] + size]
                VW_RightUp = [VW_LeftUp[0] + size, VW_LeftUp[1]]
                VW_RightDown = [VW_LeftUp[0] + size, VW_LeftUp[1] + size]
<<<<<<< HEAD
                
                obstacles_vertex_list.extend([VW_LeftUp, VW_LeftDown, VW_RightUp, VW_RightDown])
                obstacles_line_list.extend([[VW_LeftUp, VW_LeftDown], [VW_LeftUp, VW_RightUp], [VW_RightUp, VW_RightDown], [VW_RightDown, VW_LeftDown]])
=======
            
                obstacles_vertex_list.extend([VW_LeftUp, VW_LeftDown, VW_RightUp, VW_RightDown])
                obstacles_line_list.extend([[VW_LeftUp, VW_LeftDown], [VW_LeftUp, VW_RightUp], [VW_RightUp, VW_RightDown], [VW_RightDown, VW_LeftDown]])

>>>>>>> e0e05d466632c38eef4f25cfb2a451b9c8935c95
    return obstacles_vertex_list, obstacles_line_list




def set_vertex_list(obstacle_list, carAgent, wall_edge):
        """
        頂点のリストを作成し返す関数
        """
        start = carAgent[0].copy()
        goal = carAgent[1].copy()
        vertex_list = [start, goal]
<<<<<<< HEAD

        vertex_list.extend(obstacle_list)
        vertex_list.extend(wall_edge)
        #print("ver"+str(vertex_list))
=======
        
        vertex_list.extend(obstacle_list)
        vertex_list.extend(wall_edge)

        print("ver"+str(vertex_list))
>>>>>>> e0e05d466632c38eef4f25cfb2a451b9c8935c95
        return vertex_list


def visibility_graph(vertex_list, obstacle_line_list):
    """
<<<<<<< HEAD

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

=======

    O(n^3)の素朴な可視グラフ法を実行し可視グラフのリストを返す関数
    """
    visibility_graph_list = []
    #cnt=0
    #vertex_listが2次元ではなく1次元になっているので確認＆修正
    #print(vertex_list)
    for index, vertex_u in enumerate(vertex_list):  # 頂点とインデックスのペアを取得 O(n)
        for goal_index, vertex_v in enumerate(vertex_list[index + 1:], index + 1):  # インデックスの次の頂点から順番に取り出す O(n)
            #vertex_u, vertex_vをつなぐ線分をLineとし
            Line = [index,goal_index]

            #障害物との交差しているかのFlagを立てる
            cross = False

>>>>>>> e0e05d466632c38eef4f25cfb2a451b9c8935c95
            #全ての障害物の各辺に対し
            for obstacle_Line in obstacle_line_list: #O(n)
                #print(obstacle_Line)
                #print(vertex_v[0])
                #障害物の各辺に対し衝突を判定する
                #外積による線分交差判定
                s = (vertex_v[0] - vertex_u[0])*(obstacle_Line[0][1] - vertex_u[1]) - (obstacle_Line[0][0] - vertex_u[0]) * (vertex_v[1] - vertex_u[1])#外積の計算
                t = (vertex_v[0] - vertex_u[0])*(obstacle_Line[1][1] - vertex_u[1]) - (obstacle_Line[1][0] - vertex_u[0]) * (vertex_v[1] - vertex_u[1])
                
<<<<<<< HEAD
                if s * t < 0.0:
=======
                #cnt+=1
                #print(str(cnt))
                if s * t < 0:
>>>>>>> e0e05d466632c38eef4f25cfb2a451b9c8935c95
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


<<<<<<< HEAD
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
=======

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

        #print("collision::"+str(collision))    
        return collision


def set_wall():
    wall_edge_list = []
    #設置する壁の考慮すべきエッジをlistにまとめる
    wall_edge_list = setting.wall_edge

    wall_line_list = setting.wall_line
    return wall_edge_list, wall_line_list
>>>>>>> e0e05d466632c38eef4f25cfb2a451b9c8935c95




def main():
    #solution_list = vw.main()
<<<<<<< HEAD
<<<<<<< HEAD
    solution_list = [0.12807571169837018, 1.446460077397614, 0.21220710989775826, 0.919509667225882, 0.4725977492749429, 0.76309856975463, 0.8894529019224491, 0.1834743585238885, 1.9831858421554305, 0.1619855834570696, 0.6264130728301109, 0.6319109273592178, 0.250855702892377, 0.40470334989922785, 0.3935632407029115, 0.035516500784098515]
=======
    solution_list = [0.5076311447131427, 0.7563196660654035, 0.7254599797399031, 0.49421900182459266, 0.6392765104528157, 0.5494367629618895, 1.5466656320930698, 1.3446976077799442, 0.7945376667338466, 1.654188859713017, 0.23270557992566587, 0.5944521344152265, 0.7332902007851281, 0.27097563263181734, 0.6721005822570647, 0.9500005032307233]
>>>>>>> 22250fd67639fa38eee7cd2106ea5664055c30be
    vw_list = np.array(solution_list).reshape(4,setting.VWnum).tolist()
=======
    solution_list = [0.7046529106516692, 0.7414379033390146, 1.7963305316295382, 0.00723861325036701, 0.5435356674902923, 1.5221485624143458, 0.16079068713896083, 1.070680339968169, 0.53861981244324, 0.2934772141614258, 0.5337647887341432, 0.17887054510638753, 0.6639228344962633, 0.6783791245638147, 0.34320404617215305, 0.7658618893590432, 1.1764062897299539, 0.10416876295172117, 0.13581214608076353, 1.331994353174494, 0.22591842924040595, 0.9964074513916079, 0.21709934817729937, 0.49226665787252166, 0.7650827945420631, 0.18015725490435175, 0.2597086615687907, 0.5206479189835629, 0.6209859958246235, 0.08064530979541562, 0.5077964722735415, 0.3204572900492129, 0.1623428751784468, 0.6849070484814133, 0.1889496680650048, 0.5422998821470386, 0.2850233044388897, 0.6420250634630047, 0.24643729615878773, 0.6742578305740159, 0.9628399868429377, 0.43425191262400675, 0.8021027631482329, 0.9448991773201261, 0.47737154809220184, 1.1603988638211449, 0.8438889576697612, 0.2943131646321311, 1.620445737596455, 1.79113618063909, 0.365518914688443, 0.8699791859891194, 0.17348569518447388, 0.6089322621493889, 0.6031345963608301, 1.7346447712432362, 0.6992219901573791, 0.40288331863425464, 1.1565014739556483, 0.8512710506891601, 0.23491725312585787, 0.6503092699969987, 0.3283694938683328, 0.27957422037478796]
    #vw_list = np.array(solution_list)
    #vw_list = np.array(solution_list).reshape(4,setting.VWnum**2).tolist()
    vw_list = np.array(solution_list).reshape(4,4,4).tolist()
    print(vw_list)

    f = open('data_single.txt', 'x', encoding='UTF-8')
    f.writelines(str(vw_list))
    f.writelines('\n')
=======
    solution_list = [0.47453239893057964, 0.5098964558099106, 0.9359031084198701, 0.21607521209894887, 1.1773540872651616, 0.011349372092898546, 0.5841750334502656, 0.2321845211363558, 0.5162700187978769, 1.1645811701343183, 0.8932849100846982, 0.5281673667945721, 0.2315716969861169, 0.9437347006192403, 1.6927743663936954, 0.8930353904761033]
    #vw_list = np.array(solution_list)
    #vw_list = np.array(solution_list).reshape(4,setting.VWnum**2).tolist()
    vw_list = np.array(solution_list).reshape(4,4).tolist()
    print(vw_list)

    f = open('data_single.txt', 'a', encoding='UTF-8')
    f.writelines(str(vw_list))
    f.writelines('\n')
    f.writelines('\n')
>>>>>>> e0e05d466632c38eef4f25cfb2a451b9c8935c95

>>>>>>> 16980ca4676e1340b9194fde184c3d491faf5fa8
    
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

<<<<<<< HEAD
    car1_x0 = setting.car1_STARTtoGOAL[0][0]
    car1_y0 = setting.car1_STARTtoGOAL[0][1]
    car2_x0 = setting.car2_STARTtoGOAL[0][0]
    car2_y0 = setting.car2_STARTtoGOAL[0][1]
    car3_x0 = setting.car3_STARTtoGOAL[0][0]
    car3_y0 = setting.car3_STARTtoGOAL[0][1]
    car4_x0 = setting.car4_STARTtoGOAL[0][0]
    car4_y0 = setting.car4_STARTtoGOAL[0][1]
=======
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
>>>>>>> 16980ca4676e1340b9194fde184c3d491faf5fa8
        
    # ##4vw用
    # for i in range(4):
    #     car_vw = np.array(vw_list[i]).reshape(setting.VWnum,setting.VWnum).tolist()
    #     vw_point_x = setting.VWfield_x
    #     vw_point_y = setting.VWfield_y
    #     list1 = []
    #     list2 = []
    #     list3 = []
    #     list4 = []
<<<<<<< HEAD

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
    # #共通vw用
    # car1_VW_list, car1_vw_line_list = set_virtual_wall(vw_list)
    # car2_VW_list, car2_vw_line_list = set_virtual_wall(vw_list)
    # car3_VW_list, car3_vw_line_list = set_virtual_wall(vw_list)
    # car4_VW_list, car4_vw_line_list = set_virtual_wall(vw_list)
    
    
=======

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
    # #共通vw用


>>>>>>> e0e05d466632c38eef4f25cfb2a451b9c8935c95
    car_VW_list, car_vw_line_list = set_virtual_wall(vw_list)

    vw_num = len(car_VW_list)/4
    f.writelines("vw_num::"+str(vw_num))
    f.writelines('\n')
<<<<<<< HEAD
    # print("car1_vw:"+str(car1_VW_list))
    # print("car2_vw:"+str(car2_VW_list))
    # print("car3_vw:"+str(car3_VW_list))
    # print("car4_vw:"+str(car4_VW_list))
=======
>>>>>>> e0e05d466632c38eef4f25cfb2a451b9c8935c95

    # car1_start_goal_list = setting.car1_STARTtoGOAL
    # car2_start_goal_list = setting.car2_STARTtoGOAL
    # car3_start_goal_list = setting.car3_STARTtoGOAL
    # car4_start_goal_list = setting.car4_STARTtoGOAL

    cars_tuple = ((setting.car1_STARTtoGOAL[0],setting.car1_STARTtoGOAL[1]), (setting.car2_STARTtoGOAL[0],setting.car2_STARTtoGOAL[1]), (setting.car3_STARTtoGOAL[0],setting.car3_STARTtoGOAL[1]), (setting.car4_STARTtoGOAL[0],setting.car4_STARTtoGOAL[1]))
<<<<<<< HEAD
    
    wall_edge, wall_line = set_wall()

=======

    wall_edge, wall_line = set_wall()

>>>>>>> e0e05d466632c38eef4f25cfb2a451b9c8935c95
    car1_vertex_list = set_vertex_list(car_VW_list, cars_tuple[0], wall_edge)
    car2_vertex_list = set_vertex_list(car_VW_list, cars_tuple[1], wall_edge)
    car3_vertex_list = set_vertex_list(car_VW_list, cars_tuple[2], wall_edge)
    car4_vertex_list = set_vertex_list(car_VW_list, cars_tuple[3], wall_edge)


<<<<<<< HEAD
    car1_vis_graph = visibility_graph(car1_vertex_list, car1_vw_line_list)
    car2_vis_graph = visibility_graph(car2_vertex_list, car2_vw_line_list)
    car3_vis_graph = visibility_graph(car3_vertex_list, car3_vw_line_list)
    car4_vis_graph = visibility_graph(car4_vertex_list, car4_vw_line_list)
=======
    car1_vis_graph = visibility_graph(car1_vertex_list, car_vw_line_list)
    car2_vis_graph = visibility_graph(car2_vertex_list, car_vw_line_list)
    car3_vis_graph = visibility_graph(car3_vertex_list, car_vw_line_list)
    car4_vis_graph = visibility_graph(car4_vertex_list, car_vw_line_list)
>>>>>>> e0e05d466632c38eef4f25cfb2a451b9c8935c95

    car1_shortest_path, car1_shortest_length = dijkstra(car1_vis_graph)
    car2_shortest_path, car2_shortest_length = dijkstra(car2_vis_graph)
    car3_shortest_path, car3_shortest_length = dijkstra(car3_vis_graph)
    car4_shortest_path, car4_shortest_length = dijkstra(car4_vis_graph)
<<<<<<< HEAD
    
    # f.writelines("car1_vw:"+str(car1_VW_list))
    # f.writelines('\n')
    # f.writelines("car2_vw:"+str(car2_VW_list))
    # f.writelines('\n')
    # f.writelines("car3_vw:"+str(car3_VW_list))
    # f.writelines('\n')
    # f.writelines("car4_vw:"+str(car4_VW_list))
    # f.writelines('\n')
    # f.writelines('\n')

=======

    collision = collision_CarToCar(car1_vertex_list, car1_shortest_path, car2_vertex_list, car2_shortest_path, car3_vertex_list, car3_shortest_path, car4_vertex_list, car4_shortest_path)
    f.writelines("collision::"+str(collision))
    f.writelines('\n')
    # f.writelines("car1_vw:"+str(car1_VW_list))
    # f.writelines('\n')
    # f.writelines("car2_vw:"+str(car2_VW_list))
    # f.writelines('\n')
    # f.writelines("car3_vw:"+str(car3_VW_list))
    # f.writelines('\n')
    # f.writelines("car4_vw:"+str(car4_VW_list))
    # f.writelines('\n')
    # f.writelines('\n')

>>>>>>> e0e05d466632c38eef4f25cfb2a451b9c8935c95

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
<<<<<<< HEAD
    all_len = car1_shortest_length + car2_shortest_length + car3_shortest_length + car4_shortest_length
    f.writelines("all_len::"+str(all_len))
    f.writelines('\n')

    collision = collision_CarToCar(car1_vertex_list, car1_shortest_path, car2_vertex_list, car2_shortest_path, car3_vertex_list, car3_shortest_path, car4_vertex_list, car4_shortest_path)
    f.writelines("collision::"+str(collision))
    f.writelines('\n')


    f.close()

<<<<<<< HEAD
    #car1の最短経路描画
    for i in range(len(car1_shortest_path)-1):
         canvas.create_line(car1_vertex_list[car1_shortest_path[i]][0],car1_vertex_list[car1_shortest_path[i]][1], car1_vertex_list[car1_shortest_path[i+1]][0],car1_vertex_list[car1_shortest_path[i+1]][1], fill = "red", width = 3)    

    for i in range(len(car2_shortest_path)-1):
         canvas.create_line(car2_vertex_list[car2_shortest_path[i]][0],car2_vertex_list[car2_shortest_path[i]][1], car2_vertex_list[car2_shortest_path[i+1]][0],car2_vertex_list[car2_shortest_path[i+1]][1], fill = "blue", width = 3)
=======
    
=======
    #     f.writelines('\n')
>>>>>>> 16980ca4676e1340b9194fde184c3d491faf5fa8
    
    all_path_length = car1_shortest_length + car2_shortest_length + car3_shortest_length + car4_shortest_length
    f.writelines("all_len::"+str(all_path_length))
    f.writelines('\n')

    f.close()

    
>>>>>>> e0e05d466632c38eef4f25cfb2a451b9c8935c95
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
        
<<<<<<< HEAD
        canvas.coords(car1, car1_x0, car1_y0, car1_x0+x_size, car1_y0+y_size)
        canvas.coords(car2, car2_x0, car2_y0, car2_x0+y_size, car2_y0+x_size)
        canvas.coords(car3, car3_x0, car3_y0, car3_x0+x_size, car3_y0+y_size)
        canvas.coords(car4, car4_x0, car4_y0, car4_x0+y_size, car4_y0+x_size)


<<<<<<< HEAD
        # for i in range(len(car1_shortest_path)-1):
        #     #canvas.coords(car1, car1_vertex_list[car1_shortest_path[i]][0],car1_vertex_list[car1_shortest_path[i]][1], car1_vertex_list[car1_shortest_path[i+1]][0],car1_vertex_list[car1_shortest_path[i+1]][1])
        #     canvas.move(car1, car1_vertex_list[car1_shortest_path[i+1]][0] - car1_vertex_list[car1_shortest_path[i]][0], car1_vertex_list[car1_shortest_path[i+1]][1] - car1_vertex_list[car1_shortest_path[i]][1])
        #     canvas.after(10000, move)
=======
        for i in range(len(car1_shortest_path)-1):
            #canvas.coords(car1, car1_vertex_list[car1_shortest_path[i]][0],car1_vertex_list[car1_shortest_path[i]][1], car1_vertex_list[car1_shortest_path[i+1]][0],car1_vertex_list[car1_shortest_path[i+1]][1])
            canvas.move(car1, car1_vertex_list[car1_shortest_path[i+1]][0] - car1_vertex_list[car1_shortest_path[i]][0], car1_vertex_list[car1_shortest_path[i+1]][1] - car1_vertex_list[car1_shortest_path[i]][1])
            #canvas.after(10000, canvas.move)
>>>>>>> 22250fd67639fa38eee7cd2106ea5664055c30be

        #car1_x0+=5
        #car2_y0+=5
        #car3_x0-=5
        #car4_y0-=5
        #time.sleep(0.02)
        tk.update() #ウインド画面を更新
        
        if car1_x0 >= 630:
            break
=======
    #     if car1_x0 >= 630:
    #         break
>>>>>>> 16980ca4676e1340b9194fde184c3d491faf5fa8
        

    # tk.mainloop()

if __name__ == '__main__':
    main()