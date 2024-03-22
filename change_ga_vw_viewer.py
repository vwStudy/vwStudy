import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import math

import ga
import setting

import time

class VW():
    """
    Virtual Wallを管理するクラス
    """
    def __init__(self, x, y, size):
        self.x = x
        self.y = y
        self.size = size
    
    def set_virtual_wall(GA_list, VWsize = setting.VWsize):
        """
        遺伝的アルゴリズムの結果に対応したVWを設置する関数,VWの4つの頂点のlistと障害物の線分のlistを返す
        """
        #print("galist"+str(GA_list))
        size = VWsize
        field_x = setting.VWfield_x
        field_y = setting.VWfield_y
        obstacles_vertex_list = []
        obstacles_line_list = []
        total_num_obstacles = 0
        n = 1
        #indexにインデックスをdeploy_checkには値(0,1)が入る.
        for index, oneDivisionList in enumerate(GA_list):
            for twoDivisionIndex, deploy_check in enumerate(oneDivisionList):
                if deploy_check >= 1:
                    total_num_obstacles += 1
                    #VWの左上, 左下, 右上, 右下を設定
                    VW_LeftUp = [(field_x + (size * twoDivisionIndex)) * n, (field_y + (size * index)) * n]
                    VW_LeftDown = [VW_LeftUp[0] * n, (VW_LeftUp[1] + size) * n]
                    VW_RightUp = [(VW_LeftUp[0] + size) * n, VW_LeftUp[1] * n]
                    VW_RightDown = [(VW_LeftUp[0] + size) * n, (VW_LeftUp[1] + size) * n]
                    
                    obstacles_vertex_list.extend([VW_LeftUp, VW_LeftDown, VW_RightUp, VW_RightDown])
                    obstacles_line_list.extend([[VW_LeftUp, VW_LeftDown], [VW_LeftUp, VW_RightUp], [VW_RightUp, VW_RightDown], [VW_RightDown, VW_LeftDown]])
        return obstacles_vertex_list, obstacles_line_list

    def vw_result(solution):
        """
        GeneticalAlgorism用の関数
        """
        #print("genom2::::::", genom)
        car_ga_array = [[[]]*setting.VWnum]
        ga_array = np.array(solution.reshape(setting.car_num, setting.VWnum, setting.VWnum))
        for i in range(len(ga_array)):
            for j in range(len(ga_array[i])):
                l = list(ga_array[i][j])
                car_ga_array[i][j] = l

        car_VW_list, car_vw_line_list = VW.set_virtual_wall(car_ga_array[0])
        combine_vw_start= time.time()
        print(car_VW_list)
        plot_VW_list = car_VW_list[0::4]
        print(plot_VW_list)
        car_VW_list = combining_vw(car_VW_list)
        # car_vw_line_list = combining_vw(car_vw_line_list)
        combine_vw_end = time.time()
        combine_time_diff = combine_vw_end - combine_vw_start

        cars_tuple = (CarAgent(setting.car1_STARTtoGOAL[0],setting.car1_STARTtoGOAL[1]), CarAgent(setting.car2_STARTtoGOAL[0],setting.car2_STARTtoGOAL[1]), CarAgent(setting.car3_STARTtoGOAL[0],setting.car3_STARTtoGOAL[1]), CarAgent(setting.car4_STARTtoGOAL[0],setting.car4_STARTtoGOAL[1]))

        wall_edge_list, wall_line_list = Environment.set_wall()

        create_vertex_start = time.time()
        #頂点のlistを作成
        car1_vertex_list = Environment.set_vertex_list(car_VW_list, cars_tuple[0], wall_edge_list)
        car2_vertex_list = Environment.set_vertex_list(car_VW_list, cars_tuple[1], wall_edge_list)
        car3_vertex_list = Environment.set_vertex_list(car_VW_list, cars_tuple[2], wall_edge_list)
        car4_vertex_list = Environment.set_vertex_list(car_VW_list, cars_tuple[3], wall_edge_list)
        create_vertex_end = time.time()
        vertex_time_diff = create_vertex_end - create_vertex_start
        #print("vertex::",vertex_time_diff)

        visibility_start = time.time()
        #可視グラフ, ダイクストラ法を実行
        car1_vis_graph = Execution.visibility_graph(car1_vertex_list, car_vw_line_list)
        car2_vis_graph = Execution.visibility_graph(car2_vertex_list, car_vw_line_list)
        car3_vis_graph = Execution.visibility_graph(car3_vertex_list, car_vw_line_list)
        car4_vis_graph = Execution.visibility_graph(car4_vertex_list, car_vw_line_list)
        visibility_end = time.time()
        visibility_time_diff = visibility_end - visibility_start
        #print("visibility::",visibility_time_diff)
        
        dijkstra_start = time.time()
        car1_shortest_path, car1_shortest_length = Execution.dijkstra(car1_vis_graph)
        car2_shortest_path, car2_shortest_length = Execution.dijkstra(car2_vis_graph)
        car3_shortest_path, car3_shortest_length = Execution.dijkstra(car3_vis_graph)
        car4_shortest_path, car4_shortest_length = Execution.dijkstra(car4_vis_graph)
        dijkstra_end = time.time()
        dijkstra_time_diff = dijkstra_end - dijkstra_start
        #print("dijkstra::",dijkstra_time_diff)        

        flag1 = False 
        flag2 = False
        flag3 = False
        flag4 = False
        collision = 0
        num1 = 0
        num2 = 0
        num3 = 0
        num4 = 0 
        move_count1 = 0
        move_count2 = 0
        move_count3 = 0
        move_count4 = 0
        need_move1 = 0
        need_move2 = 0
        need_move3 = 0
        need_move4 = 0
        carb_rate1 = 0
        carb_rate2 = 0
        carb_rate3 = 0
        carb_rate4 = 0
        car1_start_position = setting.car1_STARTtoGOAL[0].copy()
        car2_start_position = setting.car2_STARTtoGOAL[0].copy()
        car3_start_position = setting.car3_STARTtoGOAL[0].copy()
        car4_start_position = setting.car4_STARTtoGOAL[0].copy()
        car1_position = car1_start_position
        car2_position = car2_start_position
        car3_position = car3_start_position
        car4_position = car4_start_position
        move_count = 0
        curve_count1 = 0
        curve_count2 = 0
        curve_count3 = 0
        curve_count4 = 0

        cars_position_list = [[],[],[],[]]
        while(flag1==False and flag2==False and flag3==False and flag4==False):
            car1_position, flag1, num1, move_count1, need_move1, carb_rate1, curve_count1 = cars_tuple[0].car_move(car1_vertex_list, car1_shortest_path, car1_position, num1, move_count1, need_move1, carb_rate1, curve_count1)
            car2_position, flag2, num2, move_count2, need_move2, carb_rate2, curve_count2 = cars_tuple[1].car_move(car2_vertex_list, car2_shortest_path, car2_position, num2, move_count2, need_move2, carb_rate2, curve_count2)
            car3_position, flag3, num3, move_count3, need_move3, carb_rate3, curve_count3 = cars_tuple[2].car_move(car3_vertex_list, car3_shortest_path, car3_position, num3, move_count3, need_move3, carb_rate3, curve_count3)
            car4_position, flag4, num4, move_count4, need_move4, carb_rate4, curve_count4 = cars_tuple[3].car_move(car4_vertex_list, car4_shortest_path, car4_position, num4, move_count4, need_move4, carb_rate4, curve_count4)
            cars_position_list[0].append(car1_position.copy())
            cars_position_list[1].append(car2_position.copy())
            cars_position_list[2].append(car3_position.copy())
            cars_position_list[3].append(car4_position.copy())
            #car1_position=position1.copy()
            # print(car1_position)
            collision = Environment.collision_CarToCar(car1_position, car2_position, car3_position, car4_position, collision)

        cars_path_list = []
        car_path_tmp_list = []
        for path in car1_shortest_path:
            #print("car1 :",car1_vertex_list[path])
            car_path_tmp_list.append(car1_vertex_list[path])
        cars_path_list.append(car_path_tmp_list)
        car_path_tmp_list=[]

        for path in car2_shortest_path:
            #print("car2 :",car2_vertex_list[path])
            car_path_tmp_list.append(car2_vertex_list[path])
        cars_path_list.append(car_path_tmp_list)
        car_path_tmp_list=[]

        for path in car3_shortest_path:
            #print("car3 :",car3_vertex_list[path])
            car_path_tmp_list.append(car3_vertex_list[path])
        cars_path_list.append(car_path_tmp_list)
        car_path_tmp_list=[]

        for path in car4_shortest_path:
            #print("car4 :",car4_vertex_list[path])
            car_path_tmp_list.append(car4_vertex_list[path])
        cars_path_list.append(car_path_tmp_list)

        total_num_obstacles = len(car_VW_list)/4

        #全ての経路長を足す
        all_path_length = car1_shortest_length + car2_shortest_length + car3_shortest_length + car4_shortest_length

        return cars_position_list, plot_VW_list, wall_edge_list


class Environment():
    def __init__(self, obstacle_x, obstacle_y, width, height):
        self.x = obstacle_x
        self.y = obstacle_y
        self.width = width
        self.height = height
    
    def set_wall():
        """
        
        """
        wall_edge_list = []
        #設置する壁の考慮すべきエッジをlistにまとめる
        wall_edge_list = setting.wall_edge_list

        wall_line_list = setting.wall_line_list
        return wall_edge_list, wall_line_list

    def collision_CarToCar(car1, car2, car3, car4, collision):
        r = np.sqrt((setting.car_length/2)**2 + (setting.car_width/2)**2)


        collision_checker1to2 = np.sqrt((car2[0]-car1[0])**2 + (car2[1]-car1[1])**2)
        collision_checker1to3 = np.sqrt((car3[0]-car1[0])**2 + (car3[1]-car1[1])**2)
        collision_checker1to4 = np.sqrt((car4[0]-car1[0])**2 + (car4[1]-car1[1])**2)
        collision_checker2to3 = np.sqrt((car2[0]-car3[0])**2 + (car2[1]-car3[1])**2)
        collision_checker2to4 = np.sqrt((car2[0]-car4[0])**2 + (car2[1]-car4[1])**2)
        collision_checker3to4 = np.sqrt((car2[0]-car4[0])**2 + (car2[1]-car4[1])**2)
        
        # print("hannkkei",r)
        if collision_checker1to2 <= r:
            collision += 1
        elif collision_checker1to3 <= r:
            collision += 1
        elif collision_checker1to4 <= r:
            collision += 1
        elif collision_checker2to3 <= r:
            collision += 1
        elif collision_checker2to4 <= r:
            collision += 1
        elif collision_checker3to4 <= r:
            collision += 1

        return collision


    def set_vertex_list(obstacle_list, carAgent, wall_edge):
        """
        
        頂点のリストを作成し返す関数
        """
        start = carAgent.start.copy()
        goal = carAgent.goal.copy()

        vertex_list = [start, goal]

        vertex_list.extend(obstacle_list)
        vertex_list.extend(wall_edge)

        return vertex_list

class CarAgent():
    def __init__(self, start, goal):
        self.start = start
        self.goal = goal
        self.x = start[0]
        self.y = start[1]
        self.position = start
        self.goal_x = goal[0]
        self.goal_y = goal[1]
        self.goal_flag = False
        self.car_width = setting.car_width #根拠のある数値にする

    def car_move(self, car_vertex_list, car_shortest_path, car_position, num, move_count, need_move, curve_rate, curve_count):
        next_num = 0

        #車両の変化可能角度
        car_angle = setting.car_angle
        
        if num >= len(car_shortest_path):
            self.goal_flag = True
        
        elif num < len(car_shortest_path):
            node = car_vertex_list[car_shortest_path[num]]
            if num == 0:
                if len(car_shortest_path) > num+2:
                    after_node = car_vertex_list[car_shortest_path[num+1]]
                    curve_angle, length = calculate_two_vec_angle(node, after_node, car_vertex_list[car_shortest_path[num+2]])
                    curve_angle = 180-curve_angle
                    print("curve_angle",curve_angle)
                    if curve_angle <= car_angle:
                        car_angle = curve_angle
                    
                    else:
                        car_angle = setting.car_angle

                    need_move = math.ceil(length/setting.speed)
                    print(int(need_move))
                    
                    curve_rate = math.ceil(curve_angle/car_angle)
                    print("curve_rate",curve_rate)

            if node[0] - car_position[0] == 0 and (node[1] - car_position[1]) >= 0:
                car_position[1] += setting.speed
                if car_position[1] >= node[1]:
                    car_position[0] = node[0]
                    car_position[1] = node[1]
                    next_num = num + 1
                
            elif node[0] - car_position[0] == 0 and (node[1] - car_position[1]) <= 0:
                car_position[1] -= setting.speed
                if car_position[1] <= node[1]:
                    car_position[0] = node[0]
                    car_position[1] = node[1]
                    next_num = num + 1
                
            elif node[1] - car_position[1] == 0 and (node[0] - car_position[0]) >= 0:
                car_position[0] += setting.speed
                if car_position[0] <= node[0]:
                    car_position[0] = node[0]
                    car_position[1] = node[1]
                    next_num = num + 1
                
            elif node[1] - car_position[1] == 0 and (node[0] - car_position[0]) <= 0:
                car_position[0] -= setting.speed
                if car_position[0] >= node[0]:
                    car_position[0] = node[0]
                    car_position[1] = node[1]
                    next_num = num + 1
                    
            elif abs(node[0] - car_position[0]) == 0 and (node[1] - car_position[1]) >= 0 and (need_move - move_count) <= curve_rate and (curve_rate - curve_count) != 0:
                car_position[1] += setting.speed * np.sin(np.arctan(np.deg2rad(car_angle)))
                curve_count += 1
                if car_position[1] >= node[1]:
                    car_position[0] = node[0]
                    car_position[1] = node[1]
                    next_num = num + 1
                
            elif abs(node[0] - car_position[0]) == 0 and (node[1] - car_position[1]) <= 0 and (need_move - move_count) <= curve_rate and (curve_rate - curve_count) != 0:
                car_position[1] -= setting.speed * np.sin(np.arctan(np.deg2rad(car_angle)))
                curve_count += 1
                if car_position[1] <= node[1]:
                    car_position[0] = node[0]
                    car_position[1] = node[1]
                    next_num = num + 1
                
            elif abs(node[1] - car_position[1]) == 0 and (node[0] - car_position[0]) >= 0 and (need_move - move_count) <= curve_rate and (curve_rate - curve_count) != 0:
                car_position[0] += setting.speed *  np.cos(np.arctan(np.deg2rad(car_angle)))
                curve_count += 1
                if car_position[0] <= node[0]:
                    car_position[0] = node[0]
                    car_position[1] = node[1]
                    next_num = num + 1
                
            elif abs(node[1] - car_position[1]) == 0 and (node[0] - car_position[0]) <= 0 and (need_move - move_count) <= curve_rate and (curve_rate - curve_count) != 0:
                car_position[0] -= setting.speed * np.cos(np.arctan(np.deg2rad(car_angle)))
                curve_count += 1
                if car_position[0] >= node[0]:
                    car_position[0] = node[0]
                    car_position[1] = node[1]
                    next_num = num + 1
                
            else:
                if (need_move - move_count) <= curve_rate and (curve_rate - curve_count) != 0:
                    rad = np.arctan(abs(node[1] - car_position[1])/abs(node[0] - car_position[0]) - np.deg2rad(car_angle))
                    curve_count += 1

                    rad = np.arctan(abs(node[1] - car_position[1])/abs(node[0] - car_position[0]))
                # print("carposi0",car_position[0])
                # print("carposi1",car_position[1])
                # print("node0",node[0])
                # print("node1",node[1])
                if  car_position[0] > node[0] and car_position[1] > node[1]:    
                    
                    car_position[0] -= np.cos(rad) * setting.speed
                    car_position[1] -= np.sin(rad) * setting.speed
                    
                    if car_position[0] <= node[0] and car_position[1] <= node[1]:
                        
                        car_position[0] = node[0]
                        car_position[1] = node[1]
                        
                        next_num= num + 1
                    
                elif car_position[0] < node[0] and car_position[1] > node[1]:
                
                    car_position[0] += np.cos(rad) * setting.speed
                    car_position[1] -= np.sin(rad) * setting.speed

                    if car_position[0] >= node[0] and car_position[1] <= node[1]:
                        
                        car_position[0] = node[0]
                        car_position[1] = node[1]
                        
                        next_num= num + 1

                elif car_position[0] > node[0] and car_position[1] < node[1]:
                    
                    car_position[0] -= np.cos(rad) * setting.speed
                    car_position[1] += np.sin(rad) * setting.speed

                    if car_position[0] <= node[0] and car_position[1] >= node[1]:
                        
                        car_position[0] = node[0]
                        car_position[1] = node[1]
                        
                        next_num= num + 1

                elif car_position[0] < node[0] and car_position[1] < node[1]:
                    
                    car_position[0] += np.cos(rad) * setting.speed
                    car_position[1] += np.sin(rad) * setting.speed

                    if car_position[0] >= node[0] and car_position[1] >= node[1]:
                        
                        car_position[0] = node[0]
                        car_position[1] = node[1]
                        
                        next_num= num + 1

        # elif car_position[0] == node[0] and car_position[1] == node[1]:
        #     if len(car_shortest_path-1):
        #        self.flag = True
            move_count += 1
            
            if num >= next_num:
                next_num = num
            
            else:
                next_num = num + 1
                curve_count = 0
                if len(car_shortest_path) > num+2:
                    after_node = car_vertex_list[car_shortest_path[num+1]]
                    curve_angle, length = calculate_two_vec_angle(node, after_node, car_vertex_list[car_shortest_path[num+2]])
                    curve_angle = 180-curve_angle
                    print("curve_angle",curve_angle)
                    print("curve_rate",curve_rate)
                    
                    if curve_angle <= car_angle:
                        car_angle = curve_angle

                    need_move = math.ceil(length/setting.speed)
                    print(int(need_move))
                    
                    curve_rate = math.ceil(curve_angle/car_angle)

        # print("caaaaaaaa", car_position)    
        return car_position ,self.goal_flag, next_num, move_count, need_move, curve_rate, curve_count

class Execution():
    def set_obstacle(self):
        self.Obstacle_1 = Environment()
        self.Obstacle_2 = Environment()
        self.Obstacle_3 = Environment()
        self.Obstacle_4 = Environment()

    def visibility_graph(vertex_list, obstacle_line_list):

        """
    
        O(n^3)の素朴な可視グラフ法を実行し可視グラフのリストを返す関数
        """
        visibility_graph_list = []

        for index, vertex_u in enumerate(vertex_list):  # 頂点とインデックスのペアを取得 O(n)
            for goal_index, vertex_v in enumerate(vertex_list[index + 1:], index + 1):  # インデックスの次の頂点から順番に取り出す O(n)
                #vertex_u, vertex_vをつなぐ線分をLineとし
                Line = [index,goal_index]

                #障害物との交差しているかのFlagを立てる
                cross = False

                #全ての障害物の各辺に対し
                for obstacle_Line in obstacle_line_list: #O(n)

                    #障害物の各辺に対し衝突を判定する
                    #外積による線分交差判定
                    s = (vertex_v[0] - vertex_u[0])*(obstacle_Line[0][1] - vertex_u[1]) - (obstacle_Line[0][0] - vertex_u[0]) * (vertex_v[1] - vertex_u[1])#外積の計算
                    t = (vertex_v[0] - vertex_u[0])*(obstacle_Line[1][1] - vertex_u[1]) - (obstacle_Line[1][0] - vertex_u[0]) * (vertex_v[1] - vertex_u[1])
                    
                    if s * t < 0:
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

def main():
    solution = np.array([0,0,1,0,0, 0,0,1,0,0, 1,1,1,1,1, 0,0,1,0,0, 0,0,1,0,0])
    cars_position_list, plot_vw_list, wall_edge_list = VW.vw_result(solution)

    # plt.figure()
    # plt.title("car1")
    x = []
    y = []
    print(setting.VWsize)
    for i in range(len(cars_position_list[0])):
        x.append(cars_position_list[0][i][0])
        y.append(cars_position_list[0][i][1])
    for j in range(len(plot_vw_list)):
        vw = patches.Rectangle(plot_vw_list[j], setting.VWsize, setting.VWsize)

    # a = plt.plot(x, y)

    # plt.figure()
    # plt.title("car2")
    x1 = []
    y1 = []
    for i in range(len(cars_position_list[1])):
        x1.append(cars_position_list[1][i][0])
        y1.append(cars_position_list[1][i][1])
    # b = plt.plot(x1, y1)

    # plt.figure()
    # plt.title("car3")
    x2 = []
    y2 = []
    for i in range(len(cars_position_list[2])):
        x2.append(cars_position_list[2][i][0])
        y2.append(cars_position_list[2][i][1])
    # c = plt.plot(x2, y2)

    # plt.figure()
    # plt.title("car4")
    x3 = []
    y3 = []
    for i in range(len(cars_position_list[3])):
        x3.append(cars_position_list[3][i][0])
        y3.append(cars_position_list[3][i][1])
    # d = plt.plot(x3, y3)

    fig_all_path = plt.figure()
    plt.title("all_path")
    ax = plt.axes()
    ax.set_aspect('equal', "datalim")
    plt.plot(x,y)
    plt.plot(x1,y1)
    plt.plot(x2,y2)
    plt.plot(x3,y3)
    wall_x = []
    wall_y = []
    for i in range(len(wall_edge_list)):
        wall_x.append(wall_edge_list[i][0])
        wall_y.append(wall_edge_list[i][1])
    plt.scatter(wall_x,wall_y)
    for j in range(len(plot_vw_list)):
        vw = patches.Rectangle(plot_vw_list[j], setting.VWsize, setting.VWsize)
        ax.add_patch(vw)
    plt.show()


def combining_vw(Vw_list):
    """
    
    list内の重複を消す関数,VWの4点のいずれかが重複していたら削除しVWの疑似的な結合を行う
    """
    seen = []
    return [position for position in Vw_list if position not in seen and not seen.append(position)]

def calculate_two_vec_angle(pre_position, position, move_position):
    #配列に変換
    pre_position = np.array(pre_position)
    position = np.array(position)
    move_position = np.array(move_position)
    
    # NumPy配列に変換
    vec_a = np.array(position-pre_position)
    vec_b = np.array(move_position-position)

    # 内積を計算
    inner = np.inner(vec_a, vec_b)

    # 長さを計算
    vec_a_norm = np.linalg.norm(vec_a)
    vec_b_norm = np.linalg.norm(vec_b)

    vec_a_norm = round(vec_a_norm,5)
    vec_b_norm = round(vec_b_norm,5)

    theta = inner/(vec_a_norm*vec_b_norm)

    after_angle = np.rad2deg(np.arccos(np.clip(theta, -1.0, 1.0)))

    return after_angle, vec_a_norm


if __name__ == '__main__':
    sum_fitness = 0
    sum_collision = 0
    sum_all_path_length = 0
    sum_total_num_obstacles = 0
    sum_time = 0
    # for i in range(100):
    start = time.time()
    main()
    end = time.time()

    time_diff = end - start
    #print("time:" , time_diff)
    sum_time += time_diff