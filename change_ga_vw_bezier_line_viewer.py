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
        # print(car_VW_list)
        plot_VW_list = car_VW_list[0::4]
        # print(plot_VW_list)
        car_VW_list = combining_vw(car_VW_list)
        # car_vw_line_list = combining_vw(car_vw_line_list)
        combine_vw_end = time.time()
        combine_time_diff = combine_vw_end - combine_vw_start

        cars_tuple = (CarAgent(setting.car1_STARTtoGOAL[0],setting.car1_STARTtoGOAL[1]), CarAgent(setting.car2_STARTtoGOAL[0],setting.car2_STARTtoGOAL[1]), CarAgent(setting.car3_STARTtoGOAL[0],setting.car3_STARTtoGOAL[1]), CarAgent(setting.car4_STARTtoGOAL[0],setting.car4_STARTtoGOAL[1]))

        wall_edge_list, wall_line_list = Environment.set_wall()

        car_vw_line_list.extend(wall_line_list)

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
        car1_vis_graph = Execution.visibility_graph(car1_vertex_list, car_vw_line_list, wall_line_list)
        car2_vis_graph = Execution.visibility_graph(car2_vertex_list, car_vw_line_list, wall_line_list)
        car3_vis_graph = Execution.visibility_graph(car3_vertex_list, car_vw_line_list, wall_line_list)
        car4_vis_graph = Execution.visibility_graph(car4_vertex_list, car_vw_line_list, wall_line_list)
        visibility_end = time.time()
        visibility_time_diff = visibility_end - visibility_start
        print("visibility::",visibility_time_diff)
        
        dijkstra_start = time.time()
        car1_shortest_path, car1_shortest_length = Execution.dijkstra(car1_vis_graph)
        car2_shortest_path, car2_shortest_length = Execution.dijkstra(car2_vis_graph)
        car3_shortest_path, car3_shortest_length = Execution.dijkstra(car3_vis_graph)
        car4_shortest_path, car4_shortest_length = Execution.dijkstra(car4_vis_graph)
        dijkstra_end = time.time()
        dijkstra_time_diff = dijkstra_end - dijkstra_start
        print("dijkstra::",dijkstra_time_diff)  

        #ベジェ曲線の長さを格納
        car1_length = 0
        car2_length = 0
        car3_length = 0
        car4_length = 0

        #経路追従処理
        bezier_time_start = time.time()
        car1_point = []

        for i in range(len(car1_shortest_path)):
            car1_point.append(car1_vertex_list[car1_shortest_path[i]])
            
            if i+3 <= len(car1_shortest_path)-1:
                cross_point = line_cross_point(car1_vertex_list[car1_shortest_path[i]],car1_vertex_list[car1_shortest_path[i+1]],car1_vertex_list[car1_shortest_path[i+2]],car1_vertex_list[car1_shortest_path[i+3]])
                car1_point.append(list(cross_point))

        px1, py1 = bezie_curve(car1_point)
        car1_path = []
        for i in range(len(px1)):
            # if i == 0 or i//setting.speed == 0 or i == len(px1)-1:
            car1_path.append([px1[i],py1[i]])
            car1_length += np.linalg.norm(np.array(car1_path[i]) - np.array(car1_path[i-1]))

        # print("car1_path",car1_path,"car1_path_len",len(car1_path),"car1_length",car1_length)

        car2_point = []
        for i in range(len(car2_shortest_path)):
            car2_point.append(car2_vertex_list[car2_shortest_path[i]])
        
            if i+3 <= len(car2_shortest_path)-1:
                cross_point = line_cross_point(car2_vertex_list[car2_shortest_path[i]],car2_vertex_list[car2_shortest_path[i+1]],car2_vertex_list[car2_shortest_path[i+2]],car2_vertex_list[car2_shortest_path[i+3]])
                car2_point.append(list(cross_point))
        print("car2_shortest_path", car2_shortest_path)
        px2, py2 = bezie_curve(car2_point)
        car2_path = []
        for i in range(0,len(px2)):
            # if i == 0 or i//setting.speed == 0 or i == len(px2)-1:
            car2_path.append([px2[i],py2[i]])
            car2_length += np.linalg.norm(np.array(car2_path[i]) - np.array(car2_path[i-1]))

        car3_point = []
        for i in range(len(car3_shortest_path)):
            car3_point.append(car3_vertex_list[car3_shortest_path[i]])
            if i+3 <= len(car3_shortest_path)-1:
                cross_point = line_cross_point(car3_vertex_list[car3_shortest_path[i]],car3_vertex_list[car3_shortest_path[i+1]],car3_vertex_list[car3_shortest_path[i+2]],car3_vertex_list[car3_shortest_path[i+3]])
                car3_point.append(list(cross_point))

        px3, py3 = bezie_curve(car3_point)
        car3_path = []
        for i in range(0,len(px3)):
            # if i == 0 or i//setting.speed == 0 or i == len(px3)-1:
            car3_path.append([px3[i],py3[i]])
            car3_length += np.linalg.norm(np.array(car3_path[i]) - np.array(car3_path[i-1]))

        car4_point = []
        for i in range(len(car4_shortest_path)):
            car4_point.append(car4_vertex_list[car4_shortest_path[i]])
            if i+3 <= len(car4_shortest_path)-1:
                cross_point = line_cross_point(car4_vertex_list[car4_shortest_path[i]],car4_vertex_list[car4_shortest_path[i+1]],car4_vertex_list[car4_shortest_path[i+2]],car4_vertex_list[car4_shortest_path[i+3]])
                car4_point.append(list(cross_point))

        px4, py4 = bezie_curve(car4_point)
        car4_path = []
        for i in range(0,len(px4)):
            # if i == 0 or i//setting.speed == 0 or i == len(px4)-1:
            car4_path.append([px4[i],py4[i]])
            car4_length += np.linalg.norm(np.array(car4_path[i]) - np.array(car4_path[i-1]))
        bezier_time_end = time.time()
        bezier_time = bezier_time_end - bezier_time_start
        print(bezier_time)

        flag1 = False 
        flag2 = False
        flag3 = False
        flag4 = False
        collision = 0
        num1 = 0
        num2 = 0
        num3 = 0
        num4 = 0 
        car1_start_position = setting.car1_STARTtoGOAL[0].copy()
        car2_start_position = setting.car2_STARTtoGOAL[0].copy()
        car3_start_position = setting.car3_STARTtoGOAL[0].copy()
        car4_start_position = setting.car4_STARTtoGOAL[0].copy()
        car1_position = car1_start_position
        car2_position = car2_start_position
        car3_position = car3_start_position
        car4_position = car4_start_position
    
        shurp_curve = False

        cars_position_list = [[],[],[],[]]
        while(flag1==False and flag2==False and flag3==False and flag4==False):
            car1_position, flag1, num1, shurp_curve = cars_tuple[0].car_move(car1_path, car1_position, num1, shurp_curve)
            car2_position, flag2, num2, shurp_curve = cars_tuple[1].car_move(car2_path, car2_position, num2, shurp_curve)
            car3_position, flag3, num3, shurp_curve = cars_tuple[2].car_move(car3_path, car3_position, num3, shurp_curve)
            car4_position, flag4, num4, shurp_curve = cars_tuple[3].car_move(car4_path, car4_position, num4, shurp_curve)
            cars_position_list[0].append(car1_position.copy())
            cars_position_list[1].append(car2_position.copy())
            cars_position_list[2].append(car3_position.copy())
            cars_position_list[3].append(car4_position.copy())
            # print("car1_position", car1_position)
            collision = Environment.collision_CarToCar(car1_position, car2_position, car3_position, car4_position, collision)

        all_path_length = car1_length + car2_length + car3_length + car4_length
        print(all_path_length)

        return cars_position_list, plot_VW_list, wall_edge_list, car1_vertex_list, car1_shortest_path, car1_path, px1, py1, car2_path, px2, py2, car3_path, px3, py3, car4_path, px4, py4


class Environment():
    def __init__(self, obstacle_x, obstacle_y, width, height):
        self.x = obstacle_x
        self.y = obstacle_y
        self.width = width
        self.height = height
    
    def set_wall():
        """
        
        """
        # wall_edge_list = []
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

    def car_move(self, car_path, car_position, num, shurp_curve):
          
        if num >= (len(car_path)-1):
            self.goal_flag = True
        
        elif num < (len(car_path)):
            node = car_path[num+1]

            if num+2 < len(car_path):
                after_angle = calculate_two_vec_angle(car_position, node, car_path[num + 2])
                if after_angle > setting.car_angle:
                    shurp_curve = True

            if node[0] - car_position[0] == 0 and (node[1] - car_position[1]) >= 0:
                car_position[1] += setting.speed
                if car_position[1] >= node[1]:
                    car_position[0] = node[0]
                    car_position[1] = node[1]
                    num += 1
                
            elif node[0] - car_position[0] == 0 and (node[1] - car_position[1]) <= 0:
                car_position[1] -= setting.speed
                if car_position[1] <= node[1]:
                    car_position[0] = node[0]
                    car_position[1] = node[1]
                    num += 1
                
            elif node[1] - car_position[1] == 0 and (node[0] - car_position[0]) >= 0:
                car_position[0] += setting.speed
                if car_position[0] >= node[0]:
                    car_position[0] = node[0]
                    car_position[1] = node[1]
                    num += 1
                
            elif node[1] - car_position[1] == 0 and (node[0] - car_position[0]) <= 0:
                car_position[0] -= setting.speed
                if car_position[0] <= node[0]:
                    car_position[0] = node[0]
                    car_position[1] = node[1]
                    num += 1
                
            else:
                rad = np.arctan(abs(node[1] - car_position[1])/abs(node[0] - car_position[0]))

                if  car_position[0] > node[0] and car_position[1] > node[1]:    
                    car_position[0] -= np.cos(rad) * setting.speed
                    car_position[1] -= np.sin(rad) * setting.speed
                    
                    if car_position[0] <= node[0] and car_position[1] <= node[1]:
                        
                        car_position[0] = node[0]
                        car_position[1] = node[1]
                        
                        num += 1
                    
                elif car_position[0] < node[0] and car_position[1] > node[1]:
                
                    car_position[0] += np.cos(rad) * setting.speed
                    car_position[1] -= np.sin(rad) * setting.speed

                    if car_position[0] >= node[0] and car_position[1] <= node[1]:
                        
                        car_position[0] = node[0]
                        car_position[1] = node[1]
                        
                        num += 1

                elif car_position[0] > node[0] and car_position[1] < node[1]:
                    
                    car_position[0] -= np.cos(rad) * setting.speed
                    car_position[1] += np.sin(rad) * setting.speed

                    if car_position[0] <= node[0] and car_position[1] >= node[1]:
                        
                        car_position[0] = node[0]
                        car_position[1] = node[1]
                        
                        num += 1

                elif car_position[0] < node[0] and car_position[1] < node[1]:
                    
                    car_position[0] += np.cos(rad) * setting.speed
                    car_position[1] += np.sin(rad) * setting.speed

                    if car_position[0] >= node[0] and car_position[1] >= node[1]:
                        
                        car_position[0] = node[0]
                        car_position[1] = node[1]
                        
                        num += 1

        # print("caaaaaaaa", car_position)    
        return car_position ,self.goal_flag, num, shurp_curve

class Execution():
    def set_obstacle(self):
        self.Obstacle_1 = Environment()
        self.Obstacle_2 = Environment()
        self.Obstacle_3 = Environment()
        self.Obstacle_4 = Environment()

    def visibility_graph(vertex_list, obstacle_line_list, wall_line_list):

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

                    # s = (obstacle_Line[1][0] - obstacle_Line[0][0])*(vertex_u[1] - obstacle_Line[0][1]) - (vertex_u[0] - obstacle_Line[0][0]) * (obstacle_Line[1][1] - obstacle_Line[0][1])#外積の計算
                    # t = (obstacle_Line[1][0] - obstacle_Line[0][0])*(vertex_v[1] - obstacle_Line[0][1]) - (vertex_v[0] - obstacle_Line[0][0]) * (obstacle_Line[1][1] - obstacle_Line[0][1])
                    # if s * t < 0:
                    #     #障害物との衝突が検出された時点で障害物と衝突判定のfor文を抜ける
                    #     cross = True
                    #     continue
                
                
                # for wall_line in wall_line_list:

                #     s = (wall_line[1][0] - wall_line[0][0])*(vertex_u[1] - wall_line[0][1]) - (vertex_u[0] - wall_line[0][0]) * (wall_line[1][1] - wall_line[0][1])#外積の計算
                #     t = (wall_line[1][0] - wall_line[0][0])*(vertex_v[1] - wall_line[0][1]) - (vertex_v[0] - wall_line[0][0]) * (wall_line[1][1] - wall_line[0][1])
                #     if s * t < 0:
                #         #障害物との衝突が検出された時点で障害物と衝突判定のfor文を抜ける
                #         cross = True
                #         continue
                

                if cross == False:
                    #衝突が発生しなかった場合、経路長を計算し追加
                    Line.append(np.sqrt(((vertex_v[0] - vertex_u[0])**2 + (vertex_v[1] - vertex_u[1])**2)))
            
                    visibility_graph_list.append(Line)

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
    solution = np.array([0, 1, 0, 1, 1, 1, 0, 1, 0])
    cars_position_list, plot_vw_list, wall_edge_list, car1_vertex_list, car1_shortest_path, car1_path, px1, py1, car2_path, px2, py2, car3_path, px3, py3, car4_path, px4, py4 = VW.vw_result(solution)
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
    
    wx = []
    wy = []
    for i in range(len(setting.wall_line_list)):
        wx.append(setting.wall_line_list[i][0][0])
        wx.append(setting.wall_line_list[i][1][0])
        wy.append(setting.wall_line_list[i][0][1])
        wy.append(setting.wall_line_list[i][1][1])
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
    plt.plot(wx,wy)
    # plt.plot(px1, py1)
    # plt.plot(px2, py2)
    # plt.plot(px3, py3)
    # plt.plot(px4, py4)
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
    # print("after_angle",after_angle)

    return after_angle

###
#ベジェ曲線を描く関数
###

# Bernstein多項式を計算する関数
def bernstein(n, t):
    B = []
    for k in range(n + 1):
        # 二項係数を計算してからBernstein多項式を計算
        nCk = math.factorial(n) / (math.factorial(k) * math.factorial(n - k))
        B.append(nCk * t ** k * (1 - t) ** (n - k))
    return B
 
# ベジェ曲線を描く関数
def bezie_curve(Q):
    n = len(Q) - 1
    dt = 0.01
    t = np.arange(0, 1 + dt, dt)
    B = bernstein(n, t)
    px = 0
    py = 0
    for i in range(len(Q)):
        px += np.dot(B[i], Q[i][0])
        py += np.dot(B[i], Q[i][1])
    return px, py

def line_cross_point(P0, P1, Q0, Q1):
    x0, y0 = P0
    x1, y1 = P1
    x2, y2 = Q0
    x3, y3 = Q1
    a0 = x1 - x0; b0 = y1 - y0
    a2 = x3 - x2; b2 = y3 - y2

    d = a0*b2 - a2*b0
    if d == 0:
        # two lines are parallel
        return None

    # s = sn/d
    sn = b2 * (x2-x0) - a2 * (y2-y0)
    # t = tn/d
    #tn = b0 * (x2-x0) - a0 * (y2-y0)
    return x0 + a0*sn/d, y0 + b0*sn/d


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