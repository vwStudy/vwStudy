import numpy as np
import networkx as nx
import copy

from geneticalgorithm2 import geneticalgorithm2 as ga
import setting

class VW():
    """
    
<<<<<<< HEAD
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
=======
    Virtual Wallを管理するクラス
    """
    def __init__(self, x, y, size):
        self.x = x
        self.y = y
        self.size = size
    
    def set_virtual_wall(GA_list):
>>>>>>> e0e05d466632c38eef4f25cfb2a451b9c8935c95
        """
<<<<<<< HEAD

=======
>>>>>>> e5af316b37f81eff312bdb7b952e9cf2ed0c24a2
        遺伝的アルゴリズムの結果に対応したVWを設置する関数,VWの4つの頂点のlistと障害物の線分のlistを返す
        """
        print("galist"+str(GA_list))
        size = setting.VWsize
        field_x = setting.VWfield_x
        field_y = setting.VWfield_y
        obstacles_vertex_list = []
        obstacles_line_list = []
        total_num_obstacles = 0
<<<<<<< HEAD
<<<<<<< HEAD
        #indexにインデックスをdeploy_checkには値(0,1)が入る.
        for index, oneDivisionList in enumerate(GA_list):
            for twoDivisionIndex, deploy_check in enumerate(oneDivisionList):
                if 1 == deploy_check:
=======
        
        ga_list = [GA_list]
        # for i in GA_list:
        #     print(GA_list[i])
        #     if GA_list[i] >=1.0:
        #         ga_list.append(1)
        #     elif GA_list[i]<1:
        #         ga_list.append(0)
        # print("galist"+str(ga_list))

=======
>>>>>>> 16980ca4676e1340b9194fde184c3d491faf5fa8
        #indexにインデックスをdeploy_checkには値(0,1)が入る.
        for index, oneDivisionList in enumerate(GA_list):
            for twoDivisionIndex, deploy_check in enumerate(oneDivisionList):
                if deploy_check >= 1:
>>>>>>> e5af316b37f81eff312bdb7b952e9cf2ed0c24a2
                    total_num_obstacles += 1
                    #VWの左上, 左下, 右上, 右下を設定
                    VW_LeftUp = [(field_x + (size * twoDivisionIndex)), (field_y + (size * index))]
                    VW_LeftDown = [VW_LeftUp[0], VW_LeftUp[1] + size]
                    VW_RightUp = [VW_LeftUp[0] + size, VW_LeftUp[1]]
                    VW_RightDown = [VW_LeftUp[0] + size, VW_LeftUp[1] + size]
<<<<<<< HEAD
                
                    obstacles_vertex_list.extend([VW_LeftUp, VW_LeftDown, VW_RightUp, VW_RightDown])
                    obstacles_line_list.extend([[VW_LeftUp, VW_LeftDown], [VW_LeftUp, VW_RightUp], [VW_RightUp, VW_RightDown], [VW_RightUp, VW_LeftDown]])

=======
                    
                    obstacles_vertex_list.extend([VW_LeftUp, VW_LeftDown, VW_RightUp, VW_RightDown])
<<<<<<< HEAD
                    obstacles_line_list.extend([[VW_LeftUp, VW_LeftDown], [VW_LeftUp, VW_RightUp], [VW_RightUp, VW_RightDown], [VW_RightUp, VW_LeftDown]])
>>>>>>> e5af316b37f81eff312bdb7b952e9cf2ed0c24a2
=======
                    obstacles_line_list.extend([[VW_LeftUp, VW_LeftDown], [VW_LeftUp, VW_RightUp], [VW_RightUp, VW_RightDown], [VW_RightDown, VW_LeftDown]])
>>>>>>> 16980ca4676e1340b9194fde184c3d491faf5fa8
        return obstacles_vertex_list, obstacles_line_list


class Environment():
    def __init__(self, obstacle_x, obstacle_y, width, height):
        self.x = obstacle_x
        self.y = obstacle_y
        self.width = width
        self.height = height
    
    def set_wall():
        wall_edge_list = []
        #設置する壁の考慮すべきエッジをlistにまとめる
        wall_edge_list = setting.wall_edge

        wall_line_list = setting.wall_line
        return wall_edge_list, wall_line_list


    def collision_CarToCar(car1_vertex_list, car1_shortest_path, car2_vertex_list, car2_shortest_path, car3_vertex_list, car3_shortest_path, car4_vertex_list, car4_shortest_path):
        """
<<<<<<< HEAD
<<<<<<< HEAD
        
        頂点のリストを作成し返す関数
        """
        vertex_list = car_start_goal_list
        
        vertex_list.extend(obstacle_list)
        
=======
=======
        
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
                if carTocar_distance <= 12:
                    collision += 1
            
            if index <= len(car3_node_move_list)-1: 
                carTocar_distance = np.sqrt(((car3_node_move_list[index][0] - move_pos[0])**2) + ((car3_node_move_list[index][1] - move_pos[1])**2))
                if carTocar_distance <= 12:
                    collision += 1
            
            if index <= len(car4_node_move_list)-1: 
                carTocar_distance = np.sqrt(((car4_node_move_list[index][0] - move_pos[0])**2) + ((car4_node_move_list[index][1] - move_pos[1])**2))
                if carTocar_distance <= 12:
                    collision += 1
        
        for index, move_pos in enumerate(car2_node_move_list):
            if index <= len(car3_node_move_list)-1: 
                carTocar_distance = np.sqrt(((car3_node_move_list[index][0] - move_pos[0])**2) + ((car3_node_move_list[index][1] - move_pos[1])**2))
                if carTocar_distance <= 12:
                    collision += 1
            
            if index <= len(car4_node_move_list)-1: 
                carTocar_distance = np.sqrt(((car4_node_move_list[index][0] - move_pos[0])**2) + ((car4_node_move_list[index][1] - move_pos[1])**2))
                if carTocar_distance <= 12:
                    collision += 1
        
        for index, move_pos in enumerate(car3_node_move_list):
            if index <= len(car4_node_move_list)-1: 
                carTocar_distance = np.sqrt(((car4_node_move_list[index][0] - move_pos[0])**2) + ((car4_node_move_list[index][1] - move_pos[1])**2))
                if carTocar_distance <= 12:
                    collision += 1

        #print("collision::"+str(collision))    
        return collision

    def set_vertex_list(obstacle_list, carAgent, wall_edge):
        """
        
>>>>>>> 16980ca4676e1340b9194fde184c3d491faf5fa8
        頂点のリストを作成し返す関数
        """
        start = carAgent.start.copy()
        goal = carAgent.goal.copy()
        vertex_list = [start, goal]

        vertex_list.extend(obstacle_list)
<<<<<<< HEAD
>>>>>>> e5af316b37f81eff312bdb7b952e9cf2ed0c24a2
=======
        vertex_list.extend(wall_edge)
<<<<<<< HEAD
        #print("ver"+str(vertex_list))
        return vertex_list
=======

>>>>>>> 16980ca4676e1340b9194fde184c3d491faf5fa8
        return vertex_list

class CarAgent():
    def __init__(self, start, goal):
        self.start = start
        self.goal = goal
        self.x = start[0]
        self.y = start[1]
        self.goal_x = goal[0]
        self.goal_y = goal[1]
        self.speed = setting.speed #根拠のある数値にする
        self.car_width = setting.car_width #根拠のある数値にする

    # def move(self,dx,dy):
    #     rad = np.arctan(abs(dy - self.y)/abs(dx - self.x))
    #     dig = np.degrees(rad)
    #     if dx == self.x and dy == self.y:
    #         self.x += 0
    #         self.y += 0
    #     else:
    #         if  dx > self.x and dy > self.y:
    #             self.x += (np.cos(np.radians(dig))*self.speed)
    #             self.y += (np.sin(np.radians(dig))*self.speed)
    #         elif dx < self.x and dy > self.y:
    #             self.x -= (np.cos(np.radians(dig))*self.speed)
    #             self.y += (np.sin(np.radians(dig))*self.speed) 
    #         elif dx > self.x and dy < self.y:
    #             self.x += (np.cos(np.radians(dig))*self.speed)
    #             self.y -= (np.sin(np.radians(dig))*self.speed)  
    #         elif dx < self.dx and dy < self.dy:
    #             self.x -= (np.cos(np.radians(dig))*self.speed)
    #             self.y -= (np.sin(np.radians(dig))*self.speed)
>>>>>>> e0e05d466632c38eef4f25cfb2a451b9c8935c95


class Execution():
    def set_obstacle(self):
        self.Obstacle_1 = Environment()
        self.Obstacle_2 = Environment()
        self.Obstacle_3 = Environment()
        self.Obstacle_4 = Environment()

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
=======
    def visibility_graph(vertex_list, obstacle_line_list):
>>>>>>> e0e05d466632c38eef4f25cfb2a451b9c8935c95
        
        # cross = False
        # visibility_graph_list = []
        # for index, vertex_u in enumerate(vertex_list):
        #     for goal_index, vertex_v in enumerate(vertex_list[index + 1:], index + 1):
        #         Line = [index,goal_index]

        #         for obstacle_Line in obstacle_line_list:
        #             a = (vertex_v[1]-vertex_u[1])/(vertex_v[0]-vertex_u[0])
        #             b = vertex_u[1] - (a * vertex_v[0])
        #             #print(a)
        #             #print(obstacle_line_list[0][0][0])
                    
        #             check1 = (a * obstacle_Line[0][0]) + b - obstacle_Line[0][1]
        #             check2 = (a * obstacle_Line[1][0]) + b - obstacle_Line[1][1]

        #             if(check1 * check2) < 0:
        #                 cross = True
        #                 break

        #         if cross == False:
        #             Line.append(np.sqrt(((vertex_v[0] - vertex_u[0])**2 + (vertex_v[1] - vertex_u[1])**2)))
            
        #             visibility_graph_list.append(tuple(Line))

        # return visibility_graph_list
                
                



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
<<<<<<< HEAD
        
<<<<<<< HEAD
        #print(visibility_graph_list)

=======
>>>>>>> e5af316b37f81eff312bdb7b952e9cf2ed0c24a2
=======

>>>>>>> 16980ca4676e1340b9194fde184c3d491faf5fa8
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
def main():
    #solution_list = vw.main()
<<<<<<< HEAD
    solution_list = [0.6886917157974817, 1.7329621140729117, 0.621635982483528, 1.653291418411026, 1.869981314816912, 0.9946807098912214, 1.875357795465527, 1.3673799684304846, 1.8570713433403065, 0.027529367888641687, 0.2720953065838445, 1.6032967146085195, 0.5670306144517703, 0.0406742338244368, 1.0190273894113842, 0.795520416516071, 0.33904478122228987, 1.2584934127856424, 1.033738383265325, 0.44162554397710063, 0.18527236901158317, 0.8203674377614019, 0.5254761930474774, 0.6335307297759929, 1.2881038421862552, 0.9113234543294748, 1.5398149070226534, 0.5995173281635564, 0.8763542532327442, 1.9545771299140902, 1.459356932823466, 1.4000558708339292]
    vw_list = np.array(solution_list).reshape(2,setting.VWnum**2).tolist()
=======
    solution_list = [0.15180780696053775, 0.8073581697399996, 1.477140413972299, 1.7997473606463872, 1.0130161896814958, 0.26786757982117493, 1.0266994861594445, 0.4157493750389958, 0.010426577803933412, 0.9765359409995915, 1.8540325152413883, 1.1274910728838243, 1.319590569693116, 1.449654406400599, 0.7196989355074781, 0.40116572861447586, 1.103637222919178, 0.6316405381188614, 1.8954056909891641, 1.4433933049373924, 1.9603248808320768, 0.437992781451644, 1.346275285338597, 1.690349870159493, 1.7552874033168995, 1.9486448531247733, 0.8688180227629614, 0.7212801312284212, 1.741517473148413, 0.8485090716355492, 0.3994523653006583, 1.6528081659718425, 1.1258270971857633, 1.8550045699408813, 1.462493894194725, 1.9709985177161566, 0.9894612303823174, 1.6528036258281, 0.40434071749076805, 0.08032099437243967, 1.277660414263412, 0.6809461778289496, 0.6245100542117898, 1.9633590339998621, 1.2463694298511374, 1.5782856369275582, 1.7220368477728998, 1.7435997734660043, 1.0716963642320276, 0.18906432709643428, 1.9294901555002868, 1.6542194682350937, 1.6763364360574062, 0.5223517545529688, 0.16544317976261103, 1.3738313562569902, 0.6211227771028829, 1.0955529700729336, 0.6052226082282239, 0.17161299756871373, 1.2232498457569274, 0.2584960988345433, 1.4834255322762293, 0.7920366501625498]
    vw_list = np.array(solution_list).reshape(4,setting.VWnum**2).tolist()
>>>>>>> e5af316b37f81eff312bdb7b952e9cf2ed0c24a2
=======

def set_wall():
        wall_edge_list = []
        #設置する壁の考慮すべきエッジをlistにまとめる
        wall_edge_list = setting.wall_edge

        wall_line_list = setting.wall_line
        return wall_edge_list, wall_line_list


def collision_CarToCar(car1_vertex_list, car1_shortest_path, car2_vertex_list, car2_shortest_path, car3_vertex_list, car3_shortest_path, car4_vertex_list, car4_shortest_path):
    """
>>>>>>> 16980ca4676e1340b9194fde184c3d491faf5fa8
    
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
<<<<<<< HEAD
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

<<<<<<< HEAD
<<<<<<< HEAD

=======
>>>>>>> e5af316b37f81eff312bdb7b952e9cf2ed0c24a2
    car1_x0 = setting.car1_STARTtoGOAL[0][0]
    car1_y0 = setting.car1_STARTtoGOAL[0][1]
    car2_x0 = setting.car2_STARTtoGOAL[0][0]
    car2_y0 = setting.car2_STARTtoGOAL[0][1]
    car3_x0 = setting.car3_STARTtoGOAL[0][0]
    car3_y0 = setting.car3_STARTtoGOAL[0][1]
    car4_x0 = setting.car4_STARTtoGOAL[0][0]
    car4_y0 = setting.car4_STARTtoGOAL[0][1]
<<<<<<< HEAD

    vw_point_x1 = 380
    vw_point_y1 = 200
        
    for i in range(2):
        car_vw = np.array(vw_list[i]).reshape(setting.VWnum,setting.VWnum).tolist()
        vw_point_x1 = 380
        vw_point_y1 = 200
=======
        
    for i in range(4):
        car_vw = np.array(vw_list[i]).reshape(setting.VWnum,setting.VWnum).tolist()
        vw_point_x = setting.VWfield_x
        vw_point_y = setting.VWfield_y
>>>>>>> e5af316b37f81eff312bdb7b952e9cf2ed0c24a2
        list1 = []
        list2 = []
        list3 = []
        list4 = []

        for j in range(setting.VWnum):
            for k in range(setting.VWnum):
                if car_vw[j][k]>=1:
                    if i==0:
<<<<<<< HEAD
                        canvas.create_rectangle(vw_point_x1, vw_point_y1, vw_point_x1+25, vw_point_y1+25,fill='red')
                        #list1.extend()
                        
                    elif i==1:
                        canvas.create_rectangle(vw_point_x1, vw_point_y1, vw_point_x1+25, vw_point_y1+25,fill='blue')
                        
                    elif i==2:
                        canvas.create_rectangle(vw_point_x1, vw_point_y1, vw_point_x1+25, vw_point_y1+25,fill='yellow')
                    
                    elif i==3:
                        canvas.create_rectangle(vw_point_x1, vw_point_y1, vw_point_x1+25, vw_point_y1+25,fill='black')
                    
                    #list1.extend([setting.car1_start, setting.car1_goal, [vw_point_x1, vw_point_y1], [vw_point_x1, vw_point_y1+25], [vw_point_x1+25, vw_point_y1], [vw_point_x1+25, vw_point_y1+25]])##始点,終点と頂点リスト
                    #list2.extend([setting.car2_start, setting.car2_goal, [vw_point_x1,vw_point_y1], [vw_point_x1, vw_point_y1+25], [vw_point_x1+25, vw_point_y1], [vw_point_x1+25, vw_point_y1+25]])
                    #list3.extend([setting.car3_start, setting.car3_goal, [vw_point_x1,vw_point_y1], [vw_point_x1, vw_point_y1+25], [vw_point_x1+25, vw_point_y1], [vw_point_x1+25, vw_point_y1+25]])
                    #list4.extend([setting.car4_start, setting.car4_goal, [vw_point_x1,vw_point_y1], [vw_point_x1, vw_point_y1+25], [vw_point_x1+25, vw_point_y1], [vw_point_x1+25, vw_point_y1+25]])

                vw_point_x1 += 25
            vw_point_x1 = 380
            vw_point_y1 += 25
    

    car1_VW_list, car1_vw_line_list = set_virtual_wall(vw_list)
    #print(car1_VW_list)
    car1_start_goal_list=[setting.car1_STARTtoGOAL[0], setting.car1_STARTtoGOAL[1]]
    car1_vertex_list = set_vertex_list(car1_VW_list, car1_start_goal_list)
    #print(car1_vertex_list)
    car1_vis_graph = visibility_graph(car1_vertex_list, car1_vw_line_list)
    car1_shortest_path, car1_shortest_length = dijkstra(car1_vis_graph)
=======
                        canvas.create_rectangle(vw_point_x, vw_point_y, vw_point_x+setting.VWsize, vw_point_y+setting.VWsize,fill='red')
=======
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
>>>>>>> 16980ca4676e1340b9194fde184c3d491faf5fa8
                        
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

<<<<<<< HEAD
    #car1の最短経路描画
    for i in range(len(car1_shortest_path)-1):
         canvas.create_line(car1_vertex_list[car1_shortest_path[i]][0],car1_vertex_list[car1_shortest_path[i]][1], car1_vertex_list[car1_shortest_path[i+1]][0],car1_vertex_list[car1_shortest_path[i+1]][1], fill = "red", width = 5)
    #canvas.create_line(car1_vertex_list,car1_vertex_list, car1_vertex_list, car1_vertex_list, fill = "red", width = 5)
    #canvas.create_line(car1_vertex_list, car1_vertex_list, 860, 240, fill = "red", width = 5)
>>>>>>> e5af316b37f81eff312bdb7b952e9cf2ed0c24a2

    x_size=25
    y_size=10

<<<<<<< HEAD
    #  #計算部分
=======
    #計算部分
>>>>>>> e5af316b37f81eff312bdb7b952e9cf2ed0c24a2
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
        
<<<<<<< HEAD
        if car1_x0 >= 270:
            break
        
    for i in range(2):
        #car_vw = np.array(vw_list[i]).reshape(setting.VWnum,setting.VWnum).tolist()

        vw_point_x1 = 380
        vw_point_y1 = 200
        list1 = []
        list2 = []
        list3 = []
        list4 = []
        t=100
        for j in range(setting.VWnum):
            for k in range(setting.VWnum):
                if car_vw[j][k]>=1:
                    if i==0:

                        #計算部分
                        while True:
                            #CarAgent_1.move(270,0)
                            canvas.coords(car1, car1_x0, car1_y0, car1_x0+x_size, car1_y0+y_size)
                            car1_mv=move(car1_x0, car1_y0, vw_point_x1, vw_point_y1)
                            car1_x0+=car1_mv[0]
                            car1_y0+=car1_mv[1]
                            print("testes")
                            time.sleep(0.02)
                            tk.update() #ウインド画面を更新
                            t-=1
                            if t == 0:
                                break
                        
                        
                    # elif i==1:
                    #     canvas.coords(car2, car2_x0, car2_y0, car2_x0+y_size, car2_y0+x_size)
                    #     canvas.coords(car3, car3_x0, car3_y0, car3_x0+x_size, car3_y0+y_size)
                    #     canvas.coords(car4, car4_x0, car4_y0, car4_x0+y_size, car4_y0+x_size)
                        
                    # elif i==2:
                        
                    
                    # elif i==3:

                    
                    # list1.extend([setting.car1_start, setting.car1_goal, [vw_point_x1, vw_point_y1], [vw_point_x1, vw_point_y1+25], [vw_point_x1+25, vw_point_y1], [vw_point_x1+25, vw_point_y1+25]])##始点,終点と頂点リスト
                    # list2.extend([setting.car2_start, setting.car2_goal, [vw_point_x1,vw_point_y1], [vw_point_x1, vw_point_y1+25], [vw_point_x1+25, vw_point_y1], [vw_point_x1+25, vw_point_y1+25]])
                    # list3.extend([setting.car3_start, setting.car3_goal, [vw_point_x1,vw_point_y1], [vw_point_x1, vw_point_y1+25], [vw_point_x1+25, vw_point_y1], [vw_point_x1+25, vw_point_y1+25]])
                    # list4.extend([setting.car4_start, setting.car4_goal, [vw_point_x1,vw_point_y1], [vw_point_x1, vw_point_y1+25], [vw_point_x1+25, vw_point_y1], [vw_point_x1+25, vw_point_y1+25]])

            vw_point_x1 += 25
            vw_point_x1 = 380
            vw_point_y1 += 25
=======
        if car1_x0 >= 500:
            break
        
>>>>>>> e5af316b37f81eff312bdb7b952e9cf2ed0c24a2
=======

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
>>>>>>> 16980ca4676e1340b9194fde184c3d491faf5fa8

    # for i in range(len(car4_shortest_path)):
    #     print("car4_path::"+"x:"+str(car4_vertex_list[car4_shortest_path[i]][0])+"y:"+str(car4_vertex_list[car4_shortest_path[i]][1]))
    #     f.writelines("car4_path::"+"x:"+str(car4_vertex_list[car4_shortest_path[i]][0])+"y:"+str(car4_vertex_list[car4_shortest_path[i]][1]))
    all_len = car1_shortest_length + car2_shortest_length + car3_shortest_length + car4_shortest_length
    f.writelines("all_len::"+str(all_len))
    f.writelines('\n')

    collision = collision_CarToCar(car1_vertex_list, car1_shortest_path, car2_vertex_list, car2_shortest_path, car3_vertex_list, car3_shortest_path, car4_vertex_list, car4_shortest_path)
    f.writelines("collision::"+str(collision))
    f.writelines('\n')


=======
       
    solution_list = [0.65299908, 0.79354912, 0.99713758, 1.39533659, 0.37356927, 0.35385083, 0.12938773, 0.97594239, 1.95422334, 1.02231169, 0.34340037, 1.05288289, 0.48973104, 0.5105005, 0.01938685, 1.32540365, 0.4375187, 0.59207165, 1.33210779, 0.33710088, 0.45667739, 0.1335461, 0.2746958, 0.43775444, 0.20156037, 0.41580347, 0.82473599, 0.32031398, 1.08542453, 0.95115266, 1.95612487, 0.3162209, 0.56626899, 0.98849229, 0.57902751, 0.82553665, 0.4987426, 1.87678322, 1.48214428, 0.69990249, 0.71310027, 1.44443687, 0.28491696, 1.38618651, 1.23986736, 1.54998751, 0.91767542, 1.93551015, 0.39815773, 0.73598948, 0.62282179, 0.29777548, 1.82376795, 0.61624243, 0.61721746, 1.50084867, 0.06609298, 0.76970635, 0.49609562, 1.78343203, 0.48185538, 0.01592708, 0.3158078, 1.63121802, 0.57185095, 0.77827377, 1.39009879, 0.73430812, 0.23544991, 0.72736795, 0.00978192, 0.77675913, 1.87989639, 0.84358179, 1.25528747, 0.66523101, 1.88152103, 0.60478594, 0.77135695, 0.53182687, 0.31641489, 0.43876778, 0.06155524, 0.75978956, 0.55561951, 0.74671976, 0.64984071, 1.34505708, 0.83151844, 1.38207621, 0.61451895, 0.41683822, 0.42520947, 0.37894489, 0.60532481, 0.7160464, 0.75272739, 0.70437766, 0.52533959, 0.25556862]
    for index, sol in enumerate(solution_list):
        if sol >= 1:
            sol=1
            solution_list[index] = sol
        elif sol < 1:
            sol=0
            solution_list[index] = sol
        else:
            sol=5
            solution_list[index] = sol
    vw_list = np.array(solution_list).reshape(4,5,5).tolist()
    #print("vw"+str(vw_list[0]))

    #ToDo 以下の処理は変える必要がある
    #遺伝的アルゴリズムの結果に対しVWを設置
    car1_VW_list, car1_vw_line_list = VW.set_virtual_wall(vw_list[0])
    car2_VW_list, car2_vw_line_list = VW.set_virtual_wall(vw_list[1])
    car3_VW_list, car3_vw_line_list = VW.set_virtual_wall(vw_list[2])
    car4_VW_list, car4_vw_line_list = VW.set_virtual_wall(vw_list[3])
    #print("car1::"+str(car1_VW_list))

    #CarAgentにODを設定
    cars_tuple = (CarAgent(setting.car1_STARTtoGOAL[0],setting.car1_STARTtoGOAL[1]), CarAgent(setting.car2_STARTtoGOAL[0],setting.car2_STARTtoGOAL[1]), CarAgent(setting.car3_STARTtoGOAL[0],setting.car3_STARTtoGOAL[1]), CarAgent(setting.car4_STARTtoGOAL[0],setting.car4_STARTtoGOAL[1]))
    # print(setting.car1_STARTtoGOAL[0],setting.car1_STARTtoGOAL[1])
    # print(setting.car2_STARTtoGOAL[0],setting.car2_STARTtoGOAL[1])

    wall_edge, wall_line = Environment.set_wall()

    #頂点のlistを作成
    car1_vertex_list = Environment.set_vertex_list(car1_VW_list, cars_tuple[0], wall_edge)
    car2_vertex_list = Environment.set_vertex_list(car2_VW_list, cars_tuple[1], wall_edge)
    car3_vertex_list = Environment.set_vertex_list(car3_VW_list, cars_tuple[2], wall_edge)
    car4_vertex_list = Environment.set_vertex_list(car4_VW_list, cars_tuple[3], wall_edge)
    #print(car1_vertex_list)
    # print(car2_vertex_list)

    #可視グラフ, ダイクストラ法を実行
    car1_vis_graph = Execution.visibility_graph(car1_vertex_list, car1_vw_line_list)
    car2_vis_graph = Execution.visibility_graph(car2_vertex_list, car2_vw_line_list)
    car3_vis_graph = Execution.visibility_graph(car3_vertex_list, car3_vw_line_list)
    car4_vis_graph = Execution.visibility_graph(car4_vertex_list, car4_vw_line_list)

    # print(car1_vis_graph)
    # print(car2_vis_graph)
    # print(car3_vis_graph)
    # print(car4_vis_graph)

    car1_shortest_path, car1_shortest_length = Execution.dijkstra(car1_vis_graph)
    car2_shortest_path, car2_shortest_length = Execution.dijkstra(car2_vis_graph)
    car3_shortest_path, car3_shortest_length = Execution.dijkstra(car3_vis_graph)
    car4_shortest_path, car4_shortest_length = Execution.dijkstra(car4_vis_graph)

    # for i in range(len(car1_shortest_path)):
    #     print("car1_path::"+"x:"+str(car1_vertex_list[car1_shortest_path[i]][0])+"y:"+str(car1_vertex_list[car1_shortest_path[i]][1]))
    
    # for i in range(len(car2_shortest_path)):
    #     print("car2_path::"+"x:"+str(car2_vertex_list[car2_shortest_path[i]][0])+"y:"+str(car2_vertex_list[car2_shortest_path[i]][1]))

    # for i in range(len(car3_shortest_path)):
    #     print("car3_path::"+"x:"+str(car3_vertex_list[car3_shortest_path[i]][0])+"y:"+str(car3_vertex_list[car3_shortest_path[i]][1]))

    # for i in range(len(car4_shortest_path)):
    #     print("car4_path::"+"x:"+str(car4_vertex_list[car4_shortest_path[i]][0])+"y:"+str(car4_vertex_list[car4_shortest_path[i]][1]))
    
    #車両の衝突判定
    collision = Environment.collision_CarToCar(car1_vertex_list, car1_shortest_path, car2_vertex_list, car2_shortest_path, car3_vertex_list, car3_shortest_path, car4_vertex_list, car4_shortest_path)
    #print(collision)

    total_num_obstacles = len(car1_VW_list)/4 + len(car2_VW_list)/4 + len(car3_VW_list)/4 + len(car4_VW_list)/4
    #print(total_num_obstacles)
    
    print("collision::"+str(collision))
    #全ての経路長を足す
    all_path_length = car1_shortest_length + car2_shortest_length + car3_shortest_length + car4_shortest_length
    print("all_len::"+str(all_path_length))

    f = open('data.txt', 'a', encoding='UTF-8')
    f.writelines(str(vw_list))
    f.writelines('\n')
    f.writelines("vw_num::"+str(total_num_obstacles))
    f.writelines('\n')
    f.writelines("collision::"+str(collision))
    f.writelines('\n')
    f.writelines("all_len::"+str(all_path_length))
    f.writelines('\n')

>>>>>>> e0e05d466632c38eef4f25cfb2a451b9c8935c95
    f.close()
if __name__ == '__main__':
    main()