import numpy as np
import setting
import pygame
import networkx as nx
from pygame.locals import *

class CarAgent:
    def __init__(self, screen, color, start_x_position, start_y_position, goal_x_position, goal_y_position, length, width):
        self.screen = screen
        self.color = color
        self.start_x_position = start_x_position
        self.start_y_position = start_y_position
        self.goal_x_position = goal_x_position
        self.goal_y_position = goal_y_position
        self.length = length
        self.width = width

    def get_StarttoGoal(self):
        return [[self.start_x_position, self.start_y_position], [self.goal_x_position, self.goal_y_position]]
    
    def ceate_car(self):
        x_position = self.start_x_position - (self.length/2)
        y_position = self.start_y_position - (self.width/2)
        pygame.draw.rect(self.screen, self.color, (x_position, y_position, self.length, self.width))

class Environment:
    def __init__(self, screen, x_position, y_position, width, height):
        self.screen = screen
        self.x_position = x_position
        self.y_position = y_position
        self.width = width
        self.height = height

    def set_vertex_list(obstacle_list, carAgent):
        """
        
        頂点のリストを作成し返す関数
        """
        start = carAgent.start.copy()
        goal = carAgent.goal.copy()
        vertex_list = [start, goal]

        vertex_list.extend(obstacle_list)
        return vertex_list
    
    def create_wall(self):
        pygame.draw.rect(self.screen, 'green',(self.x_position, self.y_position, self.width, self.height))

class VW:
    def __init__(self, screen, color, x_position, y_position, width, height):
        self.screen = screen
        self.color = color
        self.x_position = x_position
        self.y_position = y_position
        self.width = width
        self.height = height
    
    def set_virtual_wall(GA_list):##ここのプログラムをviewように書き換える
        """
        遺伝的アルゴリズムの結果に対応したVWを設置する関数,VWの4つの頂点のlistと障害物の線分のlistを返す
        """
        #print("galist"+str(GA_list))
        size = setting.VWsize
        field_x = setting.VWfield_x
        field_y = setting.VWfield_y
        obstacles_vertex_list = []
        obstacles_line_list = []
        total_num_obstacles = 0
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

# class Environment():
#     def __init__(self, obstacle_x, obstacle_y, width, height):
#         self.x = obstacle_x
#         self.y = obstacle_y
#         self.width = width
#         self.height = height
 
#     def set_vertex_list(obstacle_list, carAgent):
#         """
        
#         頂点のリストを作成し返す関数
#         """
#         start = carAgent.start.copy()
#         goal = carAgent.goal.copy()
#         vertex_list = [start, goal]

#         vertex_list.extend(obstacle_list)

#         return vertex_list


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
    cnt=0
    pygame.init()
    solution = np.array([1.89215129, 0.74894772, 0.00993403, 0.4199251, 0.71996361, 0.11393614, 1.55424998, 1.17790473, 0.23565381, 1.62690657, 0.11434647, 0.40075553, 1.44282191, 0.37405164, 1.43584013, 1.12043762, 1.76458347, 0.21921504, 0.48557488, 0.24243188, 0.99663108, 0.06455254, 0.99025981, 0.09090406, 0.25918536, 1.17238867, 0.61052081, 0.78857954, 1.43374165, 0.03068778, 0.59235409, 1.4372173, 1.82472237, 1.68819216, 0.11070665, 0.50956055, 1.6063743, 1.69169765, 1.50513445, 0.41652798, 1.66132287, 0.8683496, 0.0913576, 0.89012811, 0.40333341, 1.90217085, 0.88446913, 1.46940589, 0.74473358, 1.1326151, 0.54214587, 1.88746749, 0.96678426, 1.30171531, 0.43203435, 0.91851862, 0.38745088, 0.95620997, 1.49637253, 1.13973887, 1.6127253, 0.81776396, 0.55856443, 1.52546627, 0.54047958, 0.09572321, 1.03539512, 0.75229951, 0.71139677, 0.94998286, 0.52285394, 1.31682183, 0.34987624, 0.63019523, 1.37561236, 0.00076496, 0.40855242, 0.32186973, 0.74139705, 0.95536255, 0.76562882, 0.28632164, 1.99401695, 1.48164543, 0.52481142, 0.72743079, 0.82246726, 0.8803177, 0.29447183, 0.57471207, 0.52836044, 1.39142544, 0.09069547, 0.73895736, 0.17709275, 0.38624017, 0.09992985, 0.84092342, 0.10688538, 1.22615924, 0.48971551, 0.0905245, 1.29424943, 0.17590815, 0.03759963, 0.75275473, 0.34013247, 1.206067, 0.36606145, 0.72392373, 0.71267159, 1.61220819, 1.35560996, 0.59429754, 0.36609347, 0.0328301, 0.37162267, 1.02334579, 0.99002976, 0.14533639, 1.06312891, 0.31134226, 0.49147947, 1.0402685, 0.86610513, 0.89727141, 0.26376824, 1.17139419, 1.49717405, 0.85128727, 1.49728721, 1.96399671, 0.27378594, 1.78319852, 0.8641858, 1.51344239, 0.4718716, 1.92716412, 1.84697095, 1.52300925, 1.15447391, 1.26916816, 0.59056653, 0.69357074])
    car_ga = np.reshape(solution, (setting.car_num, setting.VWnum))

    screen = pygame.display.set_mode((1200, 600))
    screen.fill((255,255,255))
    leftup_wall1 = Environment(screen, 0, 0, setting.wall_edge_list[0][0], setting.wall_edge_list[0][1])
    leftup_wall1.create_wall()
    leftup_wall2 = Environment(screen, 0, 0, setting.wall_edge_list[1][0], setting.wall_edge_list[1][1])
    leftup_wall2.create_wall()
    # rightup_wall1 = Environment(screen, setting.wall_edge_list[2][0], setting.wall_edge_list[2][1], 1000, )
    # rightup_wall2 = Environment()
    # leftdown_wall1 = Environment()
    # leftdown_wall2 = Environment.create_wall()
    # rightdown_wall1 = Environment.create_wall()
    # rightdown_wall2 = Environment.create_wall()
    
    car1 = CarAgent(screen, 'red', setting.car1_STARTtoGOAL[0][0], setting.car1_STARTtoGOAL[0][1], setting.car1_STARTtoGOAL[1][0], setting.car1_STARTtoGOAL[1][1], setting.car_length, setting.car_width)
    car1.ceate_car()
    car2 = CarAgent(screen, 'blue', setting.car2_STARTtoGOAL[0][0], setting.car2_STARTtoGOAL[0][1], setting.car2_STARTtoGOAL[1][0], setting.car2_STARTtoGOAL[1][1], setting.car_length, setting.car_width)
    car2.ceate_car()
    car3 = CarAgent(screen, 'yellow', setting.car3_STARTtoGOAL[0][0], setting.car3_STARTtoGOAL[0][1], setting.car3_STARTtoGOAL[1][0], setting.car3_STARTtoGOAL[1][1], setting.car_length, setting.car_width)
    car3.ceate_car()
    car4 = CarAgent(screen, 'black', setting.car4_STARTtoGOAL[0][0], setting.car4_STARTtoGOAL[0][1], setting.car4_STARTtoGOAL[1][0], setting.car4_STARTtoGOAL[1][1], setting.car_length, setting.car_width)
    car4.ceate_car()

    
    cars_tuple = (car1.get_StarttoGoal(), car2.get_StarttoGoal(), car3.get_StarttoGoal(), car4.get_StarttoGoal())
    #cars_tuple = (CarAgent.set_StarttoGoal(setting.car1_STARTtoGOAL[0],setting.car1_STARTtoGOAL[1]))    
    #cars_tuple = (CarAgent(setting.car1_STARTtoGOAL[0],setting.car1_STARTtoGOAL[1]), CarAgent(setting.car2_STARTtoGOAL[0],setting.car2_STARTtoGOAL[1]), CarAgent(setting.car3_STARTtoGOAL[0],setting.car3_STARTtoGOAL[1]), CarAgent(setting.car4_STARTtoGOAL[0],setting.car4_STARTtoGOAL[1]))
    car1_VW_list, car1_vw_line_list = VW.set_virtual_wall(car_ga[0])#このカッコ内に最終的なvwを入れる
    car2_VW_list, car2_vw_line_list = VW.set_virtual_wall(car_ga[1])
    car3_VW_list, car3_vw_line_list = VW.set_virtual_wall(car_ga[2])
    car4_VW_list, car4_vw_line_list = VW.set_virtual_wall(car_ga[3])
    ##print("car1::"+str(car1_VW_list))


    #頂点のlistを作成
    car1_vertex_list = Environment.set_vertex_list(car1_VW_list, cars_tuple[0])
    car2_vertex_list = Environment.set_vertex_list(car2_VW_list, cars_tuple[1])
    car3_vertex_list = Environment.set_vertex_list(car3_VW_list, cars_tuple[2])
    car4_vertex_list = Environment.set_vertex_list(car4_VW_list, cars_tuple[3])
    ##print(car1_vertex_list)
    # print(car2_vertex_list)

    #可視グラフ, ダイクストラ法を実行
    car1_vis_graph = Execution.visibility_graph(car1_vertex_list, car1_vw_line_list)
    car2_vis_graph = Execution.visibility_graph(car2_vertex_list, car2_vw_line_list)
    car3_vis_graph = Execution.visibility_graph(car3_vertex_list, car3_vw_line_list)
    car4_vis_graph = Execution.visibility_graph(car4_vertex_list, car4_vw_line_list)

    # # print(car1_vis_graph)
    # # print(car2_vis_graph)
    # # print(car3_vis_graph)
    # # print(car4_vis_graph)

    car1_shortest_path, car1_shortest_length = Execution.dijkstra(car1_vis_graph)
    car2_shortest_path, car2_shortest_length = Execution.dijkstra(car2_vis_graph)
    car3_shortest_path, car3_shortest_length = Execution.dijkstra(car3_vis_graph)
    car4_shortest_path, car4_shortest_length = Execution.dijkstra(car4_vis_graph)



    while True:
        a
        #pygame.draw.rect(screen, (0,0,0),(px,py,50,50))#第一引数screenオブジェクト,第二引数図形のRGB,第三引数図形の形(左上のx座標,y座標,横幅,縦幅)
        #px+=2
        pygame.display.update()
        cnt+=1
        if cnt==9000:
            break

if __name__ == "__main__":
    main()