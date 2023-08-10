import numpy as np
import networkx as nx

from geneticalgorithm2 import geneticalgorithm2 as ga
import setting
import random

class VW():
    """
    Virtual Wallを管理するクラス
    """
    def __init__(self, x, y, size):
        self.x = x
        self.y = y
        self.size = size
    
    def set_virtual_wall(GA_list):
        """

        遺伝的アルゴリズムの結果に対応したVWを設置する関数,VWの4つの頂点のlistと障害物の線分のlistを返す
        """
        size = 5#60
        obstacles_vertex_list = []
        obstacles_line_list = []

        #indexにインデックスをdeploy_checkには値(0,1)が入る.
        for index, oneDivisionList in enumerate(GA_list):
            for twoDivisionIndex, deploy_check in enumerate(oneDivisionList):
                if 1 == deploy_check:

                    #VWの左上, 左下, 右上, 右下を設定
                    VW_LeftUp = [(400 + (size) * twoDivisionIndex), (299 + (size * index))]
                    VW_LeftDown = [VW_LeftUp[0], VW_LeftUp[1] + size]
                    VW_RightUp = [VW_LeftUp[0] + size, VW_LeftUp[1]]
                    VW_RightDown = [VW_LeftUp[0] + size, VW_LeftUp[1] + size]
                
                    obstacles_vertex_list.extend([VW_LeftUp, VW_LeftDown, VW_RightUp, VW_RightDown])
                    obstacles_line_list.extend([[VW_LeftUp, VW_LeftDown], [VW_LeftUp, VW_RightUp], [VW_RightUp, VW_RightDown], [VW_RightUp, VW_LeftDown]])

        return obstacles_vertex_list, obstacles_line_list

    def ga_function(p):
        """
        
        GeneticalAlgorism用の関数
        """
        car1_vw = []
        car2_vw = []
        car3_vw = []
        car4_vw = []
        for i in range(setting.VWnum):
            car1_vw.append([])
            car2_vw.append([])
            car3_vw.append([])
            car4_vw.append([])
            for j in range(setting.VWnum):
                car1_vw[i].append(int(p[j+i*setting.VWnum+(setting.VWnum**2)*0]))
                car2_vw[i].append(int(p[j+i*setting.VWnum+(setting.VWnum**2)*1]))
                car3_vw[i].append(int(p[j+i*setting.VWnum+(setting.VWnum**2)*2]))
                car4_vw[i].append(int(p[j+i*setting.VWnum+(setting.VWnum**2)*3]))
        array = np.array(car1_vw)
        array2 = np.array(car2_vw)
        array3 = np.array(car3_vw)
        array4 = np.array(car4_vw)
        #ToDo 以下の処理は変える必要がある

        #遺伝的アルゴリズムの結果に対しVWを設置
        car1_VW_list, car1_vw_line_list = VW.set_virtual_wall(array)
        car2_VW_list, car2_vw_line_list = VW.set_virtual_wall(array2)
        car3_VW_list, car3_vw_line_list = VW.set_virtual_wall(array3)
        car4_VW_list, car4_vw_line_list = VW.set_virtual_wall(array4)

        #CarAgentにODを設定
        car1 = CarAgent([100, 450],[1000, 450])
        car2 = CarAgent([550, 0], [551, 900])
        car3 = CarAgent([1000,450],[100, 450])
        car4 = CarAgent([550, 900],[551, 0])

        #頂点のlistを作成
        car1_vertex_list = Environment.set_vertex_list(car1_VW_list, car1)
        car2_vertex_list = Environment.set_vertex_list(car2_VW_list, car2)
        car3_vertex_list = Environment.set_vertex_list(car3_VW_list, car3)
        car4_vertex_list = Environment.set_vertex_list(car4_VW_list, car4)

        #可視グラフ, ダイクストラ法を実行
        car1_vis_graph = Execution.visibility_graph(car1_vertex_list, car1_vw_line_list)
        car2_vis_graph = Execution.visibility_graph(car2_vertex_list, car2_vw_line_list)
        car3_vis_graph = Execution.visibility_graph(car3_vertex_list, car3_vw_line_list)
        car4_vis_graph = Execution.visibility_graph(car4_vertex_list, car4_vw_line_list)

        car1_shortest_path, car1_shortest_length = Execution.dijkstra(car1_vis_graph)
        car2_shortest_path, car2_shortest_length = Execution.dijkstra(car2_vis_graph)
        car3_shortest_path, car3_shortest_length = Execution.dijkstra(car3_vis_graph)
        car4_shortest_path, car4_shortest_length = Execution.dijkstra(car4_vis_graph)
        
        #車両の衝突判定


        #全ての経路長を足す
        all_path_length = car1_shortest_length + car2_shortest_length + car3_shortest_length + car4_shortest_length

        return all_path_length

class Environment():
    def __init__(self, obstacle_x, obstacle_y, width, height):
        self.x = obstacle_x
        self.y = obstacle_y
        self.width = width
        self.height = height
    
    def set_wall():
        edge_list = []
        
        return edge_list
    
    def set_vertex_list(obstacle_list, carAgent):
        """
        
        頂点のリストを作成し返す関数
        """
        vertex_list = [carAgent.start, carAgent.goal]

        vertex_list.extend(obstacle_list)

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

    def move(self,dx,dy):
        rad = np.arctan(abs(dy - self.y)/abs(dx - self.x))
        dig = np.degrees(rad)
        if dx == self.x and dy == self.y:
            self.x += 0
            self.y += 0
        else:
            if  dx > self.x and dy > self.y:
                self.x += (np.cos(np.radians(dig))*self.speed)
                self.y += (np.sin(np.radians(dig))*self.speed)
            elif dx < self.x and dy > self.y:
                self.x -= (np.cos(np.radians(dig))*self.speed)
                self.y += (np.sin(np.radians(dig))*self.speed) 
            elif dx > self.x and dy < self.y:
                self.x += (np.cos(np.radians(dig))*self.speed)
                self.y -= (np.sin(np.radians(dig))*self.speed)  
            elif dx < self.dx and dy < self.dy:
                self.x -= (np.cos(np.radians(dig))*self.speed)
                self.y -= (np.sin(np.radians(dig))*self.speed)


class Execution():
    def set_obstacle(self):
        self.Obstacle_1 = Environment()
        self.Obstacle_2 = Environment()
        self.Obstacle_3 = Environment()
        self.Obstacle_4 = Environment()

    def set_caragent(self):
        self.CarAgent_1 = CarAgent([0, 50], [100, 50])
        self.CarAgent_2 = CarAgent([50, 100], [50, 0])
        self.CarAgent_3 = CarAgent([100, 50], [0, 50])
        self.CarAgent_4 = CarAgent([50, 0], [50, 100])

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
    solution_list = []

    varbound_list = [[0,2]] * (setting.VWnum**2 * 4) #この中にGAで出た値を入れていく,[0, 2]は0~1の値が入るという意味

    varbound = np.array(varbound_list)

    ga_model = ga(function=VW.ga_function,
            dimension=((setting.VWnum**2) * 4),
            variable_type='real',
            variable_boundaries=varbound,
            algorithm_parameters=setting.params
    )
    ga_run = print(ga_model)
    convergence = ga_model.report
    solution = ga_model.result
    print(str(setting.VWnum) + "vw")

    for key, value in setting.params.items():
        print(str(key) + "：" + str(value))
    for i in solution['variable']:
        solution_list.append(i)
    #print(solution_list)
    #print((solution['variable']),"2222") # x, y の最適値
    #print(solution['score'],"最小値") # x, y の最適値での関数の値
    print(ga_run)

    return solution_list

if __name__ == '__main__':
    main()