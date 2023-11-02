import numpy as np
import setting
import pygame
import sys
import networkx as nx
from pygame.locals import *

###やること
#車の移動
#車、壁の整形

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
    
    def create_car(self):
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
        start = carAgent[0].copy()
        goal = carAgent[1].copy()
        vertex_list = [start, goal]
        vertex_list.extend(obstacle_list)

        return vertex_list
    
    def create_wall(self):
        pygame.draw.rect(self.screen, 'green',(self.x_position, self.y_position, self.width, self.height))#第一引数screenオブジェクト,第二引数図形のRGB,第三引数図形の形(左上のx座標,y座標,横幅,縦幅)

    
class VW:

    def create_VW(screen, color, left_up, right_down):
        screen = pygame.display.set_mode((900, 500))
        screen.fill((255,255,255))

        ##pygame.draw.rect(screen, color, (left_up[0], left_up[1], abs(right_down[0]-left_up[0]), abs(right_down[1]-left_up[1])))
        
        rect1 = pygame.Rect(left_up[0], left_up[1], abs(right_down[0]-left_up[0]), abs(right_down[1]-left_up[1]))
        surface = pygame.Surface(left_up[0], left_up[1], pygame.SRCALPHA)##四角の大きさと透明を定義
        surface.fill(color)
        screen.blit(surface, rect1)
        
        #pygame.draw.rect(surface, color, rect1)
        #screen.blit(surface, rect1)
        
        
    
    def set_virtual_wall(screen, color, GA_list):##ここのプログラムをviewように書き換える
        """
        遺伝的アルゴリズムの結果に対応したVWを設置する関数,VWの4つの頂点のlistと障害物の線分のlistを返す
        """
        size = setting.VWsize
        field_x = setting.VWfield_x
        field_y = setting.VWfield_y
        obstacles_vertex_list = []
        obstacles_line_list = []
        total_num_obstacles = 0

        ga_list = np.reshape(GA_list, (setting.VWnum, setting.VWnum))
        #indexにインデックスをdeploy_checkには値(0,1)が入る.
        
        for index, oneDivisionList in enumerate(ga_list):
            for twoDivisionIndex, deploy_check in enumerate(oneDivisionList):
                if deploy_check >= 1:
                    total_num_obstacles += 1
                    #VWの左上, 左下, 右上, 右下を設定
                    VW_LeftUp = [(field_x + (size * twoDivisionIndex)), (field_y + (size * index))]
                    VW_LeftDown = [VW_LeftUp[0], VW_LeftUp[1] + size]
                    VW_RightUp = [VW_LeftUp[0] + size, VW_LeftUp[1]]
                    VW_RightDown = [VW_LeftUp[0] + size, VW_LeftUp[1] + size]
                    
                    VW.create_VW(screen, color, VW_LeftUp, VW_RightDown)
                    obstacles_vertex_list.extend([VW_LeftUp, VW_LeftDown, VW_RightUp, VW_RightDown])
                    obstacles_line_list.extend([[VW_LeftUp, VW_LeftDown], [VW_LeftUp, VW_RightUp], [VW_RightUp, VW_RightDown], [VW_RightDown, VW_LeftDown]])
        return obstacles_vertex_list, obstacles_line_list

class Execution:
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
    
    def create_line(screen, color, line_list, path_list):
        coordinate_list=[]
        for i in path_list:
            coordinate_list.append(line_list[i])
        
        for i in range(1, len(path_list)):
            pygame.draw.line(screen, color, (coordinate_list[i-1][0], coordinate_list[i-1][1]), (coordinate_list[i][0], coordinate_list[i][1]), 5)



def car_move(car_pos, line_list, path_list, num):
        
        if line_list[path_list[num]][0] == car_pos.x:
            if car_pos.y >  line_list[path_list[num]][1]:
                car_pos.y -= abs(car_pos.y-line_list[path_list[num]][1]) * setting.speed
            elif car_pos.y <  line_list[path_list[num]][1]:
                car_pos.y += abs(car_pos.y-line_list[path_list[num]][1]) * setting.speed

        elif line_list[path_list[num]][1] == car_pos.y:
            if car_pos.x >  line_list[path_list[num]][0]:
                car_pos.x -= abs(car_pos.x-line_list[path_list[num]][0]) * setting.speed
            elif car_pos.x <  line_list[path_list[num]][0]:
                car_pos.x += abs(car_pos.x-line_list[path_list[num]][0]) * setting.speed

        elif line_list[path_list[num]][0] != car_pos.x and line_list[path_list[num]][1] != car_pos.y:
            rad = np.arctan(abs(line_list[path_list[num]][1] - car_pos.y)/abs(line_list[path_list[num]][0] - car_pos.x))
            
            if  car_pos.x > line_list[path_list[num]][0] and car_pos.y > line_list[path_list[num]][1]:    
                car_pos.x -= abs(line_list[path_list[num]][0] - car_pos.x) * setting.speed
                #car_pos.x -= np.cos(rad) * setting.speed
                car_pos.y -= abs(line_list[path_list[num]][1] - car_pos.y) * setting.speed
                #car_pos.y -= np.sin(rad) * setting.speed
                
            
            elif car_pos.y > line_list[path_list[num]][1] and car_pos.x < line_list[path_list[num]][0]:
                car_pos.x += abs(line_list[path_list[num]][0] - car_pos.x) * setting.speed
                #car_pos.x += np.cos(rad) * setting.speed
                car_pos.y -= abs(line_list[path_list[num]][1] - car_pos.y) * setting.speed
                #car_pos.y -= np.sin(rad) * setting.speed
                

            elif car_pos.x > line_list[path_list[num]][0] and car_pos.y < line_list[path_list[num]][1]:
                car_pos.x -= abs(line_list[path_list[num]][0] - car_pos.x) * setting.speed
                #car_pos.x -= np.cos(rad) * setting.speed
                car_pos.y += abs(line_list[path_list[num]][1] - car_pos.y) * setting.speed
                #car_pos.y += np.sin(rad) * setting.speed
                
            elif car_pos.x < line_list[path_list[num]][0] and car_pos.y < line_list[path_list[num]][1]:
                car_pos.x += abs(line_list[path_list[num]][0] - car_pos.x) * setting.speed
                #car_pos.x += np.cos(rad) * setting.speed
                car_pos.y += abs(line_list[path_list[num]][1] - car_pos.y) * setting.speed
                #car_pos.y += np.sin(rad) * setting.speed

        return car_pos

def main():
    pygame.init()
    sol=[]
    solution = np.array([0.28957490124621077, 0.18916372212123278, 0.18036897508857708, 1.2269931939387937, 0.6988031102767378, 0.0599651494195772, 0.8898685201908574, 0.6532286540585921, 1.8465827349197292, 0.1265794826963449, 0.48602232512326604, 0.9858331230094728, 1.076974231081579, 0.767791269013129, 0.3575931059841906, 1.875541124470679, 0.5931751482531826, 0.6960168394917174, 0.9656572400996317, 0.18931443572604434, 0.5047101039314619, 1.6553000874484058, 1.9488618454005755, 1.9808619450427343, 1.029854726978918, 1.4200700438836817, 1.50628051744972, 0.7344295144652173, 0.20406466896858522, 0.627163359086192, 0.7888080722699997, 0.2645778241999164, 1.6854387071590575, 0.28517424445627326, 1.8596334549475082, 0.6596646896383598, 0.5988709468296147, 0.6846150091300247, 0.004957418739257591, 1.984843910159457, 0.38043939181296027, 1.5707341416994796, 1.733931504177465, 0.6637708944048981, 1.2946066096495221, 0.6557051840340549, 0.35687200393148943, 0.4615515030176922, 0.8467045846450709, 0.700316913709345, 1.1835116306654805, 0.6782482513762937, 1.114440196400099, 1.4406206516704065, 0.7120046856709261, 1.9659255525083184, 1.127789381045057, 1.760071605966028, 0.6975922526617411, 0.6094491568273794, 1.069824882764921, 0.29729510889662736, 0.5832966259255974, 0.266335564562191, 0.9351103951459794, 0.7237124500219925, 1.652866165014291, 0.23790090571931577, 0.9607260853945607, 0.6698791092742449, 0.1563676632519535, 0.9498876917417051, 0.48291658199561005, 0.551669613478361, 1.6454641596850228, 1.1427279516890438, 0.5350993734009446, 0.3102040077661117, 1.494815269960007, 1.24618538887623, 1.8680361752055665, 1.300251006880613, 1.100612530121019, 0.8436867001204551, 0.7473405112128426, 0.8677576895976982, 0.8836152389623517, 0.6414825426107793, 1.45003500885623, 1.7997942247908905, 0.8939755783089138, 1.1069319199243772, 1.4887914463623133, 1.5232762520645686, 1.8045288197942748, 1.5553959023278405, 0.916574447725353, 0.40176383360686074, 0.13521661258564155, 0.44169659110770243])
    for i in solution:
        sol.append(int(i))
    
    car_ga = np.reshape(sol, (setting.car_num, setting.VWnum**2))

    screen = pygame.display.set_mode((900, 500))
    screen.fill((255,255,255))

    leftup_wall1 = Environment(screen, setting.wall[0][0], setting.wall[0][1], 349, 199)
    leftup_wall2 = Environment(screen, setting.wall[1][0], setting.wall[1][1], 399, 149)    
    rightup_wall1 = Environment(screen, setting.wall[2][0], setting.wall[2][1], 900, 149)
    rightup_wall2 = Environment(screen, setting.wall[3][0], setting.wall[3][1], 900, 199)
    leftdown_wall1 = Environment(screen, setting.wall[4][0], setting.wall[4][1], 349, 500)
    leftdown_wall2 = Environment(screen, setting.wall[5][0], setting.wall[5][1], 399, 500)
    rightdown_wall1 = Environment(screen, setting.wall[6][0], setting.wall[6][1], 900, 199)
    rightdown_wall2 = Environment(screen, setting.wall[7][0], setting.wall[7][1], 900, 199)

    leftup_wall1.create_wall()
    leftup_wall2.create_wall()
    rightup_wall1.create_wall()
    rightup_wall2.create_wall()
    leftdown_wall1.create_wall()
    leftdown_wall2.create_wall()
    rightdown_wall1.create_wall()
    rightdown_wall2.create_wall()
    
    car1 = CarAgent(screen, 'red', setting.car1_STARTtoGOAL[0][0], setting.car1_STARTtoGOAL[0][1], setting.car1_STARTtoGOAL[1][0], setting.car1_STARTtoGOAL[1][1], setting.car_length, setting.car_width)
    car2 = CarAgent(screen, 'blue', setting.car2_STARTtoGOAL[0][0], setting.car2_STARTtoGOAL[0][1], setting.car2_STARTtoGOAL[1][0], setting.car2_STARTtoGOAL[1][1], setting.car_width, setting.car_length)
    car3 = CarAgent(screen, 'yellow', setting.car3_STARTtoGOAL[0][0], setting.car3_STARTtoGOAL[0][1], setting.car3_STARTtoGOAL[1][0], setting.car3_STARTtoGOAL[1][1], setting.car_length, setting.car_width)
    car4 = CarAgent(screen, 'black', setting.car4_STARTtoGOAL[0][0], setting.car4_STARTtoGOAL[0][1], setting.car4_STARTtoGOAL[1][0], setting.car4_STARTtoGOAL[1][1], setting.car_width, setting.car_length)

    cars_tuple = (car1.get_StarttoGoal(), car2.get_StarttoGoal(), car3.get_StarttoGoal(), car4.get_StarttoGoal())

    car1_VW_list, car1_vw_line_list = VW.set_virtual_wall(screen, 'red', car_ga[0])#このカッコ内に最終的なvwを入れる
    car2_VW_list, car2_vw_line_list = VW.set_virtual_wall(screen, 'blue', car_ga[1])
    car3_VW_list, car3_vw_line_list = VW.set_virtual_wall(screen, 'yellow', car_ga[2])
    car4_VW_list, car4_vw_line_list = VW.set_virtual_wall(screen, 'black', car_ga[3])

    #頂点のlistを作成
    car1_vertex_list = Environment.set_vertex_list(car1_VW_list, cars_tuple[0])
    car2_vertex_list = Environment.set_vertex_list(car2_VW_list, cars_tuple[1])
    car3_vertex_list = Environment.set_vertex_list(car3_VW_list, cars_tuple[2])
    car4_vertex_list = Environment.set_vertex_list(car4_VW_list, cars_tuple[3])

    #可視グラフ, ダイクストラ法を実行
    car1_vis_graph = Execution.visibility_graph(car1_vertex_list, car1_vw_line_list)
    car2_vis_graph = Execution.visibility_graph(car2_vertex_list, car2_vw_line_list)
    car3_vis_graph = Execution.visibility_graph(car3_vertex_list, car3_vw_line_list)
    car4_vis_graph = Execution.visibility_graph(car4_vertex_list, car4_vw_line_list)

    car1_shortest_path, car1_shortest_length = Execution.dijkstra(car1_vis_graph)
    car2_shortest_path, car2_shortest_length = Execution.dijkstra(car2_vis_graph)
    car3_shortest_path, car3_shortest_length = Execution.dijkstra(car3_vis_graph)
    car4_shortest_path, car4_shortest_length = Execution.dijkstra(car4_vis_graph)

    Execution.create_line(screen, 'red', car1_vertex_list, car1_shortest_path)
    Execution.create_line(screen, 'blue', car2_vertex_list, car2_shortest_path)
    Execution.create_line(screen, 'yellow', car3_vertex_list, car3_shortest_path)
    Execution.create_line(screen, 'black', car4_vertex_list, car4_shortest_path)


    car1_pos = pygame.Rect(car1_vertex_list[car1_shortest_path[0]][0] - (setting.car_length/2), car1_vertex_list[car1_shortest_path[0]][1] - (setting.car_width/2), setting.car_length, setting.car_width)
    car2_pos = pygame.Rect(car2_vertex_list[car2_shortest_path[0]][0] - (setting.car_width/2), car2_vertex_list[car2_shortest_path[0]][1] - (setting.car_length/2), setting.car_width, setting.car_length)
    car3_pos = pygame.Rect(car3_vertex_list[car3_shortest_path[0]][0] - (setting.car_length/2), car3_vertex_list[car3_shortest_path[0]][1] - (setting.car_width/2), setting.car_length, setting.car_width)
    car4_pos = pygame.Rect(car4_vertex_list[car4_shortest_path[0]][0] - (setting.car_width/2), car4_vertex_list[car4_shortest_path[0]][1] - (setting.car_length/2), setting.car_width, setting.car_length)
    
    cnt=0
    num1=0
    num2=0
    num3=0
    num4=0
    #rect_surface = pygame.Surface((setting.car_length, setting.car_width), pygame.SRCALPHA)
    vw_red = (255, 0, 0, 128) 
    vw_blue = (0, 0, 255, 100)
    vw_green = (0, 255, 0, 100)
    vw_yellow = (255, 255, 0, 128)
    
    red = 'red' 
    blue = 'blue'
    green = 'green'
    yellow = 'yellow'

    screen = pygame.display.set_mode((900, 500))
    screen.fill((255, 255, 255))
    frame_delay = 700

    while True:

        VW.set_virtual_wall(screen, vw_red, car_ga[0])
        VW.set_virtual_wall(screen, vw_blue, car_ga[1])
        VW.set_virtual_wall(screen, vw_green, car_ga[2])
        VW.set_virtual_wall(screen, vw_yellow, car_ga[3])

        leftup_wall1.create_wall()
        leftup_wall2.create_wall()
        rightup_wall1.create_wall()
        rightup_wall2.create_wall()
        leftdown_wall1.create_wall()
        leftdown_wall2.create_wall()
        rightdown_wall1.create_wall()
        rightdown_wall2.create_wall()
        
        Execution.create_line(screen, red, car1_vertex_list, car1_shortest_path)
        Execution.create_line(screen, blue, car2_vertex_list, car2_shortest_path)
        Execution.create_line(screen, green, car3_vertex_list, car3_shortest_path)
        Execution.create_line(screen, yellow, car4_vertex_list, car4_shortest_path)

        if num1 == 0:
            pygame.draw.rect(screen, red, car1_pos)
        if num2 == 0:
            pygame.draw.rect(screen, blue, car2_pos)
        if num3 == 0:
            pygame.draw.rect(screen, green, car3_pos)
        if num4 == 0:
            pygame.draw.rect(screen, yellow, car4_pos)

        else:
            car1_pos = car_move(car1_pos, car1_vertex_list, car1_shortest_path, num1)
            car2_pos = car_move(car2_pos, car2_vertex_list, car2_shortest_path, num2)
            car3_pos = car_move(car3_pos, car3_vertex_list, car3_shortest_path, num3)
            car4_pos = car_move(car4_pos, car4_vertex_list, car4_shortest_path, num4)
            
            pygame.draw.rect(screen, red, car1_pos)
            pygame.draw.rect(screen, blue, car2_pos)
            pygame.draw.rect(screen, green, car3_pos)
            pygame.draw.rect(screen, yellow, car4_pos) 

        pygame.display.update()
        pygame.time.delay(frame_delay)

        if num1 < len(car1_shortest_path)-1:
            num1+=1
        if num2 < len(car2_shortest_path)-1:
            num2+=1
        if num3 < len(car3_shortest_path)-1:
            num3+=1
        if num4 < len(car4_shortest_path)-1:
            num4+=1

        cnt+=1
        if cnt==50000:
            break

    
if __name__ == "__main__":
    main()