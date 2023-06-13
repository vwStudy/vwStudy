from operator import truediv
from os import remove
from turtle import update
from pygame import Color
import pygame
import random
from pygame.locals import *
import sys
import numpy as np
import math
import time
import itertools
import heapq
from math import sqrt
from functools import lru_cache
import numpy as np
from geneticalgorithm2 import geneticalgorithm2 as ga
pygame.font.init()
pygame.init()

# 定数群
BOX_TOP_X = 100        # ゲーム領域の左上X座標
BOX_TOP_Y = 100        # ゲーム領域の左上Y座標
BOX_WIDTH = 1500        # ゲーム領域の幅
BOX_HEIGHT = 1500       # ゲーム領域の高さ

DURATION = 0.05        # 描画間隔

FPS = 60     # Frame per Second 毎秒のフレーム数
D = 50
down_speed = 1
up_speed = 2.5
co = 0
co2 = 10000000
# width = 400

class CarAgent(pygame.sprite.Sprite):# ← pygame.sprite.Sprite
    def __init__(self, screen, x, y, des,Obstacle_grp,color):
        pygame.sprite.Sprite.__init__(self)
        # super().__init__() # ←この一行
        self.screen = screen
        self.x=x
        self.y=y
        self.start_x=x
        self.start_y=y
        self.speed = 3
        self.des_x, self.des_y = des
        self.trajectory = ([],[])
        self.Obstacle_grp = Obstacle_grp
        self.rect = pygame.Rect(self.x, self.y, D, D)
        self.image = pygame.Surface((10, 10))
        self.image.fill(color)
        self.image = pygame.transform.scale(self.image, (10, 10))
        self.root = []
        self.relay_pos_x = x
        self.relay_pos_y = y
        self.count = 0
        self.scored = False
        self.co2 = 0
    
    def update(self):
        self.move(self.relay_pos_x,self.relay_pos_y)
        if abs(self.rect.x - self.relay_pos_x) < 5 and abs(self.rect.y - self.relay_pos_y) < 5:
            if self.count < len(self.root) - 1:
                self.count += 1
                self.relay_pos_x = self.root[self.count][0]
                self.relay_pos_y = self.root[self.count][1]

    def move(self,des_x,des_y):
        if self.rect.x - self.des_x < 5 and self.rect.y - self.des_y < 5:
            self.scored = True
        if self.rect.x == self.des_x and self.rect.y == self.des_y:
            self.speed = 0
        elif des_x == self.x:
            if  self.x > des_x and self.y > des_y:
                self.y -= (self.speed)
            elif self.y > des_y:
                self.y -= (self.speed) 
            elif self.x > des_x:
                self.y += (self.speed)  
            else:
                self.y += (self.speed)
            self.rect.x = self.x
            self.rect.y = self.y
        else:
            if abs(des_y - self.y) != 0 or abs(des_x - self.x) != 0:
                rad = np.arctan(abs(des_y - self.y)/abs(des_x - self.x))
                dig = math.degrees(rad)
                if  self.x > des_x and self.y > des_y:
                    self.x -= (math.cos(math.radians(dig))*self.speed)
                    self.y -= (math.sin(math.radians(dig))*self.speed)
                elif self.y > des_y:
                    self.x += (math.cos(math.radians(dig))*self.speed)
                    self.y -= (math.sin(math.radians(dig))*self.speed) 
                elif self.x > des_x:
                    self.x -= (math.cos(math.radians(dig))*self.speed)
                    self.y += (math.sin(math.radians(dig))*self.speed)  
                else:
                    self.x += (math.cos(math.radians(dig))*self.speed)
                    self.y += (math.sin(math.radians(dig))*self.speed)
                self.rect.x = self.x
                self.rect.y = self.y
            

    def draw(self):#
        # 描画位置を移動させる
        self.graph_drawing()
        self.screen.blit(self.image, (self.rect.x, self.rect.y))

    def graph_drawing(self):#
        # 描画位置を移動させる
        obs_posi = []
        start_posi = []
        col_li = []
        total_line_posi = []
        start_posi.append((self.x,self.y))
        start_posi.append((self.des_x,self.des_y))
        dijkstra = []
        for obs in self.Obstacle_grp:
            obs_posi.append((obs.rect.x,obs.rect.y))
            start_posi.append((obs.rect.x,obs.rect.y))
            start_posi.append((obs.rect.x + obs.width,obs.rect.y))
            start_posi.append((obs.rect.x,obs.rect.y + obs.height))
            start_posi.append((obs.rect.x + obs.width,obs.rect.y + obs.height))
            # for wid, wid_2 in itertools.product(obs_width, obs_width):
            #     start_posi.append((obs.rect.x + wid,obs.rect.y + wid_2))
        # print(start_posi,"seed")
        for obs,str, end in itertools.product(self.Obstacle_grp,start_posi, start_posi):
            left_ob_x = obs.rect.x
            right_ob_x = obs.rect.x + obs.width
            top_ob_y = obs.rect.y
            bottom_ob_y = obs.rect.y + obs.height
            if str[0] < end[0]:
                if self.collision(str[0],str[1],end[0],end[1],left_ob_x,right_ob_x,top_ob_y,bottom_ob_y,str[0],end[0]) == True:
                    # print(obs_posi[obs],str,end,"t")
                    col_li.append((str,end))
                # else:
                #     # pygame.draw.line(self.screen,(0,95,0),(j[0],j[1]),(500,700))
                #     print(obs_posi[obs],str,end,"else")
            else:
                if self.collision(str[0],str[1],end[0],end[1],left_ob_x,right_ob_x,top_ob_y,bottom_ob_y,end[0],str[0]) == True:
                    # print(obs_posi[obs],str,end,"t")
                    col_li.append((str,end))
        for str, end in itertools.product(start_posi, start_posi):
            total_line_posi.append((str,end))
        result = list(set(total_line_posi) - set(col_li))
        for line in result:
            pygame.draw.line(self.screen,(0,95,0),(line[0][0],line[0][1]),(line[1][0],line[1][1]))
        for sta in range(len(start_posi)):
            dijkstra.append([])
            for line in result:
                if line[0] == start_posi[sta]:
                    dis_x = abs(line[0][0] -line[1][0])**2
                    dis_y = abs(line[0][1] -line[1][1])**2
                    distan = sqrt(dis_x + dis_y)
                    dijkstra[sta].append([start_posi.index(line[1]),distan])
        result_li,costdis = self.execution_dij(dijkstra,1)
        # print(result_li)
        for res in range(len(result_li) - 1):
            pygame.draw.line(self.screen, (250,0,0),(start_posi[result_li[res]][0],start_posi[result_li[res]][1]),(start_posi[result_li[res + 1]][0],start_posi[result_li[res + 1]][1]),10)
        for res in range(len(result_li)):
            self.root.append((start_posi[result_li[res]][0],start_posi[result_li[res]][1]))
        return costdis,self.root

    def execution_dij(self,edges,Goal):
        node_num = len(edges)
        opt_root, opt_cost = self.dijkstra(edges, node_num, Goal)    #道順とコストを出力させている

        #出力を見やすく整理するための変換用辞書型リストの作成
        root_converter = {}
        cost_converter = {}
        for i in range(node_num):
            root_converter[i] = i
            cost_converter[i] = opt_cost[i]

        arrow = " → "
        result = ""
        result_li = []
        for i in range(len(opt_root)):
            if i > 0:
                result += arrow
            result += f"{root_converter[opt_root[i]]}({cost_converter[opt_root[i]]})"
            result_li.append(root_converter[opt_root[i]])
        self.co2 += cost_converter[opt_root[-1]]
        co2 = self.co2
        # print(cost_converter[opt_root[-1]])
        # print(self.co2)
        print(f"ノード(そこまでのコスト)\n\n{result}")
        return result_li,cost_converter[opt_root[-1]]

    def dijkstra(self,edges, num_node, Goal):
        """ 経路の表現
                [終点, 辺の値]
                A, B, C, D, ... → 0, 1, 2, ...とする """
        node = [float('inf')] * num_node    #スタート地点以外の値は∞で初期化
        node[0] = 0     #スタートは0で初期化

        node_name = []
        heapq.heappush(node_name, [0, [0]])

        while len(node_name) > 0:
            #ヒープから取り出し
            _, min_point = heapq.heappop(node_name)
            last = min_point[-1]
            if last == Goal:
                return min_point, node  #道順とコストを出力させている

            #経路の要素を各変数に格納することで，視覚的に見やすくする
            for factor in edges[last]:
                goal = factor[0]   #終点
                cost  = factor[1]   #コスト

                #更新条件
                if node[last] + cost < node[goal]:
                    node[goal] = node[last] + cost     #更新
                    #ヒープに登録
                    heapq.heappush(node_name, [node[last] + cost, min_point + [goal]])

        return []
    
    def collision(self,xstart,ystart,xgool,ygool,left_ob_x,right_ob_x,top_ob_y,bottom_ob_y,start_x,end_x):
        if abs((xgool) - xstart) <= 10:
                # print((xgool) - xstart)
            if ystart < ygool:
                if left_ob_x < xstart < right_ob_x and (ystart <= top_ob_y <= ygool or ystart <= bottom_ob_y <= ygool):#障害物に当たるか判定
                    return True
            else:
                if left_ob_x < xstart < right_ob_x and (ygool <= top_ob_y <= ystart or ygool <= bottom_ob_y <= ystart):#障害物に当たるか判定
                    return True
        if ((xgool) - xstart) == 0:
            if ystart < ygool:
                if top_ob_y < ystart + 0.1 < bottom_ob_y and left_ob_x < xstart < right_ob_x:
                    return True
            else:
                if top_ob_y < ystart - 0.1 < bottom_ob_y and left_ob_x < xstart < right_ob_x:
                    return True
        for x in range(int(start_x),int(end_x)):
            # if -1 < ((ygool) - ystart) < 1 and abs(int(xgool) - xstart) >= 1:
            #     if top_ob_y < ystart < bottom_ob_y:#障害物に当たるか判定
            #         return True
            # if abs((xgool) - xstart) <= 1:
            #     # print((xgool) - xstart)
            #     if left_ob_x <= xstart <= right_ob_x:#障害物に当たるか判定
            #         return True
            # else:
            if int(xstart) <= xgool:
                b = ystart - (-(-(int(ygool) - ystart) / (int(xgool) - xstart))) * xstart
            else:
                b = ygool - (-(-(int(ygool) - ystart) / (int(xgool) - xstart))) * xgool
            y = -(-(int(ygool) - ystart) / (int(xgool) - xstart)) * x + b
            if top_ob_y < y < bottom_ob_y and left_ob_x < x < right_ob_x:#障害物に当たるか判定
                return True

    

class Trajectory(pygame.sprite.Sprite):# ← pygame.sprite.Sprite
    def __init__(self, screen,CarAgent_grp):
        pygame.sprite.Sprite.__init__(self)
        # super().__init__() # ←この一行
        self.screen = screen
        self.CarAgent_grp = CarAgent_grp
        self.reset_x = 1000000
        self.reset_y = 1000000
    
    def draw(self):
        Trajectory_li = []
        for car in ((self.CarAgent_grp)):
            # print(Carg[i].rect.x)
            if abs(car.rect.x - self.reset_x) > 10 or abs(car.rect.y - self.reset_y) > 10:#10以上進んだら
                if len(car.trajectory[0]) < 1:
                    car.trajectory[0].append(car.rect.x)#その時点の座標をrootに格納
                    car.trajectory[1].append(car.rect.y)#
                    self.reset_x = car.rect.x#過去の比較座標を更新
                    self.reset_y = car.rect.y#
                else:
                    if abs(car.trajectory[0][-1] - car.rect.x) > 10 or abs(car.trajectory[1][-1] - car.rect.y) > 10:
                        car.trajectory[0].append(car.rect.x)#その時点の座標をrootに格納
                        car.trajectory[1].append(car.rect.y)#
                        self.reset_x = car.rect.x#過去の比較座標を更新
                        self.reset_y = car.rect.y#
            for tra in range(len(car.trajectory[0])):
                Trajectory_rect = pygame.draw.rect(self.screen, (255,0,255), (car.trajectory[0][tra],car.trajectory[1][tra],5,5), 2)
                Trajectory_li.append(Trajectory_rect)

class Obstacle(pygame.sprite.Sprite):# ← pygame.sprite.Sprite
    def __init__(self, screen,x,y,width,height,color):
        pygame.sprite.Sprite.__init__(self)
        # super().__init__() # ←この一行
        self.screen = screen
        self.x=x
        self.y=y
        self.width = width
        self.height = height
        self.rect = pygame.Rect(self.x, self.y, D, D)
        self.image = pygame.Surface((self.width, self.height),flags=pygame.SRCALPHA)
        self.image.fill(color)
        self.image = pygame.transform.scale(self.image, (self.width, self.height))

    def draw(self):#
        # 描画位置を移動させる
        self.screen.blit(self.image, (self.rect.x, self.rect.y))
        
class Window:
    def __init__(self, w, h):#初期化
        pygame.sprite.Sprite.__init__(self)
        self.width = w
        self.height = h
        self.reset_x = 1000000
        self.reset_y = 1000000
        self.ga_li = []
        self.ga_li2 = []
        self.ga_li3 = []
        self.ga_li4 = []
        self.co = 0
        self.li = []

    def set_screen(self):#個々のクラスを呼び出し値をいれてセットする
        self.screen = pygame.display.set_mode((1200, 800))
        self.clock = pygame.time.Clock()
        self.co = 0
        # print(self.ga_li2,"sineisienisnei")
        return len(self.ga_li2)

    def init_Obstacles(self):
        self.Obstacle_1 = Obstacle(self.screen,-1100,0,1400,400,"green")
        self.Obstacle_2 = Obstacle(self.screen,-900,-100,1400,400,"green")
        self.Obstacle_3 = Obstacle(self.screen,-1100,500,1400,400,"green")
        self.Obstacle_4 = Obstacle(self.screen,-900,600,1400,400,"green")
        self.Obstacle_5 = Obstacle(self.screen,600,-100,1400,400,"green")
        self.Obstacle_6 = Obstacle(self.screen,800,0,1400,400,"green")
        self.Obstacle_7 = Obstacle(self.screen,800,500,1400,400,"green")
        self.Obstacle_8 = Obstacle(self.screen,600,600,1400,400,"green")
        self.Obstacle_9 = Obstacle(self.screen,525,425,10,10,"green")
        self.Obstacle_10 = Obstacle(self.screen,350,200,200,100,"green")
        self.Obstacle_11 = Obstacle(self.screen,670,450,120,100,"green")
        self.Obstacle_12 = Obstacle(self.screen,530,479,70,62,(127,255,0,128))
        self.Obstacle_grp = pygame.sprite.Group(self.Obstacle_1,self.Obstacle_2,self.Obstacle_3,
            self.Obstacle_4,self.Obstacle_5,self.Obstacle_6,self.Obstacle_7,self.Obstacle_8
            # ,self.Obstacle_9
            # ,self.Obstacle_10,self.Obstacle_11
        )
        self.Obstacle_grp_2 = pygame.sprite.Group(self.Obstacle_1,self.Obstacle_2,self.Obstacle_3,
            self.Obstacle_4,self.Obstacle_5,self.Obstacle_6,self.Obstacle_7,self.Obstacle_8
        )
        self.Obstacle_grp_3 = pygame.sprite.Group(self.Obstacle_1,self.Obstacle_2,self.Obstacle_3,
            self.Obstacle_4,self.Obstacle_5,self.Obstacle_6,self.Obstacle_7,self.Obstacle_8
        )
        self.Obstacle_grp_4 = pygame.sprite.Group(self.Obstacle_1,self.Obstacle_2,self.Obstacle_3,
            self.Obstacle_4,self.Obstacle_5,self.Obstacle_6,self.Obstacle_7,self.Obstacle_8
        )
        self.set_obstacles(self.ga_li,self.Obstacle_grp,(255,255,0,128))
        self.set_obstacles(self.ga_li2,self.Obstacle_grp_2,(255,20,147,128))
        self.set_obstacles(self.ga_li3,self.Obstacle_grp_3,(0,0,255,128))
        self.set_obstacles(self.ga_li4,self.Obstacle_grp_4,(127,255,0,128))
        return self.ga_li,self.ga_li2,self.ga_li3,self.ga_li4
    
    def set_obstacles(self,obs_list,Obstacle_grp,color):
        for obs in obs_list:
            obstacles = Obstacle(self.screen,obs[0],obs[1],obs[2]*60,62,color)
            Obstacle_grp.add(obstacles)

    def init_CarAgents(self):    
        self.CarAgent_1 = CarAgent(self.screen, 100, 450, (1000,450),self.Obstacle_grp,(255,255,0,128))
        self.CarAgent_2 = CarAgent(self.screen, 550, 0, (551,900),self.Obstacle_grp_2,(255,20,147,128))
        self.CarAgent_3 = CarAgent(self.screen, 1000, 450, (100,450),self.Obstacle_grp_3,(0,0,255,128))
        self.CarAgent_4 = CarAgent(self.screen, 550, 900, (551,0),self.Obstacle_grp_4,(127,255,0,128))
        # self.CarAgent_5 = CarAgent(self.screen, 1150, 0, (0,300),self.Obstacle_grp)
        # self.CarAgent_6 = CarAgent(self.screen, 0, 100, (800,750))
        # self.CarAgent_7 = CarAgent(self.screen, 200, 0, (1150,600))
        # self.CarAgent_8 = CarAgent(self.screen, 1000, 0, (100,750))
        # self.CarAgent_9 = CarAgent(self.screen, 100, 750, (300,0))
        # self.CarAgent_10 = CarAgent(self.screen, 1150, 200, (0,700))
        # self.CarAgent_11 = CarAgent(self.screen, 100, 100, (100,750))
        # self.CarAgent_12 = CarAgent(self.screen, 300, 0, (1150,500))
        # self.CarAgent_13 = CarAgent(self.screen, 1100, 0, (0,750))
        # self.CarAgent_14 = CarAgent(self.screen, 0, 750, (400,0))
        # self.CarAgent_15 = CarAgent(self.screen, 1150, 100, (0,600))
        # self.CarAgent_grp = pygame.sprite.Group(self.CarAgent_1,self.CarAgent_2,self.CarAgent_3,
        #     self.CarAgent_4,self.CarAgent_5,self.CarAgent_6,self.CarAgent_7,self.CarAgent_8,
        #     self.CarAgent_9,self.CarAgent_10,self.CarAgent_12,self.CarAgent_13,self.CarAgent_14,
        #     self.CarAgent_15
        # )
        self.CarAgent_grp = pygame.sprite.Group(self.CarAgent_1,self.CarAgent_2,self.CarAgent_3,self.CarAgent_4
        # ,self.CarAgent_3,
            # self.CarAgent_4
            # ,self.CarAgent_5
            )
        # self.CarAgent_1.graph_drawing()
        return self.CarAgent_1.graph_drawing(),self.CarAgent_2.graph_drawing(),self.CarAgent_3.graph_drawing(),self.CarAgent_4.graph_drawing()

    
    def init_Trajectory(self):
        self.Trajectory = Trajectory(self.screen,self.CarAgent_grp)

    def x_update(self,CarAgent_a,CarAgent_b,speed):
        if CarAgent_a.x > CarAgent_b.x:
            CarAgent_a.x += speed
            CarAgent_b.x -= speed
        else:
            CarAgent_a.x -= speed
            CarAgent_b.x += speed
    
    def y_update(self,CarAgent_a,CarAgent_b,speed):
        if CarAgent_a.y > CarAgent_b.y:
            CarAgent_a.y += speed
            CarAgent_b.y -= speed
        else:
            CarAgent_a.y -= speed
            CarAgent_b.y += speed

    def distance(self,Cars):
        if abs(Cars.rect.x - Cars.des_x) > 40 or abs(Cars.rect.y - Cars.des_y) > 40 == True:
            return True
            
    #車同士の衝突回避
    def collision_avoidance(self):
        # if len(self.ga_li2) > 0:
        for i, j in itertools.product(self.CarAgent_grp, self.CarAgent_grp):
            if i != j:
            # if i != j and self.distance(i) == True and self.distance(j) == True:#iとjが別の車なら
                # if self.distan(i) == True and self.distan(j) == True:#目的地から半径20以上離れていたら
                if (abs(i.rect.x - j.rect.x) < 30 and abs(i.rect.y - j.rect.y) < 30):#他の車との距離が半径20以内になったら
                    self.co += 1
                    i.speed = down_speed
                    j.speed = down_speed
                    if i.des_x == j.des_x and i.des_y == j.des_y:#もし目的地が同じ場合
                        self.x_update(i,j,up_speed)
                    
                    elif (i.des_y > j.des_y) and (i.des_x > j.des_x):#i番目の車の目的地がj番目よりも右側でかつ下
                        i.y -= up_speed
                        j.y += up_speed

                    elif (i.des_y > j.des_y) and (i.des_x < j.des_x):#i番目の車の目的地がj番目よりも右側でかつ下
                        i.y += up_speed
                        j.y -= up_speed
                        # self.x_update(i,j,up_speed)

                    elif i.des_y > j.des_y:#iの目的地の方が下にあったら
                        self.x_update(i,j,up_speed)
                    
                    elif i.des_x > j.des_x:#iの目的地の方が右にあったら
                        self.y_update(i,j,up_speed)
                else:
                    i.speed = up_speed
                    j.speed = up_speed
        # print(self.co)
        return self.co

    def genetic_algorithm(self,li,li2,li3,li4):
        self.genetic(self.ga_li,li)
        self.genetic(self.ga_li2,li2)
        self.genetic(self.ga_li3,li3)
        self.genetic(self.ga_li4,li4)

    def genetic(self,result_li,inp_list):
        list = []
        if len(list) != 0:
            list.clear()
        if len(result_li) != 0:
            result_li.clear()
        for hor in range(len(inp_list)):
            for ver in range(len(inp_list[hor])):
                if inp_list[hor][ver] >= 1:
                    list.append((hor,ver))
        if len(list) != 0:
            tmp = [list[0]]
            result = []
            for i in range(len(list) - 1):
                if (list[i+1][0]*10 + list[i+1][1]) - (list[i][0]*10 + list[i][1]) == 1:
                    tmp.append(list[i+1])
                else:
                    if len(tmp) > 0:
                        result.append(tmp)
                    tmp = []
                    tmp.append(list[i+1])
            result.append(tmp)
            for i in result:
                result_li.append(((400 + (300/len(inp_list))*i[0][1]),(299 + (300/len(inp_list))*i[0][0]),len(i)))
        else:
            result_li = []

    def animate(self,li,li2,li3,li4):
        self.set_screen()
        time_sta = time.time()
        self.screen.fill((255,255,255)) 
        self.genetic_algorithm(li,li2,li3,li4)
        self.init_Obstacles()
        self.init_CarAgents()
        # self.CarAgent_1.graph_drawing()
        # self.CarAgent_2.graph_drawing()
        # self.CarAgent_3.graph_drawing()
        # self.CarAgent_4.graph_drawing()
        self.CarAgent_grp.draw(self.screen)
        self.Obstacle_grp.draw(self.screen)
        self.Obstacle_grp_2.draw(self.screen)
        self.Obstacle_grp_3.draw(self.screen)
        self.Obstacle_grp_4.draw(self.screen)
        LOOP = True
        while LOOP:  # メインループ 
            # self.screen.fill((255,255,255))        # 画面を白に塗りつぶし
            self.CarAgent_grp.update()
            self.collision_avoidance()
            self.CarAgent_grp.draw(self.screen)
            # for car in self.CarAgent_grp:
            #     if abs(car.rect.x - car.des_x) < 20 and abs(car.rect.y - car.des_y) < 20:
            #         LOOP = False
            end_time = time.time()
            if end_time - time_sta >= 1000:
                LOOP = False
            # print(self.CarAgent_1.co2)
            # self.Trajectory.draw()
            # self.genetic_algorithm()
            for event in pygame.event.get():
                if event.type == KEYDOWN:  # キーを押したとき
                    # ESCキーならスクリプトを終了
                    if event.key == K_SPACE:
                        # running = False
                        time.sleep(5)
                    elif event.key == K_ESCAPE:
                        LOOP = False
                # 「閉じる」ボタンを処理する
                if event.type == pygame.QUIT: LOOP = False
            self.clock.tick(FPS)      # 毎秒の呼び出し回数に合わせて遅延
            # pressed_keys = pygame.key.get_pressed() # キー情報を取得
            pygame.display.flip()
            # 処理を書く（ここでは1秒停止する）
            # time.sleep(1)
            # 時間計測終了
            # if self.CarAgent_1.scored == True and self.CarAgent_2.scored == True and self.CarAgent_3.scored == True and self.CarAgent_4.scored == True:
            #     time_end = time.time()
            #     # 経過時間（秒）
            #     tim = time_end - time_sta
            #     print(tim)
            # break
Window = Window(BOX_WIDTH, BOX_HEIGHT)
solution = np.array([0.80825889, 1.48287366, 0.47925562, 0.03002901, 1.10485758, 1.83200447,
 1.1018567,  0.36450556, 1.02908719 ,0.18579785 ,1.44079271, 0.52411895,
 1.53065029, 1.12952957, 0.38010908, 1.58786373, 1.55581453, 1.59752236,
 0.67175093, 0.13904362, 0.68519541, 1.39833483, 1.15975029, 1.94351491,
 1.99549235, 0.10268501, 1.0438208,  0.91390503, 0.6888834,  0.99074394,
 1.20925376, 0.06963154, 0.90469312, 0.71300569, 0.6386213,  1.7774898,
 0.75091063, 0.54561435, 1.32425624, 1.44097974, 0.88050017, 0.32031769,
 1.29298957, 0.29531788, 1.05707445, 0.92699635, 0.75052749,1.64327548,
 0.73826433, 0.40319125, 0.70025869, 1.66716409, 0.64702599, 1.00392754,
 1.52620789, 0.34371884, 0.2860897,  1.27623767, 1.60313721, 0.78366351,
 0.96418461, 0.65651162, 0.05112923, 1.15933113, 0.74871602, 0.3919063,
 1.40449296, 0.12404012, 1.58651798, 0.3191169,  0.46570577, 0.41561198,
 0.40911742, 0.87709731, 0.74164299, 1.37893553, 0.34373078, 1.71255558,
 1.39023442, 1.27252871, 1.45386516, 0.45474301, 1.70129096, 1.42073424,
 1.10448793, 0.69553373, 0.58134021, 1.24807603, 1.94336819, 1.87716898,
 0.31840698, 0.68377002, 1.31018842, 1.95652142, 0.25316386, 0.33606638,
 1.02910473, 0.68433226, 0.35927104, 0.90871585])
liga = np.zeros((5,5))
liga2 = np.zeros((5,5))
liga3 = np.zeros((5,5))
liga4 = np.zeros((5,5))
    
def shaping_list(list,n):
    for hor, ver in itertools.product(range(len(list)), range(len(list))):
        list[hor][ver] = int(solution[hor*len(list) + ver + len(list)*len(list)*n])
count_1 = 0
for i in solution:
    if i >= 1:
        count_1 += 1
print(count_1)
shaping_list(liga,0)
shaping_list(liga2,1)
shaping_list(liga3,2)
shaping_list(liga4,3)
print(liga,"car1")
print(liga2,"car2")
print(liga3,"car3")
print(liga4,"car4")
# Window.set_screen()
# Window.genetic_algorithm(liga,liga2,liga3,liga4)
# Window.init_Obstacles()
# Window.init_CarAgents()
# Window.init_Trajectory()
# # Window.gatest(li)
Window.animate(liga,liga2,liga3,liga4)
