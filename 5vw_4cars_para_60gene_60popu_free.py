import time
import pygame
from pygame.locals import *
import numpy as np
import math
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

# パラメータ（指定しなければデフォルト）
params = {
    'max_num_iteration': 60,
    'population_size': 60,
}

#VWのセルを指定する変数、VWnum * VWnumのセル数になる
VWnum = 5

class CarAgent(pygame.sprite.Sprite):# ← pygame.sprite.Sprite
    def __init__(self, screen, x, y, des,Obstacle_grp,color):
        pygame.sprite.Sprite.__init__(self)
        # super().__init__() # ←この一行
        self.screen = screen
        self.x = self.start_x = self.relay_pos_x = x
        self.y = self.start_y = self.relay_pos_y = y
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
        self.scored = False
        self.count = 0
        self.co2 = 0
    
    def update(self):
        self.move(self.relay_pos_x,self.relay_pos_y)
        if abs(self.rect.x - self.relay_pos_x) < 5 and abs(self.rect.y - self.relay_pos_y) < 5:
            if self.count < len(self.root) - 1:
                self.count += 1
                self.relay_pos_x = self.root[self.count][0]
                self.relay_pos_y = self.root[self.count][1]

    def move(self,des_x,des_y):
        if self.rect.x == self.des_x and self.rect.y == self.des_y:
            self.speed = 0
        elif des_x == self.x:
            if  self.x > des_x and self.y > des_y:
                self.y -= (self.speed)
            elif self.x > des_x:
                self.y += (self.speed)  
            else:
                self.y += (self.speed)

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
                self.rect.x,self.rect.y =self.x,self.y
            

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

        for obs,str, end in itertools.product(self.Obstacle_grp,start_posi, start_posi):
            left_ob_x = obs.rect.x
            right_ob_x = obs.rect.x + obs.width
            top_ob_y = obs.rect.y
            bottom_ob_y = obs.rect.y + obs.height

            obs_posi.append((left_ob_x,top_ob_y))
            start_posi.extend([(left_ob_x,top_ob_y),(right_ob_x,top_ob_y),(left_ob_x,bottom_ob_y),(right_ob_x,bottom_ob_y)])

            if str[0] < end[0]:
                if self.collision(str[0],str[1],end[0],end[1],left_ob_x,right_ob_x,top_ob_y,bottom_ob_y,str[0],end[0]) == True:
                    col_li.append((str,end))
            else:
                if self.collision(str[0],str[1],end[0],end[1],left_ob_x,right_ob_x,top_ob_y,bottom_ob_y,end[0],str[0]) == True:
                    col_li.append((str,end))
        for str, end in itertools.product(start_posi, start_posi):
            total_line_posi.append((str,end))
        result = list(set(total_line_posi) - set(col_li))
        for sta in range(len(start_posi)):
            dijkstra.append([])
            for line in result:
                if line[0] == start_posi[sta]:
                    dis_x = abs(line[0][0] -line[1][0])**2
                    dis_y = abs(line[0][1] -line[1][1])**2
                    distan = sqrt(dis_x + dis_y)
                    dijkstra[sta].append([start_posi.index(line[1]),distan])
        result_li,costdis = self.execution_dij(dijkstra,1)
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
        self.screen = screen
        self.CarAgent_grp = CarAgent_grp
        self.reset_x = 1000000
        self.reset_y = 1000000
    
    def draw(self):
        Trajectory_li = []
        for car in ((self.CarAgent_grp)):
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

        return len(self.ga_li2)

    def init_Obstacles(self):
        #左上の縦置き
        self.Obstacle_1 = Obstacle(self.screen,0,0,0,0,"green")
        #左上の横置き
        self.Obstacle_2 = Obstacle(self.screen,0,0,0,0,"green")
        #左下の縦置き
        self.Obstacle_3 = Obstacle(self.screen,0,0,0,0,"green")
        #左下の横置き
        self.Obstacle_4 = Obstacle(self.screen,0,0,0,0,"green")
        #右上の横置き
        self.Obstacle_5 = Obstacle(self.screen,0,0,0,0,"green")
        #右上の縦置き
        self.Obstacle_6 = Obstacle(self.screen,0,0,0,0,"green")
        #右下の縦
        self.Obstacle_7 = Obstacle(self.screen,0,0,0,0,"green")
        #右下の横
        self.Obstacle_8 = Obstacle(self.screen,0,0,0,0,"green")
        # self.Obstacle_9 = Obstacle(self.screen,525,425,10,10,"green")
        # self.Obstacle_10 = Obstacle(self.screen,350,200,200,100,"green")
        # self.Obstacle_11 = Obstacle(self.screen,670,450,120,100,"green")
        # self.Obstacle_12 = Obstacle(self.screen,530,479,70,62,(127,255,0,128))
        self.Obstacle_grp = pygame.sprite.Group(self.Obstacle_1,self.Obstacle_2,self.Obstacle_3,
            self.Obstacle_4,self.Obstacle_5,self.Obstacle_6,self.Obstacle_7,self.Obstacle_8

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

        self.CarAgent_grp = pygame.sprite.Group(self.CarAgent_1,self.CarAgent_2,self.CarAgent_3,self.CarAgent_4)

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
        for i, j in itertools.product(self.CarAgent_grp, self.CarAgent_grp):
            if i != j:
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
        LOOP = True
        while LOOP:  # メインループ 
            self.CarAgent_grp.update()
            self.collision_avoidance()
            for car in self.CarAgent_grp:
                if abs(car.rect.x - car.des_x) < 20 and abs(car.rect.y - car.des_y) < 20:
                    LOOP = False
            end_time = time.time()
            if end_time - time_sta >= 1000:
                LOOP = False

        return self.collision_avoidance(),end_time - time_sta,self.init_Obstacles(),self.init_CarAgents()

Window = Window(BOX_WIDTH, BOX_HEIGHT)
licore = []
lili = []
def f(p):
    li = []
    li2 = []
    li3 = []
    li4 = []
    for i in range(VWnum):
        li.append([])
        li2.append([])
        li3.append([])
        li4.append([])
        for j in range(VWnum):
            li[i].append(int(p[j+i*VWnum+(VWnum**2)*0]))
            li2[i].append(int(p[j+i*VWnum+(VWnum**2)*1]))
            li3[i].append(int(p[j+i*VWnum+(VWnum**2)*2]))
            li4[i].append(int(p[j+i*VWnum+(VWnum**2)*3]))
    array = np.array(li)
    array2 = np.array(li2)
    array3 = np.array(li3)
    array4 = np.array(li4)
    # Window.set_screen()
    collision,time,(obs,obs2,obs3,obs4),((distance,root),(distance_2,root_2),(distance_3,root_3),(distance_4,root_4)) = Window.animate(array,array2,array3,array4) 
    result = [distance,distance_2,distance_3,distance_4]
    out = 0
    for res in result:
        if res >= 3000:
            out += 1000000
    print(li,"car1")
    print(li2,"car2")
    print(li3,"car3")
    print(li4,"car4")
    print(root,"root")
    print(root_2,"root2")
    print(root_3,"root3")
    print(root_4,"root4")
    total_num_obstacles = len(obs) + len(obs2)+len(obs3)+len(obs4)
    print(total_num_obstacles,"障害物数")
    print((total_num_obstacles/(VWnum*(VWnum**2-1))),"VW%")
    print(time)
    print(distance,"distance")
    print(distance_2,"distance2")
    print(distance_3,"distance3")
    print(distance_4,"distance4")
    print(collision,"衝突")
    all_distance = distance + distance_2 + distance_3 + distance_4
    print(all_distance*((total_num_obstacles/(VWnum*((VWnum**2)-1)))) + collision*1000000 + out,"ans")

    return all_distance*((total_num_obstacles/(VWnum*((VWnum**2)-1)))) + collision*1000000 + out

var_list = [[0,2]] * (VWnum**2 * 4) #この中にGAで出た値を入れていく,[0, 2]は0~1の値が入るという意味

varbound = np.array(var_list)
model = ga(function=f,
           dimension=((VWnum**2) * 4),
           variable_type='real',
           variable_boundaries=varbound,
           algorithm_parameters=params
)
model.run()
convergence = model.report
solution = model.result
#print((solution['variable']),"2222") # x, y の最適値
solution_list = []
for i in solution['variable']:
    solution_list.append(i)
print(solution_list)
print(solution['score'],"最小値") # x, y の最適値での関数の値