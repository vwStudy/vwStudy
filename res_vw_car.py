import heapq
from math import sqrt
import math
import numpy as np
import itertools
import pygame
import time

BOX_WIDTH = 1500        # ゲーム領域の幅
BOX_HEIGHT = 1500       # ゲーム領域の高さ
FPS = 60     # Frame per Second 毎秒のフレーム数
D = 50

down_speed = 1 
up_speed = 2.5

#ここに学習した結果を入力
solution = np.array(
 [0.32775185, 1.29517524, 1.17635555, 1.75909224, 0.29091859, 
  1.22809437, 1.6594809,  0.3005342,  1.5276873,  1.05395578, 
  1.69455444, 1.37084135, 1.00598338, 1.61256377, 0.20082736, 
  0.84292587, 1.77407938, 1.50213264, 1.29044015, 1.53904479, 
  0.67616924, 0.59144169, 0.84500169, 0.58393948, 0.8166541, 
  0.32303221, 0.08856333, 0.84662365, 1.95846297, 1.2097168, 
  0.4436809,  0.5617083,  0.43774157, 1.76028569, 0.12332012, 
  0.52090038, 1.35292256, 1.3566661,  1.77396311, 1.4959235,  
  0.62931747, 0.75509154, 0.1494494,  0.71015476, 0.01414997, 
  1.83923629, 0.91167422, 1.38646788, 1.80669418, 1.77626711, 
  1.40693494, 0.5013669,  0.95567003, 0.30089151, 0.16989127,
  0.39645503, 1.96163675, 1.7690657,  1.87673159, 0.55797495, 
  0.67333689, 1.07846513, 1.41365751, 1.04786171, 1.11835186, 
  0.26661077, 0.78897786, 0.33749054, 1.45993427, 0.92760418, 
  1.25795343, 1.5863175, 1.7787776,  1.40363355, 1.61546408, 
  1.79182455, 1.84097154, 0.24333971, 0.43336357, 0.0833296, 
  0.38261609, 0.29235074, 0.56338378, 0.89524012, 1.18773819, 
  0.89238898, 0.61152485, 1.60412188, 0.61215721, 1.11641841,
  0.16843995, 0.28591366, 0.2713051,  0.20058605, 0.62870875, 
  1.91719306, 0.09302912, 0.29040787, 0.41761321, 1.76614666])

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
        for obs,str, end in itertools.product(self.Obstacle_grp,start_posi, start_posi):
            left_ob_x = obs.rect.x
            right_ob_x = obs.rect.x + obs.width
            top_ob_y = obs.rect.y
            bottom_ob_y = obs.rect.y + obs.height
            if str[0] < end[0]:
                if self.collision(str[0],str[1],end[0],end[1],left_ob_x,right_ob_x,top_ob_y,bottom_ob_y,str[0],end[0]) == True:
                    col_li.append((str,end))
            else:
                if self.collision(str[0],str[1],end[0],end[1],left_ob_x,right_ob_x,top_ob_y,bottom_ob_y,end[0],str[0]) == True:
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
        for res in range(len(result_li) - 1):
            pygame.draw.line(self.screen, (250,0,0),(start_posi[result_li[res]][0],start_posi[result_li[res]][1]),(start_posi[result_li[res + 1]][0],start_posi[result_li[res + 1]][1]),10)
        for res in range(len(result_li)):
            self.root.append((start_posi[result_li[res]][0],start_posi[result_li[res]][1]))
        return costdis,self.root
    
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
    

class Trajectory(pygame.sprite.Sprite):# ← pygame.sprite.Sprite
    def __init__(self, screen,CarAgent_grp):
        pygame.sprite.Sprite.__init__(self)
        # super().__init__() # ←この一行
        self.screen = screen
        self.CarAgent_grp = CarAgent_grp
        self.reset_x = 1000000
        self.reset_y = 1000000
    

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
            self.Obstacle_4,self.Obstacle_5,self.Obstacle_6,self.Obstacle_7,self.Obstacle_8)
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
        self.CarAgent_grp.draw(self.screen)
        self.Obstacle_grp.draw(self.screen)
        self.Obstacle_grp_2.draw(self.screen)
        self.Obstacle_grp_3.draw(self.screen)
        self.Obstacle_grp_4.draw(self.screen)
        LOOP = True
        while LOOP:  # メインループ 
            self.CarAgent_grp.update()
            self.collision_avoidance()
            self.CarAgent_grp.draw(self.screen)
            end_time = time.time()
            if end_time - time_sta >= 1000:
                LOOP = False
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:  # キーを押したとき
                    # ESCキーならスクリプトを終了
                    if event.key == pygame.K_SPACE:
                        # running = False
                        time.sleep(5)
                    elif event.key == pygame.K_ESCAPE:
                        LOOP = False
                # 「閉じる」ボタンを処理する
                if event.type == pygame.QUIT: LOOP = False
            self.clock.tick(FPS)      # 毎秒の呼び出し回数に合わせて遅延
            pygame.display.flip()

Window = Window(BOX_WIDTH, BOX_HEIGHT)
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

#結果表示
print(count_1)
shaping_list(liga,0)
shaping_list(liga2,1)
shaping_list(liga3,2)
shaping_list(liga4,3)
print(liga,"car1")
print(liga2,"car2")
print(liga3,"car3")
print(liga4,"car4")

Window.animate(liga,liga2,liga3,liga4)