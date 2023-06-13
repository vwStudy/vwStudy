import heapq
from math import sqrt
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
 [1.25403725,1.63707607,0.34537117,0.46692902,1.86810347,
 1.18568906,0.67364596,1.89324646,0.82830462,0.3523431,
 0.15432086,0.29573098,1.06439061,1.59870093,1.97580755,
 1.12003864,1.54358525,1.82082465,0.02086383,0.4825695,
 0.36392434,0.82723439,0.36418933,1.64793356,1.08520321,
 1.70873908,1.21419134,1.22159324,0.15354872,0.8169798,
 0.7977669,1.71697157,1.75583134,1.0969045,1.80250816,
 0.87715171,0.03766605,0.16870925,1.57438554,1.11382887,
 1.16325384,1.5546369,1.50400887,0.61948955,1.4716717,
 1.08904872,1.37802616,1.29530233,0.79034857,1.41454885,
 0.16134286,1.34528155,0.16194565,1.44138628,1.60876101,
 0.78751937,1.31930033,1.84901679,1.68121327,0.35231061,
 1.81483864,1.20759885,1.82751383,0.88787637,0.61179138,
 0.66469404,1.25423522,0.13764197,0.19998714,0.52783636,
 1.6902009,0.80272668,0.07944935,0.99847551,1.20465835,
 1.67703867,0.07523967,0.42963693,0.12791744,0.30956578,
 0.59011986,1.6090654,1.26059346,1.89184595,1.22218197,
 1.72165271,0.80212005,0.28289763,0.21556561,0.08818326,
 0.48089746,0.58732784,0.21430227,0.47507705,0.47387369,
 0.05656618,0.7913301,0.91735949,0.79300591,1.08533279])


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