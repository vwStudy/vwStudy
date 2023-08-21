import time
from tkinter import *
import numpy as np
import networkx as nx
import math
from geneticalgorithm2 import geneticalgorithm2 as ga

import vw
import setting


def set_virtual_wall(GA_list):
    """
    遺伝的アルゴリズムの結果に対応したVWを設置する関数,VWの4つの頂点のlistと障害物の線分のlistを返す
    """
    size = setting.VWsize
    field_x = setting.VWfield_x
    field_y = setting.VWfield_y
    obstacles_vertex_list = []
    obstacles_line_list = []
    total_num_obstacles = 0
    
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
                obstacles_line_list.extend([[VW_LeftUp, VW_LeftDown], [VW_LeftUp, VW_RightUp], [VW_RightUp, VW_RightDown], [VW_RightUp, VW_LeftDown]])
    return obstacles_vertex_list, obstacles_line_list




def set_vertex_list(obstacle_list, carAgent):
        """
        頂点のリストを作成し返す関数
        """
        start = carAgent[0].copy()
        goal = carAgent[1].copy()
        vertex_list = [start, goal]
        new_vertex_list = vertex_list.copy()
        new_vertex_list.extend(obstacle_list)

        #print("ver"+str(vertex_list))
        return new_vertex_list


def visibility_graph(vertex_list, obstacle_line_list):
    """

    O(n^3)の素朴な可視グラフ法を実行し可視グラフのリストを返す関数
    """
    visibility_graph_list = []
    cnt=0
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
                
                cnt+=1
                print(str(cnt))
                if s * t > 0:
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
    #solution_list = vw.main()
    solution_list = [0.4830308765178599, 0.9823179155949318, 0.23567058072789449, 0.6598978654778107, 1.0185323699863469, 1.8528592118363258, 1.3254095580883782, 0.07231107218265032, 0.6969338202475376, 0.5241456276917202, 0.9898407591921201, 0.3699841452160064, 0.17969433196739948, 1.8928912642292177, 0.780119540444121, 0.8691953448472354, 0.4907781266242711, 0.30085318780961756, 0.7338622785164621, 1.6133852231780121, 0.08598944635611594, 0.6797483083504425, 0.968107451520396, 0.8628160921941983, 0.4969412301135474, 0.5425910927531294, 1.0541232663439746, 0.4127038771580383, 0.7483661993126998, 0.4019831232548665, 0.5672979240177336, 0.8656969843196494, 0.6232815075534692, 0.2787298859493809, 0.8173828226631601, 0.9629021143971079, 0.39599922324417297, 0.797161276609919, 0.5061948832590522, 0.29768938575899506, 1.6399824688227047, 1.4129856328778407, 1.6970252447383622, 1.3843950347263418, 0.9719502834176259, 0.5975288893536095, 1.4878235054579352, 1.9576941402061414, 0.9376777266708521, 1.3821252113260702, 0.3404917808383774, 0.4291680951462318, 0.4326969706107202, 1.2649833432348534, 0.9885591203881934, 0.27296443748417776, 0.9513397878684813, 0.7887986705510384, 1.5042781145369593, 1.1097019277090325, 0.33459463382818333, 1.8511033754899477, 0.08198471361729887, 0.4740049956265806]
    #vw_list = np.array(solution_list)
    #vw_list = np.array(solution_list).reshape(4,setting.VWnum**2).tolist()
    vw_list = np.array(solution_list).reshape(4,4,4).tolist()
    print(vw_list)

    f = open('data.txt', 'w', encoding='UTF-8')
    f.writelines(str(vw_list))
    f.writelines('\n')
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
    print("car1_vw:"+str(car1_VW_list))
    print("car2_vw:"+str(car2_VW_list))
    print("car3_vw:"+str(car3_VW_list))
    print("car4_vw:"+str(car4_VW_list))

    # car1_start_goal_list = setting.car1_STARTtoGOAL
    # car2_start_goal_list = setting.car2_STARTtoGOAL
    # car3_start_goal_list = setting.car3_STARTtoGOAL
    # car4_start_goal_list = setting.car4_STARTtoGOAL

    cars_tuple = ((setting.car1_STARTtoGOAL[0],setting.car1_STARTtoGOAL[1]), (setting.car2_STARTtoGOAL[0],setting.car2_STARTtoGOAL[1]), (setting.car3_STARTtoGOAL[0],setting.car3_STARTtoGOAL[1]), (setting.car4_STARTtoGOAL[0],setting.car4_STARTtoGOAL[1]))

    car1_vertex_list = set_vertex_list(car1_VW_list, cars_tuple[0])
    car2_vertex_list = set_vertex_list(car2_VW_list, cars_tuple[1])
    car3_vertex_list = set_vertex_list(car3_VW_list, cars_tuple[2])
    car4_vertex_list = set_vertex_list(car4_VW_list, cars_tuple[3])


    car1_vis_graph = visibility_graph(car1_vertex_list, car1_vw_line_list)
    car2_vis_graph = visibility_graph(car2_vertex_list, car2_vw_line_list)
    car3_vis_graph = visibility_graph(car3_vertex_list, car3_vw_line_list)
    car4_vis_graph = visibility_graph(car4_vertex_list, car4_vw_line_list)

    car1_shortest_path, car1_shortest_length = dijkstra(car1_vis_graph)
    car2_shortest_path, car2_shortest_length = dijkstra(car2_vis_graph)
    car3_shortest_path, car3_shortest_length = dijkstra(car3_vis_graph)
    car4_shortest_path, car4_shortest_length = dijkstra(car4_vis_graph)


    
    f.writelines("car1_vw:"+str(car1_VW_list))
    f.writelines('\n')
    f.writelines("car2_vw:"+str(car2_VW_list))
    f.writelines('\n')
    f.writelines("car3_vw:"+str(car3_VW_list))
    f.writelines('\n')
    f.writelines("car4_vw:"+str(car4_VW_list))
    f.writelines('\n')
    f.writelines('\n')


    for i in range(len(car1_shortest_path)):
        print("car1_path::"+"x:"+str(car1_vertex_list[car1_shortest_path[i]][0])+"y:"+str(car1_vertex_list[car1_shortest_path[i]][1]))
        f.writelines("car1_path::"+"x:"+str(car1_vertex_list[car1_shortest_path[i]][0])+"y:"+str(car1_vertex_list[car1_shortest_path[i]][1]))
        f.writelines('\n')
        
    for i in range(len(car2_shortest_path)):
        print("car2_path::"+"x:"+str(car2_vertex_list[car2_shortest_path[i]][0])+"y:"+str(car2_vertex_list[car2_shortest_path[i]][1]))
        f.writelines("car2_path::"+"x:"+str(car2_vertex_list[car2_shortest_path[i]][0])+"y:"+str(car2_vertex_list[car2_shortest_path[i]][1]))
        f.writelines('\n')
    
    for i in range(len(car3_shortest_path)):
        print("car3_path::"+"x:"+str(car3_vertex_list[car3_shortest_path[i]][0])+"y:"+str(car3_vertex_list[car3_shortest_path[i]][1]))
        f.writelines("car3_path::"+"x:"+str(car3_vertex_list[car3_shortest_path[i]][0])+"y:"+str(car3_vertex_list[car3_shortest_path[i]][1]))
        f.writelines('\n')

    for i in range(len(car4_shortest_path)):
        print("car4_path::"+"x:"+str(car4_vertex_list[car4_shortest_path[i]][0])+"y:"+str(car4_vertex_list[car4_shortest_path[i]][1]))
        f.writelines("car4_path::"+"x:"+str(car4_vertex_list[car4_shortest_path[i]][0])+"y:"+str(car4_vertex_list[car4_shortest_path[i]][1]))
        f.writelines('\n')
    
    f.close()

    
    # print(car1_shortest_path)
    # print("testttt2222")

    # #car1の最短経路描画
    # for i in range(len(car1_shortest_path)-1):
    #      canvas.create_line(car1_vertex_list[car1_shortest_path[i]][0],car1_vertex_list[car1_shortest_path[i]][1], car1_vertex_list[car1_shortest_path[i+1]][0],car1_vertex_list[car1_shortest_path[i+1]][1], fill = "red", width = 5)
    # #canvas.create_line(car1_vertex_list,car1_vertex_list, car1_vertex_list, car1_vertex_list, fill = "red", width = 5)
    # #canvas.create_line(car1_vertex_list, car1_vertex_list, 860, 240, fill = "red", width = 5)
    # for i in range(len(car2_shortest_path)-1):
    #      canvas.create_line(car2_vertex_list[car2_shortest_path[i]][0],car2_vertex_list[car2_shortest_path[i]][1], car2_vertex_list[car2_shortest_path[i+1]][0],car2_vertex_list[car2_shortest_path[i+1]][1], fill = "blue", width = 5)
    
    # for i in range(len(car3_shortest_path)-1):
    #      canvas.create_line(car3_vertex_list[car3_shortest_path[i]][0],car3_vertex_list[car3_shortest_path[i]][1], car3_vertex_list[car3_shortest_path[i+1]][0],car3_vertex_list[car3_shortest_path[i+1]][1], fill = "yellow", width = 5)         
    
    # for i in range(len(car4_shortest_path)-1):
    #      canvas.create_line(car4_vertex_list[car4_shortest_path[i]][0],car4_vertex_list[car4_shortest_path[i]][1], car4_vertex_list[car4_shortest_path[i+1]][0],car4_vertex_list[car4_shortest_path[i+1]][1], fill = "black", width = 5)
         
    
    # x_size=25
    # y_size=10

    # #計算部分
    # while True:
    #     #CarAgent_1.move(270,0)
    #     canvas.coords(car1, car1_x0, car1_y0, car1_x0+x_size, car1_y0+y_size)
    #     canvas.coords(car2, car2_x0, car2_y0, car2_x0+y_size, car2_y0+x_size)
    #     canvas.coords(car3, car3_x0, car3_y0, car3_x0+x_size, car3_y0+y_size)
    #     canvas.coords(car4, car4_x0, car4_y0, car4_x0+y_size, car4_y0+x_size)
    #     car1_x0+=5
    #     car2_y0+=5
    #     car3_x0-=5
    #     car4_y0-=5
    #     time.sleep(0.02)
    #     tk.update() #ウインド画面を更新
        
    #     if car1_x0 >= 630:
    #         break
        

    # tk.mainloop()

if __name__ == '__main__':
    main()