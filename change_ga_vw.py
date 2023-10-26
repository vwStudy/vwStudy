import numpy as np
import networkx as nx
import itertools

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

    def GA_function(genom):
        """
        GeneticalAlgorism用の関数
        """
        car_ga_array = [[[],[],[]],[[],[],[]],[[],[],[]],[[],[],[]]]
        ga_array = np.array(genom.reshape(4, 3, 3))
        for i in range(len(ga_array)):
            for j in range(len(ga_array[i])):
                l = list(ga_array[i][j])
                car_ga_array[i][j] = l

        # print(car_ga_array)

        # print("vw"+str(car_ga_array))

        #ToDo 以下の処理は変える必要がある
        #遺伝的アルゴリズムの結果に対しVWを設置
        car1_VW_list, car1_vw_line_list = VW.set_virtual_wall(car_ga_array[0])
        car2_VW_list, car2_vw_line_list = VW.set_virtual_wall(car_ga_array[1])
        car3_VW_list, car3_vw_line_list = VW.set_virtual_wall(car_ga_array[2])
        car4_VW_list, car4_vw_line_list = VW.set_virtual_wall(car_ga_array[3])

        print(car1_VW_list)
        # print(car2_VW_list)

        #CarAgentにODを設定
        cars_tuple = (CarAgent(setting.car1_STARTtoGOAL[0],setting.car1_STARTtoGOAL[1]), CarAgent(setting.car2_STARTtoGOAL[0],setting.car2_STARTtoGOAL[1]), CarAgent(setting.car3_STARTtoGOAL[0],setting.car3_STARTtoGOAL[1]), CarAgent(setting.car4_STARTtoGOAL[0],setting.car4_STARTtoGOAL[1]))
        # print(setting.car1_STARTtoGOAL[0],setting.car1_STARTtoGOAL[1])
        # print(setting.car2_STARTtoGOAL[0],setting.car2_STARTtoGOAL[1])

        wall_edge_list, wall_line_list = Environment.set_wall()

        # car1_vw_line_list.extend(wall_line_list)
        # car2_vw_line_list.extend(wall_line_list)
        # car3_vw_line_list.extend(wall_line_list)
        # car4_vw_line_list.extend(wall_line_list)


        # print(car1_VW_list)
        
        #頂点のlistを作成
        car1_vertex_list = Environment.set_vertex_list(car1_VW_list, cars_tuple[0], wall_edge_list)
        car2_vertex_list = Environment.set_vertex_list(car2_VW_list, cars_tuple[1], wall_edge_list)
        car3_vertex_list = Environment.set_vertex_list(car3_VW_list, cars_tuple[2], wall_edge_list)
        car4_vertex_list = Environment.set_vertex_list(car4_VW_list, cars_tuple[3], wall_edge_list)

        #可視グラフ, ダイクストラ法を実行
        car1_vis_graph = Execution.visibility_graph(car1_vertex_list, car1_vw_line_list)
        car2_vis_graph = Execution.visibility_graph(car2_vertex_list, car2_vw_line_list)
        car3_vis_graph = Execution.visibility_graph(car3_vertex_list, car3_vw_line_list)
        car4_vis_graph = Execution.visibility_graph(car4_vertex_list, car4_vw_line_list)

        car1_shortest_path, car1_shortest_length = Execution.dijkstra(car1_vis_graph)
        car2_shortest_path, car2_shortest_length = Execution.dijkstra(car2_vis_graph)
        car3_shortest_path, car3_shortest_length = Execution.dijkstra(car3_vis_graph)
        car4_shortest_path, car4_shortest_length = Execution.dijkstra(car4_vis_graph)

        for path in car1_shortest_path:
            print("car1 :",car1_vertex_list[path])

        for path in car2_shortest_path:
            print("car2 :",car2_vertex_list[path])
        
        for path in car3_shortest_path:
            print("car3 :",car3_vertex_list[path])

        for path in car4_shortest_path:
            print("car4 :",car4_vertex_list[path])
        
        collision = Environment.collision_CarToCar(car1_vertex_list, car1_shortest_path, car2_vertex_list, car2_shortest_path, car3_vertex_list, car3_shortest_path, car4_vertex_list, car4_shortest_path)

        print("collision::"+str(collision))

        total_num_obstacles = len(car1_VW_list)/4 + len(car2_VW_list)/4 + len(car3_VW_list)/4 + len(car4_VW_list)/4
        #print(total_num_obstacles)
        
        #print("collision::"+str(collision))
        #全ての経路長を足す
        all_path_length = car1_shortest_length + car2_shortest_length + car3_shortest_length + car4_shortest_length
        # print("all_len::"+str(all_path_length))

        return all_path_length * (total_num_obstacles / (setting.car_num * (setting.VWnum ** 2))) + collision * 1000000

    def single_GA_function(genom):
        """
        GeneticalAlgorism用の関数
        """
        car_ga_array = [[[],[],[]]]
        ga_array = np.array(genom.reshape(1, 3, 3))
        for i in range(len(ga_array)):
            for j in range(len(ga_array[i])):
                l = list(ga_array[i][j])
                car_ga_array[i][j] = l

        # print(car_ga_array)

        # print("vw"+str(car_ga_array))

        #ToDo 以下の処理は変える必要がある
        #遺伝的アルゴリズムの結果に対しVWを設置
        car1_VW_list, car1_vw_line_list = VW.set_virtual_wall(car_ga_array[0])
        car2_VW_list, car2_vw_line_list = VW.set_virtual_wall(car_ga_array[0])
        car3_VW_list, car3_vw_line_list = VW.set_virtual_wall(car_ga_array[0])
        car4_VW_list, car4_vw_line_list = VW.set_virtual_wall(car_ga_array[0])

        print(car1_VW_list)
        # print(car2_VW_list)

        #CarAgentにODを設定
        cars_tuple = (CarAgent(setting.car1_STARTtoGOAL[0],setting.car1_STARTtoGOAL[1]), CarAgent(setting.car2_STARTtoGOAL[0],setting.car2_STARTtoGOAL[1]), CarAgent(setting.car3_STARTtoGOAL[0],setting.car3_STARTtoGOAL[1]), CarAgent(setting.car4_STARTtoGOAL[0],setting.car4_STARTtoGOAL[1]))
        # print(setting.car1_STARTtoGOAL[0],setting.car1_STARTtoGOAL[1])
        # print(setting.car2_STARTtoGOAL[0],setting.car2_STARTtoGOAL[1])

        wall_edge_list, wall_line_list = Environment.set_wall()

        # car1_vw_line_list.extend(wall_line_list)
        # car2_vw_line_list.extend(wall_line_list)
        # car3_vw_line_list.extend(wall_line_list)
        # car4_vw_line_list.extend(wall_line_list)


        # print(car1_VW_list)
        
        #頂点のlistを作成
        car1_vertex_list = Environment.set_vertex_list(car1_VW_list, cars_tuple[0], wall_edge_list)
        car2_vertex_list = Environment.set_vertex_list(car2_VW_list, cars_tuple[1], wall_edge_list)
        car3_vertex_list = Environment.set_vertex_list(car3_VW_list, cars_tuple[2], wall_edge_list)
        car4_vertex_list = Environment.set_vertex_list(car4_VW_list, cars_tuple[3], wall_edge_list)

        #可視グラフ, ダイクストラ法を実行
        car1_vis_graph = Execution.visibility_graph(car1_vertex_list, car1_vw_line_list)
        car2_vis_graph = Execution.visibility_graph(car2_vertex_list, car2_vw_line_list)
        car3_vis_graph = Execution.visibility_graph(car3_vertex_list, car3_vw_line_list)
        car4_vis_graph = Execution.visibility_graph(car4_vertex_list, car4_vw_line_list)

        car1_shortest_path, car1_shortest_length = Execution.dijkstra(car1_vis_graph)
        car2_shortest_path, car2_shortest_length = Execution.dijkstra(car2_vis_graph)
        car3_shortest_path, car3_shortest_length = Execution.dijkstra(car3_vis_graph)
        car4_shortest_path, car4_shortest_length = Execution.dijkstra(car4_vis_graph)

        for path in car1_shortest_path:
            print("car1 :",car1_vertex_list[path])

        for path in car2_shortest_path:
            print("car2 :",car2_vertex_list[path])
        
        for path in car3_shortest_path:
            print("car3 :",car3_vertex_list[path])

        for path in car4_shortest_path:
            print("car4 :",car4_vertex_list[path])
        
        collision = Environment.collision_CarToCar(car1_vertex_list, car1_shortest_path, car2_vertex_list, car2_shortest_path, car3_vertex_list, car3_shortest_path, car4_vertex_list, car4_shortest_path)

        print("collision::"+str(collision))

        total_num_obstacles = len(car1_VW_list)/4 + len(car2_VW_list)/4 + len(car3_VW_list)/4 + len(car4_VW_list)/4
        #print(total_num_obstacles)
        
        #print("collision::"+str(collision))
        #全ての経路長を足す
        all_path_length = car1_shortest_length + car2_shortest_length + car3_shortest_length + car4_shortest_length
        # print("all_len::"+str(all_path_length))

        return all_path_length * (total_num_obstacles / (setting.car_num * (setting.VWnum ** 2))) + collision * 1000000



    # def GA_function(genom):
    #     """
        
    #     """
    #     car_ga_array = [[[],[],[],[]],[[],[],[],[]],[[],[],[],[]],[[],[],[],[]]]
    #     ga_array = np.array(genom.reshape(4, 4, 4))
    #     for i in range(len(ga_array)):
    #         for j in range(len(ga_array[i])):
    #             l = list(ga_array[i][j])
    #             car_ga_array[i][j] = l

    #     #ToDo 以下の処理は変える必要がある
    #     #遺伝的アルゴリズムの結果に対しVWを設置
    #     car1_vw_list, car1_vw_line_list = VW.set_virtual_wall(car_ga_array[0])
    #     car2_vw_list, car2_vw_line_list = VW.set_virtual_wall(car_ga_array[1])
    #     car3_vw_list, car3_vw_line_list = VW.set_virtual_wall(car_ga_array[2])
    #     car4_vw_list, car4_vw_line_list = VW.set_virtual_wall(car_ga_array[3])
    #     ##print("car1::"+str(car1_VW_list))

    #     #CarAgentにODを設定
    #     cars_tuple = (CarAgent(setting.car1_STARTtoGOAL[0],setting.car1_STARTtoGOAL[1]), CarAgent(setting.car2_STARTtoGOAL[0],setting.car2_STARTtoGOAL[1]), CarAgent(setting.car3_STARTtoGOAL[0],setting.car3_STARTtoGOAL[1]), CarAgent(setting.car4_STARTtoGOAL[0],setting.car4_STARTtoGOAL[1]))
    #     # print(setting.car1_STARTtoGOAL[0],setting.car1_STARTtoGOAL[1])
    #     # print(setting.car2_STARTtoGOAL[0],setting.car2_STARTtoGOAL[1])

    #     wall_edge_list, wall_line_list = Environment.set_wall()

    #     car1_vw_line_list.extend(wall_line_list)
    #     car2_vw_line_list.extend(wall_line_list)
    #     car3_vw_line_list.extend(wall_line_list)
    #     car4_vw_line_list.extend(wall_line_list)

    #     #頂点のlistを作成
    #     car1_vertex_list = Environment.set_vertex_list(car1_vw_list, cars_tuple[0], wall_edge_list)
    #     car2_vertex_list = Environment.set_vertex_list(car2_vw_list, cars_tuple[1], wall_edge_list)
    #     car3_vertex_list = Environment.set_vertex_list(car3_vw_list, cars_tuple[2], wall_edge_list)
    #     car4_vertex_list = Environment.set_vertex_list(car4_vw_list, cars_tuple[3], wall_edge_list)
    #     # print(car1_vertex_list)
    #     # print(car2_vertex_list)

    #     #可視グラフ, ダイクストラ法を実行
    #     car1_vis_graph = Execution.visibility_graph(car1_vertex_list, car1_vw_line_list)
    #     car2_vis_graph = Execution.visibility_graph(car2_vertex_list, car2_vw_line_list)
    #     car3_vis_graph = Execution.visibility_graph(car3_vertex_list, car3_vw_line_list)
    #     car4_vis_graph = Execution.visibility_graph(car4_vertex_list, car4_vw_line_list)

    #     # print(car1_vis_graph)
    #     # print(car2_vis_graph)
    #     # print(car3_vis_graph)
    #     # print(car4_vis_graph)

    #     car1_shortest_path, car1_shortest_length = Execution.dijkstra(car1_vis_graph)
    #     car2_shortest_path, car2_shortest_length = Execution.dijkstra(car2_vis_graph)
    #     car3_shortest_path, car3_shortest_length = Execution.dijkstra(car3_vis_graph)
    #     car4_shortest_path, car4_shortest_length = Execution.dijkstra(car4_vis_graph)

    #     for path in car1_shortest_path:
    #         print("car1 :",car1_vertex_list[path])

    #     for path in car2_shortest_path:
    #         print("car2 :",car2_vertex_list[path])
            
    #     for path in car3_shortest_path:
    #         print("car3 :",car3_vertex_list[path])

    #     for path in car4_shortest_path:
    #         print("car4 :",car4_vertex_list[path])
            
    #     #車両の衝突判定
    #     car1_move_list = Environment.car_move(car1_vertex_list, car1_shortest_path)
    #     car2_move_list = Environment.car_move(car2_vertex_list, car2_shortest_path)
    #     car3_move_list = Environment.car_move(car3_vertex_list, car3_shortest_path)
    #     car4_move_list = Environment.car_move(car4_vertex_list, car4_shortest_path)

    #     collision = Environment.cars_collision(car1_move_list,car2_move_list,car3_move_list,car4_move_list)
            
    #     print("collision::"+str(collision))

    #     total_num_obstacles = len(car1_vw_list)/4 + len(car2_vw_list)/4 + len(car3_vw_list)/4 + len(car4_vw_list)/4
    #     #print(total_num_obstacles)

    #     #全ての経路長を足す
    #     all_path_length = car1_shortest_length + car2_shortest_length + car3_shortest_length + car4_shortest_length
    #     print("all_len::"+str(all_path_length))

    #     print(all_path_length * (total_num_obstacles / (setting.car_num * (setting.VWnum ** 2))) + collision * 100000)
    #     return all_path_length * (total_num_obstacles / (setting.car_num * (setting.VWnum ** 2))) + collision * 100000, collision, all_path_length
    
    def two_steps_ga_function(genom,two_steps_list,zeros_list):
        """
        GeneticalAlgorism用の関数
        """

        ga_array = np.array(genom.reshape(1, len(two_steps_list), 9))

        car_ga_array = zeros_list
        
        for i in range(len(two_steps_list)):
            car_ga_array[0][two_steps_list[i]] = list(ga_array[0][i])

        print(car_ga_array)

        # print("vw"+str(car_ga_array))

        VWsize = (setting.VWfield / ((setting.VWsize**2) **2) )
        print(VWsize)

        #ToDo 以下の処理は変える必要がある
        #遺伝的アルゴリズムの結果に対しVWを設置
        car1_VW_list, car1_vw_line_list = VW.set_virtual_wall(car_ga_array[0], VWsize)
        car2_VW_list, car2_vw_line_list = VW.set_virtual_wall(car_ga_array[0], VWsize)
        car3_VW_list, car3_vw_line_list = VW.set_virtual_wall(car_ga_array[0], VWsize)
        car4_VW_list, car4_vw_line_list = VW.set_virtual_wall(car_ga_array[0], VWsize)

        print(car1_VW_list)
        # print(car2_VW_list)

        #CarAgentにODを設定
        cars_tuple = (CarAgent(setting.car1_STARTtoGOAL[0],setting.car1_STARTtoGOAL[1]), CarAgent(setting.car2_STARTtoGOAL[0],setting.car2_STARTtoGOAL[1]), CarAgent(setting.car3_STARTtoGOAL[0],setting.car3_STARTtoGOAL[1]), CarAgent(setting.car4_STARTtoGOAL[0],setting.car4_STARTtoGOAL[1]))
        # print(setting.car1_STARTtoGOAL[0],setting.car1_STARTtoGOAL[1])
        # print(setting.car2_STARTtoGOAL[0],setting.car2_STARTtoGOAL[1])

        wall_edge_list, wall_line_list = Environment.set_wall()

        # car1_vw_line_list.extend(wall_line_list)
        # car2_vw_line_list.extend(wall_line_list)
        # car3_vw_line_list.extend(wall_line_list)
        # car4_vw_line_list.extend(wall_line_list)


        # print(car1_VW_list)
        
        #頂点のlistを作成
        car1_vertex_list = Environment.set_vertex_list(car1_VW_list, cars_tuple[0], wall_edge_list)
        car2_vertex_list = Environment.set_vertex_list(car2_VW_list, cars_tuple[1], wall_edge_list)
        car3_vertex_list = Environment.set_vertex_list(car3_VW_list, cars_tuple[2], wall_edge_list)
        car4_vertex_list = Environment.set_vertex_list(car4_VW_list, cars_tuple[3], wall_edge_list)

        #可視グラフ, ダイクストラ法を実行
        car1_vis_graph = Execution.visibility_graph(car1_vertex_list, car1_vw_line_list)
        car2_vis_graph = Execution.visibility_graph(car2_vertex_list, car2_vw_line_list)
        car3_vis_graph = Execution.visibility_graph(car3_vertex_list, car3_vw_line_list)
        car4_vis_graph = Execution.visibility_graph(car4_vertex_list, car4_vw_line_list)

        car1_shortest_path, car1_shortest_length = Execution.dijkstra(car1_vis_graph)
        car2_shortest_path, car2_shortest_length = Execution.dijkstra(car2_vis_graph)
        car3_shortest_path, car3_shortest_length = Execution.dijkstra(car3_vis_graph)
        car4_shortest_path, car4_shortest_length = Execution.dijkstra(car4_vis_graph)

        for path in car1_shortest_path:
            print("car1 :",car1_vertex_list[path])

        for path in car2_shortest_path:
            print("car2 :",car2_vertex_list[path])
        
        for path in car3_shortest_path:
            print("car3 :",car3_vertex_list[path])

        for path in car4_shortest_path:
            print("car4 :",car4_vertex_list[path])
        
        collision = Environment.collision_CarToCar(car1_vertex_list, car1_shortest_path, car2_vertex_list, car2_shortest_path, car3_vertex_list, car3_shortest_path, car4_vertex_list, car4_shortest_path)

        print("collision::"+str(collision))

        total_num_obstacles = len(car1_VW_list)/4 + len(car2_VW_list)/4 + len(car3_VW_list)/4 + len(car4_VW_list)/4
        #print(total_num_obstacles)
        
        #print("collision::"+str(collision))
        #全ての経路長を足す
        all_path_length = car1_shortest_length + car2_shortest_length + car3_shortest_length + car4_shortest_length
        # print("all_len::"+str(all_path_length))

        return all_path_length * (total_num_obstacles / (setting.car_num * (setting.VWnum ** 2))) + collision * 1000000

    def two_steps_ga_setting(best):
        """
        GeneticalAlgorism用の関数
        """

        two_steps_list = []

        for index, cell in enumerate(best):
            if cell == 1:
                two_steps_list.append(index)
        
        print(two_steps_list)
        
        two_VWnum = 3
        gene_size = len(two_steps_list) * setting.VWnum
        VWsize = setting.VWsize/two_VWnum
        zeros_list = [[[0] * (two_VWnum)**2] * 9]

        return two_steps_list, zeros_list

class Environment():
    def __init__(self, obstacle_x, obstacle_y, width, height):
        self.x = obstacle_x
        self.y = obstacle_y
        self.width = width
        self.height = height
    
    def set_wall():
        """
        
        """
        wall_edge_list = []
        #設置する壁の考慮すべきエッジをlistにまとめる
        wall_edge_list = setting.wall_edge_list

        wall_line_list = setting.wall_line_list
        return wall_edge_list, wall_line_list

    def car_move(car_vertex_list, car_shortest_path):

        car_node_move_list = []

        for index in range(0, len(car_shortest_path)-1):
            start = car_vertex_list[car_shortest_path[index]]
            node = car_vertex_list[car_shortest_path[index + 1]]
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

                car_node_move_list.append([int(car_position[0]),int(car_position[1])])
            
        return car_node_move_list
    
    def cars_collision(car1_node_move_list, car2_node_move_list, car3_node_move_list, car4_node_move_list):
        
        car_node_move_list = [car1_node_move_list,car2_node_move_list,car3_node_move_list,car4_node_move_list]
        index_list = [0,1,2,3]

        collision = 0
        for pair in itertools.combinations(index_list, 2):
            for index, move_pos in enumerate(car_node_move_list[pair[0]]):
                if index < len(car_node_move_list[pair[1]]): 
                    carTocar_distance = np.sqrt(((car_node_move_list[pair[0]][index][0] - move_pos[0])**2) + ((car_node_move_list[pair[1]][index][1] - move_pos[1])**2))

                    if carTocar_distance <= 29:
                        collision += 1
                
                        
        return collision

    def collision_CarToCar(car1_vertex_list, car1_shortest_path, car2_vertex_list, car2_shortest_path, car3_vertex_list, car3_shortest_path, 
                           car4_vertex_list, car4_shortest_path):
        """
        
        車同士の衝突判定
        """
        collision = 0
        speed = setting.speed
        #speed = random.uniform(2.7, 3.3)#-10%~+10%
        #speed = random.uniform(2.4, 3.6)#-20%~+20%
        #speed = random.uniform(2.1, 3.9)#-30%~+30%
        #print("speed::"+str(speed))
        #同じ速度で目標地点に動いた場合の予測地点をlistにまとめる

        car1_node_move_list = []
        for index in range(0, len(car1_shortest_path)-1):
            start = car1_vertex_list[car1_shortest_path[index]]
            node = car1_vertex_list[car1_shortest_path[index + 1]]
            car_position = start
            while car_position != node:
                if node[0] == start[0]:
                    if  start[0] > node[0] and start[1] > node[1]:
                        car_position[1] -= (speed)
                    elif start[1] > node[1]:
                        car_position[1] -= (speed) 
                    elif start[0] > node[0]:
                        car_position[1] += (speed)  
                    else:
                        car_position[1] += (speed)
                else:
                    #移動角度の計算
                    rad = np.arctan(abs(node[1] - start[1])/abs(node[0] - start[0]))
                    if  start[0] > node[0] and start[1] > node[1]:    
                        car_position[0] -= np.cos(rad) * speed
                        car_position[1] -= np.sin(rad) * speed
                    elif start[1] > node[1]:
                        car_position[0] += np.cos(rad) * speed
                        car_position[1] -= np.sin(rad) * speed
                    elif start[0] > node[0]:
                        car_position[0] -= np.cos(rad) * speed
                        car_position[1] += np.sin(rad) * speed
                    else:
                        car_position[0] += np.cos(rad) * speed
                        car_position[1] += np.sin(rad) * speed
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
                        car_position[1] -= (speed)
                    elif start[1] > node[1]:
                        car_position[1] -= (speed) 
                    elif start[0] > node[0]:
                        car_position[1] += (speed)  
                    else:
                        car_position[1] += (speed)
                else:
                    #移動角度の計算
                    rad = np.arctan(abs(node[1] - start[1])/abs(node[0] - start[0])) 
                    if  start[0] > node[0] and start[1] > node[1]:    
                        car_position[0] -= np.cos(rad) * speed
                        car_position[1] -= np.sin(rad) * speed
                    elif start[1] > node[1]:
                        car_position[0] += np.cos(rad) * speed
                        car_position[1] -= np.sin(rad) * speed
                    elif start[0] > node[0]:
                        car_position[0] -= np.cos(rad) * speed
                        car_position[1] += np.sin(rad) * speed
                    else:
                        car_position[0] += np.cos(rad) * speed
                        car_position[1] += np.sin(rad) * speed
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
                        car_position[1] -= (speed)
                    elif start[1] > node[1]:
                        car_position[1] -= (speed) 
                    elif start[0] > node[0]:
                        car_position[1] += (speed)  
                    else:
                        car_position[1] += (speed)
                else:
                    #移動角度の計算
                    rad = np.arctan(abs(node[1] - start[1])/abs(node[0] - start[0]))
                    if  start[0] > node[0] and start[1] > node[1]:    
                        car_position[0] -= np.cos(rad) * speed
                        car_position[1] -= np.sin(rad) * speed
                    elif start[1] > node[1]:
                        car_position[0] += np.cos(rad) * speed
                        car_position[1] -= np.sin(rad) * speed
                    elif start[0] > node[0]:
                        car_position[0] -= np.cos(rad) * speed
                        car_position[1] += np.sin(rad) * speed
                    else:
                        car_position[0] += np.cos(rad) * speed
                        car_position[1] += np.sin(rad) * speed
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
                        car_position[1] -= (speed)
                    elif start[1] > node[1]:
                        car_position[1] -= (speed) 
                    elif start[0] > node[0]:
                        car_position[1] += (speed)  
                    else:
                        car_position[1] += (speed)
                else:
                    #移動角度の計算
                    rad = np.arctan(abs(node[1] - start[1])/abs(node[0] - start[0]))
                    if  start[0] > node[0] and start[1] > node[1]:    
                        car_position[0] -= np.cos(rad) * speed
                        car_position[1] -= np.sin(rad) * speed
                    elif start[1] > node[1]:
                        car_position[0] += np.cos(rad) * speed
                        car_position[1] -= np.sin(rad) * speed
                    elif start[0] > node[0]:
                        car_position[0] -= np.cos(rad) * speed
                        car_position[1] += np.sin(rad) * speed
                    else:
                        car_position[0] += np.cos(rad) * speed
                        car_position[1] += np.sin(rad) * speed
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
        self.goal_x = goal[0]
        self.goal_y = goal[1]
        self.car_width = setting.car_width #根拠のある数値にする

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
    best, best_gene = ga.main()
    print(best_gene.genom)
    two_steps_list, zeros_list = VW.two_steps_ga_setting(best_gene.genom)
    gene_size=len(two_steps_list*9)
    two_steps_best, two_steps_best_gene = ga.set_paramater_ga(setting.population_size, setting.generation_size, gene_size, two_steps_list, zeros_list)
    
    ga_resalt = zeros_list
    
    best_genom = np.array(two_steps_best_gene.genom.reshape(1, len(two_steps_list), 9))

    for i in range(len(two_steps_list)):
        ga_resalt[0][two_steps_list[i]] = list(best_genom[0][i])

    print(ga_resalt)

# def main():
#     best, best_gene = ga.main()
#     print(best_gene.genom)
#     print(best_gene.get_fitness())

if __name__ == '__main__':
    start = time.time()
    main()
    end = time.time()

    time_diff = end - start
    print(time_diff)
    