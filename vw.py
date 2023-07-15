import numpy as np
import math

from geneticalgorithm2 import geneticalgorithm2 as ga
import setting

class VW():
    def __init__(self):
        a=0

    def ga_function(p):
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
        collision,time,(obs,obs2,obs3,obs4),((distance,root),(distance_2,root_2),(distance_3,root_3),(distance_4,root_4)) = Window.animate(array,array2,array3,array4) 
        result = [distance,distance_2,distance_3,distance_4]
        out = 0
        for res in result:
            if res >= 3000:
                out += 1000000

        total_num_obstacles = len(obs) + len(obs2)+len(obs3)+len(obs4)
        print(car1_vw,"car1")
        print(car2_vw,"car2")
        print(car3_vw,"car3")
        print(car4_vw,"car4")
        print(root,"root")
        print(root_2,"root2")
        print(root_3,"root3")
        print(root_4,"root4")
        print(total_num_obstacles,"障害物数")
        print((total_num_obstacles/(setting.VWnum*(setting.VWnum**2-1))),"VW%")
        print(time)
        print(distance,"distance")
        print(distance_2,"distance2")
        print(distance_3,"distance3")
        print(distance_4,"distance4")
        print(collision,"衝突")
        all_distance = distance + distance_2 + distance_3 + distance_4
        print(all_distance*((total_num_obstacles/(setting.VWnum*((setting.VWnum**2)-1)))) + collision*1000000 + out,"ans")
        return all_distance*((total_num_obstacles/(setting.VWnum*((setting.VWnum**2)-1)))) + collision*1000000 + out

class Environment():
    def __init__(self, obstacle_x, obstacle_y, width, height):
        self.x = obstacle_x
        self.y = obstacle_y
        self.width = width
        self.height = height
        

class CarAgent():
    def __init__(self, start_x, start_y, goal_x, goal_y):
        self.x = self.dx = start_x
        self.y = self.dy = start_y
        self.speed = setting.speed #根拠のある数値にする
        self.car_width = setting.car_width #根拠のある数値にする
        self.car_length = setting.car_length #根拠のある数値にする
        self.car_root = []

    def move(self,dx,dy):
        rad = np.arctan(abs(dy - self.y)/abs(dx - self.x))
        dig = math.degrees(rad)
        if dx == self.x and dy == self.y:
            self.x += 0
            self.y += 0
        else:
            if  dx > self.x and dy > self.y:
                self.x += (math.cos(math.radians(dig))*self.speed)
                self.y += (math.sin(math.radians(dig))*self.speed)
            elif dx < self.x and dy > self.y:
                self.x -= (math.cos(math.radians(dig))*self.speed)
                self.y += (math.sin(math.radians(dig))*self.speed) 
            elif dx > self.x and dy < self.y:
                self.x += (math.cos(math.radians(dig))*self.speed)
                self.y -= (math.sin(math.radians(dig))*self.speed)  
            elif dx < self.dx and dy < self.dy:
                self.x -= (math.cos(math.radians(dig))*self.speed)
                self.y -= (math.sin(math.radians(dig))*self.speed)

class Execution():
    def set_obstacle(self):
        self.Obstacle_1 = Environment()
        self.Obstacle_2 = Environment()
        self.Obstacle_3 = Environment()
        self.Obstacle_4 = Environment()

    def set_caragent(self):
        self.CarAgent_1 = CarAgent(0, 50, 100, 50)
        self.CarAgent_2 = CarAgent(50, 100, 50, 0)
        self.CarAgent_3 = CarAgent(100, 50, 0, 50)
        self.CarAgent_4 = CarAgent(50, 0, 50, 100)

def main():
    solution_list = []
    var_list = [[0,2]] * (setting.VWnum**2 * 4) #この中にGAで出た値を入れていく,[0, 2]は0~1の値が入るという意味
    varbound = np.array(var_list)
    ga_model = ga(function=VW.ga_function,
            dimension=((setting.VWnum**2) * 4),
            variable_type='real',
            variable_boundaries=varbound,
            algorithm_parameters=setting.params
    )
    ga_model.run()
    convergence = ga_model.report
    solution = ga_model.result
    print(str(setting.VWnum) + "vw")
    for key, value in setting.params.items():
        print(str(key) + "：" + str(value))
    for i in solution['variable']:
        solution_list.append(i)
    print(solution_list)
    #print((solution['variable']),"2222") # x, y の最適値
    print(solution['score'],"最小値") # x, y の最適値での関数の値

    def visibility_graph():
        j=0

    def dijkstra():
        i=0

if __name__ == '__main__':
    main()