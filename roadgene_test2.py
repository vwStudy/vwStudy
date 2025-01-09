import numpy as np
import random
import artificial_potential_method as apm
import copy
import setting
import csv

class Car:
    def __init__(self, start_position, end_position, check_position, radius, step_size=1):
        self.position = np.array(start_position).copy()
        self.start_position = np.array(start_position)
        self.end_position = np.array(end_position)
        self.check_position = np.array(check_position)
        self.step_size = step_size
        self.radius = radius
        self.distance_travelled = 0.0
        self.reached_end = False
        self.collision_count = 0
        self.obstacle_collision_count = 0
        self.interval = 0
        self.decision = False


    def start_update_position(self, obstacles):
        direction = self.check_position - self.position
        norm_direction = direction / (np.linalg.norm(direction) + 1e-10)
        potential_step = norm_direction * self.step_size/4
        
        # #壁の人工ポテンシャル法
        for obstacle in obstacles:
            obs_distance = np.linalg.norm(self.position - obstacle.position) + 1e-10
            if obs_distance <= 1.5*(self.radius+obstacle.radius):
                #目的の棚か確認
                self.decision = Car.check_shelf(obstacle.position, self.check_position)
                if self.decision == True:
                    continue
                else:
                    self.obstacle_collision_count += 1
                    next_x, next_y  = apm.cal_route(self.position, self.end_position, obstacle)
                    potential_step[0] += next_x
                    potential_step[1] += next_y       

        # 新しい位置を更新
        new_position = self.position + potential_step
        move_distance = np.linalg.norm(potential_step)
        self.distance_travelled += move_distance
        if np.linalg.norm(new_position - self.check_position) < self.step_size:
            self.position = self.check_position.copy()
            self.reached_end = True
        else:
            self.position = new_position

    def update_position(self, other_cars, obstacles):
        #スタートから設置物まで
        if self.decision == False:
            direction = self.check_position - self.position
            norm_direction = direction / (np.linalg.norm(direction) + 1e-10)
            potential_step = norm_direction * self.step_size/1.5
            #他の車の人工ポテンシャル法
            for car in other_cars:
                if car != self and car.reached_end == False:
                    car_distance = np.linalg.norm(self.position - car.position) + 1e-10
                    if car_distance <= 2*self.radius:
                        next_x, next_y = apm.car_cal_route(self.position, self.check_position, car)
                        potential_step[0] += next_x
                        potential_step[1] += next_y
                        self.collision_count += 1
            #壁の人工ポテンシャル法
            for obstacle in obstacles:
                obs_distance = np.linalg.norm(self.position - obstacle.position) + 1e-10
                if obs_distance <= 1.1*(self.radius+obstacle.radius):#1.8でラウンドアバウトはいけたはず
                    #目的の棚か確認
                    self.decision = Car.check_shelf(obstacle.position, self.check_position)
                    if self.decision == True:
                        continue
                    else:
                        self.obstacle_collision_count += 1
                        next_x, next_y  = apm.cal_route(self.position, self.check_position, obstacle)
                        potential_step[0] += next_x
                        potential_step[1] += next_y 
            # 新しい位置を更新
            new_position = self.position + potential_step
            move_distance = np.linalg.norm(potential_step)
            self.distance_travelled += move_distance
            self.position = new_position

        #設置物からゴールまで
        else:        
            direction = self.end_position - self.position
            norm_direction = direction / (np.linalg.norm(direction) + 1e-10)
            potential_step = norm_direction * self.step_size/1.5
            #他の車の人工ポテンシャル法
            for car in other_cars:
                if car != self and car.reached_end == False:
                    car_distance = np.linalg.norm(self.position - car.position) + 1e-10
                    if car_distance <= 2*self.radius:
                        next_x, next_y = apm.car_cal_route(self.position, self.end_position, car)
                        potential_step[0] += next_x
                        potential_step[1] += next_y
                        self.collision_count += 1                    
            #壁の人工ポテンシャル法
            for obstacle in obstacles:
                obs_distance = np.linalg.norm(self.position - obstacle.position) + 1e-10
                if obs_distance <= 1.5*(self.radius+obstacle.radius):#1.8でラウンドアバウトはいけたはず                
                    self.obstacle_collision_count += 1
                    next_x, next_y  = apm.cal_route(self.position, self.end_position, obstacle)
                    potential_step[0] += next_x
                    potential_step[1] += next_y  
            # 新しい位置を更新
            new_position = self.position + potential_step
            move_distance = np.linalg.norm(potential_step)
            self.distance_travelled += move_distance 
            if np.linalg.norm(new_position - self.end_position) <= 2*self.step_size:
                self.position = self.end_position.copy()
                self.reached_end = True
            else:
                self.position = new_position
    
    def check_shelf(obs_pos, check_pos):
        if obs_pos[0] == check_pos[0] and obs_pos[1] == check_pos[1]:
            return True

        # elif obstacle.position[0] == 10.5 and obstacle.position[1] == 19.5:
        #     return True
        
        # elif obstacle.position[0] == 22.5 and obstacle.position[1] == 19.5:
        #     return True
        
        # elif obstacle.position[0] == 16.5 and obstacle.position[1] == 13.5:
        #     return True
        
        # elif obstacle.position[0] == 19.5 and obstacle.position[1] == 1.5:
        #     return True
        
        # elif obstacle.position[0] == 1.5 and obstacle.position[1] == 28.5:
        #     return True

        # elif obstacle.position[0] == 25.5 and obstacle.position[1] == 4.5:
        #     return True
        
        # elif obstacle.position[0] == 10.5 and obstacle.position[1] == 25.5:
        #     return True
        
        # elif obstacle.position[0] == 7.5 and obstacle.position[1] == 7.5:
        #     return True
        
        # elif obstacle.position[0] == 28.5 and obstacle.position[1] == 16.5:
        #     return True
        
        else: return False


class Obstacle:
    def __init__(self, position, radius, tag):
        self.position = np.array(position)
        self.radius = radius
        self.tag = tag
    
    def single_GA_function(genom):
        
        """
        GeneticalAlgorism用の関数
        """
        
        #遺伝的アルゴリズムの結果に対しVWを設置
        #15*15
        # x = 1
        # y = 29
        #10*10
        # x=1.5
        # y=28.5
        #6*6(30,30)
        # x=3
        # y=27
        #5*5(30,30)
        # x=3
        # y=27
        #5*5(20,20)
        #x = 2
        #y = 18
        
        # obs_radius = 0.75
        obs_radius = 0.5
        vw = setting.VWnum
        obs_list = []
        #obstacle_array = np.array()gmeno.reshape(1,1800)
        total_num_obstacles = 0

        for i in genom:
            row = (i - 1) // 40  # 上から何行目か (0-indexed)
            col = (i - 1) % 40   # 左から何列目か (0-indexed)
    # 中心座標を計算
            x = col + 0.5
            y = 40 - row - 0.5

            # print(i)
            # a = i%39
            # x = a-0.5
            # if i%39 == 0:
            #     b = int(i/39)
            #     y = 40.5-b      
            # else:
            #     b = int(i/39)
            #     y = 39.5-b

            # def is_excluded(x, y):
            #     return 0 <= x <= 2 and 19 <= y <= 23
            
            # for row in range(40):
            #     for col in range(40):
            #         x = col + 0.5
            #         y = 40 - row - 0.5
            #         if not is_excluded(col, 40 - row - 1):
            #             obs_list.append(Obstacle(np.array([x,y]), obs_radius, total_num_obstacles))
            #         else:
            #             return 
            ##ここで数字と座標を一致させて棚を配置する
            obs_list.append(Obstacle(np.array([x,y]), obs_radius, total_num_obstacles))
        simulation = Simulation(obs_list)
        simulation.simulate_movement(obs_list)
        #遺伝的アルゴリズムの最適解ではなく、一番最後の配列を持ってきている可能性あり
        simulation.save_data(obs_list)
        collision_counts=0
        distances = simulation.get_distances()
        collision_counts_list = simulation.get_collision_counts()
        
        for i, (car_collision_count, obstacle_collision_count) in enumerate(collision_counts_list):
            collision_counts += car_collision_count + obstacle_collision_count
        # for obs in obs_list:
        #     print("obs",obs.position)
        #return sum(distances) + collision_counts * 1000000+ (1/len_obs)*10, collision_counts, distances
        #return sum(distances) + car_collision_count * 10000 + obstacle_collision_count * 10000 + (1/len_obs)*100, collision_counts, distances

        # if len(obs_list)==728:
            #return sum(distances) + car_collision_count * 10000 + obstacle_collision_count * 10000 + (1/len(obs_list))*1000, collision_counts, distances
        return sum(distances) + collision_counts * 10000, collision_counts, distances
        #return sum(distances) + collision_counts * 10000
        # else:
        #     return sum(distances)*10000000 + collision_counts * 10000000, collision_counts, distances
class Simulation:
    def __init__(self, obs_list, step_size=1.0, car_radius=0.5, x_max=40, y_max=40):
        self.num_cars = obs_list
        self.step_size = step_size
        self.car_radius = car_radius
        self.x_max = x_max
        self.y_max = y_max
        self.cars_list = []
        self.completion_count = 0
        self.trajectory = []
        self.interval_list = []

        for i in range(len(self.num_cars)):
            start_pos = np.array([0.0,20.0])#入口
            goal_pos = np.array([0.0,22.0])#出口
            check_pos = obs_list[i].position

            self.cars_list.append(Car(start_pos, goal_pos, check_pos, self.car_radius, self.step_size))
        
    def update_positions(self, obs_list, interval):
        # #乱数を振って閾値以下だったら車を生成
        # if 0.5 < random.random():
        #     if 0.5 < random.random():
        #         self.cars.append(Car(np.array([0,15]), np.array([20,15]), self.step_size, self.car_radius))
        #     else:    
        #         self.cars.append(Car(np.array([13,20]), np.array([13,0]), self.step_size, self.car_radius))

        for i, car in enumerate(self.cars_list):
            if car.reached_end == True:
                continue
            
            elif car.position[0] == car.start_position[0] and car.position[1] == car.start_position[1]:
  
                if interval%5 == 0:
                    car.start_update_position(obs_list)          
                else:
                    continue

            else: car.update_position(self.cars_list, obs_list)
            
            interval += 1

            if car.reached_end == True:
                self.completion_count += 1
        
        self.trajectory.append([car.position.copy() for car in self.cars_list])
        
    def simulate_movement(self,obs_list):
        interval = 5
        cnt=0
        while self.completion_count != self.num_cars:
            self.update_positions(obs_list, interval)
            interval+=1
            cnt+=1
            if cnt > setting.genom_size*10+300: return
        return np.array(self.trajectory)

    def save_data(self, obs_list):
        np.save('trajectory_test.npy', np.array(self.trajectory))
        np.save('obstacles_test.npy', np.array([obstacle.position for obstacle in obs_list]))
        np.save('end_positions_test.npy', np.array([car.end_position for car in self.cars_list]))

    def get_distances(self):
        return [car.distance_travelled for car in self.cars_list]

    def get_collision_counts(self):
        return [(car.collision_count, car.obstacle_collision_count) for car in self.cars_list]
