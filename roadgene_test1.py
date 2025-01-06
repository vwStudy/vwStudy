import numpy as np
import random
import artificial_potential_method as apm
import copy
import setting
import csv

class Car:
    def __init__(self, start_position, end_position, check_position, step_size=1, radius=1.0):
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
        potential_step = norm_direction * self.step_size/1.5
        
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
        x=1.5
        y=28.5
        #6*6(30,30)
        # x=3
        # y=27
        #5*5(30,30)
        # x=3
        # y=27
        #5*5(20,20)
        #x = 2
        #y = 18
        
        obs_radius = 1.5
        vw = setting.VWnum
        obs_list = []
        obstacle_array = np.array(genom.reshape(vw, vw))
        total_num_obstacles = 0
        print("obs ",obstacle_array)
        # obstacle_array[0][0] = 1 
        # obstacle_array[1][3] = 1
        # obstacle_array[2][4] = 1
        # obstacle_array[3][3] = 1
        # obstacle_array[3][7] = 1
        # obstacle_array[4][9] = 1
        # obstacle_array[5][5] = 1
        # obstacle_array[7][2] = 1
        # obstacle_array[8][8] = 1
        # obstacle_array[9][6] = 1

        for i in range(vw):
            for j in range(vw):
                if obstacle_array[i][j] == 1:
                    total_num_obstacles += 1
                    obs_list.append(Obstacle(np.array([x,y]), obs_radius, total_num_obstacles))
                x += 2*obs_radius
            y -= 2*obs_radius
            x = obs_radius
        
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

        if len(obs_list)==25:
            #return sum(distances) + car_collision_count * 10000 + obstacle_collision_count * 10000 + (1/len(obs_list))*1000, collision_counts, distances
            return sum(distances) + collision_counts * 10000000, collision_counts, distances
        else:
            return sum(distances)*10000000 + collision_counts * 10000000, collision_counts, distances
class Simulation:
    def __init__(self, obs_list, step_size=1.0, car_radius=1, x_max=30, y_max=30):
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
            number = i
            start_pos = np.array([0.0,14.0])#入口
            goal_pos = np.array([0.0,16.0])#出口
            check_pos = obs_list[i].position
            print("check", check_pos)
            # if i == 0:
            #     check_pos = np.array([13.5, 22.5])
                
            # elif i == 1:
            #     check_pos = np.array([10.5, 19.5])
            
            # elif i == 2:
            #     check_pos = np.array([22.5, 19.5])
            
            # elif i == 3:
            #     check_pos = np.array([16.5, 13.5])
            
            # elif i == 4:
            #     check_pos = np.array([19.5, 1.5])
            
            # elif i == 5:
            #     check_pos = np.array([1.5, 28.5])

            # elif i == 6:
            #     check_pos = np.array([10.5, 25.5])

            # elif i == 7:
            #     check_pos = np.array([28.5, 16.5])

            # elif i == 8:
            #     check_pos = np.array([7.5, 7.5])

            # elif i == 9:
            #     check_pos = np.array([25.5, 4.5])

            self.cars_list.append(Car(start_pos, goal_pos, check_pos, self.step_size, self.car_radius))
        
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
            
            if car.position[0] == car.start_position[0] and car.position[1] == car.start_position[1]:
  
                if interval%5 == 0:
                    car.start_update_position(obs_list)
                    interval += 1             
                else:
                    continue

            else: car.update_position(self.cars_list, obs_list)
            
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
            if cnt > 100: return
        return np.array(self.trajectory)

    def save_data(self, obs_list):
        np.save('trajectory_25.npy', np.array(self.trajectory))
        np.save('obstacles_25.npy', np.array([obstacle.position for obstacle in obs_list]))
        np.save('end_positions_25.npy', np.array([car.end_position for car in self.cars_list]))

    def get_distances(self):
        return [car.distance_travelled for car in self.cars_list]

    def get_collision_counts(self):
        return [(car.collision_count, car.obstacle_collision_count) for car in self.cars_list]
