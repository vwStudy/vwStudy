import numpy as np
import random
import artificial_potential_method as apm
import copy
import setting
import csv

class Car:
    def __init__(self, start_position, end_position, step_size=1.0, radius=1.0):
        self.position = np.array(start_position).copy()
        self.start_position = np.array(start_position)
        self.end_position = np.array(end_position)
        self.step_size = step_size
        self.radius = radius
        self.distance_travelled = 0.0
        self.reached_end = False
        self.collision_count = 0
        self.obstacle_collision_count = 0
        self.interval = 0


    def start_update_position(self, obstacles):
        direction = self.end_position - self.position
        norm_direction = direction / (np.linalg.norm(direction) + 1e-10)
        potential_step = norm_direction * self.step_size

        #壁の人工ポテンシャル法
        for obstacle in obstacles:
            obs_distance = np.linalg.norm(self.position - obstacle.position) + 1e-10
            if obs_distance <= obstacle.radius+self.radius:
                self.obstacle_collision_count += 1
            #if obs_distance < 1.4*obstacle.radius:
            if obs_distance < 2*obstacle.radius:
                next_x, next_y  = apm.cal_route(self.position, self.end_position, obstacle)
                potential_step[0] += next_x
                potential_step[1] += next_y

        # 新しい位置を更新
        new_position = self.position + potential_step
        move_distance = np.linalg.norm(potential_step)
        self.distance_travelled += move_distance
        if np.linalg.norm(new_position - self.end_position) < self.step_size:
            self.position = self.end_position.copy()
            self.reached_end = True
        else:
            self.position = new_position

    def update_position(self, other_cars, obstacles):
        direction = self.end_position - self.position
        norm_direction = direction / (np.linalg.norm(direction) + 1e-10)
        potential_step = norm_direction * self.step_size

        #他の車の人工ポテンシャル法
        for car in other_cars:
            if car != self and car.reached_end == False:
                car_distance = np.linalg.norm(self.position - car.position) + 1e-10
                if car_distance <= 2*self.radius:
                    self.collision_count += 1
                #if car_distance < 1.5*self.radius:
                if car_distance < 2.7*self.radius:    
                    next_x, next_y = apm.car_cal_route(self.position, self.end_position, car)
                    potential_step[0] += next_x
                    potential_step[1] += next_y

        #壁の人工ポテンシャル法
        for obstacle in obstacles:
            obs_distance = np.linalg.norm(self.position - obstacle.position) + 1e-10
            # if obs_distance <= obstacle.radius + self.radius:
            if obs_distance <= 2.2*obstacle.radius:
                self.obstacle_collision_count += 1
            #if obs_distance < 1.5*obstacle.radius:
            if obs_distance < 2.5*obstacle.radius:
                next_x, next_y  = apm.cal_route(self.position, self.end_position, obstacle)
                potential_step[0] += next_x
                potential_step[1] += next_y


        # 新しい位置を更新
        new_position = self.position + potential_step
        move_distance = np.linalg.norm(potential_step)
        self.distance_travelled += move_distance 
        
        # new_position[0]= round(new_position[0], 2)
        # new_position[1] = round(new_position[1],2)
        if np.linalg.norm(new_position - self.end_position) < self.step_size:
            self.position = self.end_position.copy()
            self.reached_end = True
        else:
            self.position = new_position
        
    # def reset_position(self, start_position, end_position):
    #     self.position = np.array(start_position)
    #     self.start_position = np.array(start_position)
    #     self.end_position = np.array(end_position)
    #     #self.distance_travelled = 0.0
    #     self.reached_end = False

class Obstacle:
    def __init__(self, position, radius):
        self.position = np.array(position)
        self.radius = radius
    
    def single_GA_function(genom):
        
        """
        GeneticalAlgorism用の関数
        """
        
        #ToDo 以下の処理は変える必要がある
        #遺伝的アルゴリズムの結果に対しVWを設置
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
        obs_list = []
        obstacle_array = np.array(genom.reshape(10,10))
        total_num_obstacles = 0
        for i in range(10):
            for j in range(10):
                if obstacle_array[i][j] >= 1:
                    # if obstacle_array[0][2] >= 1 or obstacle_array[2][0] >= 1 or obstacle_array[2][4] >= 1 or obstacle_array[4][2] >= 1:
                    #     obstacle_array[0][2] = 0
                    #     obstacle_array[2][0] = 0
                    #     obstacle_array[2][4] = 0
                    #     obstacle_array[4][2] = 0
                    # else: 
                    #     obs_list.append(Obstacle(np.array([x,y]), obs_radius))
                    #     total_num_obstacles += 1
                    obs_list.append(Obstacle(np.array([x,y]), obs_radius))
                    total_num_obstacles += 1
                x += 3
            y -= 3
            x = 3
        
        simulation = Simulation()
        simulation.simulate_movement(obs_list)
        print("doing")
        #遺伝的アルゴリズムの最適解ではなく、一番最後の配列を持ってきている可能性あり
        simulation.save_data(obs_list)

        distances = simulation.get_distances()
        collision_counts = simulation.get_collision_counts()
        for i, (car_collision_count, obstacle_collision_count) in enumerate(collision_counts):
            collision_counts = car_collision_count + obstacle_collision_count
        print("car_collision",car_collision_count)
        print("obs_collision",obstacle_collision_count)
        print("distances",sum(distances))
        #print("obs_len",len(obs_list))
        # for obs in obs_list:
        #     print("obs",obs.position)
        #return sum(distances) + collision_counts * 1000000+ (1/len_obs)*10, collision_counts, distances
        #return sum(distances) + car_collision_count * 10000 + obstacle_collision_count * 10000 + (1/len_obs)*100, collision_counts, distances
        vwnum=setting.VWnum
        if len(obs_list)>0:
            #return sum(distances) + car_collision_count * 10000 + obstacle_collision_count * 10000 + (1/len(obs_list))*1000, collision_counts, distances
            return sum(distances)*vwnum/len(obs_list) + car_collision_count * 10000000 + obstacle_collision_count * 10000000, collision_counts, distances
        else:
            return sum(distances) + car_collision_count * 10000000 + obstacle_collision_count * 10000000 , collision_counts, distances
class Simulation:
    def __init__(self, num_cars=15, num_obstacles=25, step_size=1.0, car_radius=1, x_max=30, y_max=30):
        self.num_cars = num_cars
        self.step_size = step_size
        self.car_radius = car_radius
        self.x_max = x_max
        self.y_max = y_max
        self.cars_list = []
        self.completion_count = 0
        self.trajectory = []
        self.interval_list = []

        cnt=1
        for _ in range(num_cars):
            #スタートゴール用+-ランダム
            rand_posi = random.randint(0, 3)
            rand_posi2 = random.randint(0, 3)
            
            rand1 =random.random()
            rand2 =random.random()
            if rand1>0.5:
            # if cnt<=7: 
                #左側スタート
                #start_pos = np.array([0.0,15.0+rand_posi])
                #start_pos = np.array([0.0,15.0])
                start_pos = np.array([0.0,5.0])
                #右側ゴール
                goal_pos = np.array([30.0,14.0])
                #goal_pos = np.array([30.0,15.0+rand_posi2])
                #下側ゴール
                #goal_pos = np.array([14.0+rand_posi2,0.0])
                # cnt+=1
                self.cars_list.append(Car(start_pos, goal_pos, self.step_size, self.car_radius))
            else:
                #上側スタート
                #start_pos = np.array([14.0+rand_posi,30.0])
                #start_pos = np.array([14.0,30.0])
                start_pos = np.array([4.0,30.0])
                #下側ゴール
                #goal_pos = np.array([14.0+rand_posi2,0.0])
                goal_pos = np.array([14.0,0.0])
                #右側ゴール
                #goal_pos = np.array([30.0,15.0+rand_posi2])
                #5叉路
                # if rand2>0.5:
                #     # goal_pos = np.array([6.0+rand_posi2,0.0])
                #     goal_pos = np.array([6.0,0.0])
                # else:
                #     # goal_pos = np.array([21.0+rand_posi2,0.0])
                #     goal_pos = np.array([21.0,0.0])
                self.cars_list.append(Car(start_pos, goal_pos, self.step_size, self.car_radius))
        
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
                
            ##以下で再生成
                
                # rand_posi = np.random.randint(0,4)
                # rand_posi_nega = np.random.randint(-3,4)
                # rand_nega = np.random.randint(-3,1)
            
                # if random.random()>0.5:
                #     start_pos = np.array([0+rand_posi,15+rand_posi_nega])
                #     goal_pos = np.array([20,15+rand_posi_nega])
                #     #左側スタート
                #     self.cars_list.append(Car(start_pos, goal_pos, self.step_size, self.car_radius))
                # else:
                #     start_pos = np.array([13+rand_posi_nega,20+rand_nega])
                #     goal_pos = np.array([13+rand_posi_nega,0])
                #     #上側スタート
                #     self.cars_list.append(Car(start_pos, goal_pos, self.step_size, self.car_radius))

                #print(self.completion_count)
                #print("comp",self.completion_count)
                # for _ in range(10):
                #     per_posi = random.randint(0,6)
                #     per_posi_nega = random.randint(-5,5)
                #     per_nega = random.randint(-5,0)
                #     car.reset_position(np.array([0+per_posi,15+per_posi_nega]), np.array([20,15]))
                #     car.reset_position(np.array([13+per_posi_nega,20+per_nega]), np.array([13, 0]))



                # for i in range(5):
                #     car.reset_position(np.array([0+i,15]), np.array([20,15]))
                #     if i > 2:
                #         car.reset_position(np.array([0+i,15-i]), np.array([10,20]))

                # for j in range(5):    
                #     car.reset_position(np.array([11+j,20]), np.array([13,0]))
                #     if j > 2:
                #         car.reset_position(np.array([0+i,15-i]), np.array([10,20]))

                # if self.completion_count%2 == 0:
                #     car.reset_position(np.array([0,15]), np.array([20,15]))
                # else:
                #     for _ in range(time):
                #         print("")
                #     car.reset_position(np.array([13,20]), np.array([13,0]))


                    #car.reset_position(np.array([13,20]), np.array([13,0]))
                # car.reset_position(np.random.randint(0, 20, size=2), np.random.randint(0, 19, size=2))
        
        self.trajectory.append([car.position.copy() for car in self.cars_list])
        
    def simulate_movement(self,obs_list):
        interval = 5
        cnt=0
        while self.completion_count != self.num_cars:
            self.update_positions(obs_list, interval)
            interval+=1
            cnt+=1
            if cnt >1000: return
        return np.array(self.trajectory)

    def save_data(self,obs_list):
        np.save('trajectory_test.npy', np.array(self.trajectory))
        ##np.save('obstacles.npy', np.array([obstacle.position for obstacle in self.obstacles]))
        np.save('obstacles_test.npy', np.array([obstacle.position for obstacle in obs_list]))
        #print("test2",np.array([obstacle.position for obstacle in obs_list]))
        np.save('end_positions_test.npy', np.array([car.end_position for car in self.cars_list]))


    def get_distances(self):
        return [car.distance_travelled for car in self.cars_list]

    def get_collision_counts(self):
        return [(car.collision_count, car.obstacle_collision_count) for car in self.cars_list]


