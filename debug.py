
import setting
import numpy as np

class CarAgent():
    def __init__(self, start, goal):
        self.start = start
        self.goal = goal
        self.x = start[0]
        self.y = start[1]
        self.goal_x = goal[0]
        self.goal_y = goal[1]
        self.flag = False
        self.car_width = setting.car_width #根拠のある数値にする
    
    def car_move(self, car_shortest_path, car_position, num):

        node = setting.car1_STARTtoGOAL[num] 
        #node = car_vertex_list[car_shortest_path[num]]

        if num == car_shortest_path-1:
                self.flag = True
        
        elif num < car_shortest_path-1:
            node = setting.car1_STARTtoGOAL[num]

            if abs(node[0] - car_position[0])==0:
                None
            else:
                rad = np.arctan(abs(node[1] - car_position[1])/abs(node[0] - car_position[0]))
                if  car_position[0] > node[0] and car_position[1] > node[1]:    
                    car_position[0] -= np.cos(rad) * setting.speed
                    car_position[1] -= np.sin(rad) * setting.speed
                elif car_position[0] < node[0] and car_position[1] > node[1]:
                    car_position[0] += np.cos(rad) * setting.speed
                    car_position[1] -= np.sin(rad) * setting.speed
                elif car_position[0] > node[0] and car_position[1] < node[1]:
                    car_position[0] -= np.cos(rad) * setting.speed
                    car_position[1] += np.sin(rad) * setting.speed
                elif car_position[0] > node[0] and car_position[1] > node[1]:
                    car_position[0] += np.cos(rad) * setting.speed
                    car_position[1] += np.sin(rad) * setting.speed
            


        # elif car_position[0] == node[0] and car_position[1] == node[1]:
        #     if len(car_shortest_path-1):
        #        self.flag = True
            
            
        return car_position ,self.flag



def main():

    num = 2
    car1 = setting.car1_STARTtoGOAL[0]
    cars_tuple = (CarAgent(setting.car1_STARTtoGOAL[0],setting.car1_STARTtoGOAL[1]), 20)
    for i in range(num):
        car1, flag= cars_tuple[0].car_move(num,car1,i)
        print(car1)
    # cars_tuple[1].car_move
    # cars_tuple[2].car_move
    # cars_tuple[3].car_move

if __name__ == '__main__':
    main()

