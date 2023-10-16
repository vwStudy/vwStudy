import numpy
import setting
import pygame
from pygame.locals import *

class CarAgent():
    def __init__(self, start_position, goal_position):
        self.start_position = start_position
        self.goal_position = goal_position

#     def get_position(self):
# # class vw():
class environment():
    def __init__(self, x_position, y_position, width, height):
        self.x_position = x_position
        self.y_position = y_position



def main():
    pygame.init()
    cnt=0
    screen = pygame.display.set_mode((2000, 1000))
    px=120
    py=100
    
    while True:
        screen.fill((255,255,255))
        
        pygame.draw.rect(screen, (0,0,0),(10,100,50,50))#第一引数screenオブジェクト,第二引数図形のRGB,第三引数図形の形(左上のx座標,y座標,横幅,縦幅)
        px+=1
        pygame.display.update()
        cnt+=1
        if cnt==1100:
            break

if __name__ == "__main__":
    main()