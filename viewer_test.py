import numpy as np
import pygame
import sys
from pygame.locals import *

import pygame
import sys

flg = 1
frame_delay = 200
pygame.init()
while flg<10000:
    # Pygameの初期化
    # pygame.init()
    # ウィンドウのサイズを指定
    window_size = (400, 400)
    screen = pygame.display.set_mode(window_size)
    screen.fill((255, 255, 255))

    # 半透明の赤い四角を描画するSurfaceを作成
    surface = pygame.Surface((80, 500), pygame.SRCALPHA)
    pygame.draw.rect(surface, (255, 0, 0, 100), pygame.Rect(50, 50, 100, 128))  # 赤い四角 (128はアルファ値)

    # ウィンドウにSurfaceを描画
    screen.blit(surface, (200, 200))  # Surfaceをウィンドウ上の指定位置に描画

    flg += 1
    pygame.display.update()
    pygame.time.delay(frame_delay)
# # ゲームループ
# running = True
# while running:
#     for event in pygame.event.get():
#         if event.type == pygame.QUIT:
#             running = False

#     pygame.display.flip()


