import pygame

# 初期化
pygame.init()

# ウィンドウのサイズ
width, height = 800, 600

# ウィンドウの作成
screen = pygame.display.set_mode((width, height))

# サーフェスを作成し、RGBAフォーマットに設定
surface = pygame.Surface((80, 80), pygame.SRCALPHA)

# 半透明の色 (RGBA形式)
color = (255, 0, 0, 128)  # この場合、赤色でアルファ値が128（半透明）

# 正方形を描画
pygame.draw.rect(surface, color, pygame.Rect(0, 0, 80, 80))

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # ウィンドウをクリア
    screen.fill((255, 255, 255))

    # サーフェスをウィンドウに描画
    screen.blit(surface, (360, 420))

    pygame.display.flip()

pygame.quit()
