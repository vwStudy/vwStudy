import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import heapq

# グリッドサイズの設定 (幅, 高さ)
GRID_SIZE = (30, 30)

# 障害物の位置の設定（自由に変更可能）
obstacles = [(i, j) for i in range(0,30) for j in range(20,30)]

# エージェントのスタート位置とゴール位置の設定（自由に変更可能）
agents_info = [
    {'start': (0, 15), 'goal': (29, 15)},
    {'start': (29, 14), 'goal': (0, 14)},
    {'start': (14, 29), 'goal': (0, 15)},
    {'start': (15, 29), 'goal': (29, 15)}
]

# グリッドの初期化 (高さ, 幅)
grid = np.zeros((GRID_SIZE[1], GRID_SIZE[0]))

# 障害物をグリッドに配置
for obs in obstacles:
    x, y = obs
    grid[y][x] = 1  # grid[y][x]

# エージェントのリストを作成
agents = []
for info in agents_info:
    agent = {'start': info['start'], 'goal': info['goal'], 'path': []}
    agents.append(agent)

# エージェントに優先度を割り当て（必要に応じて変更可能）
agents = sorted(agents, key=lambda x: x['start'])

def heuristic(a, b):
    """マンハッタン距離の計算"""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star(start, goal, grid, occupied):
    """A*アルゴリズムの実装"""
    neighbors = [(0,1),(1,0),(0,-1),(-1,0)]
    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))
    
    while oheap:
        current = heapq.heappop(oheap)[1]
        if current == goal:
            data = [current]
            while current in came_from:
                current = came_from[current]
                data.append(current)
            return data[::-1]
        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0]+i, current[1]+j
            tentative_g_score = gscore[current] + 1
            if 0 <= neighbor[0] < GRID_SIZE[0] and 0 <= neighbor[1] < GRID_SIZE[1]:
                if grid[neighbor[1]][neighbor[0]] == 1:
                    continue
            else:
                continue
            if neighbor in occupied:
                continue
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
            if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
    return None

# エージェントの経路計画
occupied = set()
for idx, agent in enumerate(agents):
    path = a_star(agent['start'], agent['goal'], grid, occupied)
    if not path:
        print(f"エージェント{idx}の経路が見つかりませんでした。")
        continue
    agent['path'] = path
    occupied.update(path)  # 高優先度のエージェントの経路を占有領域として追加

# アニメーションの作成
fig, ax = plt.subplots()
ax.set_xlim(0, GRID_SIZE[0])
ax.set_ylim(0, GRID_SIZE[1])
ax.set_xticks(np.arange(0, GRID_SIZE[0], 1))
ax.set_yticks(np.arange(0, GRID_SIZE[1], 1))
ax.grid(True)

# 障害物の描画
for obs in obstacles:
    x, y = obs
    rect = plt.Rectangle((x, y), 1, 1, color='black')
    ax.add_patch(rect)

# エージェントの初期位置の設定
agent_patches = []
colors = ['red', 'blue', 'green', 'orange', 'purple', 'cyan']  # エージェントの色を定義
for idx, agent in enumerate(agents):
    color = colors[idx % len(colors)]  # エージェントの色を割り当て
    patch = plt.Circle((agent['start'][0]+0.5, agent['start'][1]+0.5), 0.3, color=color)
    agent_patches.append(patch)
    ax.add_patch(patch)
    # ゴール位置を星印で表示
    plt.plot(agent['goal'][0]+0.5, agent['goal'][1]+0.5, marker='*', color=color, markersize=15)

def animate(i):
    for idx, agent in enumerate(agents):
        if i < len(agent['path']):
            pos = agent['path'][i]
            agent_patches[idx].center = (pos[0]+0.5, pos[1]+0.5)
        else:
            # ゴールに到達したらその場に留まる
            pos = agent['goal']
            agent_patches[idx].center = (pos[0]+0.5, pos[1]+0.5)
    return agent_patches

# 最大フレーム数を計算（最も長い経路の長さ）
max_frames = max(len(agent['path']) for agent in agents)

ani = animation.FuncAnimation(fig, animate, frames=max_frames, interval=500, blit=True, repeat=False)
plt.gca().invert_yaxis()
plt.show()
