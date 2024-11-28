import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle, Rectangle
from heapq import heappush, heappop
from copy import deepcopy
import itertools


# CBS Node Class
class CBSNode:
    def __init__(self, constraints, paths, cost):
        self.constraints = constraints  # 衝突制約リスト
        self.paths = paths  # 各エージェントの現在の経路
        self.cost = cost  # 総コスト
    
    def __lt__(self, other):
        return self.cost < other.cost  # コストで優先度を比較


# A* Algorithm for individual pathfinding
def a_star(graph, start, goal, constraints, agent_id):
    open_list = [(0, start, [start])]  # (コスト, 現在の位置, 経路)
    closed_set = set()
    
    while open_list:
        cost, current, path = heappop(open_list)
        
        if current == goal:
            print(current)
            return path  # 目的地に到達
        
        if (current, len(path) - 1) in constraints.get(agent_id, []):
            continue  # 制約に違反する経路は除外

        for neighbor in graph[current]:
            if neighbor not in closed_set:
                new_path = path + [neighbor]
                heappush(open_list, (cost + 1, neighbor, new_path))
                closed_set.add(neighbor)
    
    return None  # 目的地に到達できなかった場合


# Calculate total cost of paths
def get_total_cost(paths):
    return sum(len(path) for path in paths)


# Detect collision between two paths
def detect_collision(path1, path2):
    max_time = max(len(path1), len(path2))
    for t in range(max_time):
        pos1 = path1[min(t, len(path1) - 1)]
        pos2 = path2[min(t, len(path2) - 1)]
        if pos1 == pos2:
            return t, pos1  # 衝突時間と位置を返す
        if t > 0 and pos1 == path2[min(t - 1, len(path2) - 1)] and pos2 == path1[min(t - 1, len(path1) - 1)]:
            return t, (pos1, pos2)  # 道路上での衝突
    
    return None  # 衝突なし


# CBS Algorithm
def CBS(graph, starts, goals):
    root = CBSNode({}, [], 0)  # 初期ノード
    for i in range(len(starts)):
        path = a_star(graph, starts[i], goals[i], {}, i)
        if path is None:
            return None  # 経路が見つからない
        root.paths.append(path)
    root.cost = get_total_cost(root.paths)
    
    open_list = [root]
    while open_list:
        node = heappop(open_list)
        
        # 衝突を検出
        collision = None
        for (i, path1), (j, path2) in itertools.combinations(enumerate(node.paths), 2):
            collision = detect_collision(path1, path2)
            if collision:
                break
        
        if collision is None:
            return node.paths  # 衝突がない場合、解を返す
        
        # 衝突を解決するための新しい制約を生成
        t, loc = collision
        constraints1 = deepcopy(node.constraints)
        constraints2 = deepcopy(node.constraints)
        
        if isinstance(loc, tuple):
            constraints1.setdefault(i, []).append((loc[0], t))
            constraints2.setdefault(j, []).append((loc[1], t))
        else:
            constraints1.setdefault(i, []).append((loc, t))
            constraints2.setdefault(j, []).append((loc, t))

        # 各エージェントに制約を適用して新しいノードを作成
        for constraints, agent_id in [(constraints1, i), (constraints2, j)]:
            new_paths = deepcopy(node.paths)
            path = a_star(graph, starts[agent_id], goals[agent_id], constraints, agent_id)
            if path is not None:
                new_paths[agent_id] = path
                new_cost = get_total_cost(new_paths)
                heappush(open_list, CBSNode(constraints, new_paths, new_cost))
    
    return None  # 解が見つからない場合


# AnimationVisualizer for CBS Results
class AnimationVisualizer:
    def __init__(self, graph, paths, starts, goals, obstacles=None):
        self.graph = graph
        self.paths = paths
        self.starts = starts
        self.goals = goals
        self.obstacles = obstacles if obstacles else []
        self.num_agents = len(paths)
        self.grid_size = self.get_grid_size()
        
        # プロットの設定
        self.fig, self.ax = plt.subplots()
        self.setup_plot()

    def get_grid_size(self):
        max_x = max(node[0] for node in self.graph.keys())
        max_y = max(node[1] for node in self.graph.keys())
        return max_x + 1, max_y + 1

    def setup_plot(self):
        # self.ax.set_xlim(-0.5, self.grid_size[0] - 0.5)
        # self.ax.set_ylim(-0.5, self.grid_size[1] - 0.5)
        self.ax.set_xlim(0, self.grid_size[0])
        self.ax.set_ylim(0, self.grid_size[1])
        
        self.ax.set_xticks(range(self.grid_size[0]))
        self.ax.set_yticks(range(self.grid_size[1]))
        self.ax.grid(True)
        self.ax.set_aspect('equal')
        
        # ゴール位置を矩形で表示
        for goal in self.goals:
            # self.ax.add_patch(Rectangle((goal[0] - 0.5, goal[1] - 0.5), 1, 1, color='yellow', alpha=0.3))
            self.ax.add_patch(Rectangle((goal[0], goal[1]), 1, 1, color='yellow', alpha=0.3))

        # 障害物を描画
        for obstacle in self.obstacles:
            #self.ax.add_patch(Rectangle((obstacle[0] - 0.5, obstacle[1] - 0.5), 1, 1, color='black'))
            self.ax.add_patch(Rectangle((obstacle[0], obstacle[1]), 1, 1, color='black'))

        # エージェントの円形パッチを作成
        self.agent_patches = [
            Circle(self.starts[i], 0.3, color=f'C{i}') for i in range(self.num_agents)
        ]
        for patch in self.agent_patches:
            self.ax.add_patch(patch)

    def animate(self, frame):
        for i, patch in enumerate(self.agent_patches):
            pos = self.paths[i][min(frame, len(self.paths[i]) - 1)]
            patch.center = pos
        return self.agent_patches



    def save_animation(self, filename="output.mp4", fps=2):
        """アニメーションを録画して保存します。
        
        Args:
            filename (str): 保存するファイル名 (例: "output.mp4")
            fps (int): フレームレート (1秒あたりのフレーム数)
        """
        max_frames = max(len(path) for path in self.paths)
        ani = animation.FuncAnimation(
            self.fig, self.animate, frames=max_frames, interval=1000 // fps, blit=True, repeat=False
        )
        ani.save(filename, writer="ffmpeg", fps=fps)
        print(f"Animation saved to {filename}")



    def display_animation(self):
        max_frames = max(len(path) for path in self.paths)
        ani = animation.FuncAnimation(
            self.fig, self.animate, frames=max_frames, interval=500, blit=True, repeat=False
        )
        plt.show()



# Run and Visualize CBS
def visualize_cbs_result(graph, starts, goals):
    paths = CBS(graph, starts, goals)
    if paths is None:
        print("No solution found")
        return
    visualizer = AnimationVisualizer(graph, paths, starts, goals, obstacles=obstacles)
    visualizer.save_animation(filename="cbs_simulation.mp4", fps=2)
    visualizer.display_animation()




def generate_grid_graph(size, obstacles=None):
    """指定されたサイズのグリッドグラフを生成し、障害物を設置します。
    
    Args:
        size (int): グリッドのサイズ (例: 30なら30x30のグリッド)
        obstacles (list): 障害物の位置 [(x1, y1), (x2, y2), ...]
    
    Returns:
        dict: グラフ構造 {ノード: 隣接ノードリスト}
    """
    graph = {}
    if obstacles is None:
        obstacles = []

    for x in range(size):
        for y in range(size):
            if (x, y) in obstacles:
                continue  # 障害物の位置はスキップ

            neighbors = []
            if x > 0 and (x - 1, y) not in obstacles:  # 左隣
                neighbors.append((x - 1, y))
            if x < size - 1 and (x + 1, y) not in obstacles:  # 右隣
                neighbors.append((x + 1, y))
            if y > 0 and (x, y - 1) not in obstacles:  # 上隣
                neighbors.append((x, y - 1))
            if y < size - 1 and (x, y + 1) not in obstacles:  # 下隣
                neighbors.append((x, y + 1))
            
            graph[(x, y)] = neighbors

    return graph

# 障害物の位置を定義

obstacles = []
for y in range(13):
    for x in range(31): 
        obstacles.append((x,y))

print(obstacles)


# グリッドを生成 (30x30)
grid_size = 31
graph = generate_grid_graph(grid_size, obstacles)

# エージェント数
num_agents = 5

starts = [(0, 15), (30, 15)]
goals = [(30, 14), (0, 14)]

# CBS 結果を可視化
visualize_cbs_result(graph, starts, goals)


