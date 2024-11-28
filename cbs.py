import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle
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
            return path  # 目的地に到達
        
        if (current, len(path) - 1) in constraints.get(agent_id, []):
            continue  # 制約に違反する経路は除外

        for neighbor in graph[current]:
            if neighbor not in closed_set:
                new_path = path + [neighbor]
                heappush(open_list, (cost + 1, neighbor, new_path))
                closed_set.add(neighbor)
    #print(current)
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
    root = CBSNode([], [], 0)  # 初期ノード
    for i in range(len(starts)):
        path = a_star(graph, starts[i], goals[i], {}, i)
        if path is None:
            return None  # 経路が見つからない
        root.paths.append(path)
    print("start",i)
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
            constraints1.append((i, t, loc[0], loc[1]))  # エージェントiの制約
            constraints2.append((j, t, loc[1], loc[0]))  # エージェントjの制約
        else:
            constraints1.append((i, t, loc))  # エージェントiの制約
            constraints2.append((j, t, loc))  # エージェントjの制約

        # 各エージェントに制約を適用して新しいノードを作成
        for constraints, agent_id in [(constraints1, i), (constraints2, j)]:
            new_paths = deepcopy(node.paths)
            path = a_star(graph, starts[agent_id], goals[agent_id], {agent_id: constraints}, agent_id)
            if path is not None:
                new_paths[agent_id] = path
                new_cost = get_total_cost(new_paths)
                heappush(open_list, CBSNode(constraints, new_paths, new_cost))
    
    return None  # 解が見つからない場合

# AnimationVisualizer for CBS Results
class AnimationVisualizer:
    def __init__(self, graph, paths):
        self.graph = graph
        self.paths = paths
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
        self.ax.set_xlim(-0.5, self.grid_size[0] - 0.5)
        self.ax.set_ylim(-0.5, self.grid_size[1] - 0.5)
        self.ax.set_xticks(range(self.grid_size[0]))
        self.ax.set_yticks(range(self.grid_size[1]))
        self.ax.grid(True)
        self.ax.set_aspect('equal')
        
        # エージェントの円形パッチを作成
        self.agent_patches = [
            Circle(self.paths[i][0], 0.3, color=f'C{i}') for i in range(self.num_agents)
        ]
        for patch in self.agent_patches:
            self.ax.add_patch(patch)

    def animate(self, frame):
        for i, patch in enumerate(self.agent_patches):
            pos = self.paths[i][min(frame, len(self.paths[i]) - 1)]
            patch.center = pos
        return self.agent_patches

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
    visualizer = AnimationVisualizer(graph, paths)
    visualizer.display_animation()

# Example Graph, Start, and Goal
graph = {
    (0, 0): [(0, 0), (0, 0)],
    (0, 1): [(0, 0), (0, 1)],
    (1, 0): [(1, 0), (0, 0)],
    (1, 1): [(1, 1), (1, 1)]
}
starts = [(0, 0), (1, 1)]
goals = [(1, 1), (0, 0)]

# Visualize the results
visualize_cbs_result(graph, starts, goals)
