import numpy as np
import networkx as nx
import math
import random
import time
import copy
import setting
import ga 
import change_ga_vw_adhoc

# シミュレーションモード設定（"traffic_flow"または"fourcars_scenario"）
SIMULATION_MODE = "fourcars_scenario"

# シミュレーションモードを設定する関数
def set_simulation_mode(mode):
    """
    シミュレーションモードを設定する
    
    Parameters:
    - mode: "traffic_flow"または"fixed_scenario"
    """
    global SIMULATION_MODE
    if mode in ["traffic_flow", "fixed_scenario"]:
        SIMULATION_MODE = mode
        print(f"シミュレーションモードを '{mode}' に設定しました")
    else:
        print(f"警告: 無効なモード '{mode}'。'traffic_flow'または'fixed_scenario'を指定してください")

class SimulationConfig:
    """シミュレーションの設定を一元管理するクラス"""
    def __init__(self):
        # 基本設定
        self.car_width = setting.car_width
        self.car_length = setting.car_length
        self.base_speed = setting.speed
        
        # フィールド設定
        self.field_width = 900
        self.field_height = 500
        
        # 交差点設定
        self.intersection_center = [450, 233]
        self.intersection_size = setting.VWfield
        self.road_width = setting.VWfield
        
        # 安全マージン計算のパラメータ
        self.reaction_time = 0.2
        self.braking_factor = 0.1
        
        # シミュレーション状態
        self.max_steps = 500
        
        # VW設定
        self.vw_field_size = setting.VWnum
        self.vw_field = setting.VWfield
        self.vw_size = self.vw_field / self.vw_field_size
        self.vw_field_x = setting.VWfield_x
        self.vw_field_y = setting.VWfield_y
        
        # 車両の最大旋回角度
        self.car_angle = setting.car_angle

class Vehicle:
    """単一車両の状態と振る舞いを管理するクラス"""
    def __init__(self, start, goal, vehicle_id=0, vehicle_type="straight", spawn_direction=None, spawn_lane_idx=None):
        self.id = vehicle_id
        self.start = start.copy()
        self.goal = goal.copy()
        self.position = start.copy()
        self.velocity = [0.0, 0.0]
        self.target_velocity = [0.0, 0.0]
        self.reached = False
        self.stopped_time = 0
        self.slowed_down = False
        self.last_angle = 0.0
        self.type = vehicle_type
        self.waypoints = []
        self.current_waypoint_idx = -1
        
        # スポーン情報
        self.spawn_direction = spawn_direction
        self.spawn_lane_idx = spawn_lane_idx
        self.spawn_info = {
            "direction": spawn_direction,
            "lane_idx": spawn_lane_idx
        } if spawn_direction and spawn_lane_idx is not None else None

    def update_position(self):
        """車両の位置を更新"""
        if self.reached:
            return
        self.position[0] += self.velocity[0]
        self.position[1] += self.velocity[1]
    
    def set_velocity(self, vel_x, vel_y):
        """速度を設定"""
        self.velocity[0] = vel_x
        self.velocity[1] = vel_y
    
    def get_speed(self):
        """現在の速度の大きさを返す"""
        return np.linalg.norm(self.velocity)
    
    def set_waypoints(self, waypoints):
        """経路のウェイポイントを設定"""
        self.waypoints = waypoints
        if waypoints:
            self.current_waypoint_idx = 0
     
    def is_at_waypoint(self, threshold=8.0):
        """現在のウェイポイントに到達したかどうかを判定"""
        if not self.waypoints or self.current_waypoint_idx < 0:
            return False
        if self.current_waypoint_idx >= len(self.waypoints):
            return True
        waypoint = self.waypoints[self.current_waypoint_idx]
        dist = np.sqrt(
            (self.position[0] - waypoint[0])**2 + 
            (self.position[1] - waypoint[1])**2
        )
        return dist < threshold
    
    def advance_to_next_waypoint(self):
        """次のウェイポイントに進む"""
        if self.waypoints and self.current_waypoint_idx < len(self.waypoints):
            self.current_waypoint_idx += 1
    
    def is_reached_goal(self, threshold=5.0):
        """目標位置に到達したかどうかを判定"""
        if self.waypoints:
            if self.current_waypoint_idx >= len(self.waypoints):
                dist = np.sqrt(
                    (self.position[0] - self.goal[0])**2 + 
                    (self.position[1] - self.goal[1])**2
                )
                return dist < threshold
            return False
        dist = np.sqrt(
            (self.position[0] - self.goal[0])**2 + 
            (self.position[1] - self.goal[1])**2
        )
        return dist < threshold
    
    def is_almost_stopped(self, threshold=0.5):
        """車両がほぼ停止しているかどうかを判定"""
        return self.get_speed() < threshold

class CarAgent:
    """車両エージェント（経路生成用の簡易クラス）"""
    def __init__(self, start, goal):
        self.start = start
        self.goal = goal
        self.position = start

class CollisionDetector:
    """車両間の衝突を検出・予測するクラス"""
    def __init__(self, config):
        self.config = config
    
    def detect_collisions(self, vehicles):
        """実際の衝突を検出"""
        collisions = 0
        active_vehicles = [v for v in vehicles if not v.reached]
        
        for i in range(len(active_vehicles)):
            for j in range(i+1, len(active_vehicles)):
                v1 = active_vehicles[i]
                v2 = active_vehicles[j]
                
                dist = np.sqrt(
                    (v1.position[0] - v2.position[0])**2 + 
                    (v1.position[1] - v2.position[1])**2
                )
                
                if dist < (self.config.car_width/2):
                    collisions += 1
        
        return collisions
    
    def predict_collisions(self, vehicles, prediction_steps=10):
        """将来の衝突を予測"""
        predictions = []
        active_vehicles = [v for v in vehicles if not v.reached]
        
        potential_pairs = self._filter_potential_collision_pairs(active_vehicles)
        
        for i, j in potential_pairs:
            v1 = active_vehicles[i]
            v2 = active_vehicles[j]
            
            future_positions1 = self._predict_future_positions(v1, prediction_steps)
            future_positions2 = self._predict_future_positions(v2, prediction_steps)
            
            for step in range(min(len(future_positions1), len(future_positions2))):
                pos1 = future_positions1[step]
                pos2 = future_positions2[step]
                
                dist = np.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
                
                speed1 = v1.get_speed()
                speed2 = v2.get_speed()
                
                if speed1 > 0 and speed2 > 0:
                    v1_dir = v1.velocity.copy()
                    v2_dir = v2.velocity.copy()
                    v1_dir_norm = v1_dir / speed1
                    v2_dir_norm = v2_dir / speed2
                    
                    cos_angle = np.dot(v1_dir_norm, v2_dir_norm)
                    angle = np.arccos(np.clip(cos_angle, -1.0, 1.0))
                else:
                    angle = 0
                
                safety_margin = self._calculate_safety_margin(speed1, speed2, angle)
                
                if dist < (self.config.car_width + safety_margin):
                    predictions.append({
                        "vehicles": (i, j),
                        "time_step": step,
                        "distance": dist,
                        "future_positions": (pos1.copy(), pos2.copy()),
                        "in_intersection": self._is_in_intersection(pos1) or self._is_in_intersection(pos2)
                    })
                    break
        
        return predictions
    
    def _filter_potential_collision_pairs(self, vehicles):
        """潜在的な衝突の可能性があるペアのみを抽出"""
        pairs = []
        
        for i in range(len(vehicles)):
            for j in range(i+1, len(vehicles)):
                v1 = vehicles[i]
                v2 = vehicles[j]
                
                dist = np.sqrt(
                    (v1.position[0] - v2.position[0])**2 + 
                    (v1.position[1] - v2.position[1])**2
                )
                
                max_speed = max(v1.get_speed(), v2.get_speed())
                max_prediction_dist = self.config.car_width * 3 + max_speed * 10
                
                if dist < max_prediction_dist:
                    pairs.append((i, j))
        
        return pairs
    
    def _predict_future_positions(self, vehicle, steps):
        """車両の将来位置を予測"""
        future_positions = []
        pos = vehicle.position.copy()
        vel = vehicle.velocity.copy()
        
        if vehicle.waypoints and vehicle.current_waypoint_idx >= 0:
            wp_idx = vehicle.current_waypoint_idx
            
            for _ in range(steps):
                if wp_idx < len(vehicle.waypoints):
                    waypoint = vehicle.waypoints[wp_idx]
                    
                    dx = waypoint[0] - pos[0]
                    dy = waypoint[1] - pos[1]
                    dist = np.sqrt(dx**2 + dy**2)
                    
                    if dist > 0:
                        dir_x = dx / dist
                        dir_y = dy / dist
                        speed = np.linalg.norm(vel)
                        new_vel = [dir_x * speed, dir_y * speed]
                        pos[0] += new_vel[0]
                        pos[1] += new_vel[1]
                        
                        new_dist = np.sqrt((waypoint[0] - pos[0])**2 + (waypoint[1] - pos[1])**2)
                        if new_dist < 5:
                            wp_idx += 1
                    else:
                        wp_idx += 1
                
                future_positions.append(pos.copy())
        else:
            for _ in range(steps):
                pos[0] += vel[0]
                pos[1] += vel[1]
                future_positions.append(pos.copy())
        
        return future_positions
    
    def _calculate_safety_margin(self, speed1, speed2, angle):
        """安全マージンを計算"""
        margin1 = self._single_vehicle_safety_margin(speed1)
        margin2 = self._single_vehicle_safety_margin(speed2)
        angle_factor = 1.0 + (1.0 - np.cos(angle)) / 2.0
        return (margin1 + margin2) * angle_factor
    
    def _single_vehicle_safety_margin(self, speed):
        """単一車両の安全マージンを計算"""
        base_margin = self.config.car_length
        reaction_distance = speed * self.config.reaction_time
        braking_distance = self.config.braking_factor * (speed ** 2)
        return base_margin + reaction_distance + braking_distance
    
    def _is_in_intersection(self, position):
        """位置が交差点内かどうかを判定"""
        ic_x, ic_y = self.config.intersection_center
        half_size = self.config.intersection_size / 2
        
        return (ic_x - half_size <= position[0] <= ic_x + half_size and 
                ic_y - half_size <= position[1] <= ic_y + half_size)

class VirtualWallManager:
    """仮想障害物（VW）の管理クラス - 個別VWと共通VWの両方をサポート"""
    def __init__(self, config):
        self.config = config
        self.vw_field_size = config.vw_field_size
        self.vw_size = config.vw_field / config.vw_field_size
        
        # 共通VW用のグリッド
        self.common_vw_grid = np.zeros((self.vw_field_size, self.vw_field_size))
        self.common_vw_vertices = []
        self.common_vw_lines = []
        
        # 個別VW用のグリッド（スポーン地点ごと）
        self.individual_vw_grids = {
            "left": [np.zeros((self.vw_field_size, self.vw_field_size)) for _ in range(3)],
            "right": [np.zeros((self.vw_field_size, self.vw_field_size)) for _ in range(3)],
            "top": [np.zeros((self.vw_field_size, self.vw_field_size)) for _ in range(3)],
            "bottom": [np.zeros((self.vw_field_size, self.vw_field_size)) for _ in range(3)]
        }
        self.individual_vw_vertices = {
            "left": [[] for _ in range(3)],
            "right": [[] for _ in range(3)],
            "top": [[] for _ in range(3)],
            "bottom": [[] for _ in range(3)]
        }
        self.individual_vw_lines = {
            "left": [[] for _ in range(3)],
            "right": [[] for _ in range(3)],
            "top": [[] for _ in range(3)],
            "bottom": [[] for _ in range(3)]
        }
        
        # VWモード（共通または個別）
        self.vw_mode = "common"  # デフォルトは共通モード
    
    def set_vw_mode(self, mode):
        """VWモードを設定（'common'または'individual'）"""
        if mode in ["common", "individual"]:
            self.vw_mode = mode
        else:
            raise ValueError("無効なVWモードです。'common'または'individual'を指定してください。")
    
    def set_common_vw(self, grid):
        """共通VWを設定"""
        if grid.shape != (self.vw_field_size, self.vw_field_size):
            raise ValueError(f"グリッドサイズは({self.vw_field_size}, {self.vw_field_size})である必要があります")
        
        self.common_vw_grid = grid.copy()
        self._update_vw_geometry(self.common_vw_grid, result_type="common")
    
    def set_individual_vw(self, direction, lane_idx, grid):
        """個別VWを設定"""
        if grid.shape != (self.vw_field_size, self.vw_field_size):
            raise ValueError(f"グリッドサイズは({self.vw_field_size}, {self.vw_field_size})である必要があります")
        
        if direction not in self.individual_vw_grids:
            raise ValueError(f"無効な方向: {direction}")
        
        if lane_idx < 0 or lane_idx >= 3:
            raise ValueError(f"車線インデックスは0-2の範囲である必要があります: {lane_idx}")
        
        self.individual_vw_grids[direction][lane_idx] = grid.copy()
        self._update_vw_geometry(grid, result_type="individual", direction=direction, lane_idx=lane_idx)
    
    def _update_vw_geometry(self, grid, result_type="common", direction=None, lane_idx=None):
        """グリッドに基づいてVWの頂点と線分を更新"""
        vertices = []
        lines = []
        
        # フィールドの左上隅（原点）
        field_x = self.config.intersection_center[0] - (self.vw_field_size * self.vw_size) / 2
        field_y = self.config.intersection_center[1] - (self.vw_field_size * self.vw_size) / 2
        
        # グリッドをスキャンしてVWを作成
        for i in range(self.vw_field_size):
            for j in range(self.vw_field_size):
                if grid[i, j] >= 1:
                    # VWの左上座標
                    vw_left_up = [field_x + (self.vw_size * j), field_y + (self.vw_size * i)]
                    
                    # VWの四隅の座標
                    vw_left_down = [vw_left_up[0], vw_left_up[1] + self.vw_size]
                    vw_right_up = [vw_left_up[0] + self.vw_size, vw_left_up[1]]
                    vw_right_down = [vw_left_up[0] + self.vw_size, vw_left_up[1] + self.vw_size]
                    
                    # 頂点を追加
                    vertices.extend([vw_left_up, vw_left_down, vw_right_up, vw_right_down])
                    
                    # 線分を追加
                    lines.extend([
                        [vw_left_up, vw_left_down],
                        [vw_left_up, vw_right_up],
                        [vw_right_up, vw_right_down],
                        [vw_right_down, vw_left_down]
                    ])
        
        # 結果を保存
        if result_type == "common":
            self.common_vw_vertices = vertices
            self.common_vw_lines = lines
        elif result_type == "individual" and direction is not None and lane_idx is not None:
            self.individual_vw_vertices[direction][lane_idx] = vertices
            self.individual_vw_lines[direction][lane_idx] = lines
    
    def get_vw_for_vehicle(self, vehicle_spawn_info=None):
        """車両のスポーン情報に基づいて適切なVWを取得"""
        if self.vw_mode == "common":
            return self.common_vw_vertices, self.common_vw_lines
        else:  # individual
            if vehicle_spawn_info is None:
                return self.common_vw_vertices, self.common_vw_lines
            
            direction = vehicle_spawn_info["direction"]
            lane_idx = vehicle_spawn_info["lane_idx"]
            
            return (self.individual_vw_vertices[direction][lane_idx],
                    self.individual_vw_lines[direction][lane_idx])

class PathFinder:
    """経路計算を担当するクラス"""
    def set_wall():
        """壁情報を取得"""
        wall_edge_list = setting.wall_edge_list
        wall_line_list = setting.wall_line_list
        return wall_edge_list, wall_line_list
    
    def set_vertex_list(obstacle_list, car_agent, wall_edge):
        """頂点リストを作成"""
        start = car_agent.start.copy()
        goal = car_agent.goal.copy()
        vertex_list = [start, goal]
        vertex_list.extend(obstacle_list)
        vertex_list.extend(wall_edge)
        return vertex_list
    
    @staticmethod
    def visibility_graph(vertex_list, obstacle_line_list):
        """可視グラフを計算"""
        visibility_graph_list = []
        
        for index, vertex_u in enumerate(vertex_list):
            for goal_index, vertex_v in enumerate(vertex_list[index + 1:], index + 1):
                Line = [index, goal_index]
                cross = False
                
                for obstacle_Line in obstacle_line_list:
                    s = (vertex_v[0] - vertex_u[0])*(obstacle_Line[0][1] - vertex_u[1]) - (obstacle_Line[0][0] - vertex_u[0]) * (vertex_v[1] - vertex_u[1])
                    t = (vertex_v[0] - vertex_u[0])*(obstacle_Line[1][1] - vertex_u[1]) - (obstacle_Line[1][0] - vertex_u[0]) * (vertex_v[1] - vertex_u[1])
                    
                    if s * t < 0:
                        cross = True
                        break
                
                if not cross:
                    Line.append(np.sqrt(((vertex_v[0] - vertex_u[0])**2 + (vertex_v[1] - vertex_u[1])**2)))
                    visibility_graph_list.append(tuple(Line))
        
        return visibility_graph_list
    
    @staticmethod
    def dijkstra(visibility_graph_list):
        """ダイクストラ法を使った最短経路計算"""
        nx_Graph = nx.Graph()
        nx_Graph.add_weighted_edges_from(visibility_graph_list)
        origin_node = 0
        destination_node = 1
        
        try:
            shortest_path = nx.dijkstra_path(nx_Graph, origin_node, destination_node)
            shortest_length = nx.dijkstra_path_length(nx_Graph, origin_node, destination_node)
        except (nx.NetworkXNoPath, nx.NodeNotFound):
            # 経路が見つからない場合
            shortest_path = [0, 1]
            shortest_length = float('inf')
            
        return shortest_path, shortest_length
    
    @staticmethod
    def collision_CarToCar(car1_vertices, car1_path, car2_vertices, car2_path, 
                            car3_vertices, car3_path, car4_vertices, car4_path):
        """車両間の衝突を検出"""
        # 衝突回数
        collision = 0
        
        # 車両のサイズに基づく衝突判定距離
        r = np.sqrt((setting.car_length/2)**2 + (setting.car_width/2)**2)
        
        # 経路上の各位置で衝突チェック
        for i in range(min(len(car1_path), len(car2_path), len(car3_path), len(car4_path))):
            # 各車両の現在位置
            if i < len(car1_path): car1_pos = car1_vertices[car1_path[i]]
            else: car1_pos = car1_vertices[-1]
            
            if i < len(car2_path): car2_pos = car2_vertices[car2_path[i]]
            else: car2_pos = car2_vertices[-1]
            
            if i < len(car3_path): car3_pos = car3_vertices[car3_path[i]]
            else: car3_pos = car3_vertices[-1]
            
            if i < len(car4_path): car4_pos = car4_vertices[car4_path[i]]
            else: car4_pos = car4_vertices[-1]
            
            # 車両間の距離を計算して衝突判定
            if np.sqrt((car1_pos[0] - car2_pos[0])**2 + (car1_pos[1] - car2_pos[1])**2) <= r:
                collision += 1
            if np.sqrt((car1_pos[0] - car3_pos[0])**2 + (car1_pos[1] - car3_pos[1])**2) <= r:
                collision += 1
            if np.sqrt((car1_pos[0] - car4_pos[0])**2 + (car1_pos[1] - car4_pos[1])**2) <= r:
                collision += 1
            if np.sqrt((car2_pos[0] - car3_pos[0])**2 + (car2_pos[1] - car3_pos[1])**2) <= r:
                collision += 1
            if np.sqrt((car2_pos[0] - car4_pos[0])**2 + (car2_pos[1] - car4_pos[1])**2) <= r:
                collision += 1
            if np.sqrt((car3_pos[0] - car4_pos[0])**2 + (car3_pos[1] - car4_pos[1])**2) <= r:
                collision += 1
        
        return collision

class TrafficGenerator:
    """交通流を生成する機能を提供するクラス"""
    def __init__(self, config):
        self.config = config
        self.spawn_points = self._create_spawn_points()
        
        # 方向選択の確率設定
        self.direction_probabilities = {
            "straight": 0.5,  # 直進の確率
            "right_turn": 0.5,  # 右折の確率
            "left_turn": 0.5   # 左折の確率
        }
        
        # 車両ID用カウンター
        self.next_vehicle_id = 0

        # 各スポーン地点の最終生成時間を記録
        self.last_spawn_times = {
            "left": [0, 0, 0],  # 各車線の最終生成時間
            "right": [0, 0, 0],
            "top": [0, 0, 0],
            "bottom": [0, 0, 0]
        }
        
        # 最小車両間隔（ステップ数）
        self.min_spawn_interval = 30  # 例：30ステップ
        
        # 安全距離（px）
        self.safe_distance = 40  # 車両長の約2倍
    
    def _create_spawn_points(self):
        """各方向に3つのスポーンポイントを作成"""
        # 交差点中心
        ic_x, ic_y = self.config.intersection_center
        road_width = self.config.road_width
        
        # スポーンポイントからの距離（交差点より十分離れた位置）
        distance_from_center = 150
        
        # 車線の幅（道路幅の1/3とする）
        lane_width = road_width / 3
        
        # 各方向のスポーンポイント
        spawn_points = {
            "left": [
                [ic_x - distance_from_center, ic_y - lane_width],  # 左側上車線
                [ic_x - distance_from_center, ic_y],               # 左側中央車線
                [ic_x - distance_from_center, ic_y + lane_width]   # 左側下車線
            ],
            "right": [
                [ic_x + distance_from_center, ic_y - lane_width],  # 右側上車線
                [ic_x + distance_from_center, ic_y],               # 右側中央車線
                [ic_x + distance_from_center, ic_y + lane_width]   # 右側下車線
            ],
            "top": [
                [ic_x - lane_width, ic_y - distance_from_center],  # 上側左車線
                [ic_x, ic_y - distance_from_center],               # 上側中央車線
                [ic_x + lane_width, ic_y - distance_from_center]   # 上側右車線
            ],
            "bottom": [
                [ic_x - lane_width, ic_y + distance_from_center],  # 下側左車線
                [ic_x, ic_y + distance_from_center],               # 下側中央車線
                [ic_x + lane_width, ic_y + distance_from_center]   # 下側右車線
            ]
        }
        
        return spawn_points
    
    def select_random_spawn_point(self):
        """ランダムな方向と車線からスポーンポイントを選択"""
        # ランダムな方向を選択
        direction = random.choice(list(self.spawn_points.keys()))
        
        # 選択した方向からランダムな車線を選択
        lane_idx = random.randint(0, 2)
        
        # スポーンポイントを返す
        spawn_point = self.spawn_points[direction][lane_idx]
        
        return spawn_point, direction, lane_idx
    
    def select_destination(self, spawn_direction, spawn_lane_idx):
        """スポーン方向と車線に基づいて目的地を選択"""
        # ランダムな進行方向を確率に基づいて選択
        movement_type = random.choices(
            list(self.direction_probabilities.keys()),
            weights=list(self.direction_probabilities.values())
        )[0]
        
        # スポーン方向に基づいて目的地の方向を決定
        if movement_type == "straight":
            # 直進：反対側へ
            if spawn_direction == "left":
                dest_direction = "right"
            elif spawn_direction == "right":
                dest_direction = "left"
            elif spawn_direction == "top":
                dest_direction = "bottom"
            else:  # bottom
                dest_direction = "top"
                
            # 同じ車線を維持
            dest_lane_idx = spawn_lane_idx
            
        elif movement_type == "right_turn":
            # 右折：時計回りの方向へ
            if spawn_direction == "left":
                dest_direction = "top"
            elif spawn_direction == "top":
                dest_direction = "right"
            elif spawn_direction == "right":
                dest_direction = "bottom"
            else:  # bottom
                dest_direction = "left"
                
            # 車線の対応関係を設定（右折時の自然な流れを表現）
            if spawn_direction in ["left", "right"]:
                # 左右からの右折：上下方向の対応車線へ
                dest_lane_idx = spawn_lane_idx
            else:
                # 上下からの右折：左右方向の対応車線へ（車線の反転が必要）
                dest_lane_idx = 2 - spawn_lane_idx
            
        else:  # left_turn
            # 左折：反時計回りの方向へ
            if spawn_direction == "left":
                dest_direction = "bottom"
            elif spawn_direction == "bottom":
                dest_direction = "right"
            elif spawn_direction == "right":
                dest_direction = "top"
            else:  # top
                dest_direction = "left"
                
            # 車線の対応関係を設定（左折時の自然な流れを表現）
            if spawn_direction in ["left", "right"]:
                # 左右からの左折：上下方向の対応車線へ（車線の反転が必要）
                dest_lane_idx = 2 - spawn_lane_idx
            else:
                # 上下からの左折：左右方向の対応車線へ
                dest_lane_idx = spawn_lane_idx
        
        # 目的地のスポーンポイントを取得
        destination = self.spawn_points[dest_direction][dest_lane_idx]
        
        return destination, movement_type
    
    def generate_random_vehicle(self):
        """ランダムなスポーンポイントと目的地で車両を生成"""
        # ランダムなスポーンポイントを選択
        spawn_point, spawn_direction, spawn_lane_idx = self.select_random_spawn_point()
        
        # スポーン方向に基づいて目的地を選択
        destination, movement_type = self.select_destination(spawn_direction, spawn_lane_idx)
        
        # 車両オブジェクトを作成
        vehicle = Vehicle(
            start=spawn_point,
            goal=destination,
            vehicle_id=self.next_vehicle_id,
            vehicle_type=movement_type,
            spawn_direction=spawn_direction,
            spawn_lane_idx=spawn_lane_idx
        )
        
        # 車両IDを更新
        self.next_vehicle_id += 1
        
        return vehicle, spawn_direction, movement_type
    
    def generate_random_vehicles(self, num_vehicles):
        """指定された数のランダムな車両を生成"""
        vehicles = []
        
        for _ in range(num_vehicles):
            vehicle, _, _ = self.generate_random_vehicle()
            vehicles.append(vehicle)
        
        return vehicles
    
    def generate_vehicle_if_needed(self, simulation, spawn_rate=0.1, max_vehicles=20):
        """継続的に新しい車両を生成する機能（安全間隔を考慮）"""
        current_time = simulation.time_steps
        
        # 車両数が最大値を超えていないか確認
        active_vehicles = sum(1 for v in simulation.vehicles if not v.reached)
        if active_vehicles >= max_vehicles:
            return None
        
        # 確率に基づいて生成するかどうか決定
        if random.random() >= spawn_rate:
            return None
            
        # 利用可能なスポーン地点を収集
        available_spawn_points = []
        
        for direction in self.spawn_points.keys():
            for lane_idx in range(3):
                # 時間条件：最後の生成から十分な時間が経過しているか
                time_condition = (current_time - self.last_spawn_times[direction][lane_idx] 
                                 >= self.min_spawn_interval)
                
                if time_condition:
                    # 距離条件：前方に既存の車両がないか確認
                    spawn_point = self.spawn_points[direction][lane_idx]
                    is_safe = self._check_spawn_safety(spawn_point, direction, simulation)
                    
                    if is_safe:
                        available_spawn_points.append((direction, lane_idx))
        
        # 利用可能なスポーン地点がなければ生成しない
        if not available_spawn_points:
            return None
            
        # ランダムに1つのスポーン地点を選択
        spawn_direction, spawn_lane_idx = random.choice(available_spawn_points)
        spawn_point = self.spawn_points[spawn_direction][spawn_lane_idx]
        
        # 目的地を選択
        destination, movement_type = self.select_destination(spawn_direction, spawn_lane_idx)
        
        # 車両オブジェクトを作成
        vehicle = Vehicle(
            start=spawn_point,
            goal=destination,
            vehicle_id=self.next_vehicle_id,
            vehicle_type=movement_type,
            spawn_direction=spawn_direction,
            spawn_lane_idx=spawn_lane_idx
        )
        
        # 最終生成時間を更新
        self.last_spawn_times[spawn_direction][spawn_lane_idx] = current_time
        
        # 車両IDを更新
        self.next_vehicle_id += 1
        
        return vehicle
    
    def _check_spawn_safety(self, spawn_point, direction, simulation):
        """スポーンポイントの安全性をチェック（前方の車両との距離）"""
        # 進行方向ベクトルを取得
        direction_vector = self._get_direction_vector(direction)
        
        # 各車両について前方にいるか確認
        for vehicle in simulation.vehicles:
            if vehicle.reached:
                continue
                
            # 車両の位置とスポーン地点の距離を計算
            dx = vehicle.position[0] - spawn_point[0]
            dy = vehicle.position[1] - spawn_point[1]
            dist = np.sqrt(dx**2 + dy**2)
            
            # 安全距離内に車両がいる場合
            if dist < self.safe_distance:
                # 進行方向に沿って前方にいるかチェック
                dot_product = dx * direction_vector[0] + dy * direction_vector[1]
                if dot_product > 0:  # 進行方向の前方に車両がいる
                    return False
        
        return True
    
    def _get_direction_vector(self, direction):
        """指定された方向の単位ベクトルを返す"""
        if direction == "left":
            return [1, 0]  # 右向き
        elif direction == "right":
            return [-1, 0]  # 左向き
        elif direction == "top":
            return [0, 1]  # 下向き
        else:  # bottom
            return [0, -1]  # 上向き
        
    def adjust_spawn_rate_by_congestion(self, simulation, base_rate=0.1):
        """交通密度に応じて生成率を調整"""
        # 交差点内の車両数をカウント
        vehicles_in_intersection = 0
        
        for vehicle in simulation.vehicles:
            if not vehicle.reached:
                # 交差点内かどうかを判定
                in_intersection = simulation._is_in_intersection(vehicle.position)
                if in_intersection:
                    vehicles_in_intersection += 1
        
        # 混雑度に応じた生成率の調整
        if vehicles_in_intersection >= 6:
            return base_rate * 0.2  # 非常に混雑: 生成率を80%減
        elif vehicles_in_intersection >= 4:
            return base_rate * 0.5  # 混雑: 生成率を50%減
        elif vehicles_in_intersection >= 2:
            return base_rate * 0.7  # やや混雑: 生成率を30%減
        else:
            return base_rate  # 空いている: 通常の生成率

class CarSimulation:
    """車両シミュレーションのメインクラス"""
    def __init__(self):
        """車両シミュレーションの初期化"""
        self.config = SimulationConfig()
        self.collision_detector = CollisionDetector(self.config)
        self.path_finder = PathFinder()
        self.vw_manager = VirtualWallManager(self.config)
        self.traffic_generator = TrafficGenerator(self.config)
        
        # シミュレーション状態
        self.vehicles = []
        self.wall_vertices = []  # 壁の頂点情報
        self.wall_lines = []     # 壁の線分情報
        self.time_steps = 0
        self.collision_count = 0
        self.adhoc_count = 0
        
        # 車両の優先度
        self.priorities = []
        
        # 経路記録用
        self.trajectories = []
        
        # アドホック回避
        self.use_adhoc = True
        
        # 壁の設定（初期化時に必ず実行）
        self.setup_walls()
    
    def setup_walls(self):
        """壁や静的障害物の設定"""
        self.wall_vertices, self.wall_lines = PathFinder.set_wall()
        
        # 壁のデータが正しく取得できたか確認
        if not self.wall_vertices or not self.wall_lines:
            print("警告: 壁の情報が取得できませんでした")
            print(f"wall_vertices: {len(self.wall_vertices) if self.wall_vertices else 0}個")
            print(f"wall_lines: {len(self.wall_lines) if self.wall_lines else 0}本")
        else:
            print(f"壁データ: {len(self.wall_vertices)}頂点, {len(self.wall_lines)}線分を読み込みました")
            # 一部の壁データを表示して確認
            if self.wall_lines:
                print(f"例: 壁線分[0]: {self.wall_lines[0]}")

    def initialize_scenario(self, scenario_type="basic"):
        """シナリオに基づいて車両を初期化"""
        self.vehicles = []
        self.priorities = []
        self.trajectories = []
        
        if scenario_type == "basic":
            self.setup_basic_scenario()
        elif scenario_type == "mixed_turns":
            self.setup_mixed_turns_scenario()
        elif scenario_type == "complex_crossing":
            self.setup_complex_crossing_scenario()
        else:
            raise ValueError(f"未知のシナリオタイプ: {scenario_type}")
        
        # 優先度と軌跡の初期化
        self.priorities = [0] * len(self.vehicles)
        self.trajectories = [[] for _ in range(len(self.vehicles))]
        
        # 初期軌跡を記録
        for i, vehicle in enumerate(self.vehicles):
            self.trajectories[i].append(vehicle.position.copy())
    
    def initialize_random_traffic(self, num_vehicles):
        """ランダムな交通を初期化"""
        # ランダムな車両を生成
        self.vehicles = self.traffic_generator.generate_random_vehicles(num_vehicles)
        
        # 優先度と軌跡の初期化
        self.priorities = [0] * len(self.vehicles)
        self.trajectories = [[] for _ in range(len(self.vehicles))]
        
        # 初期軌跡を記録
        for i, vehicle in enumerate(self.vehicles):
            self.trajectories[i].append(vehicle.position.copy())
        
        # 初期速度の設定
        self._initialize_velocities()
        
        # 最適経路の計算
        self.compute_optimal_paths()

    def add_vehicle(self, vehicle):
        """新しい車両をシミュレーションに追加"""
        self.vehicles.append(vehicle)
        self.priorities.append(0)
        self.trajectories.append([vehicle.position.copy()])
        
        print(f"新しい車両{vehicle.id}を追加: {vehicle.type} タイプ, {vehicle.start}から{vehicle.goal}へ")
        
        # 新しい車両の経路を計算
        self._calculate_path_for_vehicle(len(self.vehicles) - 1)

    def _calculate_path_for_vehicle(self, vehicle_idx):
        """特定の車両の経路を計算（壁と障害物を考慮）"""
        vehicle = self.vehicles[vehicle_idx]
        if vehicle.reached:
            return
            
        # 車両のスポーン情報に基づいてVWを取得
        vw_vertices, vw_lines = self.vw_manager.get_vw_for_vehicle(
            getattr(vehicle, 'spawn_info', None)
        )
        
        # CarAgentを作成
        car_agent = CarAgent(vehicle.start, vehicle.goal)
        
        # 障害物線のリストを作成（壁とVWの両方を含める）
        obstacle_lines = []
        obstacle_lines.extend(vw_lines)
        
        # 壁を確実に追加
        if hasattr(self, 'wall_lines') and self.wall_lines:
            print(f"車両{vehicle_idx}: {len(self.wall_lines)}本の壁線分を追加")
            obstacle_lines.extend(self.wall_lines)
        else:
            # 壁情報が見つからない場合は直接取得
            wall_vertices, wall_lines = PathFinder.set_wall()
            print(f"車両{vehicle_idx}: 直接取得した{len(wall_lines)}本の壁線分を追加")
            obstacle_lines.extend(wall_lines)
            print(f"車両{vehicle_idx}: 壁情報をsetting.pyから直接取得しました")
        
        # 頂点リストを作成
        vertex_list = PathFinder.set_vertex_list(
            vw_vertices, 
            car_agent, 
            self.wall_vertices if hasattr(self, 'wall_vertices') else setting.wall_edge_list
        )
        
        # 可視グラフとダイクストラ法を使用して経路を計算する前に情報を表示
        print(f"車両{vehicle_idx}の経路計算: {len(obstacle_lines)}障害物線分, {len(vertex_list)}頂点")
        
        # 可視グラフを生成
        visibility_graph = PathFinder.visibility_graph(vertex_list, obstacle_lines)
        
        # 最短経路を計算
        try:
            shortest_path, shortest_length = PathFinder.dijkstra(visibility_graph)
            
            # 経路をウェイポイントに変換
            waypoints = [vertex_list[node_idx] for node_idx in shortest_path]
            
            # 車両に経路を設定
            vehicle.set_waypoints(waypoints)
            
            # 初期速度を設定
            self._initialize_velocity_for_vehicle(vehicle_idx)
            
            print(f"車両{vehicle_idx}の経路計算成功: 長さ={shortest_length:.2f}, ウェイポイント数={len(waypoints)}")
        except Exception as e:
            print(f"車両{vehicle_idx}の経路計算に失敗: {str(e)}")
            print("壁や障害物が経路を完全に塞いでいないか確認してください")

    def _initialize_velocity_for_vehicle(self, vehicle_idx):
        """特定の車両の初期速度を設定"""
        vehicle = self.vehicles[vehicle_idx]
        
        # 経路に沿った初期速度を計算
        if vehicle.waypoints and len(vehicle.waypoints) > 0:
            # 最初のウェイポイントへの方向
            wp = vehicle.waypoints[0]
            dx = wp[0] - vehicle.position[0]
            dy = wp[1] - vehicle.position[1]
            dist = np.sqrt(dx**2 + dy**2)
            
            if dist > 0:
                # 単位方向ベクトルに基本速度を乗算
                dir_x = dx / dist
                dir_y = dy / dist
                vehicle.velocity = [dir_x * self.config.base_speed, dir_y * self.config.base_speed]
                vehicle.target_velocity = vehicle.velocity.copy()
                
                # 進行方向の角度を保存
                vehicle.last_angle = np.arctan2(dir_y, dir_x)
        else:
            # ウェイポイントがない場合は目標位置への直線方向
            dx = vehicle.goal[0] - vehicle.position[0]
            dy = vehicle.goal[1] - vehicle.position[1]
            dist = np.sqrt(dx**2 + dy**2)
            
            if dist > 0:
                dir_x = dx / dist
                dir_y = dy / dist
                vehicle.velocity = [dir_x * self.config.base_speed, dir_y * self.config.base_speed]
                vehicle.target_velocity = vehicle.velocity.copy()
                
                # 進行方向の角度を保存
                vehicle.last_angle = np.arctan2(dir_y, dir_x)

    def setup_basic_scenario(self):
        """基本的な4台車両の直線シナリオ"""
        # 左からの車両
        self.vehicles.append(Vehicle(
            start=[324, 250],
            goal=[576, 250],
            vehicle_id=0,
            vehicle_type="straight"
        ))
        
        # 上からの車両
        self.vehicles.append(Vehicle(
            start=[450, 124],
            goal=[450, 376],
            vehicle_id=1,
            vehicle_type="straight"
        ))
        
        # 右からの車両
        self.vehicles.append(Vehicle(
            start=[576, 260],
            goal=[324, 260],
            vehicle_id=2,
            vehicle_type="straight"
        ))
        
        # 下からの車両
        self.vehicles.append(Vehicle(
            start=[460, 376],
            goal=[460, 124],
            vehicle_id=3,
            vehicle_type="straight"
        ))
        
        # 初期速度の設定
        self._initialize_velocities()
    
    def setup_mixed_turns_scenario(self):
        """右折と左折を含む混合シナリオ"""
        # 車両1: 左から来て右折（上方向へ）
        self.vehicles.append(Vehicle(
            start=[324, 250],
            goal=[450, 124],
            vehicle_id=0,
            vehicle_type="right_turn",
            spawn_direction="left",
            spawn_lane_idx=1
        ))
        
        # 車両2: 上から来て左折（右方向へ）
        self.vehicles.append(Vehicle(
            start=[450, 124],
            goal=[576, 260],
            vehicle_id=1,
            vehicle_type="left_turn",
            spawn_direction="top",
            spawn_lane_idx=1
        ))
        
        # 車両3: 右から来て直進
        self.vehicles.append(Vehicle(
            start=[576, 260],
            goal=[324, 260],
            vehicle_id=2,
            vehicle_type="straight",
            spawn_direction="right",
            spawn_lane_idx=1
        ))
        
        # 車両4: 下から来て右折（右方向へ）
        self.vehicles.append(Vehicle(
            start=[460, 376],
            goal=[576, 260],
            vehicle_id=3,
            vehicle_type="right_turn",
            spawn_direction="bottom",
            spawn_lane_idx=1
        ))
        
        # 初期速度の設定
        self._initialize_velocities()
        
    def setup_complex_crossing_scenario(self):
        """複雑な交差シナリオ - 全車両が交差点中央で交差"""
        # 交差点中心座標
        ic_x, ic_y = self.config.intersection_center
        
        # 車両1: 左側からの車両 - 対角線で右下へ
        self.vehicles.append(Vehicle(
            start=[324, 213],
            goal=[576, 253],
            vehicle_id=0,
            vehicle_type="diagonal",
            spawn_direction="left",
            spawn_lane_idx=0
        ))
        
        # 車両2: 右側からの車両 - 対角線で左上へ
        self.vehicles.append(Vehicle(
            start=[576, 253],
            goal=[324, 213],
            vehicle_id=1,
            vehicle_type="diagonal",
            spawn_direction="right",
            spawn_lane_idx=0
        ))
        
        # 車両3: 上側からの車両 - 対角線で左下へ
        self.vehicles.append(Vehicle(
            start=[430, 124],
            goal=[324, 253],
            vehicle_id=2,
            vehicle_type="diagonal",
            spawn_direction="top",
            spawn_lane_idx=0
        ))
        
        # 車両4: 下側からの車両 - 対角線で右上へ
        self.vehicles.append(Vehicle(
            start=[470, 342],
            goal=[576, 213],
            vehicle_id=3,
            vehicle_type="diagonal",
            spawn_direction="bottom",
            spawn_lane_idx=0
        ))
        
        # 初期速度の設定
        self._initialize_velocities()
    
    def _initialize_velocities(self):
        """車両の初期速度を設定"""
        for i, vehicle in enumerate(self.vehicles):
            self._initialize_velocity_for_vehicle(i)
    
    def compute_optimal_paths(self):
        """VWと壁を考慮した各車両の最適経路を計算"""
        # 壁の情報を明示的に設定
        self.wall_vertices, self.wall_lines = PathFinder.set_wall()
        
        # 壁の情報が正しく取得できたか確認
        if not self.wall_lines or len(self.wall_lines) == 0:
            print("警告: 壁の情報が取得できませんでした。setting.pyの壁設定を確認してください。")
            print("wall_edge_list:", setting.wall_edge_list)
            print("wall_line_list:", setting.wall_line_list)
        else:
            print(f"壁情報を取得しました: {len(self.wall_vertices)}頂点, {len(self.wall_lines)}線分")
        
        # 各車両の経路を計算
        for i, vehicle in enumerate(self.vehicles):
            if vehicle.reached:
                continue
            
            # 車両ごとの経路計算
            self._calculate_path_for_vehicle(i)
    
    def update_car_positions(self):
        """車両の位置を更新（ウェイポイント追従）"""
        for i, vehicle in enumerate(self.vehicles):
            if vehicle.reached:
                continue
                
            # 目標に到達したかチェック
            if vehicle.is_reached_goal():
                vehicle.reached = True
                vehicle.velocity = [0, 0]
                # print(f"車両 {i} が目標に到達しました。(ステップ: {self.time_steps})")
                continue
            
            # ウェイポイントがある場合
            if vehicle.waypoints and vehicle.current_waypoint_idx >= 0:
                # 現在のウェイポイントインデックス
                wp_idx = vehicle.current_waypoint_idx
                
                # 現在のウェイポイントがまだあれば
                if wp_idx < len(vehicle.waypoints):
                    current_wp = vehicle.waypoints[wp_idx]
                    
                    # ウェイポイントまでの距離を計算
                    dx = current_wp[0] - vehicle.position[0]
                    dy = current_wp[1] - vehicle.position[1]
                    dist_to_wp = np.sqrt(dx**2 + dy**2)
                    
                    # ウェイポイントに十分近づいたら、次のウェイポイントへ
                    if dist_to_wp < 5:  # 5pxを閾値としてウェイポイント到達判定
                        vehicle.current_waypoint_idx += 1
                        
                        # 次のウェイポイントがある場合
                        if vehicle.current_waypoint_idx < len(vehicle.waypoints):
                            next_wp = vehicle.waypoints[vehicle.current_waypoint_idx]
                            
                            # 次のウェイポイントへの方向ベクトルを計算
                            dx = next_wp[0] - vehicle.position[0]
                            dy = next_wp[1] - vehicle.position[1]
                            dist = np.sqrt(dx*dx + dy*dy)
                            
                            if dist > 0:
                                # 新しい方向ベクトル（正規化）
                                dir_x = dx / dist
                                dir_y = dy / dist
                                
                                # 車両タイプに応じた速度調整
                                speed = self.config.base_speed
                                if vehicle.type in ["right_turn", "left_turn"]:
                                    speed *= 0.8  # 曲がる時は速度を少し落とす
                                
                                # 新しい速度ベクトルを設定
                                vehicle.velocity = [dir_x * speed, dir_y * speed]
                                vehicle.target_velocity = [dir_x * speed, dir_y * speed]
                                
                                # 角度を更新
                                vehicle.last_angle = np.arctan2(dir_y, dir_x)
                    
                    # まだウェイポイントに到達していない場合
                    else:
                        # 現在のウェイポイントへの方向ベクトルを計算
                        if dist_to_wp > 0:
                            # 方向ベクトル（正規化）
                            dir_x = dx / dist_to_wp
                            dir_y = dy / dist_to_wp
                            
                            # 速度を方向ベクトルに合わせて調整
                            if not vehicle.slowed_down:
                                # 車両タイプに応じた速度調整
                                speed = self.config.base_speed
                                if vehicle.type in ["right_turn", "left_turn"]:
                                    speed *= 0.8  # 曲がる時は速度を少し落とす
                                
                                vehicle.velocity = [dir_x * speed, dir_y * speed]
                            else:
                                # アドホック回避中は速度の大きさはそのままで方向だけ調整
                                current_speed = np.linalg.norm(vehicle.velocity)
                                vehicle.velocity = [dir_x * current_speed, dir_y * current_speed]
                            
                            # 目標速度の更新
                            vehicle.target_velocity = [dir_x * self.config.base_speed, dir_y * self.config.base_speed]
                            
                            # 走行中は常に最後の角度を更新
                            vehicle.last_angle = np.arctan2(dir_y, dir_x)
                else:
                    # 全てのウェイポイントを通過した場合、最終目標へ直線移動
                    dx = vehicle.goal[0] - vehicle.position[0]
                    dy = vehicle.goal[1] - vehicle.position[1]
                    dist = np.sqrt(dx*dx + dy*dy)
                    
                    if dist > 0:
                        # 方向ベクトル（正規化）
                        dir_x = dx / dist
                        dir_y = dy / dist
                        
                        # 速度を方向ベクトルに合わせて調整
                        if not vehicle.slowed_down:
                            speed = self.config.base_speed
                            vehicle.velocity = [dir_x * speed, dir_y * speed]
                        else:
                            # 速度の大きさはそのままで方向だけ調整
                            speed = np.linalg.norm(vehicle.velocity)
                            vehicle.velocity = [dir_x * speed, dir_y * speed]
                        
                        # 目標速度も更新
                        vehicle.target_velocity = [dir_x * self.config.base_speed, dir_y * self.config.base_speed]
                        
                        # 走行中は常に最後の角度を更新
                        vehicle.last_angle = np.arctan2(dir_y, dir_x)
            
            # ウェイポイントがない場合（単純な直線移動）
            else:
                # 目標位置への方向
                dx = vehicle.goal[0] - vehicle.position[0]
                dy = vehicle.goal[1] - vehicle.position[1]
                dist = np.sqrt(dx*dx + dy*dy)
                
                if dist > 0:
                    # 方向ベクトル（正規化）
                    dir_x = dx / dist
                    dir_y = dy / dist
                    
                    # 速度を設定
                    if not vehicle.slowed_down:
                        vehicle.velocity = [dir_x * self.config.base_speed, dir_y * self.config.base_speed]
                    
                    # 目標速度の更新
                    vehicle.target_velocity = [dir_x * self.config.base_speed, dir_y * self.config.base_speed]
                    
                    # 走行中は常に最後の角度を更新
                    vehicle.last_angle = np.arctan2(dir_y, dir_x)
            
            # 車両の位置を更新
            vehicle.update_position()
            
            # 軌跡を記録
            self.trajectories[i].append(vehicle.position.copy())
    
    def update_priorities(self):
        """車両の優先度を更新（車両タイプ、位置、待ち時間に基づく）"""
        for i, vehicle in enumerate(self.vehicles):
            if vehicle.reached:
                self.priorities[i] = -999  # 到達済み車両は最低優先度
                continue
                
            # 基本スコアをリセット
            self.priorities[i] = 0
            
            # 交差点までの距離に基づく優先度
            pos = np.array(vehicle.position)
            intersection_center = np.array(self.config.intersection_center)
            dist_to_intersection = np.linalg.norm(pos - intersection_center)
            
            # 交差点内または交差点に近い車両の優先度を高く設定
            in_intersection = self._is_in_intersection(pos)
            if in_intersection:
                self.priorities[i] += 200  # 交差点内の車両は高優先
            elif dist_to_intersection < 50:
                self.priorities[i] += 150 - dist_to_intersection  # 距離が近いほど優先
                
            # 速度ベースの優先度 - 高速で移動中の車両は流れを維持
            speed = np.linalg.norm(vehicle.velocity)
            if speed > self.config.base_speed * 0.8:
                self.priorities[i] += 50  # 良好な速度で走行中の車両を優先
                
            # 待機時間ベースの公平性
            if vehicle.stopped_time > 5:
                self.priorities[i] += vehicle.stopped_time * 3  # 長く待機した車両を優先
                
            # 交差点との相対位置を計算
            vel = np.array(vehicle.velocity)
            speed = np.linalg.norm(vel)
            
            if speed > 0:
                # 進行方向の単位ベクトル
                direction = vel / speed
                
                # 交差点中心への相対ベクトル
                to_intersection = intersection_center - pos
                
                # 交差点が車両の前方にあるかどうか（内積で判定）
                forward_dist = np.dot(to_intersection, direction)
                
                # 交差点までの横方向距離（垂直成分）
                lateral_dist = np.linalg.norm(to_intersection - forward_dist * direction)
                
                # 横方向距離が小さい（＝交差点を通過する予定）かつ
                # 前方距離が正（＝交差点がまだ前方にある）場合
                if lateral_dist < self.config.road_width/2 and forward_dist > 0:
                    # 交差点に近いほど優先度を高く
                    self.priorities[i] += 100 - min(100, forward_dist)
                
                # すでに交差点を通過中の車両は最優先
                if in_intersection:
                    # 交差点の中心を過ぎた車両はより優先
                    if forward_dist < 0:
                        self.priorities[i] += 100
            
            # 長時間停止している車両の優先度を上げる（デッドロック防止）
            if vehicle.is_almost_stopped():
                vehicle.stopped_time += 1
                if vehicle.stopped_time > 10:
                    self.priorities[i] += vehicle.stopped_time * 2
            else:
                vehicle.stopped_time = 0
            
            # 既に減速している車両は優先度を下げる
            if vehicle.slowed_down:
                self.priorities[i] -= 50
            
            # 車両のタイプに基づく優先度調整
            if vehicle.type == "straight":
                self.priorities[i] += 30  # 直進車両は優先
            elif vehicle.type == "right_turn":
                self.priorities[i] += 10  # 右折は次に優先
            # 左折は追加優先度なし
    
    def _is_in_intersection(self, position):
        """位置が交差点内かどうかを判定"""
        ic_x, ic_y = self.config.intersection_center
        half_size = self.config.intersection_size / 2
        
        return (ic_x - half_size <= position[0] <= ic_x + half_size and 
                ic_y - half_size <= position[1] <= ic_y + half_size)
    
    def adjust_velocity(self, predictions):
        """衝突予測に基づいて車両の速度を調整（アドホック回避）- 改良版カウント機能付き"""
        # 衝突予測がない場合は早期リターン
        if not predictions:
            # 減速中の車両を通常速度に戻す
            self._normalize_all_speeds()
            return 0  # アドホック回避なし
        
        # 優先度を更新
        self.update_priorities()
        
        # 減速が必要な車両を記録
        cars_to_slow_down = set()
        
        # アドホック回避回数
        adhoc_count = 0
        
        # 現在のステップ
        current_time = self.time_steps
        
        # last_adhoc_timestampsがなければ初期化
        if not hasattr(self, 'last_adhoc_timestamps'):
            self.last_adhoc_timestamps = {}
        
        # 予測衝突に基づく速度調整
        for prediction in predictions:
            i, j = prediction["vehicles"]
            time_step = prediction["time_step"]
            in_intersection = prediction.get("in_intersection", False)
            
            # 既に到達している車両はスキップ
            if self.vehicles[i].reached or self.vehicles[j].reached:
                continue
            
            # 車両ペアの識別子を作成（順序に依存しないようにソート）
            pair_id = tuple(sorted([i, j]))
            
            # 衝突予測時間に基づく緊急度計算
            time_urgency = max(0.2, min(1.0, 1.0 / (time_step + 1)))
            
            # 同じペアの最後の衝突回避から一定時間（例:20ステップ）経過していない場合はカウントしない
            if pair_id in self.last_adhoc_timestamps and \
            current_time - self.last_adhoc_timestamps[pair_id] < 20:
                # 速度調整は行うが、新規カウントはしない
                if in_intersection:
                    self._handle_intersection_collision(i, j, time_urgency, cars_to_slow_down)
                else:
                    self._handle_regular_collision(i, j, time_urgency, cars_to_slow_down)
                continue
            
            # 交差点内での衝突処理
            if in_intersection:
                adhoc_occurred = self._handle_intersection_collision(i, j, time_urgency, cars_to_slow_down)
                if adhoc_occurred:
                    # タイムスタンプを更新
                    self.last_adhoc_timestamps[pair_id] = current_time
                    adhoc_count += 1
                    print(f"ステップ {self.time_steps}: 交差点内で車両{i}と{j}間のアドホック回避をカウント")
            else:
                # 交差点外での衝突処理
                adhoc_occurred = self._handle_regular_collision(i, j, time_urgency, cars_to_slow_down)
                if adhoc_occurred:
                    # タイムスタンプを更新
                    self.last_adhoc_timestamps[pair_id] = current_time
                    adhoc_count += 1
                    print(f"ステップ {self.time_steps}: 交差点外で車両{i}と{j}間のアドホック回避をカウント")
        
        # 減速していない車両の速度を正常化
        self._normalize_speeds(cars_to_slow_down)

        # 長時間減速状態の車両を強制的に加速
        for i, vehicle in enumerate(self.vehicles):
            if not vehicle.reached and vehicle.slowed_down and vehicle.stopped_time > 20:
                print(f"ステップ {self.time_steps}: 車両{i}が長時間減速状態です。強制加速します。")
                self._force_restart_vehicle(i)
            
        return adhoc_count
    
    def _handle_intersection_collision(self, i, j, time_urgency, cars_to_slow_down):
        """交差点内での衝突処理（車両タイプを考慮）"""
        # 車両タイプに基づく優先度判断
        car_i_type = self.vehicles[i].type
        car_j_type = self.vehicles[j].type
        
        # 直進と右折の特殊ケース
        if (car_i_type == "straight" and car_j_type == "right_turn"):
            # 直進車優先、右折車は適度に減速
            self._apply_moderate_deceleration(j, cars_to_slow_down)
            # print(f"ステップ {self.time_steps}: 交差点内で直進車両{i}優先、右折車両{j}は適度に減速")
            return True
        elif (car_j_type == "straight" and car_i_type == "right_turn"):
            # 直進車優先、右折車は適度に減速
            self._apply_moderate_deceleration(i, cars_to_slow_down)
            # print(f"ステップ {self.time_steps}: 交差点内で直進車両{j}優先、右折車両{i}は適度に減速")
            return True
        
        # 直進が優先、次に右折、左折は最後
        type_priority = {"straight": 3, "right_turn": 2, "left_turn": 1, "diagonal": 2}
        type_priority_i = type_priority.get(car_i_type, 0)
        type_priority_j = type_priority.get(car_j_type, 0)
        
        # 車両タイプの優先度差がある場合はそちらを優先
        if type_priority_i != type_priority_j:
            if type_priority_i > type_priority_j:
                # 車両jが減速
                self._apply_deceleration(j, time_urgency, cars_to_slow_down)
                # print(f"ステップ {self.time_steps}: 交差点内で車両{j}({car_j_type})が車両{i}({car_i_type})に譲ります")
                return True
            else:
                # 車両iが減速
                self._apply_deceleration(i, time_urgency, cars_to_slow_down)
                # print(f"ステップ {self.time_steps}: 交差点内で車両{i}({car_i_type})が車両{j}({car_j_type})に譲ります")
                return True
        else:
            # タイプ優先度が同じなら通常の優先度で判断
            if self.priorities[i] > self.priorities[j]:
                # 車両jが減速
                self._apply_deceleration(j, time_urgency, cars_to_slow_down)
                return True
            else:
                # 車両iが減速
                self._apply_deceleration(i, time_urgency, cars_to_slow_down)
                return True
    
    def _handle_regular_collision(self, i, j, time_urgency, cars_to_slow_down):
        """交差点外での衝突処理"""
        # 車両の位置と速度ベクトルを取得
        v1 = self.vehicles[i]
        v2 = self.vehicles[j]
        
        pos_i = np.array(v1.position)
        pos_j = np.array(v2.position)
        vel_i = np.array(v1.velocity)
        vel_j = np.array(v2.velocity)
        
        speed_i = np.linalg.norm(vel_i)
        speed_j = np.linalg.norm(vel_j)
        
        # 両方とも十分な速度で動いているか確認
        if speed_i < 0.01 or speed_j < 0.01:
            # どちらかが極めて遅い場合、停止中の車両を少し動かす
            if speed_i < 0.01 and not v1.reached:
                self._gently_accelerate_stopped_vehicle(i)
            
            if speed_j < 0.01 and not v2.reached:
                self._gently_accelerate_stopped_vehicle(j)
            
            return False
        
        # 進行方向ベクトル（正規化）
        dir_i = vel_i / speed_i
        dir_j = vel_j / speed_j
        
        # 相対位置ベクトル
        rel_pos = pos_j - pos_i
        dist = np.linalg.norm(rel_pos)
        
        # 車両iから見た車両jの前方距離（内積で計算）
        forward_dist_i = np.dot(rel_pos, dir_i)
        
        # 車両jから見た車両iの前方距離
        rel_pos_j = -rel_pos
        forward_dist_j = np.dot(rel_pos_j, dir_j)
        
        # 距離に基づく緊急度調整
        distance_factor = max(0.5, min(1.0, 30.0 / dist)) if dist > 0 else 1.0
        adjusted_urgency = time_urgency * distance_factor
        
        # 経路に基づく優先度判断
        if forward_dist_i > 0 and forward_dist_j <= 0:
            # 車両jは車両iの前方にあり、車両iは車両jの後方にある
            # 車両iが減速
            self._apply_deceleration(i, adjusted_urgency, cars_to_slow_down)
            # print(f"ステップ {self.time_steps}: 車両{i}が前方の車両{j}に対して減速")
            return True
        elif forward_dist_j > 0 and forward_dist_i <= 0:
            # 車両iは車両jの前方にあり、車両jは車両iの後方にある
            # 車両jが減速
            self._apply_deceleration(j, adjusted_urgency, cars_to_slow_down)
            # print(f"ステップ {self.time_steps}: 車両{j}が前方の車両{i}に対して減速")
            return True
        else:
            # 両方とも相手の前方または両方とも相手の後方にいる場合
            # （交差する経路や並行する経路の場合）
            
            # 優先度に基づく判断
            if self.priorities[i] > self.priorities[j]:
                # 車両jが減速
                self._apply_deceleration(j, adjusted_urgency, cars_to_slow_down)
                # print(f"ステップ {self.time_steps}: 優先度に基づき車両{j}が車両{i}に譲ります")
                return True
            else:
                # 車両iが減速
                self._apply_deceleration(i, adjusted_urgency, cars_to_slow_down)
                # print(f"ステップ {self.time_steps}: 優先度に基づき車両{i}が車両{j}に譲ります")
                return True
    
    def _gently_accelerate_stopped_vehicle(self, car_idx):
        """停止中の車両をわずかに動かす（デッドロック防止）"""
        vehicle = self.vehicles[car_idx]
        target_vel = vehicle.target_velocity
        target_speed = np.linalg.norm(target_vel)
        
        if target_speed > 0:
            # 目標方向に合わせて最低速度で再始動
            direction = [target_vel[0]/target_speed, target_vel[1]/target_speed]
            restart_speed = 0.3  # 最低再始動速度
            
            vehicle.velocity = [direction[0] * restart_speed, direction[1] * restart_speed]
            # print(f"ステップ {self.time_steps}: 停止中の車両{car_idx}を再始動")
    
    def _apply_deceleration(self, car_idx, time_urgency, cars_to_slow_down):
        """車両に減速を適用する標準化されたメソッド"""
        vehicle = self.vehicles[car_idx]
        speed = np.linalg.norm(vehicle.velocity)
        
        if speed < 0.1:
            return  # 既に停止中なら何もしない
        
        # 車両をスローダウン対象として記録
        cars_to_slow_down.add(car_idx)
        
        # 緊急度に基づく減速係数計算
        slow_factor = max(0.3, 1.0 - (time_urgency * 0.7))
        
        # 車両タイプに応じた調整
        if vehicle.type == "right_turn":
            slow_factor *= 0.9  # 右折はやや慎重に
        elif vehicle.type == "left_turn":
            slow_factor *= 0.8  # 左折はさらに慎重に
        
        # 方向ベクトルを維持
        direction = [vehicle.velocity[0] / speed, vehicle.velocity[1] / speed]
        
        # 新しい速度を計算（最低速度を保証）
        new_speed = speed * slow_factor
        new_speed = max(0.2, new_speed)  # 最低0.2の速度を維持
        
        # 速度を更新
        vehicle.velocity[0] = direction[0] * new_speed
        vehicle.velocity[1] = direction[1] * new_speed
        vehicle.slowed_down = True
        
        # print(f"ステップ {self.time_steps}: 車両{car_idx}が速度調整 ({speed:.2f} → {new_speed:.2f})")
    
    def _apply_moderate_deceleration(self, car_idx, cars_to_slow_down):
        """過度な減速を避けつつ適度な速度調整を行う"""
        vehicle = self.vehicles[car_idx]
        speed = np.linalg.norm(vehicle.velocity)
        
        if speed < 0.5:
            # 既に遅い場合は最低速度を保証
            direction = [vehicle.velocity[0] / (speed or 1), vehicle.velocity[1] / (speed or 1)]
            min_speed = self.config.base_speed * 0.4  # 最低速度として基本速度の40%を確保
            
            vehicle.velocity[0] = direction[0] * min_speed
            vehicle.velocity[1] = direction[1] * min_speed
            vehicle.slowed_down = True
            cars_to_slow_down.add(car_idx)
            
            # print(f"ステップ {self.time_steps}: 車両{car_idx}の最低速度を確保 ({speed:.2f} → {min_speed:.2f})")
            return
        
        # 適度な減速係数（60%の速度を維持）
        slow_factor = 0.6
        
        # 方向ベクトルを維持
        direction = [vehicle.velocity[0] / speed, vehicle.velocity[1] / speed]
        
        # 新しい速度を計算（最低速度を保証）
        new_speed = max(self.config.base_speed * 0.4, speed * slow_factor)
        
        # 速度を更新
        vehicle.velocity[0] = direction[0] * new_speed
        vehicle.velocity[1] = direction[1] * new_speed
        vehicle.slowed_down = True
        
        # 減速対象としてマーク
        cars_to_slow_down.add(car_idx)
        
        # print(f"ステップ {self.time_steps}: 車両{car_idx}に適度な減速を適用 ({speed:.2f} → {new_speed:.2f})")
    
    def _normalize_speeds(self, cars_to_slow_down):
        """減速が不要な車両の速度を正常化"""
        for i, vehicle in enumerate(self.vehicles):
            # 到達済み、減速されていない、または減速が必要な車両はスキップ
            if vehicle.reached or not vehicle.slowed_down or i in cars_to_slow_down:
                continue
                
            # 目標速度を取得
            target_vel = vehicle.target_velocity
            target_speed = np.linalg.norm(target_vel)
            
            if target_speed < 0.001:
                continue  # 目標速度がほぼゼロならスキップ
                
            # 現在の速度
            current_speed = np.linalg.norm(vehicle.velocity)
            
            # 目標速度より遅い場合は徐々に加速
            if current_speed < target_speed:
                # 加速率（目標速度の10%ずつ加速）
                acceleration_rate = 0.1
                
                # 新しい速度（目標速度を上限）
                new_speed = min(current_speed + (target_speed * acceleration_rate), target_speed)
                
                # 方向を取得
                if current_speed > 0:
                    direction = [vehicle.velocity[0] / current_speed, vehicle.velocity[1] / current_speed]
                else:
                    direction = [target_vel[0] / target_speed, target_vel[1] / target_speed]
                
                # 速度を更新
                vehicle.velocity[0] = direction[0] * new_speed
                vehicle.velocity[1] = direction[1] * new_speed
                
                # 目標速度に十分近づいたら減速状態を解除
                if new_speed >= target_speed * 0.95:
                    vehicle.slowed_down = False
                    # print(f"ステップ {self.time_steps}: 車両{i}が通常速度に戻りました")
    
    def _normalize_all_speeds(self):
        """全車両の速度を正常化（減速状態を解除）"""
        for i, vehicle in enumerate(self.vehicles):
            if vehicle.reached or not vehicle.slowed_down:
                continue
                    
            # 目標速度を取得
            target_vel = vehicle.target_velocity
            target_speed = np.linalg.norm(target_vel)
            
            if target_speed < 0.001:
                continue  # 目標速度がほぼゼロならスキップ
                    
            # 現在の速度
            current_speed = np.linalg.norm(vehicle.velocity)
            
            # 目標速度より遅い場合は徐々に加速
            if current_speed < target_speed:
                # 加速率を上げる（10%から20%に）
                acceleration_rate = 0.2
                
                # 新しい速度（目標速度を上限）
                new_speed = min(current_speed + (target_speed * acceleration_rate), target_speed)
                
                # 方向を取得
                if current_speed > 0:
                    direction = [vehicle.velocity[0] / current_speed, vehicle.velocity[1] / current_speed]
                else:
                    direction = [target_vel[0] / target_speed, target_vel[1] / target_speed]
                
                # 速度を更新
                vehicle.velocity[0] = direction[0] * new_speed
                vehicle.velocity[1] = direction[1] * new_speed
                
                # 目標速度に十分近づいたら減速状態を解除
                if new_speed >= target_speed * 0.8:  # 80%まで回復したら解除（以前は95%）
                    vehicle.slowed_down = False
                    # print(f"ステップ {self.time_steps}: 車両{i}が通常速度に戻りました")
    
    def _force_restart_vehicle(self, car_idx):
        """車両を強制的に再始動"""
        vehicle = self.vehicles[car_idx]
        
        # 目標方向への速度を設定
        if vehicle.waypoints and 0 <= vehicle.current_waypoint_idx < len(vehicle.waypoints):
            # 次のウェイポイントへの方向
            wp = vehicle.waypoints[vehicle.current_waypoint_idx]
            dx = wp[0] - vehicle.position[0]
            dy = wp[1] - vehicle.position[1]
        else:
            # 目標位置への方向
            dx = vehicle.goal[0] - vehicle.position[0]
            dy = vehicle.goal[1] - vehicle.position[1]
        
        dist = np.sqrt(dx*dx + dy*dy)
        if dist > 0:
            dir_x = dx / dist
            dir_y = dy / dist
            
            # 通常速度の80%で再始動
            restart_speed = self.config.base_speed * 0.8
            
            # 速度を更新
            vehicle.velocity = [dir_x * restart_speed, dir_y * restart_speed]
            vehicle.target_velocity = [dir_x * self.config.base_speed, dir_y * self.config.base_speed]
            vehicle.slowed_down = False
            vehicle.stopped_time = 0
            
            # print(f"ステップ {self.time_steps}: 車両{car_idx}を強制再始動 (速度: {restart_speed:.2f})")
    
    def check_for_deadlocks(self):
        """デッドロックを検出し解決（単一車両の停止も処理）"""
        # 長時間停止している車両を特定
        stopped_vehicles = []
        
        for i, vehicle in enumerate(self.vehicles):
            if vehicle.reached:
                continue
                    
            if vehicle.is_almost_stopped() and vehicle.stopped_time > 15:
                stopped_vehicles.append(i)
                print(f"停止中の車両 {i}: 位置={vehicle.position}, ウェイポイント idx={vehicle.current_waypoint_idx}")
                
                # 現在のウェイポイントまでの距離をデバッグ表示
                if vehicle.waypoints and 0 <= vehicle.current_waypoint_idx < len(vehicle.waypoints):
                    wp = vehicle.waypoints[vehicle.current_waypoint_idx]
                    dist = np.sqrt((vehicle.position[0] - wp[0])**2 + (vehicle.position[1] - wp[1])**2)
                    print(f"  現在のウェイポイントまでの距離: {dist:.2f}")
        
        # 複数車両のデッドロック処理
        if len(stopped_vehicles) >= 2:
            # print(f"ステップ {self.time_steps}: 複数車両デッドロック検出 - {stopped_vehicles}")
            
            # 最も優先度の高い車両を特定
            highest_priority_idx = max(stopped_vehicles, key=lambda idx: self.priorities[idx])
            
            # その車両を再始動
            self._force_restart_vehicle(highest_priority_idx)
            
            # 他の停止車両も緩やかに再始動
            for idx in stopped_vehicles:
                if idx != highest_priority_idx:
                    self._gently_accelerate_stopped_vehicle(idx)
        
        # 単一車両の停止処理を追加
        elif len(stopped_vehicles) == 1:
            vehicle_idx = stopped_vehicles[0]
            # print(f"ステップ {self.time_steps}: 単一車両停止検出 - 車両{vehicle_idx}")
            
            # 現在の状況を詳細に分析
            vehicle = self.vehicles[vehicle_idx]
            
            # ウェイポイントが適切か確認
            if vehicle.waypoints and 0 <= vehicle.current_waypoint_idx < len(vehicle.waypoints):
                # 現在のウェイポイントを少しスキップして次に進む
                print(f"  車両{vehicle_idx}のウェイポイントをスキップします")
                vehicle.current_waypoint_idx += 1
                
                # 速度を再設定
                self._force_restart_vehicle(vehicle_idx)
            else:
                # ウェイポイントがない場合は目標位置へ直接向かう
                self._force_direct_to_goal(vehicle_idx)

    def _force_direct_to_goal(self, car_idx):
        """ウェイポイントを無視して直接目標に向かわせる"""
        vehicle = self.vehicles[car_idx]
        
        # 目標位置への方向ベクトル
        dx = vehicle.goal[0] - vehicle.position[0]
        dy = vehicle.goal[1] - vehicle.position[1]
        dist = np.sqrt(dx*dx + dy*dy)
        
        if dist > 0:
            dir_x = dx / dist
            dir_y = dy / dist
            
            # 速度を設定（通常よりやや遅く）
            speed = self.config.base_speed * 0.6
            
            vehicle.velocity = [dir_x * speed, dir_y * speed]
            vehicle.target_velocity = [dir_x * self.config.base_speed, dir_y * self.config.base_speed]
            vehicle.slowed_down = False
            vehicle.stopped_time = 0
            
            # ウェイポイント追従をキャンセル
            vehicle.current_waypoint_idx = -1
            # print(f"ステップ {self.time_steps}: 車両{car_idx}を目標に直接向かわせます (距離: {dist:.2f})")

    def check_collisions(self):
        """シミュレーション中の実際の衝突をチェック"""
        vehicle_collisions = self.collision_detector.detect_collisions(self.vehicles)
        
        # 壁との衝突も検出
        # wall_collisions = self.check_wall_collisions()
        
        # 両方の衝突数を合計して返す
        return vehicle_collisions
        # + wall_collisions
    
    def run_simulation(self):
        """シミュレーションを実行"""
        self.time_steps = 0
        self.collision_count = 0
        self.adhoc_count = 0
        
        while not all(vehicle.reached for vehicle in self.vehicles) and self.time_steps < self.config.max_steps:
            self.time_steps += 1
            
            # デッドロック検出・解決
            self.check_for_deadlocks()
            
            # アドホック回避が有効な場合
            if self.use_adhoc:
                # 衝突予測
                collision_predictions = self.collision_detector.predict_collisions(self.vehicles)
                
                if collision_predictions:
                    # 速度調整とアドホック回避カウント
                    adhoc_occurred = self.adjust_velocity(collision_predictions)
                    self.adhoc_count += adhoc_occurred
                else:
                    # 衝突がなければ減速した車両を元の速度に戻す
                    self._normalize_all_speeds()
            
            # 車両の位置を更新
            self.update_car_positions()
            
            # 衝突検出
            new_collisions = self.check_collisions()
            
            if new_collisions > 0:
                # print(f"ステップ {self.time_steps}: {new_collisions}件の衝突発生")
                self.collision_count += new_collisions
        
        # 結果表示
        adhoc_status = "あり" if self.use_adhoc else "なし"
        print(f"\nシミュレーション終了 (アドホック回避: {adhoc_status})")
        print(f"  ステップ数: {self.time_steps}")
        print(f"  衝突回数: {self.collision_count}")
        print(f"  アドホック回避回数: {self.adhoc_count}")
        print(f"  全車両到達: {all(vehicle.reached for vehicle in self.vehicles)}")
        
        # 結果を返す
        return {
            "steps": self.time_steps,
            "collisions": self.collision_count,
            "adhoc_count": self.adhoc_count,
            "all_reached": all(vehicle.reached for vehicle in self.vehicles)
        }
    
    def run_simulation_with_continuous_traffic(self, max_steps=None, spawn_rate=0.1, max_vehicles=20):
        """継続的な車両生成を含むシミュレーションを実行"""
        if max_steps is None:
            max_steps = self.config.max_steps
        
        # 初期状態を記録
        self.time_steps = 0
        self.collision_count = 0
        self.adhoc_count = 0
        
        # シミュレーションループ
        while self.time_steps < max_steps:
            self.time_steps += 1
            
            # 混雑度に応じた生成率調整
            adjusted_rate = self.traffic_generator.adjust_spawn_rate_by_congestion(
                self, base_rate=spawn_rate
            )
            
            # 新しい車両の生成を試みる
            new_vehicle = self.traffic_generator.generate_vehicle_if_needed(
                self, spawn_rate=adjusted_rate, max_vehicles=max_vehicles
            )
            
            if new_vehicle is not None:
                # 新しい車両をシミュレーションに追加
                self.add_vehicle(new_vehicle)
            
            # デッドロック検出・解決
            self.check_for_deadlocks()
            
            # アドホック回避
            if self.use_adhoc:
                collision_predictions = self.collision_detector.predict_collisions(self.vehicles)
                
                if collision_predictions:
                    adhoc_occurred = self.adjust_velocity(collision_predictions)
                    self.adhoc_count += adhoc_occurred
                else:
                    self._normalize_all_speeds()
            
            # 車両の位置を更新
            self.update_car_positions()
            
            # 衝突検出
            new_collisions = self.check_collisions()
            
            if new_collisions > 0:
                self.collision_count += new_collisions
            
            # 到達した車両の処理（必要に応じてここで車両を削除できる）
            self._handle_arrived_vehicles()
            
            # 終了条件：すべての車両が到達し、新しい車両が生成される可能性がなくなった場合
            if all(v.reached for v in self.vehicles) and len(self.vehicles) >= max_vehicles:
                break
        
        # 結果出力
        print(f"\nシミュレーション終了")
        print(f"  ステップ数: {self.time_steps}")
        print(f"  衝突回数: {self.collision_count}")
        print(f"  アドホック回避回数: {self.adhoc_count}")
        print(f"  生成車両数: {self.traffic_generator.next_vehicle_id}")
        print(f"  到達車両数: {sum(1 for v in self.vehicles if v.reached)}")
        
        # 結果を返す
        return {
            "steps": self.time_steps,
            "collisions": self.collision_count,
            "adhoc_count": self.adhoc_count,
            "vehicles_generated": self.traffic_generator.next_vehicle_id,
            "vehicles_arrived": sum(1 for v in self.vehicles if v.reached)
        }
        
    def _handle_arrived_vehicles(self):
        """到達した車両の処理"""
        # 現在の実装では車両を削除せずに維持
        # 必要に応じて到達した車両をリストから削除する機能を追加可能
        for i, vehicle in enumerate(self.vehicles):
            if vehicle.is_reached_goal() and not vehicle.reached:
                vehicle.reached = True
                vehicle.velocity = [0, 0]
                print(f"車両{i} ({vehicle.type}) が時刻 {self.time_steps} に到達")

    def check_wall_collisions(self):
        """壁との衝突をチェック"""
        collisions = 0
        
        for vehicle in self.vehicles:
            if vehicle.reached:
                continue
                
            # 壁との最短距離を計算
            min_distance = float('inf')
            for wall_line in self.wall_lines:
                p1 = np.array(wall_line[0])
                p2 = np.array(wall_line[1])
                p = np.array(vehicle.position)
                
                # 線分と点の最短距離を計算
                line_vec = p2 - p1
                line_len = np.linalg.norm(line_vec)
                line_unit_vec = line_vec / line_len if line_len > 0 else line_vec
                
                # 点から線分の始点へのベクトル
                pv = p - p1
                
                # 射影長
                proj_len = np.dot(pv, line_unit_vec)
                
                # 射影点の計算
                if proj_len <= 0:
                    # 線分の始点が最近点
                    closest = p1
                elif proj_len >= line_len:
                    # 線分の終点が最近点
                    closest = p2
                else:
                    # 線分上の点が最近点
                    closest = p1 + line_unit_vec * proj_len
                
                # 最短距離を計算
                dist = np.linalg.norm(p - closest)
                
                if dist < min_distance:
                    min_distance = dist
                    
            # 車両の幅/2より距離が短い場合、衝突と判定
            if min_distance < self.config.car_width / 2:
                collisions += 1
                # 壁との衝突が検出された場合、車両を停止させる
                vehicle.velocity = [0, 0]
                vehicle.slowed_down = True
                print(f"警告: 車両{vehicle.id}が壁と衝突！")
        
        return collisions

class VW:
    """Virtual Wallを管理するクラス"""
    
    @staticmethod
    def set_virtual_wall(GA_list, VWsize=setting.VWsize):
        """
        遺伝的アルゴリズムの結果に対応したVWを設置する関数
        VWの4つの頂点のlistと障害物の線分のlistを返す
        """
        # 配列構造の調整（余分な次元を削除）
        if len(GA_list) == 1 and isinstance(GA_list[0], list):
            GA_list = GA_list[0]
        
        # VWグリッド配置を1行で出力
        print(f"VWグリッド配置: {np.array(GA_list).tolist()}")
        
        size = VWsize
        field_x = setting.VWfield_x
        field_y = setting.VWfield_y
        obstacles_vertex_list = []
        obstacles_line_list = []
        total_num_obstacles = 0
        
        for index, oneDivisionList in enumerate(GA_list):
            for twoDivisionIndex, deploy_check in enumerate(oneDivisionList):
                if isinstance(deploy_check, list):
                    # 三次元配列の場合（二段階VW）の処理
                    for i, row in enumerate(deploy_check):
                        for j, value in enumerate(row):
                            if value >= 1:
                                total_num_obstacles += 1
                                inner_size = size / len(deploy_check)
                                VW_LeftUp = [
                                    field_x + (size * twoDivisionIndex) + (inner_size * j), 
                                    field_y + (size * index) + (inner_size * i)
                                ]
                                VW_LeftDown = [VW_LeftUp[0], VW_LeftUp[1] + inner_size]
                                VW_RightUp = [VW_LeftUp[0] + inner_size, VW_LeftUp[1]]
                                VW_RightDown = [VW_LeftUp[0] + inner_size, VW_LeftUp[1] + inner_size]
                                
                                obstacles_vertex_list.extend([VW_LeftUp, VW_LeftDown, VW_RightUp, VW_RightDown])
                                obstacles_line_list.extend([
                                    [VW_LeftUp, VW_LeftDown], 
                                    [VW_LeftUp, VW_RightUp], 
                                    [VW_RightUp, VW_RightDown], 
                                    [VW_RightDown, VW_LeftDown]
                                ])
                elif deploy_check >= 1:
                    # 二次元配列の場合（一段階VW）の処理
                    total_num_obstacles += 1
                    VW_LeftUp = [field_x + (size * twoDivisionIndex), field_y + (size * index)]
                    VW_LeftDown = [VW_LeftUp[0], VW_LeftUp[1] + size]
                    VW_RightUp = [VW_LeftUp[0] + size, VW_LeftUp[1]]
                    VW_RightDown = [VW_LeftUp[0] + size, VW_LeftUp[1] + size]
                    
                    obstacles_vertex_list.extend([VW_LeftUp, VW_LeftDown, VW_RightUp, VW_RightDown])
                    obstacles_line_list.extend([
                        [VW_LeftUp, VW_LeftDown], 
                        [VW_LeftUp, VW_RightUp], 
                        [VW_RightUp, VW_RightDown], 
                        [VW_RightDown, VW_LeftDown]
                    ])
        
        return obstacles_vertex_list, obstacles_line_list

    def single_GA_function(genom):
        """改良版GA関数（VWnum=9対応版）"""
        # グローバル変数からシミュレーションモードを取得
        global SIMULATION_MODE
        
        # 共通VWの遺伝子長
        common_gene_length = setting.VWnum * setting.VWnum
        
        # 個別VWの総遺伝子長（4方向×3車線×VWnum×VWnum）
        individual_gene_length = 12 * setting.VWnum * setting.VWnum
        
        # 遺伝子長でモード判定
        is_individual_vw = (len(genom) == individual_gene_length)
        
        # 経路計算時間を記録
        create_path_time_start = time.time()
        
        # モードに応じて処理分岐
        if is_individual_vw:
            # 個別VWモード処理
            print(f"個別VWモードで評価（遺伝子長: {len(genom)}）")
            
            # 遺伝子を12個のVWグリッドに分割
            vw_grids = []
            vw_grid_size = setting.VWnum  # VWnumを使用（9×9）
            grid_gene_length = vw_grid_size * vw_grid_size
            
            for i in range(12):  # 12個のグリッド（4方向×3車線）
                start_idx = i * grid_gene_length
                end_idx = start_idx + grid_gene_length
                
                if end_idx <= len(genom):
                    grid_genes = genom[start_idx:end_idx]
                    grid = np.array(grid_genes).reshape(vw_grid_size, vw_grid_size)
                    vw_grids.append(grid)
                else:
                    vw_grids.append(np.zeros((vw_grid_size, vw_grid_size)))
            
            # シミュレーションの作成
            sim = CarSimulation()
            sim.use_adhoc = True
            
            # 個別VWモードを設定
            sim.vw_manager.set_vw_mode("individual")
            
            # 各方向・車線ごとのVWを設定
            directions = ["left", "right", "top", "bottom"]
            for dir_idx, direction in enumerate(directions):
                for lane_idx in range(3):
                    grid_idx = dir_idx * 3 + lane_idx
                    if grid_idx < len(vw_grids):
                        sim.vw_manager.set_individual_vw(direction, lane_idx, vw_grids[grid_idx])
            
            # 障害物の総数を計算
            total_num_obstacles = sum(np.sum(grid) for grid in vw_grids)
            
        else:
            # 共通VWモード処理（既存コード）
            print(f"共通VWモードで評価（遺伝子長: {len(genom)}）")
            
            car_ga_array = [[0] * setting.VWnum for _ in range(setting.VWnum)]
            ga_array = np.array(genom.reshape(setting.VWnum, setting.VWnum))
            
            for i in range(len(ga_array)):
                for j in range(len(ga_array[i])):
                    car_ga_array[i][j] = ga_array[i][j]
            
            # VWの設置
            car_VW_list, car_vw_line_list = VW.set_virtual_wall(car_ga_array)
            
            # シミュレーションの作成
            sim = CarSimulation()
            sim.use_adhoc = True
            
            # 共通VWモードを設定
            sim.vw_manager.set_vw_mode("common")
            
            # 共通VWグリッドを設定
            sim.vw_manager.set_common_vw(ga_array)
            
            # 障害物の総数を計算
            total_num_obstacles = np.sum(ga_array)
        
        # シミュレーションモードに応じて分岐
        if SIMULATION_MODE == "fourcars_scenario":
            # 固定台数シナリオのシミュレーション実行
            fixed_result = run_fixed_scenario_simulation(sim)
            
            # 経路計算時間を記録
            create_path_time_end = time.time()
            create_path_time = create_path_time_end - create_path_time_start
            create_path_time_dic = {"create_path_time": create_path_time}
            
            # 経路長の合計を計算
            all_path_length = sum(fixed_result["path_lengths"])
            
            # 適合度計算（固定シナリオ用）
            fitness = (
                # fixed_result["collisions"] * 100000 +  # 衝突に大きなペナルティ
                all_path_length *               # 経路長
                (total_num_obstacles / (1 * (setting.VWnum ** 2))) + # 障害物数を考慮
                fixed_result["adhoc_count"] * 100000 # アドホック回避にペナルティ
            )
            
            return fitness, fixed_result["collisions"], all_path_length, total_num_obstacles, fixed_result["paths"], fixed_result["adhoc_count"], create_path_time_dic
            
        else:
            # 交通フローシミュレーション実行
            result = sim.run_simulation_with_continuous_traffic(
                max_steps=500,
                spawn_rate=0.05,
                max_vehicles=12
            )
            
            # 経路長の概算値
            all_path_length = result["steps"] * result["vehicles_arrived"] if result["vehicles_arrived"] > 0 else float('inf')
            
            # 経路計算時間を記録
            create_path_time_end = time.time()
            create_path_time = create_path_time_end - create_path_time_start
            create_path_time_dic = {"create_path_time": create_path_time}
            
            # 適合度計算
            throughput = result["vehicles_arrived"] / result["vehicles_generated"] if result["vehicles_generated"] > 0 else 0
            efficiency = result["steps"] / result["vehicles_arrived"] if result["vehicles_arrived"] > 0 else float('inf')
            
            fitness = (
                # result["collisions"] * 100000 +  # 衝突に大きなペナルティ
                all_path_length *               # 経路長
                (total_num_obstacles / (1 * (setting.VWnum ** 2))) + # 障害物数を考慮
                result["adhoc_count"] * 100000 # アドホック回避にペナルティ
            )
            
            return fitness, result["collisions"], all_path_length, total_num_obstacles, [], result["adhoc_count"], create_path_time_dic

def traffic_flow_GA_function(genom):
    """交通フロー環境でVWパターンを評価するGA適合度関数"""
    # ゲノムをVWグリッドに変換
    car_ga_array = [[0] * setting.VWnum for _ in range(setting.VWnum)]
    ga_array = np.array(genom.reshape(setting.VWnum, setting.VWnum))
    
    for i in range(len(ga_array)):
        for j in range(len(ga_array[i])):
            car_ga_array[i][j] = ga_array[i][j]
    
    # シミュレーションの作成
    sim = CarSimulation()
    sim.use_adhoc = True
    
    # VWパターンの設定
    sim.vw_manager.set_common_vw(ga_array)
    
    # 交通フローシミュレーション実行
    result = sim.run_simulation_with_continuous_traffic(
        max_steps=500,       # 評価を高速化するために短縮
        spawn_rate=0.05,     # 新車両を生成する確率
        max_vehicles=12      # 最大車両数
    )
    
    # 障害物数の計算
    total_num_obstacles = np.sum(ga_array)
    
    # all_path_lengthの代替として「全車両の移動距離」の概算値を計算
    all_path_length = result["steps"] * result["vehicles_arrived"] if result["vehicles_arrived"] > 0 else float('inf')
    
    # cars_path_listは空のリストとする（固定車両の経路がないため）
    cars_path_list = []
    
    # 適合度計算
    throughput = result["vehicles_arrived"] / result["vehicles_generated"] if result["vehicles_generated"] > 0 else 0
    efficiency = result["steps"] / result["vehicles_arrived"] if result["vehicles_arrived"] > 0 else float('inf')
    
    fitness = (
        # result["collisions"] * 10000 +           # 衝突に大きなペナルティ
        (1 - throughput) * 1000 +                # スループットを最大化
        efficiency * 0.5 +                       # 走行時間を最小化
        result["adhoc_count"] *  +              # アドホック回避にペナルティ
        total_num_obstacles * (total_num_obstacles / (setting.VWnum ** 2)) * 20  # 過度の障害物にペナルティ
    )
    
    # 重要: 期待される6つの値のみを返す（create_path_time_dicは含めない）
    return fitness, result["collisions"], all_path_length, total_num_obstacles, cars_path_list, result["adhoc_count"]

def run_fixed_scenario_simulation(simulation):
    """
    setting.pyで定義された固定シナリオでシミュレーションを実行

    Parameters:
    - simulation: 設定済みのCarSimulationオブジェクト

    Returns:
    - 結果の辞書（衝突数、経路、アドホック回避回数など）
    """
    # 車両の初期化
    vehicles = []
    
    # setting.pyから車両数を取得
    # car_num = setting.car_num
    
    # 各車両のスタート・ゴール位置を取得
    for i in range(1, 5):
        # 動的に変数名を生成して車両の開始・目標位置を取得
        var_name = f"car{i}_STARTtoGOAL"
        if hasattr(setting, var_name):
            car_data = getattr(setting, var_name)
            
            # STARTtoGOALの形式をチェック
            if isinstance(car_data, tuple) and len(car_data) == 2:
                start = car_data[0]
                goal = car_data[1]
            elif isinstance(car_data, list) and len(car_data) == 2:
                start = car_data[0]
                goal = car_data[1]
            else:
                print(f"警告: {var_name} の形式が無効です。[[start_x, start_y], [goal_x, goal_y]]の形式が必要です")
                continue
            
            # 車両オブジェクトを作成
            vehicle = Vehicle(
                start=start,
                goal=goal,
                vehicle_id=i-1,
                vehicle_type="mixed",  # または適切なタイプ
                spawn_direction=None,  # 固定シナリオでは不要
                spawn_lane_idx=None    # 固定シナリオでは不要
            )
            vehicles.append(vehicle)
        else:
            print(f"警告: {var_name} がsetting.pyに見つかりません")
    
    # シミュレーションに車両を設定
    simulation.vehicles = vehicles
    
    # 優先度と軌跡の初期化
    simulation.priorities = [0] * len(vehicles)
    simulation.trajectories = [[] for _ in range(len(vehicles))]
    
    # 初期軌跡を記録
    for i, vehicle in enumerate(vehicles):
        simulation.trajectories[i].append(vehicle.position.copy())
    
    # 初期速度の設定
    simulation._initialize_velocities()
    
    # 壁の設定を明示的に行う
    # simulation.setup_walls()

    # 最適経路の計算
    simulation.compute_optimal_paths()
    
    # シミュレーション実行
    simulation.time_steps = 0
    simulation.collision_count = 0
    simulation.adhoc_count = 0
    
    # 各車両の経路情報
    paths = []
    path_lengths = []
    
    max_steps = 1000000  # 最大ステップ数（無限ループ防止）
    
    # シミュレーションループ
    while not all(vehicle.reached for vehicle in simulation.vehicles) and simulation.time_steps < max_steps:
        simulation.time_steps += 1
        
        # デッドロック検出・解決
        simulation.check_for_deadlocks()
        
        # アドホック回避が有効な場合
        if simulation.use_adhoc:
            # 衝突予測
            collision_predictions = simulation.collision_detector.predict_collisions(simulation.vehicles)
            
            if collision_predictions:
                # 速度調整とアドホック回避カウント
                adhoc_occurred = simulation.adjust_velocity(collision_predictions)
                simulation.adhoc_count += adhoc_occurred
            else:
                # 衝突がなければ減速した車両を元の速度に戻す
                simulation._normalize_all_speeds()
        
        # 車両の位置を更新
        simulation.update_car_positions()
        
        # 衝突検出
        new_collisions = simulation.check_collisions()
        
        if new_collisions > 0:
            simulation.collision_count += new_collisions
    
    # 各車両の経路情報を収集
    for i, trajectory in enumerate(simulation.trajectories):
        path_length = 0
        # 経路の長さを計算
        for j in range(1, len(trajectory)):
            dx = trajectory[j][0] - trajectory[j-1][0]
            dy = trajectory[j][1] - trajectory[j-1][1]
            segment_length = np.sqrt(dx*dx + dy*dy)
            path_length += segment_length
        
        paths.append(trajectory)
        path_lengths.append(path_length)
    
    # 結果を表示
    print(f"\n固定シナリオシミュレーション結果:")
    print(f"  ステップ数: {simulation.time_steps}")
    print(f"  衝突回数: {simulation.collision_count}")
    print(f"  アドホック回避回数: {simulation.adhoc_count}")
    print(f"  全車両到達: {all(vehicle.reached for vehicle in simulation.vehicles)}")
    
    # 各車両の結果を表示
    for i, vehicle in enumerate(simulation.vehicles):
        print(f"  車両 {i+1}: 到達={vehicle.reached}, 経路長={path_lengths[i]:.2f}")
    
    # 結果を返す
    return {
        "steps": simulation.time_steps,
        "collisions": simulation.collision_count,
        "adhoc_count": simulation.adhoc_count,
        "all_reached": all(vehicle.reached for vehicle in simulation.vehicles),
        "paths": paths,
        "path_lengths": path_lengths
    }

def individual_vw_ga_function(genom):
    """スポーンポイントごとの個別VWを最適化するGA適合度関数"""
    # 経路計算時間を記録
    create_path_time_start = time.time()
    
    # 遺伝子を12個のVWグリッドに分割（4方向×3車線）
    vw_grids = []
    vw_grid_size = 3  # 各グリッドは3×3
    grid_gene_length = vw_grid_size * vw_grid_size  # 各グリッドの遺伝子長 = 9
    
    # 遺伝子の分割
    for i in range(12):  # 12個のグリッド
        start_idx = i * grid_gene_length
        end_idx = start_idx + grid_gene_length
        
        if end_idx <= len(genom):
            grid_genes = genom[start_idx:end_idx]
            grid = np.array(grid_genes).reshape(vw_grid_size, vw_grid_size)
            vw_grids.append(grid)
        else:
            # 遺伝子長が足りない場合は空のグリッド
            vw_grids.append(np.zeros((vw_grid_size, vw_grid_size)))
    
    # シミュレーションの作成
    sim = CarSimulation()
    sim.use_adhoc = True
    
    # 個別VWモードを設定
    sim.vw_manager.set_vw_mode("individual")
    
    # 各方向・車線ごとのVWを設定
    directions = ["left", "right", "top", "bottom"]
    for dir_idx, direction in enumerate(directions):
        for lane_idx in range(3):
            grid_idx = dir_idx * 3 + lane_idx
            if grid_idx < len(vw_grids):
                sim.vw_manager.set_individual_vw(direction, lane_idx, vw_grids[grid_idx])
    
    # 交通フローシミュレーション実行
    result = sim.run_simulation_with_continuous_traffic(
        max_steps=200,
        spawn_rate=0.05,
        max_vehicles=12
    )
    
    # 障害物の総数を計算
    total_obstacles = sum(np.sum(grid) for grid in vw_grids)
    
    # 適合度の計算
    throughput = result["vehicles_arrived"] / result["vehicles_generated"] if result["vehicles_generated"] > 0 else 0
    efficiency = result["steps"] / result["vehicles_arrived"] if result["vehicles_arrived"] > 0 else float('inf')
    
    fitness = (
        # result["collisions"] * 10000 +       # 衝突に大きなペナルティ
        (1 - throughput) * 1000 +            # スループットを最大化
        efficiency * 0.5 +                   # 走行時間を最小化
        result["adhoc_count"] * 1000000 +          # アドホック回避にペナルティ
        total_obstacles * 5                  # 障害物数を考慮
    )
    
    # 経路長の概算
    path_length = result["steps"] * result["vehicles_arrived"] if result["vehicles_arrived"] > 0 else float('inf')
    
    # 経路計算時間を記録
    create_path_time_end = time.time()
    create_path_time = create_path_time_end - create_path_time_start
    create_path_time_dic = {"create_path_time": create_path_time}
    
    # 7つの戻り値（最後は経路計算時間辞書）
    return fitness, result["collisions"], path_length, total_obstacles, [], result["adhoc_count"], create_path_time_dic

def split_genome_to_vw_grids(genom):
    """遺伝子配列を12個のVWグリッドに分割"""
    grids = []
    
    # 各グリッドのサイズ
    vw_field_size = setting.VWnum
    grid_gene_size = vw_field_size * vw_field_size
    
    # 12個のグリッドに分割（4方向×3車線）
    for i in range(12):
        start_idx = i * grid_gene_size
        end_idx = (i + 1) * grid_gene_size
        
        # 遺伝子が十分な長さか確認
        if end_idx <= len(genom):
            grid_genes = genom[start_idx:end_idx]
            grid = np.array(grid_genes).reshape(vw_field_size, vw_field_size)
            grids.append(grid)
        else:
            # 足りない場合は空のグリッド
            grids.append(np.zeros((vw_field_size, vw_field_size)))
    
    return grids

def combining_vw(vw_list):
    """VW内の重複を消す関数"""
    seen = []
    return [position for position in vw_list if position not in seen and not seen.append(position)]

def set_all_seeds(seed):
    random.seed(seed)
    np.random.seed(seed)

def test_vw_optimization():
    """VW最適化テスト関数"""
    # 設定を表示
    print("VW最適化テスト開始")
    print(f"VWグリッドサイズ: {setting.VWnum}x{setting.VWnum}")
    print(f"人口サイズ: {setting.population_size}")
    print(f"世代数: {setting.generation_size}")
    
    # ランダムなVWグリッドで評価（単一段階）
    print("\n単一段階VW評価:")
    test_genom = np.random.randint(0, 2, setting.VWnum * setting.VWnum)
    
    start_time = time.time()
    # テスト用VWを評価
    fitness, collisions, all_path_length, total_num_obstacles, cars_path, adhoc_count, create_path_time_dic = VW.single_GA_function(test_genom)
    end_time = time.time()
    
    # 結果表示
    print("評価結果:")
    print(f"  適応度: {fitness}")
    print(f"  衝突数: {collisions}")
    print(f"  総経路長: {all_path_length}")
    print(f"  障害物数: {total_num_obstacles}")
    print(f"  アドホック回避回数: {adhoc_count}")
    print(f"  経路計算時間: {create_path_time_dic['create_path_time']:.4f}秒")
    print(f"  実行時間: {end_time - start_time:.2f}秒")

def test_continuous_traffic():
    """継続的な交通流テスト関数"""
    # シミュレーションの作成
    sim = CarSimulation()
    
    # VWの設定（ランダムパターン）
    vw_grid = np.zeros((sim.config.vw_field_size, sim.config.vw_field_size))
    
    # 交差点周辺にVWをランダムに配置
    for i in range(sim.config.vw_field_size):
        for j in range(sim.config.vw_field_size):
            # 中央部分は空けておく
            center_i = sim.config.vw_field_size // 2
            center_j = sim.config.vw_field_size // 2
            
            if abs(i - center_i) > 1 or abs(j - center_j) > 1:
                # 20%の確率でVWを配置
                if random.random() < 0.2:
                    vw_grid[i, j] = 1
    
    # VWを設定
    sim.vw_manager.set_common_vw(vw_grid)
    
    # 継続的な車両生成を伴うシミュレーション実行
    result = sim.run_simulation_with_continuous_traffic(
        max_steps=300,
        spawn_rate=0.05,  # 5%の確率で新車両を生成
        max_vehicles=15   # 最大15台まで
    )
    
    # 結果表示
    print("\n継続的な交通流シミュレーション結果:")
    print(f"  ステップ数: {result['steps']}")
    print(f"  衝突回数: {result['collisions']}")
    print(f"  アドホック回避回数: {result['adhoc_count']}")
    print(f"  生成車両数: {result['vehicles_generated']}")
    print(f"  到達車両数: {result['vehicles_arrived']}")

def test_individual_vw_patterns(best_gene):
    """最適化された個別VWパターンを検証"""
    # 最適化されたVWグリッドを取得
    optimized_grids = split_genome_to_vw_grids(best_gene.genom)
    
    # シミュレーション作成
    sim = CarSimulation()
    sim.use_adhoc = True
    
    # 個別VWモードを設定
    sim.vw_manager.set_vw_mode("individual")
    
    # 各方向・車線ごとのVWを設定
    directions = ["left", "right", "top", "bottom"]
    for dir_idx, direction in enumerate(directions):
        for lane_idx in range(3):
            grid_idx = dir_idx * 3 + lane_idx
            sim.vw_manager.set_individual_vw(direction, lane_idx, optimized_grids[grid_idx])
    
    # より長時間のシミュレーションで検証
    result = sim.run_simulation_with_continuous_traffic(
        max_steps=500,
        spawn_rate=0.05,
        max_vehicles=20
    )
    
    # 結果表示
    print("\n個別VWパターンの検証結果:")
    print(f"  ステップ数: {result['steps']}")
    print(f"  衝突回数: {result['collisions']}")
    print(f"  アドホック回避回数: {result['adhoc_count']}")
    print(f"  生成車両数: {result['vehicles_generated']}")
    print(f"  到達車両数: {result['vehicles_arrived']}")
    print(f"  スループット: {result['vehicles_arrived'] / result['vehicles_generated']:.2f}")
    
    return result

def main():
    """メイン関数"""
    print("VW最適化と交通フロー統合テスト開始")
    print(f"VWグリッドサイズ: {setting.VWnum}x{setting.VWnum}")
    
    # GA最適化の実行
    best, best_gene, generation_list = ga.main(setting.population_size, setting.generation_size, setting.genom_size)
    
    print("\n最適化結果:")
    print(f"  適応度: {best_gene.get_fitness()}")
    print(f"  衝突数: {best_gene.get_collision()}")
    print(f"  経路長: {best_gene.get_all_path_length()}")
    print(f"  障害物数: {best_gene.get_total_num_obstacles()}")
    
    # 最適化されたVWグリッドを取得
    optimized_genom = best_gene.genom
    optimized_vw_grid = np.array(optimized_genom).reshape(setting.VWnum, setting.VWnum)
    
    # 最適化されたVWグリッドを表示
    print("\n最適化されたVWグリッド配置:")
    for row in optimized_vw_grid:
        print("  " + " ".join("■" if cell >= 1 else "□" for cell in row))
    
    # 最適化されたVWを詳細表示
    optimized_vw_list, optimized_vw_lines = VW.set_virtual_wall([optimized_vw_grid.tolist()])
    
    print(f"\n最適化されたVW詳細 (総数: {len(optimized_vw_list) // 4}):")
    print(f"  VWの座標リスト: {optimized_vw_list}")
    
    print("\n最適化VWパターンでの交通フローテスト...")
    sim = CarSimulation()
    
    # 最適化されたVWグリッドを設定
    vw_grid = np.array(best_gene.genom).reshape(setting.VWnum, setting.VWnum)
    sim.vw_manager.set_common_vw(vw_grid)
    
    # 継続的な車両生成を伴うシミュレーション実行
    result = sim.run_simulation_with_continuous_traffic(
        max_steps=300,
        spawn_rate=0.05,
        max_vehicles=15
    )
    
    # 結果表示
    print("\n交通フロー結果:")
    print(f"  ステップ数: {result['steps']}")
    print(f"  衝突回数: {result['collisions']}")
    print(f"  アドホック回避回数: {result['adhoc_count']}")
    print(f"  生成車両数: {result['vehicles_generated']}")
    print(f"  到達車両数: {result['vehicles_arrived']}")
    
    return best, best_gene

def main_traffic_flow_optimization():
    """交通フローVW最適化のメイン関数"""
    print("交通フローVW最適化開始")
    print(f"VWグリッドサイズ: {setting.VWnum}x{setting.VWnum}")
    print(f"人口サイズ: {setting.population_size}")
    print(f"世代数: {setting.generation_size}")
    
    # 元の適合度関数を保存
    original_fitness = change_ga_vw_adhoc.VW.single_GA_function
    
    try:
        # 適合度関数を置き換え
        change_ga_vw_adhoc.VW.single_GA_function = traffic_flow_GA_function
        
        # GA最適化を実行
        start_time = time.time()
        best, best_gene, generation_list = ga.main(
            setting.population_size, 
            setting.generation_size, 
            setting.genom_size
        )
        end_time = time.time()
        
        print("\n最適化結果:")
        print(f"  総時間: {end_time - start_time:.2f}秒")
        print(f"  適合度: {best_gene.get_fitness()}")
        print(f"  衝突数: {best_gene.get_collision()}")
        print(f"  総移動距離: {best_gene.get_all_path_length()}")
        print(f"  障害物数: {best_gene.get_total_num_obstacles()}")
        print(f"  アドホック回避回数: {best_gene.get_adhoc_avoidance_count()}")
        
        # 最適化されたVWグリッドを表示
        optimized_genom = best_gene.genom
        optimized_vw_grid = np.array(optimized_genom).reshape(setting.VWnum, setting.VWnum)
        
        print("\n最適化されたVWグリッド:")
        for row in optimized_vw_grid:
            print("  " + " ".join("■" if cell >= 1 else "□" for cell in row))
        
        # 最終テスト実行
        print("\n最適化パターンで最終交通フローテスト実行...")
        sim = CarSimulation()
        sim.vw_manager.set_common_vw(optimized_vw_grid)
        
        # より長時間のテストで検証
        result = sim.run_simulation_with_continuous_traffic(
            max_steps=500,
            spawn_rate=0.05,
            max_vehicles=20
        )
        
        # 最終結果表示
        print("\n最終交通フロー結果:")
        print(f"  ステップ数: {result['steps']}")
        print(f"  衝突回数: {result['collisions']}")
        print(f"  アドホック回避回数: {result['adhoc_count']}")
        print(f"  生成車両数: {result['vehicles_generated']}")
        print(f"  到達車両数: {result['vehicles_arrived']}")
        print(f"  スループット: {result['vehicles_arrived'] / result['vehicles_generated']:.2f}")
        
        return best, best_gene
        
    finally:
        # 元の適合度関数を復元
        change_ga_vw_adhoc.VW.single_GA_function = original_fitness

def main_individual_vw_optimization():
    """スポーンポイントごとの個別VW最適化のメイン関数（VWnum=9対応版）"""
    
    SEED = 1

    set_all_seeds(SEED)

    # 個別VW用の遺伝子サイズ計算
    individual_genom_size = 12 * setting.VWnum * setting.VWnum
    
    print("スポーンポイントごとの個別VW最適化開始")
    print(f"個別VWグリッドサイズ: {setting.VWnum}×{setting.VWnum} (各方向・車線ごと)")
    print(f"遺伝子長: {individual_genom_size} (4方向×3車線×{setting.VWnum}×{setting.VWnum})")
    print(f"人口サイズ: {setting.population_size}")
    print(f"世代数: {setting.generation_size}")
    
    # GA最適化を実行
    start_time = time.time()
    best, best_gene, generation_list = ga.main(
        setting.population_size, 
        setting.generation_size, 
        individual_genom_size
    )
    end_time = time.time()
    
    print("\n最適化結果:")
    print(f"  総時間: {end_time - start_time:.2f}秒")
    print(f"  適合度: {best_gene.get_fitness()}")
    print(f"  衝突数: {best_gene.get_collision()}")
    print(f"  総移動距離: {best_gene.get_all_path_length()}")
    print(f"  障害物数: {best_gene.get_total_num_obstacles()}")
    print(f"  アドホック回避回数: {best_gene.get_adhoc_avoidance_count()}")
    
    # 最適化されたVWグリッドを可視化
    visualize_individual_vw_grids(best_gene.genom)
    
    return best, best_gene

def visualize_individual_vw_grids(genom):
    """最適化された個別VWグリッドを可視化（VWnum=9対応版）"""
    # 遺伝子を12個のVWグリッドに分割
    vw_grids = []
    vw_grid_size = setting.VWnum
    grid_gene_length = vw_grid_size * vw_grid_size
    
    for i in range(12):
        start_idx = i * grid_gene_length
        end_idx = start_idx + grid_gene_length
        
        if end_idx <= len(genom):
            grid_genes = genom[start_idx:end_idx]
            grid = np.array(grid_genes).reshape(vw_grid_size, vw_grid_size)
            vw_grids.append(grid)
        else:
            vw_grids.append(np.zeros((vw_grid_size, vw_grid_size)))
    
    # 各方向・車線ごとのVWグリッドの概要を表示
    directions = ["左", "右", "上", "下"]
    
    print("\n最適化された個別VWグリッド:")
    for dir_idx, direction in enumerate(directions):
        print(f"\n{direction}側:")
        for lane_idx in range(3):
            grid_idx = dir_idx * 3 + lane_idx
            if grid_idx < len(vw_grids):
                grid = vw_grids[grid_idx]
                vw_count = np.sum(grid)
                print(f"  車線 {lane_idx+1}: VW数 {int(vw_count)} / {vw_grid_size * vw_grid_size}")
                
                # グリッドが大きい場合は要約表示
                # 小さいグリッドは全表示
                for row in grid:
                    print("    " + " ".join("■" if cell >= 1 else "□" for cell in row))

def test_individual_vw_patterns(best_gene):
    """最適化された個別VWパターンを検証（VWnum=9対応版）"""
    # 遺伝子を12個のVWグリッドに分割
    vw_grids = []
    vw_grid_size = setting.VWnum
    grid_gene_length = vw_grid_size * vw_grid_size
    
    for i in range(12):
        start_idx = i * grid_gene_length
        end_idx = start_idx + grid_gene_length
        
        if end_idx <= len(best_gene.genom):
            grid_genes = best_gene.genom[start_idx:end_idx]
            grid = np.array(grid_genes).reshape(vw_grid_size, vw_grid_size)
            vw_grids.append(grid)
        else:
            vw_grids.append(np.zeros((vw_grid_size, vw_grid_size)))
    
    # シミュレーション作成
    sim = CarSimulation()
    sim.use_adhoc = True
    
    # 個別VWモードを設定
    sim.vw_manager.set_vw_mode("individual")
    
    # 各方向・車線ごとのVWを設定
    directions = ["left", "right", "top", "bottom"]
    for dir_idx, direction in enumerate(directions):
        for lane_idx in range(3):
            grid_idx = dir_idx * 3 + lane_idx
            if grid_idx < len(vw_grids):
                sim.vw_manager.set_individual_vw(direction, lane_idx, vw_grids[grid_idx])
    
    # より長時間のシミュレーションで検証
    result = sim.run_simulation_with_continuous_traffic(
        max_steps=500,
        spawn_rate=0.05,
        max_vehicles=20
    )
    
    # 結果表示
    print("\n個別VWパターンの検証結果:")
    print(f"  ステップ数: {result['steps']}")
    print(f"  衝突回数: {result['collisions']}")
    print(f"  アドホック回避回数: {result['adhoc_count']}")
    print(f"  生成車両数: {result['vehicles_generated']}")
    print(f"  到達車両数: {result['vehicles_arrived']}")
    if result['vehicles_generated'] > 0:
        print(f"  スループット: {result['vehicles_arrived'] / result['vehicles_generated']:.2f}")
    
    return result

if __name__ == '__main__':
    # main()
    # main_traffic_flow_optimization()

    # 個別VW最適化の実行
    best, best_gene = main_individual_vw_optimization()

    # 最適化されたVWパターンの検証
    test_individual_vw_patterns(best_gene)

    print(f"VWグリッド配置: {best_gene.genom}")