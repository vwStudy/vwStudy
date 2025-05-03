import numpy as np
import networkx as nx
import math

import ga
import setting
import copy

import random
import time

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
        self.reaction_time = 0.5
        self.braking_factor = 0.5
        
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
    def __init__(self, start, goal, vehicle_id=0, vehicle_type="straight"):
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
                
                if dist < self.config.car_width:
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

class TrafficGenerator:
    """交通流を生成する機能を提供するクラス"""
    def __init__(self, config):
        self.config = config
        self.spawn_points = self._create_spawn_points()
        
        # 方向選択の確率設定
        self.direction_probabilities = {
            "straight": 0.6,  # 直進の確率
            "right_turn": 0.2,  # 右折の確率
            "left_turn": 0.2   # 左折の確率
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
            vehicle_type=movement_type
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
        """
        継続的に新しい車両を生成する機能（安全間隔を考慮）
        """
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
            vehicle_type=movement_type
        )
        
        # 最終生成時間を更新
        self.last_spawn_times[spawn_direction][spawn_lane_idx] = current_time
        
        # 車両IDを更新
        self.next_vehicle_id += 1
        
        return vehicle
    
    def _check_spawn_safety(self, spawn_point, direction, simulation):
        """
        スポーンポイントの安全性をチェック（前方の車両との距離）
        """
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
        self.config = SimulationConfig()
        self.collision_detector = CollisionDetector(self.config)
        self.vehicles = []
        self.time_steps = 0
        self.collision_count = 0
        self.adhoc_count = 0
        self.priorities = []
        self.trajectories = []
        self.use_adhoc = True
        self.traffic_generator = TrafficGenerator(self.config)

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

    def add_vehicle(self, vehicle):
        """新しい車両をシミュレーションに追加"""
        self.vehicles.append(vehicle)
        self.priorities.append(0)
        self.trajectories.append([vehicle.position.copy()])
        
        # 初期速度を設定
        self._initialize_velocity_for_vehicle(len(self.vehicles) - 1)

    def _initialize_velocities(self):
        """車両の初期速度を設定"""
        for vehicle in self.vehicles:
            if vehicle.waypoints and len(vehicle.waypoints) > 0:
                wp = vehicle.waypoints[0]
                dx = wp[0] - vehicle.position[0]
                dy = wp[1] - vehicle.position[1]
                dist = np.sqrt(dx**2 + dy**2)
                
                if dist > 0:
                    dir_x = dx / dist
                    dir_y = dy / dist
                    vehicle.velocity = [dir_x * self.config.base_speed, dir_y * self.config.base_speed]
                    vehicle.target_velocity = vehicle.velocity.copy()
                    vehicle.last_angle = np.arctan2(dir_y, dir_x)
            else:
                dx = vehicle.goal[0] - vehicle.position[0]
                dy = vehicle.goal[1] - vehicle.position[1]
                dist = np.sqrt(dx**2 + dy**2)
                
                if dist > 0:
                    dir_x = dx / dist
                    dir_y = dy / dist
                    vehicle.velocity = [dir_x * self.config.base_speed, dir_y * self.config.base_speed]
                    vehicle.target_velocity = vehicle.velocity.copy()
                    vehicle.last_angle = np.arctan2(dir_y, dir_x)

    def _initialize_velocity_for_vehicle(self, vehicle_idx):
        """特定の車両の初期速度を設定"""
        vehicle = self.vehicles[vehicle_idx]
        
        if vehicle.waypoints and len(vehicle.waypoints) > 0:
            wp = vehicle.waypoints[0]
            dx = wp[0] - vehicle.position[0]
            dy = wp[1] - vehicle.position[1]
            dist = np.sqrt(dx**2 + dy**2)
            
            if dist > 0:
                dir_x = dx / dist
                dir_y = dy / dist
                vehicle.velocity = [dir_x * self.config.base_speed, dir_y * self.config.base_speed]
                vehicle.target_velocity = vehicle.velocity.copy()
                vehicle.last_angle = np.arctan2(dir_y, dir_x)
        else:
            dx = vehicle.goal[0] - vehicle.position[0]
            dy = vehicle.goal[1] - vehicle.position[1]
            dist = np.sqrt(dx**2 + dy**2)
            
            if dist > 0:
                dir_x = dx / dist
                dir_y = dy / dist
                vehicle.velocity = [dir_x * self.config.base_speed, dir_y * self.config.base_speed]
                vehicle.target_velocity = vehicle.velocity.copy()
                vehicle.last_angle = np.arctan2(dir_y, dir_x)

    def update_car_positions(self):
        """車両の位置を更新"""
        for i, vehicle in enumerate(self.vehicles):
            if vehicle.reached:
                continue
            
            if vehicle.is_reached_goal():
                vehicle.reached = True
                vehicle.velocity = [0, 0]
                continue
            
            if vehicle.waypoints and vehicle.current_waypoint_idx >= 0:
                wp_idx = vehicle.current_waypoint_idx
                
                if wp_idx < len(vehicle.waypoints):
                    current_wp = vehicle.waypoints[wp_idx]
                    
                    dx = current_wp[0] - vehicle.position[0]
                    dy = current_wp[1] - vehicle.position[1]
                    dist_to_wp = np.sqrt(dx**2 + dy**2)
                    
                    if dist_to_wp < 5:
                        vehicle.current_waypoint_idx += 1
                        
                        if vehicle.current_waypoint_idx < len(vehicle.waypoints):
                            next_wp = vehicle.waypoints[vehicle.current_waypoint_idx]
                            dx = next_wp[0] - vehicle.position[0]
                            dy = next_wp[1] - vehicle.position[1]
                            dist = np.sqrt(dx*dx + dy*dy)
                            
                            if dist > 0:
                                dir_x = dx / dist
                                dir_y = dy / dist
                                speed = self.config.base_speed
                                
                                if vehicle.type in ["right_turn", "left_turn"]:
                                    speed *= 0.8
                                
                                vehicle.velocity = [dir_x * speed, dir_y * speed]
                                vehicle.target_velocity = [dir_x * speed, dir_y * speed]
                                vehicle.last_angle = np.arctan2(dir_y, dir_x)
                    else:
                        if dist_to_wp > 0:
                            dir_x = dx / dist_to_wp
                            dir_y = dy / dist_to_wp
                            
                            if not vehicle.slowed_down:
                                speed = self.config.base_speed
                                
                                if vehicle.type in ["right_turn", "left_turn"]:
                                    speed *= 0.8
                                
                                vehicle.velocity = [dir_x * speed, dir_y * speed]
                            else:
                                current_speed = np.linalg.norm(vehicle.velocity)
                                vehicle.velocity = [dir_x * current_speed, dir_y * current_speed]
                            
                            vehicle.target_velocity = [dir_x * self.config.base_speed, dir_y * self.config.base_speed]
                            vehicle.last_angle = np.arctan2(dir_y, dir_x)
                else:
                    dx = vehicle.goal[0] - vehicle.position[0]
                    dy = vehicle.goal[1] - vehicle.position[1]
                    dist = np.sqrt(dx*dx + dy*dy)
                    
                    if dist > 0:
                        dir_x = dx / dist
                        dir_y = dy / dist
                        
                        if not vehicle.slowed_down:
                            speed = self.config.base_speed
                            vehicle.velocity = [dir_x * speed, dir_y * speed]
                        else:
                            speed = np.linalg.norm(vehicle.velocity)
                            vehicle.velocity = [dir_x * speed, dir_y * speed]
                        
                        vehicle.target_velocity = [dir_x * self.config.base_speed, dir_y * self.config.base_speed]
                        vehicle.last_angle = np.arctan2(dir_y, dir_x)
            else:
                dx = vehicle.goal[0] - vehicle.position[0]
                dy = vehicle.goal[1] - vehicle.position[1]
                dist = np.sqrt(dx*dx + dy*dy)
                
                if dist > 0:
                    dir_x = dx / dist
                    dir_y = dy / dist
                    
                    if not vehicle.slowed_down:
                        vehicle.velocity = [dir_x * self.config.base_speed, dir_y * self.config.base_speed]
                    
                    vehicle.target_velocity = [dir_x * self.config.base_speed, dir_y * self.config.base_speed]
                    vehicle.last_angle = np.arctan2(dir_y, dir_x)
            
            vehicle.update_position()
            self.trajectories[i].append(vehicle.position.copy())
    
    def update_priorities(self):
        """車両の優先度を更新"""
        for i, vehicle in enumerate(self.vehicles):
            if vehicle.reached:
                self.priorities[i] = -999
                continue
            
            self.priorities[i] = 0
            
            pos = np.array(vehicle.position)
            intersection_center = np.array(self.config.intersection_center)
            dist_to_intersection = np.linalg.norm(pos - intersection_center)
            
            in_intersection = self._is_in_intersection(pos)
            if in_intersection:
                self.priorities[i] += 200
            elif dist_to_intersection < 50:
                self.priorities[i] += 150 - dist_to_intersection
            
            speed = np.linalg.norm(vehicle.velocity)
            if speed > self.config.base_speed * 0.8:
                self.priorities[i] += 50
            
            if vehicle.stopped_time > 5:
                self.priorities[i] += vehicle.stopped_time * 3
            
            if vehicle.is_almost_stopped():
                vehicle.stopped_time += 1
                if vehicle.stopped_time > 10:
                    self.priorities[i] += vehicle.stopped_time * 2
            else:
                vehicle.stopped_time = 0
            
            if vehicle.slowed_down:
                self.priorities[i] -= 50
    
    def _is_in_intersection(self, position):
        """位置が交差点内かどうかを判定"""
        ic_x, ic_y = self.config.intersection_center
        half_size = self.config.intersection_size / 2
        
        return (ic_x - half_size <= position[0] <= ic_x + half_size and 
                ic_y - half_size <= position[1] <= ic_y + half_size)
    
    def adjust_velocity(self, predictions):
        """衝突予測に基づいて車両の速度を調整"""
        if not predictions:
            self._normalize_all_speeds()
            return 0
        
        self.update_priorities()
        cars_to_slow_down = set()
        adhoc_count = 0
        
        for prediction in predictions:
            i, j = prediction["vehicles"]
            time_step = prediction["time_step"]
            in_intersection = prediction.get("in_intersection", False)
            
            if self.vehicles[i].reached or self.vehicles[j].reached:
                continue
            
            time_urgency = max(0.2, min(1.0, 1.0 / (time_step + 1)))
            
            if in_intersection:
                adhoc_occurred = self._handle_intersection_collision(i, j, time_urgency, cars_to_slow_down)
                if adhoc_occurred:
                    adhoc_count += 1
            else:
                adhoc_occurred = self._handle_regular_collision(i, j, time_urgency, cars_to_slow_down)
                if adhoc_occurred:
                    adhoc_count += 1
        
        self._normalize_speeds(cars_to_slow_down)
        
        return adhoc_count
    
    def _handle_intersection_collision(self, i, j, time_urgency, cars_to_slow_down):
        """交差点内での衝突処理"""
        if self.priorities[i] > self.priorities[j]:
            self._apply_deceleration(j, time_urgency, cars_to_slow_down)
        else:
            self._apply_deceleration(i, time_urgency, cars_to_slow_down)
        return True
    
    def _handle_regular_collision(self, i, j, time_urgency, cars_to_slow_down):
        """交差点外での衝突処理"""
        if self.priorities[i] > self.priorities[j]:
            self._apply_deceleration(j, time_urgency, cars_to_slow_down)
            return True
        else:
            self._apply_deceleration(i, time_urgency, cars_to_slow_down)
            return True
    
    def _apply_deceleration(self, car_idx, time_urgency, cars_to_slow_down):
        """車両に減速を適用"""
        vehicle = self.vehicles[car_idx]
        speed = np.linalg.norm(vehicle.velocity)
        
        if speed < 0.1:
            return
        
        cars_to_slow_down.add(car_idx)
        slow_factor = max(0.3, 1.0 - (time_urgency * 0.7))
        
        direction = [vehicle.velocity[0] / speed, vehicle.velocity[1] / speed]
        new_speed = max(0.2, speed * slow_factor)
        
        vehicle.velocity[0] = direction[0] * new_speed
        vehicle.velocity[1] = direction[1] * new_speed
        vehicle.slowed_down = True
    
    def _normalize_speeds(self, cars_to_slow_down):
        """減速が不要な車両の速度を正常化"""
        for i, vehicle in enumerate(self.vehicles):
            if vehicle.reached or not vehicle.slowed_down or i in cars_to_slow_down:
                continue
            
            target_vel = vehicle.target_velocity
            target_speed = np.linalg.norm(target_vel)
            
            if target_speed < 0.001:
                continue
            
            current_speed = np.linalg.norm(vehicle.velocity)
            
            if current_speed < target_speed:
                acceleration_rate = 0.1
                new_speed = min(current_speed + (target_speed * acceleration_rate), target_speed)
                
                if current_speed > 0:
                    direction = [vehicle.velocity[0] / current_speed, vehicle.velocity[1] / current_speed]
                else:
                    direction = [target_vel[0] / target_speed, target_vel[1] / target_speed]
                
                vehicle.velocity[0] = direction[0] * new_speed
                vehicle.velocity[1] = direction[1] * new_speed
                
                if new_speed >= target_speed * 0.95:
                    vehicle.slowed_down = False
    
    def _normalize_all_speeds(self):
        """全車両の速度を正常化"""
        for i, vehicle in enumerate(self.vehicles):
            if vehicle.reached or not vehicle.slowed_down:
                continue
            
            target_vel = vehicle.target_velocity
            target_speed = np.linalg.norm(target_vel)
            
            if target_speed < 0.001:
                continue
            
            current_speed = np.linalg.norm(vehicle.velocity)
            
            if current_speed < target_speed:
                acceleration_rate = 0.2
                new_speed = min(current_speed + (target_speed * acceleration_rate), target_speed)
                
                if current_speed > 0:
                    direction = [vehicle.velocity[0] / current_speed, vehicle.velocity[1] / current_speed]
                else:
                    direction = [target_vel[0] / target_speed, target_vel[1] / target_speed]
                
                vehicle.velocity[0] = direction[0] * new_speed
                vehicle.velocity[1] = direction[1] * new_speed
                
                if new_speed >= target_speed * 0.8:
                    vehicle.slowed_down = False
    
    def check_collisions(self):
        """実際の衝突をチェック"""
        return self.collision_detector.detect_collisions(self.vehicles)
    
    def run_simulation(self):
        """シミュレーションを実行"""
        self.time_steps = 0
        self.collision_count = 0
        self.adhoc_count = 0
        
        while not all(vehicle.reached for vehicle in self.vehicles) and self.time_steps < self.config.max_steps:
            self.time_steps += 1
            
            if self.use_adhoc:
                collision_predictions = self.collision_detector.predict_collisions(self.vehicles)
                
                if collision_predictions:
                    adhoc_occurred = self.adjust_velocity(collision_predictions)
                    self.adhoc_count += adhoc_occurred
                else:
                    self._normalize_all_speeds()
            
            self.update_car_positions()
            new_collisions = self.check_collisions()
            
            if new_collisions > 0:
                self.collision_count += new_collisions
        
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
        
        self.time_steps = 0
        self.collision_count = 0
        self.adhoc_count = 0
        
        while self.time_steps < max_steps:
            self.time_steps += 1
            
            # 新しい車両の生成を試みる
            new_vehicle = self.traffic_generator.generate_vehicle_if_needed(
                self, spawn_rate=spawn_rate, max_vehicles=max_vehicles
            )
            
            if new_vehicle is not None:
                self.add_vehicle(new_vehicle)
            
            if self.use_adhoc:
                collision_predictions = self.collision_detector.predict_collisions(self.vehicles)
                
                if collision_predictions:
                    adhoc_occurred = self.adjust_velocity(collision_predictions)
                    self.adhoc_count += adhoc_occurred
                else:
                    self._normalize_all_speeds()
            
            self.update_car_positions()
            new_collisions = self.check_collisions()
            
            if new_collisions > 0:
                self.collision_count += new_collisions
            
            # 終了条件：すべての車両が到達し、新しい車両が生成される可能性がなくなった場合
            if all(v.reached for v in self.vehicles) and len(self.vehicles) >= max_vehicles:
                break
        
        result = {
            "steps": self.time_steps,
            "collisions": self.collision_count,
            "adhoc_count": self.adhoc_count,
            "vehicles_generated": self.traffic_generator.next_vehicle_id,
            "vehicles_arrived": sum(1 for v in self.vehicles if v.reached)
        }
        
        return result

class VW():
    """
    Virtual Wallを管理するクラス
    """
    def __init__(self, x, y, size):
        self.x = x
        self.y = y
        self.size = size
    
    def set_virtual_wall(GA_list, VWsize = setting.VWsize):
        """
        遺伝的アルゴリズムの結果に対応したVWを設置する関数,VWの4つの頂点のlistと障害物の線分のlistを返す
        """
        size = VWsize
        field_x = setting.VWfield_x
        field_y = setting.VWfield_y
        obstacles_vertex_list = []
        obstacles_line_list = []
        total_num_obstacles = 0
        n = 1
        
        for index, oneDivisionList in enumerate(GA_list):
            for twoDivisionIndex, deploy_check in enumerate(oneDivisionList):
                if deploy_check >= 1:
                    total_num_obstacles += 1
                    VW_LeftUp = [(field_x + (size * twoDivisionIndex)) * n, (field_y + (size * index)) * n]
                    VW_LeftDown = [VW_LeftUp[0] * n, (VW_LeftUp[1] + size) * n]
                    VW_RightUp = [(VW_LeftUp[0] + size) * n, VW_LeftUp[1] * n]
                    VW_RightDown = [(VW_LeftUp[0] + size) * n, (VW_LeftUp[1] + size) * n]
                    
                    obstacles_vertex_list.extend([VW_LeftUp, VW_LeftDown, VW_RightUp, VW_RightDown])
                    obstacles_line_list.extend([[VW_LeftUp, VW_LeftDown], [VW_LeftUp, VW_RightUp], [VW_RightUp, VW_RightDown], [VW_RightDown, VW_LeftDown]])
        return obstacles_vertex_list, obstacles_line_list
    
    def GA_function(genom):
        """
        GeneticalAlgorism用の関数
        """
        car_ga_array = [[[],[],[],[],[],[],[],[],[]],[[],[],[],[],[],[],[],[],[]],[[],[],[],[],[],[],[],[],[]],[[],[],[],[],[],[],[],[],[]]]
        ga_array = np.array(genom.reshape(4, setting.VWnum, setting.VWnum))
        for i in range(len(ga_array)):
            for j in range(len(ga_array[i])):
                l = list(ga_array[i][j])
                car_ga_array[i][j] = l

        # print(car_ga_array)

        # print("vw"+str(car_ga_array))

        #ToDo 以下の処理は変える必要がある
        #遺伝的アルゴリズムの結果に対しVWを設置
        car1_VW_list, car1_vw_line_list = VW.set_virtual_wall(car_ga_array[0])
        car2_VW_list, car2_vw_line_list = VW.set_virtual_wall(car_ga_array[1])
        car3_VW_list, car3_vw_line_list = VW.set_virtual_wall(car_ga_array[2])
        car4_VW_list, car4_vw_line_list = VW.set_virtual_wall(car_ga_array[3])

        # print(car1_VW_list)
        # print(car2_VW_list)

        #CarAgentにODを設定
        cars_tuple = (CarAgent(setting.car1_STARTtoGOAL[0],setting.car1_STARTtoGOAL[1]), CarAgent(setting.car2_STARTtoGOAL[0],setting.car2_STARTtoGOAL[1]), CarAgent(setting.car3_STARTtoGOAL[0],setting.car3_STARTtoGOAL[1]), CarAgent(setting.car4_STARTtoGOAL[0],setting.car4_STARTtoGOAL[1]))
        # print(setting.car1_STARTtoGOAL[0],setting.car1_STARTtoGOAL[1])
        # print(setting.car2_STARTtoGOAL[0],setting.car2_STARTtoGOAL[1])

        wall_edge_list, wall_line_list = Environment.set_wall()

        # car1_vw_line_list.extend(wall_line_list)
        # car2_vw_line_list.extend(wall_line_list)
        # car3_vw_line_list.extend(wall_line_list)
        # car4_vw_line_list.extend(wall_line_list)


        # print(car1_VW_list)
        
        create_vertex_start = time.time()
        #頂点のlistを作成
        car1_vertex_list = Environment.set_vertex_list(car1_VW_list, cars_tuple[0], wall_edge_list)
        car2_vertex_list = Environment.set_vertex_list(car2_VW_list, cars_tuple[1], wall_edge_list)
        car3_vertex_list = Environment.set_vertex_list(car3_VW_list, cars_tuple[2], wall_edge_list)
        car4_vertex_list = Environment.set_vertex_list(car4_VW_list, cars_tuple[3], wall_edge_list)
        create_vertex_end = time.time()
        vertex_time_diff = create_vertex_end - create_vertex_start
        print("vertex::",vertex_time_diff)

        visibility_start = time.time()
        #可視グラフ, ダイクストラ法を実行
        car1_vis_graph = Execution.visibility_graph(car1_vertex_list, car1_vw_line_list)
        car2_vis_graph = Execution.visibility_graph(car2_vertex_list, car2_vw_line_list)
        car3_vis_graph = Execution.visibility_graph(car3_vertex_list, car3_vw_line_list)
        car4_vis_graph = Execution.visibility_graph(car4_vertex_list, car4_vw_line_list)
        visibility_end = time.time()
        visibility_time_diff = visibility_end - visibility_start
        print("visibility::",visibility_time_diff)

        dijkstra_start = time.time()
        car1_shortest_path, car1_shortest_length = Execution.dijkstra(car1_vis_graph)
        car2_shortest_path, car2_shortest_length = Execution.dijkstra(car2_vis_graph)
        car3_shortest_path, car3_shortest_length = Execution.dijkstra(car3_vis_graph)
        car4_shortest_path, car4_shortest_length = Execution.dijkstra(car4_vis_graph)
        dijkstra_end = time.time()
        dijkstra_time_diff = dijkstra_end - dijkstra_start
        print("dijkstra::",dijkstra_time_diff)        

        cars_path_list = []
        car_path_tmp_list = []
        for path in car1_shortest_path:
            #print("car1 :",car1_vertex_list[path])
            car_path_tmp_list.append(car1_vertex_list[path])
        cars_path_list.append(car_path_tmp_list)
        
        for path in car2_shortest_path:
            #print("car2 :",car2_vertex_list[path])
            car_path_tmp_list.append(car2_vertex_list[path])
        cars_path_list.append(car_path_tmp_list)
        
        for path in car3_shortest_path:
            #print("car3 :",car3_vertex_list[path])
            car_path_tmp_list.append(car3_vertex_list[path])
        cars_path_list.append(car_path_tmp_list)

        for path in car4_shortest_path:
            #print("car4 :",car4_vertex_list[path])
            car_path_tmp_list.append(car4_vertex_list[path])
        cars_path_list.append(car_path_tmp_list)
        
        collision = Environment.collision_CarToCar(car1_vertex_list, car1_shortest_path, car2_vertex_list, car2_shortest_path, car3_vertex_list, car3_shortest_path, car4_vertex_list, car4_shortest_path)

        print("collision::"+str(collision))

        total_num_obstacles = len(car1_VW_list)/4 + len(car2_VW_list)/4 + len(car3_VW_list)/4 + len(car4_VW_list)/4
        #print(total_num_obstacles)
        
        #print("collision::"+str(collision))
        #全ての経路長を足す
        all_path_length = car1_shortest_length + car2_shortest_length + car3_shortest_length + car4_shortest_length
        # print("all_len::"+str(all_path_length))

        return all_path_length * (total_num_obstacles / (4 * (setting.VWnum ** 2))) + collision * 1000000, collision, all_path_length, total_num_obstacles, cars_path_list

    def single_GA_function(genom):
        """改良版GA関数（アドホック回避機能付き）"""
        car_ga_array = [[[]]*setting.VWnum]
        ga_array = np.array(genom.reshape(setting.car_num, setting.VWnum, setting.VWnum))
        for i in range(len(ga_array)):
            for j in range(len(ga_array[i])):
                l = list(ga_array[i][j])
                car_ga_array[i][j] = l

        # 既存の処理（VW設置、経路計算など）
        car_VW_list, car_vw_line_list = VW.set_virtual_wall(car_ga_array[0])
        
        cars_tuple = (CarAgent(setting.car1_STARTtoGOAL[0],setting.car1_STARTtoGOAL[1]), 
                     CarAgent(setting.car2_STARTtoGOAL[0],setting.car2_STARTtoGOAL[1]), 
                     CarAgent(setting.car3_STARTtoGOAL[0],setting.car3_STARTtoGOAL[1]), 
                     CarAgent(setting.car4_STARTtoGOAL[0],setting.car4_STARTtoGOAL[1]))
        
        wall_edge_list, wall_line_list = Environment.set_wall()
        
        car1_vertex_list = Environment.set_vertex_list(car_VW_list, cars_tuple[0], wall_edge_list)
        car2_vertex_list = Environment.set_vertex_list(car_VW_list, cars_tuple[1], wall_edge_list)
        car3_vertex_list = Environment.set_vertex_list(car_VW_list, cars_tuple[2], wall_edge_list)
        car4_vertex_list = Environment.set_vertex_list(car_VW_list, cars_tuple[3], wall_edge_list)
        
        car1_vis_graph = Execution.visibility_graph(car1_vertex_list, car_vw_line_list)
        car2_vis_graph = Execution.visibility_graph(car2_vertex_list, car_vw_line_list)
        car3_vis_graph = Execution.visibility_graph(car3_vertex_list, car_vw_line_list)
        car4_vis_graph = Execution.visibility_graph(car4_vertex_list, car_vw_line_list)
        
        car1_shortest_path, car1_shortest_length = Execution.dijkstra(car1_vis_graph)
        car2_shortest_path, car2_shortest_length = Execution.dijkstra(car2_vis_graph)
        car3_shortest_path, car3_shortest_length = Execution.dijkstra(car3_vis_graph)
        car4_shortest_path, car4_shortest_length = Execution.dijkstra(car4_vis_graph)

        # 新しいシミュレーションシステムを使用
        sim = CarSimulationWithVWManager()
        sim.use_adhoc = True

        # VWモードを設定（共通または個別）
        sim.vw_manager.set_vw_mode("individual")  # 個別VWモードに設定

        
        # 車両の初期化
        vehicle1 = Vehicle(setting.car1_STARTtoGOAL[0], setting.car1_STARTtoGOAL[1], 0, "straight")
        vehicle2 = Vehicle(setting.car2_STARTtoGOAL[0], setting.car2_STARTtoGOAL[1], 1, "straight")
        vehicle3 = Vehicle(setting.car3_STARTtoGOAL[0], setting.car3_STARTtoGOAL[1], 2, "straight")
        vehicle4 = Vehicle(setting.car4_STARTtoGOAL[0], setting.car4_STARTtoGOAL[1], 3, "straight")
        
        # 経路をウェイポイントとして設定
        waypoints1 = [car1_vertex_list[idx] for idx in car1_shortest_path]
        waypoints2 = [car2_vertex_list[idx] for idx in car2_shortest_path]
        waypoints3 = [car3_vertex_list[idx] for idx in car3_shortest_path]
        waypoints4 = [car4_vertex_list[idx] for idx in car4_shortest_path]
        
        vehicle1.set_waypoints(waypoints1)
        vehicle2.set_waypoints(waypoints2)
        vehicle3.set_waypoints(waypoints3)
        vehicle4.set_waypoints(waypoints4)
        
        sim.vehicles = [vehicle1, vehicle2, vehicle3, vehicle4]
        sim.priorities = [0, 0, 0, 0]
        sim.trajectories = [[], [], [], []]
        
        # 初期速度の設定
        sim._initialize_velocities()
        
        # シミュレーション実行
        result = sim.run_simulation()
        
        # 経路情報を収集
        cars_path_list = []
        for i, (path, vertex_list) in enumerate([
            (car1_shortest_path, car1_vertex_list),
            (car2_shortest_path, car2_vertex_list),
            (car3_shortest_path, car3_vertex_list),
            (car4_shortest_path, car4_vertex_list)
        ]):
            car_path_tmp_list = []
            for idx in path:
                car_path_tmp_list.append(vertex_list[idx])
            cars_path_list.append(car_path_tmp_list)
        
        # 結果の計算
        total_num_obstacles = len(car_VW_list)/4
        all_path_length = car1_shortest_length + car2_shortest_length + car3_shortest_length + car4_shortest_length
        
        # 適応度計算
        fitness = (all_path_length * (total_num_obstacles / (1 * (setting.VWnum ** 2))) + 
                  result["adhoc_count"] * 1000000)
        
        return fitness, result["collisions"], all_path_length, total_num_obstacles, cars_path_list, result["adhoc_count"]
    
    def two_steps_GA_function(genom, two_steps_list, zeros_list):
        """改良版2段階GA関数（アドホック回避機能付き）"""
        ga_array = np.array(genom.reshape(1, len(two_steps_list), 9))
        car_ga_array = zeros_list
        
        for i in range(len(two_steps_list)):
            car_ga_array[0][two_steps_list[i]] = list(ga_array[0][i])
        
        VWsize = (setting.VWfield / (setting.two_VWnum **2))
        
        # 既存の処理（VW設置、経路計算など）
        car_VW_list, car_vw_line_list = VW.set_virtual_wall(car_ga_array[0], VWsize)
        
        cars_tuple = (CarAgent(setting.car1_STARTtoGOAL[0],setting.car1_STARTtoGOAL[1]), 
                     CarAgent(setting.car2_STARTtoGOAL[0],setting.car2_STARTtoGOAL[1]), 
                     CarAgent(setting.car3_STARTtoGOAL[0],setting.car3_STARTtoGOAL[1]), 
                     CarAgent(setting.car4_STARTtoGOAL[0],setting.car4_STARTtoGOAL[1]))
        
        wall_edge_list, wall_line_list = Environment.set_wall()
        
        car1_vertex_list = Environment.set_vertex_list(car_VW_list, cars_tuple[0], wall_edge_list)
        car2_vertex_list = Environment.set_vertex_list(car_VW_list, cars_tuple[1], wall_edge_list)
        car3_vertex_list = Environment.set_vertex_list(car_VW_list, cars_tuple[2], wall_edge_list)
        car4_vertex_list = Environment.set_vertex_list(car_VW_list, cars_tuple[3], wall_edge_list)
        
        car1_vis_graph = Execution.visibility_graph(car1_vertex_list, car_vw_line_list)
        car2_vis_graph = Execution.visibility_graph(car2_vertex_list, car_vw_line_list)
        car3_vis_graph = Execution.visibility_graph(car3_vertex_list, car_vw_line_list)
        car4_vis_graph = Execution.visibility_graph(car4_vertex_list, car_vw_line_list)
        
        car1_shortest_path, car1_shortest_length = Execution.dijkstra(car1_vis_graph)
        car2_shortest_path, car2_shortest_length = Execution.dijkstra(car2_vis_graph)
        car3_shortest_path, car3_shortest_length = Execution.dijkstra(car3_vis_graph)
        car4_shortest_path, car4_shortest_length = Execution.dijkstra(car4_vis_graph)

        # 新しいシミュレーションシステムを使用
        sim = CarSimulation()
        sim.use_adhoc = True
        
        # 車両の初期化
        vehicle1 = Vehicle(setting.car1_STARTtoGOAL[0], setting.car1_STARTtoGOAL[1], 0, "straight")
        vehicle2 = Vehicle(setting.car2_STARTtoGOAL[0], setting.car2_STARTtoGOAL[1], 1, "straight")
        vehicle3 = Vehicle(setting.car3_STARTtoGOAL[0], setting.car3_STARTtoGOAL[1], 2, "straight")
        vehicle4 = Vehicle(setting.car4_STARTtoGOAL[0], setting.car4_STARTtoGOAL[1], 3, "straight")
        
        # 経路をウェイポイントとして設定
        waypoints1 = [car1_vertex_list[idx] for idx in car1_shortest_path]
        waypoints2 = [car2_vertex_list[idx] for idx in car2_shortest_path]
        waypoints3 = [car3_vertex_list[idx] for idx in car3_shortest_path]
        waypoints4 = [car4_vertex_list[idx] for idx in car4_shortest_path]
        
        vehicle1.set_waypoints(waypoints1)
        vehicle2.set_waypoints(waypoints2)
        vehicle3.set_waypoints(waypoints3)
        vehicle4.set_waypoints(waypoints4)
        
        sim.vehicles = [vehicle1, vehicle2, vehicle3, vehicle4]
        sim.priorities = [0, 0, 0, 0]
        sim.trajectories = [[], [], [], []]
        
        # 初期速度の設定
        sim._initialize_velocities()
        
        # シミュレーション実行
        result = sim.run_simulation()
        
        # 経路情報を収集
        cars_path_list = []
        for i, (path, vertex_list) in enumerate([
            (car1_shortest_path, car1_vertex_list),
            (car2_shortest_path, car2_vertex_list),
            (car3_shortest_path, car3_vertex_list),
            (car4_shortest_path, car4_vertex_list)
        ]):
            car_path_tmp_list = []
            for idx in path:
                car_path_tmp_list.append(vertex_list[idx])
            cars_path_list.append(car_path_tmp_list)
        
        # 結果の計算
        total_num_obstacles = len(car_VW_list)/4
        all_path_length = car1_shortest_length + car2_shortest_length + car3_shortest_length + car4_shortest_length
        
        # 適応度計算
        fitness = (all_path_length * (total_num_obstacles / (setting.car_num * ((len(two_steps_list)*((setting.VWnum) ** 2))))) + 
                  result["adhoc_count"] * 1000000)
        
        return fitness, result["collisions"], all_path_length, total_num_obstacles, cars_path_list, result["adhoc_count"]
    # def GA_function(genom):
    #     """
    #
    #     """
    #     car_ga_array = [[[],[],[],[]],[[],[],[],[]],[[],[],[],[]],[[],[],[],[]]]
    #     ga_array = np.array(genom.reshape(4, 4, 4))
    #     for i in range(len(ga_array)):
    #         for j in range(len(ga_array[i])):
    #             l = list(ga_array[i][j])
    #             car_ga_array[i][j] = l

    #     #ToDo 以下の処理は変える必要がある
    #     #遺伝的アルゴリズムの結果に対しVWを設置
    #     car1_vw_list, car1_vw_line_list = VW.set_virtual_wall(car_ga_array[0])
    #     car2_vw_list, car2_vw_line_list = VW.set_virtual_wall(car_ga_array[1])
    #     car3_vw_list, car3_vw_line_list = VW.set_virtual_wall(car_ga_array[2])
    #     car4_vw_list, car4_vw_line_list = VW.set_virtual_wall(car_ga_array[3])
    #     ##print("car1::"+str(car1_VW_list))

    #     #CarAgentにODを設定
    #     cars_tuple = (CarAgent(setting.car1_STARTtoGOAL[0],setting.car1_STARTtoGOAL[1]), CarAgent(setting.car2_STARTtoGOAL[0],setting.car2_STARTtoGOAL[1]), CarAgent(setting.car3_STARTtoGOAL[0],setting.car3_STARTtoGOAL[1]), CarAgent(setting.car4_STARTtoGOAL[0],setting.car4_STARTtoGOAL[1]))
    #     # print(setting.car1_STARTtoGOAL[0],setting.car1_STARTtoGOAL[1])
    #     # print(setting.car2_STARTtoGOAL[0],setting.car2_STARTtoGOAL[1])

    #     wall_edge_list, wall_line_list = Environment.set_wall()

    #     car1_vw_line_list.extend(wall_line_list)
    #     car2_vw_line_list.extend(wall_line_list)
    #     car3_vw_line_list.extend(wall_line_list)
    #     car4_vw_line_list.extend(wall_line_list)

    #     #頂点のlistを作成
    #     car1_vertex_list = Environment.set_vertex_list(car1_vw_list, cars_tuple[0], wall_edge_list)
    #     car2_vertex_list = Environment.set_vertex_list(car2_vw_list, cars_tuple[1], wall_edge_list)
    #     car3_vertex_list = Environment.set_vertex_list(car3_vw_list, cars_tuple[2], wall_edge_list)
    #     car4_vertex_list = Environment.set_vertex_list(car4_vw_list, cars_tuple[3], wall_edge_list)
    #     # print(car1_vertex_list)
    #     # print(car2_vertex_list)

    #     #可視グラフ, ダイクストラ法を実行
    #     car1_vis_graph = Execution.visibility_graph(car1_vertex_list, car1_vw_line_list)
    #     car2_vis_graph = Execution.visibility_graph(car2_vertex_list, car2_vw_line_list)
    #     car3_vis_graph = Execution.visibility_graph(car3_vertex_list, car3_vw_line_list)
    #     car4_vis_graph = Execution.visibility_graph(car4_vertex_list, car4_vw_line_list)

    #     # print(car1_vis_graph)
    #     # print(car2_vis_graph)
    #     # print(car3_vis_graph)
    #     # print(car4_vis_graph)

    #     car1_shortest_path, car1_shortest_length = Execution.dijkstra(car1_vis_graph)
    #     car2_shortest_path, car2_shortest_length = Execution.dijkstra(car2_vis_graph)
    #     car3_shortest_path, car3_shortest_length = Execution.dijkstra(car3_vis_graph)
    #     car4_shortest_path, car4_shortest_length = Execution.dijkstra(car4_vis_graph)

    #     for path in car1_shortest_path:
    #         print("car1 :",car1_vertex_list[path])

    #     for path in car2_shortest_path:
    #         print("car2 :",car2_vertex_list[path])
            
    #     for path in car3_shortest_path:
    #         print("car3 :",car3_vertex_list[path])

    #     for path in car4_shortest_path:
    #         print("car4 :",car4_vertex_list[path])
            
    #     #車両の衝突判定
    #     car1_move_list = Environment.car_move(car1_vertex_list, car1_shortest_path)
    #     car2_move_list = Environment.car_move(car2_vertex_list, car2_shortest_path)
    #     car3_move_list = Environment.car_move(car3_vertex_list, car3_shortest_path)
    #     car4_move_list = Environment.car_move(car4_vertex_list, car4_shortest_path)

    #     collision = Environment.cars_collision(car1_move_list,car2_move_list,car3_move_list,car4_move_list)
            
    #     print("collision::"+str(collision))

    #     total_num_obstacles = len(car1_vw_list)/4 + len(car2_vw_list)/4 + len(car3_vw_list)/4 + len(car4_vw_list)/4
    #     #print(total_num_obstacles)

    #     #全ての経路長を足す
    #     all_path_length = car1_shortest_length + car2_shortest_length + car3_shortest_length + car4_shortest_length
    #     print("all_len::"+str(all_path_length))

    #     print(all_path_length * (total_num_obstacles / (setting.car_num * (setting.VWnum ** 2))) + collision * 100000)
    #     return all_path_length * (total_num_obstacles / (setting.car_num * (setting.VWnum ** 2))) + collision * 100000, collision, all_path_length

    def two_steps_ga_setting(best):
        """
        GeneticalAlgorism用の関数
        """

        two_steps_list = []

        for index, cell in enumerate(best):
            if cell == 1:
                two_steps_list.append(index)
        
        # print(two_steps_list)
        
        two_VWnum = 3
        gene_size = len(two_steps_list) * setting.VWnum
        VWsize = setting.VWsize/two_VWnum
        zeros_list = [[[0] * (two_VWnum)**2] * 9]

        return two_steps_list, zeros_list
    
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
            print(f"VWモードを{mode}に設定しました")
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
        
        # 結果を適切な場所に保存
        if result_type == "common":
            self.common_vw_vertices = vertices
            self.common_vw_lines = lines
        else:  # individual
            self.individual_vw_vertices[direction][lane_idx] = vertices
            self.individual_vw_lines[direction][lane_idx] = lines
    
    def get_vw_for_vehicle(self, vehicle_spawn_info=None):
        """車両のスポーン情報に基づいて適切なVWを取得"""
        if self.vw_mode == "common":
            return self.common_vw_vertices, self.common_vw_lines
        else:  # individual
            if vehicle_spawn_info is None:
                raise ValueError("個別VWモードでは車両のスポーン情報が必要です")
            
            direction = vehicle_spawn_info["direction"]
            lane_idx = vehicle_spawn_info["lane_idx"]
            
            return (self.individual_vw_vertices[direction][lane_idx],
                    self.individual_vw_lines[direction][lane_idx])

class VehicleWithSpawnInfo(Vehicle):
    """スポーン情報を持つ車両クラス"""
    def __init__(self, start, goal, vehicle_id=0, vehicle_type="straight", 
                 spawn_direction=None, spawn_lane_idx=None):
        super().__init__(start, goal, vehicle_id, vehicle_type)
        self.spawn_direction = spawn_direction
        self.spawn_lane_idx = spawn_lane_idx
        self.spawn_info = {
            "direction": spawn_direction,
            "lane_idx": spawn_lane_idx
        }

class CarSimulationWithVWManager(CarSimulation):
    """VWマネージャーを統合した車両シミュレーションクラス"""
    def __init__(self):
        super().__init__()
        self.vw_manager = VirtualWallManager(self.config)
        self.wall_vertices = []
        self.wall_lines = []
        self.setup_walls()
    
    def setup_walls(self):
        """壁や静的障害物の設定"""
        # 既存のEnvironmentクラスを使用
        self.wall_vertices, self.wall_lines = Environment.set_wall()
    
    def compute_path_for_vehicle(self, vehicle_idx):
        """特定の車両の経路を計算（個別VWに対応）"""
        vehicle = self.vehicles[vehicle_idx]
        if vehicle.reached:
            return
        
        # 車両のスポーン情報に基づいてVWを取得
        spawn_info = getattr(vehicle, 'spawn_info', None)
        vw_vertices, vw_lines = self.vw_manager.get_vw_for_vehicle(spawn_info)
        
        # CarAgentを作成
        car_agent = CarAgent(vehicle.start, vehicle.goal)
        
        # 頂点リストを作成
        vertex_list = Environment.set_vertex_list(vw_vertices, car_agent, self.wall_vertices)
        
        # 障害物線のリストを作成
        obstacle_lines = vw_lines.copy()
        obstacle_lines.extend(self.wall_lines)
        
        # 既存のExecutionクラスの可視グラフとダイクストラ法を使用
        visibility_graph = Execution.visibility_graph(vertex_list, obstacle_lines)
        shortest_path, shortest_length = Execution.dijkstra(visibility_graph)
        
        # 経路をウェイポイントに変換
        waypoints = [vertex_list[node_idx] for node_idx in shortest_path]
        
        vehicle.set_waypoints(waypoints)
        print(f"車両{vehicle_idx}の経路: 長さ={shortest_length:.2f}, ウェイポイント数={len(waypoints)}")

class Environment():
    def __init__(self, obstacle_x, obstacle_y, width, height):
        self.x = obstacle_x
        self.y = obstacle_y
        self.width = width
        self.height = height
    
    def set_wall():
        wall_edge_list = setting.wall_edge_list
        wall_line_list = setting.wall_line_list
        return wall_edge_list, wall_line_list

    def collision_CarToCar(car1, car2, car3, car4, collision):
        r = np.sqrt((setting.car_length/2)**2 + (setting.car_width/2)**2)

        collision_checker1to2 = np.sqrt((car2[0]-car1[0])**2 + (car2[1]-car1[1])**2)
        collision_checker1to3 = np.sqrt((car3[0]-car1[0])**2 + (car3[1]-car1[1])**2)
        collision_checker1to4 = np.sqrt((car4[0]-car1[0])**2 + (car4[1]-car1[1])**2)
        collision_checker2to3 = np.sqrt((car2[0]-car3[0])**2 + (car2[1]-car3[1])**2)
        collision_checker2to4 = np.sqrt((car2[0]-car4[0])**2 + (car2[1]-car4[1])**2)
        collision_checker3to4 = np.sqrt((car2[0]-car4[0])**2 + (car2[1]-car4[1])**2)
        
        if collision_checker1to2 <= r:
            collision += 1
        elif collision_checker1to3 <= r:
            collision += 1
        elif collision_checker1to4 <= r:
            collision += 1
        elif collision_checker2to3 <= r:
            collision += 1
        elif collision_checker2to4 <= r:
            collision += 1
        elif collision_checker3to4 <= r:
            collision += 1

        return collision
    
    def set_vertex_list(obstacle_list, carAgent, wall_edge):
        start = carAgent.start.copy()
        goal = carAgent.goal.copy()
        vertex_list = [start, goal]
        vertex_list.extend(obstacle_list)
        vertex_list.extend(wall_edge)
        return vertex_list

    def set_vertex_list(obstacle_list, carAgent, wall_edge):
        """
        
        頂点のリストを作成し返す関数
        """
        start = carAgent.start.copy()
        goal = carAgent.goal.copy()

        vertex_list = [start, goal]

        vertex_list.extend(obstacle_list)
        vertex_list.extend(wall_edge)

        return vertex_list

class CarAgent():
    def __init__(self, start, goal):
        self.start = start
        self.goal = goal
        self.x = start[0]
        self.y = start[1]
        self.position = start
        self.goal_x = goal[0]
        self.goal_y = goal[1]
        self.goal_flag = False
        self.car_width = setting.car_width

    def car_move(self, car_vertex_list, car_shortest_path, car_position, num, move_count, need_move, curve_rate, curve_count):
        next_num = 0
        car_angle = setting.car_angle
        
        if num >= len(car_shortest_path):
            self.goal_flag = True
        
        elif num < len(car_shortest_path):
            node = car_vertex_list[car_shortest_path[num]]
            if num == 0:
                if len(car_shortest_path) > num+2:
                    after_node = car_vertex_list[car_shortest_path[num+1]]
                    curve_angle, length = calculate_two_vec_angle(node, after_node, car_vertex_list[car_shortest_path[num+2]])
                    if curve_angle <= car_angle:
                        car_angle = curve_angle
                    else:
                        car_angle = setting.car_angle
                    need_move = math.ceil(length/setting.speed)
                    if curve_rate <= 0:
                        curve_rate = 0
                    else:
                        curve_rate = math.ceil(curve_angle/car_angle)
                else:
                    need_move = -1
                    curve_rate = 0
            
            if abs(node[0] - car_position[0]) == 0:
                car_position[0] += setting.speed
            elif abs(node[1] - car_position[1]) == 0:
                car_position[1] += setting.speed
            else:
                if (need_move - move_count) <= curve_rate and curve_rate - curve_count != 0:
                    rad = np.arctan(abs(node[1] - car_position[1])/abs(node[0] - car_position[0])) + np.deg2rad(car_angle)
                    curve_count += 1
                else:
                    rad = np.arctan(abs(node[1] - car_position[1])/abs(node[0] - car_position[0]))
                
                if  car_position[0] > node[0] and car_position[1] > node[1]:
                    car_position[0] -= np.cos(rad) * setting.speed
                    car_position[1] -= np.sin(rad) * setting.speed
                    if car_position[0] <= node[0] or car_position[1] <= node[1]:
                        next_num= num + 1
                elif car_position[0] < node[0] and car_position[1] > node[1]:
                    car_position[0] += np.cos(rad) * setting.speed
                    car_position[1] -= np.sin(rad) * setting.speed
                    if car_position[0] >= node[0] or car_position[1] <= node[1]:
                        next_num= num + 1
                elif car_position[0] > node[0] and car_position[1] < node[1]:
                    car_position[0] -= np.cos(rad) * setting.speed
                    car_position[1] += np.sin(rad) * setting.speed
                    if car_position[0] <= node[0] or car_position[1] >= node[1]:
                        next_num= num + 1
                elif car_position[0] < node[0] and car_position[1] < node[1]:
                    car_position[0] += np.cos(rad) * setting.speed
                    car_position[1] += np.sin(rad) * setting.speed
                    if car_position[0] >= node[0] or car_position[1] >= node[1]:
                        next_num= num + 1
            
            move_count += 1
            
            if num >= next_num:
                next_num = num
            else:
                next_num = num + 1
                curve_count = 0
                if len(car_shortest_path) > num+2:
                    after_node = car_vertex_list[car_shortest_path[num+1]]
                    curve_angle, length = calculate_two_vec_angle(node, after_node, car_vertex_list[car_shortest_path[num+2]])
                    if curve_angle <= car_angle:
                        car_angle = curve_angle
                    else:
                        car_angle = setting.car_angle
                    need_move = math.ceil(length/setting.speed)
                    if curve_rate <= 0:
                        curve_rate = 0
                    else:
                        curve_rate = math.ceil(curve_angle/car_angle)
                else:
                    need_move = -1
                    curve_rate = 0
            
        return car_position ,self.goal_flag, next_num, move_count, need_move, curve_rate, curve_count

class Execution():
    def set_obstacle(self):
        self.Obstacle_1 = Environment()
        self.Obstacle_2 = Environment()
        self.Obstacle_3 = Environment()
        self.Obstacle_4 = Environment()

    def visibility_graph(vertex_list, obstacle_line_list):
        visibility_graph_list = []
        
        for index, vertex_u in enumerate(vertex_list):
            for goal_index, vertex_v in enumerate(vertex_list[index + 1:], index + 1):
                Line = [index,goal_index]
                cross = False
                
                for obstacle_Line in obstacle_line_list:
                    s = (vertex_v[0] - vertex_u[0])*(obstacle_Line[0][1] - vertex_u[1]) - (obstacle_Line[0][0] - vertex_u[0]) * (vertex_v[1] - vertex_u[1])
                    t = (vertex_v[0] - vertex_u[0])*(obstacle_Line[1][1] - vertex_u[1]) - (obstacle_Line[1][0] - vertex_u[0]) * (vertex_v[1] - vertex_u[1])
                    
                    if s * t < 0:
                        cross = True
                        continue
                
                if cross == False:
                    Line.append(np.sqrt(((vertex_v[0] - vertex_u[0])**2 + (vertex_v[1] - vertex_u[1])**2)))
                    visibility_graph_list.append(tuple(Line))
        
        return visibility_graph_list

    def dijkstra(visibility_graph_list):
        nx_Graph = nx.Graph()
        nx_Graph.add_weighted_edges_from(visibility_graph_list)
        origin_node = 0
        destination_node = 1
        shortest_path = nx.dijkstra_path(nx_Graph,origin_node,destination_node)
        shortest_length = nx.dijkstra_path_length(nx_Graph,origin_node,destination_node)
        return shortest_path, shortest_length
  
def main():
    best, best_gene, genelation_list = ga.main(setting.population_size, setting.generation_size, setting.genom_size)
    
    print("最適化結果:")
    print("----------")
    print("genom::", best_gene.genom)
    print("fitness::", best_gene.get_fitness())
    print("collision::", best_gene.get_collision())
    print("path_length::", best_gene.get_all_path_length())
    print("adhoc_avoidance_count::", best_gene.get_adhoc_avoidance_count())
    print("total_num_obstacles::", int(best_gene.get_total_num_obstacles()))
    
    return best, best_gene, genelation_list
        
def combining_vw(Vw_list):
    """
    
    list内の重複を消す関数,VWの4点のいずれかが重複していたら削除しVWの疑似的な結合を行う
    """
    seen = []
    return [position for position in Vw_list if position not in seen and not seen.append(position)]

def calculate_two_vec_angle(pre_position, position, move_position):
    pre_position = np.array(pre_position)
    position = np.array(position)
    move_position = np.array(move_position)
    
    vec_a = np.array(position-pre_position)
    vec_b = np.array(move_position-position)
    inner = np.inner(vec_a, vec_b)
    vec_a_norm = np.linalg.norm(vec_a)
    vec_b_norm = np.linalg.norm(vec_b)
    vec_a_norm = round(vec_a_norm,5)
    vec_b_norm = round(vec_b_norm,5)
    theta = inner/(vec_a_norm*vec_b_norm)
    after_angle = np.rad2deg(np.arccos(np.clip(theta, -1.0, 1.0)))
    
    return after_angle, vec_a_norm

def calculate_safety_margin(speed):
    """
    車両の速度の二乗に基づいて安全マージンを計算する
    """
    # 基本マージン（車両サイズに基づく最小距離）
    base_margin = setting.car_width
    
    # 反応距離（速度に線形比例）
    reaction_time = 2.0  # シミュレーションステップ数
    reaction_distance = speed * reaction_time
    
    # 制動距離（速度の二乗に比例）
    braking_factor = 0.5  # 制動係数
    braking_distance = braking_factor * (speed ** 2)
    
    # 合計安全マージン
    safety_margin = base_margin + reaction_distance + braking_distance
    
    return safety_margin

# 簡略化されたCarSimulationクラス - アドホック回避機能に特化
# SimpleCarSimulationクラスの修正

class SimpleCarSimulation:
    """
    アドホック回避機能を持ち、計算された経路に沿って動く車両シミュレーション
    可視化機能は含まない、遺伝的アルゴリズム評価用
    """
    def __init__(self):
        # シミュレーション設定
        self.car_width = setting.car_length
        self.car_length = setting.car_width
        self.base_speed = 3.0  # 通常速度（目標速度）
        
        # 交差点座標
        self.intersection_center = [450, 250]
        self.intersection_size = 70
        
        # 安全マージン計算のパラメータ
        self.reaction_time = 2.0
        self.braking_factor = 0.5
        
        # シミュレーション状態
        self.time_steps = 0
        self.max_steps = 300
        self.use_adhoc = True  # アドホック回避の有効/無効
        
        # 車両の初期位置と目標位置を設定ファイルから取得
        self.cars = [
            {"start": setting.car1_STARTtoGOAL[0].copy(), "goal": setting.car1_STARTtoGOAL[1].copy(), 
             "position": setting.car1_STARTtoGOAL[0].copy(), 
             "velocity": [0, 0], "target_velocity": [0, 0], 
             "reached": False, "id": 0, "stopped_time": 0, "slowed_down": False,
             "path_idx": 0, "following_path": True},
            
            {"start": setting.car2_STARTtoGOAL[0].copy(), "goal": setting.car2_STARTtoGOAL[1].copy(), 
             "position": setting.car2_STARTtoGOAL[0].copy(),
             "velocity": [0, 0], "target_velocity": [0, 0], 
             "reached": False, "id": 1, "stopped_time": 0, "slowed_down": False,
             "path_idx": 0, "following_path": True},
            
            {"start": setting.car3_STARTtoGOAL[0].copy(), "goal": setting.car3_STARTtoGOAL[1].copy(), 
             "position": setting.car3_STARTtoGOAL[0].copy(),
             "velocity": [0, 0], "target_velocity": [0, 0], 
             "reached": False, "id": 2, "stopped_time": 0, "slowed_down": False,
             "path_idx": 0, "following_path": True},
            
            {"start": setting.car4_STARTtoGOAL[0].copy(), "goal": setting.car4_STARTtoGOAL[1].copy(), 
             "position": setting.car4_STARTtoGOAL[0].copy(),
             "velocity": [0, 0], "target_velocity": [0, 0], 
             "reached": False, "id": 3, "stopped_time": 0, "slowed_down": False,
             "path_idx": 0, "following_path": True}
        ]
        
        # 初期速度ベクトルを計算
        for car in self.cars:
            self._calculate_initial_velocity(car)
        
        # 車両の優先度 (デッドロック解決のため)
        self.priorities = [0, 0, 0, 0]
        
        # 衝突回数
        self.collision_count = 0
        
        # 経路情報（後で設定される）
        self.paths = [[], [], [], []]
        self.vertex_lists = [[], [], [], []]

    def _calculate_initial_velocity(self, car):
        """
        スタートとゴールの位置から適切な初期速度ベクトルを計算
        """
        start = car["start"]
        goal = car["goal"]
        
        # 方向ベクトルを計算
        dx = goal[0] - start[0]
        dy = goal[1] - start[1]
        
        # ベクトルの長さを計算
        length = np.sqrt(dx**2 + dy**2)
        
        # ゼロ除算を回避
        if length > 0:
            # 方向ベクトルを正規化し、基本速度を掛ける
            dx = dx / length * self.base_speed
            dy = dy / length * self.base_speed
        else:
            dx, dy = 0, 0
        
        # 速度ベクトルを設定
        car["velocity"] = [dx, dy]
        car["target_velocity"] = [dx, dy]
    
    def set_paths(self, paths):
        """
        車両の経路情報を設定
        paths: 各車両のノードインデックスのリスト [car1_path, car2_path, ...]
        """
        self.paths = paths
        
        # 経路が設定されたことを記録
        for i, car in enumerate(self.cars):
            if len(paths[i]) > 0:
                car["following_path"] = True
                car["path_idx"] = 0
            else:
                car["following_path"] = False
    
    def set_vertex_lists(self, vertex_lists):
        """
        頂点リストを設定
        vertex_lists: 各車両の頂点座標のリスト [car1_vertices, car2_vertices, ...]
        """
        self.vertex_lists = vertex_lists
    
    def update_velocity_for_path_following(self, car_idx):
        """
        経路追従のための速度を更新
        """
        car = self.cars[car_idx]
        
        # 経路追従モードでない場合は処理しない
        if not car["following_path"]:
            return
        
        # 経路情報がない場合は処理しない
        if len(self.paths[car_idx]) == 0 or len(self.vertex_lists[car_idx]) == 0:
            return
        
        # 現在の目標ノードのインデックス
        path_idx = car["path_idx"]
        
        # 経路の終端に達した場合
        if path_idx >= len(self.paths[car_idx]):
            car["velocity"] = [0, 0]
            car["target_velocity"] = [0, 0]
            car["reached"] = True
            return
        
        # 目標ノードの座標を取得
        target_node_idx = self.paths[car_idx][path_idx]
        target_position = self.vertex_lists[car_idx][target_node_idx]
        
        # 現在位置と目標位置の差分
        dx = target_position[0] - car["position"][0]
        dy = target_position[1] - car["position"][1]
        
        # 目標地点までの距離
        distance = np.sqrt(dx**2 + dy**2)
        
        # 目標ノードに十分近づいた場合、次のノードへ
        if distance < 10:  # 10ピクセル以内なら到達とみなす
            car["path_idx"] += 1
            
            # 次のノードがある場合、そのノードへの速度を計算
            if car["path_idx"] < len(self.paths[car_idx]):
                next_node_idx = self.paths[car_idx][car["path_idx"]]
                next_position = self.vertex_lists[car_idx][next_node_idx]
                
                dx = next_position[0] - car["position"][0]
                dy = next_position[1] - car["position"][1]
                distance = np.sqrt(dx**2 + dy**2)
            else:
                # 経路終了
                car["velocity"] = [0, 0]
                car["target_velocity"] = [0, 0]
                car["reached"] = True
                return
        
        # 目標方向の単位ベクトルを計算
        if distance > 0:
            dx /= distance
            dy /= distance
        
        # 目標速度を設定（基本速度×方向ベクトル）
        target_velocity = [dx * self.base_speed, dy * self.base_speed]
        car["target_velocity"] = target_velocity
        
        # 現在の速度が大幅に変わらないように調整（急な方向転換を避ける）
        current_speed = np.linalg.norm(car["velocity"])
        if current_speed > 0:
            # 現在の速度と目標速度を徐々に近づける
            car["velocity"][0] = car["velocity"][0] * 0.8 + target_velocity[0] * 0.2
            car["velocity"][1] = car["velocity"][1] * 0.8 + target_velocity[1] * 0.2
        else:
            # 停止状態から始める場合はそのまま目標速度を使用
            car["velocity"] = target_velocity.copy()
    
    def calculate_safety_margin(self, speed):
        """車両の速度の二乗に基づいて安全マージンを計算"""
        base_margin = setting.car_length
        reaction_distance = speed * self.reaction_time
        braking_distance = self.braking_factor * (speed ** 2)
        return base_margin + reaction_distance + braking_distance
    
    def calculate_distance_to_goal(self, car_idx):
        """目的地までの距離を計算"""
        car = self.cars[car_idx]
        return np.sqrt(
            (car["position"][0] - car["goal"][0])**2 + 
            (car["position"][1] - car["goal"][1])**2
        )
    
    def is_in_intersection(self, position):
        """車両が交差点内にいるかチェック"""
        x, y = position
        ix, iy = self.intersection_center
        half_size = self.intersection_size / 2
        
        return (ix - half_size <= x <= ix + half_size and 
                iy - half_size <= y <= iy + half_size)
    
    def is_almost_stopped(self, velocity):
        """車両がほぼ停止しているかチェック"""
        speed = np.linalg.norm(velocity)
        return speed < 0.5  # 速度が0.5未満であれば「ほぼ停止」と判断
    
    def predict_collisions(self):
        """すべての車両ペア間の将来の衝突を予測（進行方向を考慮）"""
        predictions = []
        positions = [car["position"] for car in self.cars]
        velocities = [car["velocity"] for car in self.cars]
        
        # 将来の位置を予測
        future_positions = []
        for i in range(len(self.cars)):
            if self.cars[i]["reached"]:
                future_positions.append([])
                continue
                
            vehicle_future = []
            pos = positions[i].copy()
            vel = velocities[i].copy()
            
            for step in range(10):  # 10ステップ先まで予測
                pos = [pos[0] + vel[0], pos[1] + vel[1]]
                vehicle_future.append(pos)
            
            future_positions.append(vehicle_future)
        
        # 車両ペア間の衝突を予測
        for i in range(len(self.cars)):
            if self.cars[i]["reached"]:
                continue
                
            for j in range(i+1, len(self.cars)):
                if self.cars[j]["reached"]:
                    continue
                    
                # 速度の大きさを計算
                speed_i = np.linalg.norm(velocities[i])
                speed_j = np.linalg.norm(velocities[j])
                
                # 進行方向の相対角度を計算
                v1 = np.array(velocities[i])
                v2 = np.array(velocities[j])
                
                # ゼロ除算を回避
                angle = 0
                if speed_i > 0 and speed_j > 0:
                    cos_angle = np.dot(v1, v2) / (speed_i * speed_j)
                    angle = np.arccos(np.clip(cos_angle, -1.0, 1.0))
                
                # 相対速度に基づく安全マージンを計算
                safety_margin = self.calculate_relative_safety_margin(speed_i, speed_j, angle)
                
                # 将来の位置で衝突をチェック
                min_future_steps = min(len(future_positions[i]), len(future_positions[j]))
                for step in range(min_future_steps):
                    pos1 = future_positions[i][step]
                    pos2 = future_positions[j][step]
                    dist = np.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
                    
                    if dist < (self.car_width + safety_margin):
                        predictions.append({
                            "vehicles": (i, j),
                            "time_step": step,
                            "distance": dist
                        })
                        break
        
        return predictions
    
    def calculate_relative_safety_margin(self, speed1, speed2, angle):
        """2台の車両間の相対速度と角度に基づく安全マージンを計算"""
        # 各車両の個別の安全マージン
        margin1 = self.calculate_safety_margin(speed1)
        margin2 = self.calculate_safety_margin(speed2)
        
        # 相対速度の計算（角度を考慮）
        relative_factor = (1 - np.cos(angle)) / 2  # 0（同方向）〜1（正面衝突）
        
        # 角度が大きいほど安全マージンを大きくする
        angle_factor = 1.0 + relative_factor * 0.5
        
        return (margin1 + margin2) * angle_factor
    
    def update_priorities(self):
        """車両の優先度を更新"""
        # 各車両の状態を確認
        for i, car in enumerate(self.cars):
            if car["reached"]:
                continue
                
            # 交差点内にいる車両の優先度を高める
            if self.is_in_intersection(car["position"]):
                self.priorities[i] += 2
            
            # 目的地に近い車両の優先度を高める
            dist_to_goal = self.calculate_distance_to_goal(i)
            if dist_to_goal < 100:  # 目的地が近い
                self.priorities[i] += 1
            
            # 長時間停止している車両の優先度を高める
            if self.is_almost_stopped(car["velocity"]):
                car["stopped_time"] += 1
                if car["stopped_time"] > 10:  # 10ステップ以上停止している
                    self.priorities[i] += 3
            else:
                car["stopped_time"] = 0  # リセット
    
    def adjust_velocity(self, predictions):
        """衝突予測に基づいて車両の速度を調整（アドホック回避カウント付き）"""
        # 優先度を更新
        self.update_priorities()
        
        # アドホック回避カウント
        adhoc_avoidance_count = 0
        
        # 衝突回避が必要な車両を記録
        cars_to_slow_down = set()
        
        for prediction in predictions:
            i, j = prediction["vehicles"]
            time_step = prediction["time_step"]
            
            # 優先度を比較
            if self.priorities[i] > self.priorities[j]:
                # 車両jが減速
                cars_to_slow_down.add(j)
                
                speed = np.linalg.norm(self.cars[j]["velocity"])
                urgency_factor = 1.0 / (time_step + 1)
                slow_factor = max(0.3, 1.0 - (urgency_factor * 0.3))
                
                if speed > 0:
                    self.cars[j]["velocity"][0] *= slow_factor
                    self.cars[j]["velocity"][1] *= slow_factor
                    self.cars[j]["slowed_down"] = True
                    
                    # アドホック回避をカウント
                    adhoc_avoidance_count += 1
            else:
                # 車両iが減速
                cars_to_slow_down.add(i)
                
                speed = np.linalg.norm(self.cars[i]["velocity"])
                urgency_factor = 1.0 / (time_step + 1)
                slow_factor = max(0.3, 1.0 - (urgency_factor * 0.3))
                
                if speed > 0:
                    self.cars[i]["velocity"][0] *= slow_factor
                    self.cars[i]["velocity"][1] *= slow_factor
                    self.cars[i]["slowed_down"] = True
                    
                    # アドホック回避をカウント
                    adhoc_avoidance_count += 1
        
        # 衝突の危険がない車両は徐々に元の速度に戻す
        for i, car in enumerate(self.cars):
            if car["reached"] or not car["slowed_down"]:
                continue
                
            if i not in cars_to_slow_down:
                target_vel = car["target_velocity"]
                current_vel = car["velocity"]
                
                # 現在の速度ベクトルから大きさと方向を取得
                current_speed = np.linalg.norm(current_vel)
                target_speed = np.linalg.norm(target_vel)
                
                if current_speed < target_speed:
                    # 徐々に加速（ターゲット速度の10%ずつ加速）
                    acceleration_rate = 0.1
                    new_speed = min(current_speed + (target_speed * acceleration_rate), target_speed)
                    
                    # 方向ベクトルを計算（現在の方向を維持）
                    if current_speed > 0:
                        direction = [current_vel[0] / current_speed, current_vel[1] / current_speed]
                    else:
                        direction = [target_vel[0] / target_speed, target_vel[1] / target_speed]
                    
                    # 新しい速度ベクトルを設定
                    car["velocity"][0] = direction[0] * new_speed
                    car["velocity"][1] = direction[1] * new_speed
                    
                    # 元の速度に十分近づいたら「減速状態」をリセット
                    if new_speed >= target_speed * 0.95:
                        car["slowed_down"] = False
        
        return adhoc_avoidance_count
    
    def check_for_deadlock(self):
        """デッドロックを検出し解決を試みる"""
        # デッドロック検出: 複数の車両が交差点内で長時間停止している
        stopped_cars_in_intersection = []
        
        for i, car in enumerate(self.cars):
            if car["reached"]:
                continue
                
            if (self.is_in_intersection(car["position"]) and 
                self.is_almost_stopped(car["velocity"]) and 
                car["stopped_time"] > 20):
                stopped_cars_in_intersection.append(i)
        
        # デッドロック解決: ランダムに1台の車両を一時的に高優先度にする
        if len(stopped_cars_in_intersection) >= 2:
            car_to_prioritize = random.choice(stopped_cars_in_intersection)
            
            # 優先車両の速度を元に戻し、他の車両を減速
            self.cars[car_to_prioritize]["velocity"] = self.cars[car_to_prioritize]["target_velocity"].copy()
            self.priorities[car_to_prioritize] += 10  # 大幅に優先度を上げる
            self.cars[car_to_prioritize]["slowed_down"] = False
            
            # 他の停止車両を減速
            for car_idx in stopped_cars_in_intersection:
                if car_idx != car_to_prioritize:
                    self.cars[car_idx]["velocity"][0] *= 0.2
                    self.cars[car_idx]["velocity"][1] *= 0.2
                    self.cars[car_idx]["slowed_down"] = True
    
    def check_collisions(self):
        """シミュレーション中の実際の衝突をチェック"""
        new_collisions = 0
        for i in range(len(self.cars)):
            if self.cars[i]["reached"]:
                continue
                
            for j in range(i+1, len(self.cars)):
                if self.cars[j]["reached"]:
                    continue
                    
                dist = np.sqrt(
                    (self.cars[i]["position"][0] - self.cars[j]["position"][0])**2 + 
                    (self.cars[i]["position"][1] - self.cars[j]["position"][1])**2
                )
                
                if dist < self.car_width:
                    new_collisions += 1
        
        return new_collisions
    
    def check_goal_reached(self, car_idx):
        """車両が目標位置に到達したかチェック"""
        car = self.cars[car_idx]
        
        # 経路追従モードの場合、最後のノードに到達したかどうかで判定
        if car["following_path"] and len(self.paths[car_idx]) > 0:
            return car["path_idx"] >= len(self.paths[car_idx])
        
        # 経路追従でない場合、目標位置からの距離で判定
        goal_dist = np.sqrt(
            (car["position"][0] - car["goal"][0])**2 + 
            (car["position"][1] - car["goal"][1])**2
        )
        
        return goal_dist < 10  # 目標位置から10px以内なら到達とみなす
    
    def update_car_positions(self):
        """車両の位置を更新"""
        for i, car in enumerate(self.cars):
            if car["reached"]:
                continue
            
            # 経路追従モードの場合、経路に沿った速度を計算
            if car["following_path"]:
                self.update_velocity_for_path_following(i)
                
            # 車両が目標に到達したかチェック
            if self.check_goal_reached(i):
                car["reached"] = True
                car["velocity"] = [0, 0]
                continue
            
            # 車両の位置を更新
            car["position"][0] += car["velocity"][0]
            car["position"][1] += car["velocity"][1]
    
    def run_simulation(self):
        """シミュレーションを実行してアドホック回避回数を返す（可視化なし）"""
        # アドホック回避の総数をカウント
        total_adhoc_avoidances = 0
        
        # シミュレーションループ
        while not all(car["reached"] for car in self.cars) and self.time_steps < self.max_steps:
            self.time_steps += 1
            
            # アドホック回避が有効な場合、衝突予測と速度調整を行う
            if self.use_adhoc:
                # デッドロック検出・解決
                self.check_for_deadlock()
                
                # 衝突予測と速度調整
                collision_predictions = self.predict_collisions()
                if collision_predictions:
                    # アドホック回避のカウント
                    adhoc_count = self.adjust_velocity(collision_predictions)
                    total_adhoc_avoidances += adhoc_count
                    
                else:
                    # 衝突がなければ減速した車両を元の速度に戻す処理も実行
                    self.adjust_velocity([])
            
            # 車両の位置を更新
            self.update_car_positions()
            
            # 衝突検出
            self.collision_count += self.check_collisions()
        
        return self.time_steps, self.collision_count, total_adhoc_avoidances

if __name__ == '__main__':
    sum_fitness = 0
    sum_collision = 0
    sum_all_path_length = 0
    sum_total_num_obstacles = 0
    sum_time = 0
    
    start = time.time()
    best, best_gene, genelation_list = main()
    end = time.time()
    
    time_diff = end - start
    print("all_time:" , time_diff)
    sum_time += time_diff
    print(best_gene)
    print(list(best_gene))

    # ga.create_graph_best_all_path_length(best)
    # # ga.create_graph_best_fitness(best)
        
    #     #結果のファイルへの書き込み処理      
    # f = open('data_test_ga.txt', 'a', encoding='UTF-8')
    # f.writelines('\n')
    # f.writelines("population_size::" + str(setting.population_size) + "," + "generation_size::" + str(setting.generation_size))
    # f.writelines('\n')
    # f.writelines("genom::" + str(best_gene.genom))
    # f.writelines('\n')
    # f.writelines("fitness::"+str(best_gene.get_fitness()))
    # f.writelines('\n')
    # f.writelines("collision::"+str(best_gene.get_collision()))
    # f.writelines('\n')
    # f.writelines("path_length::"+str(best_gene.get_all_path_length()))
    # f.writelines('\n')
    # f.writelines("total_num_obstacles::"+str(int(best_gene.get_total_num_obstacles())))
    # f.writelines('\n')
    # sum_fitness += best_gene.get_fitness()
    # sum_collision += best_gene.get_collision()
    # sum_all_path_length += best_gene.get_all_path_length()
    # sum_total_num_obstacles += best_gene.get_total_num_obstacles()
    # # for i in best:
    # #     print(len(best))
    # #     f.writelines("\n")
    # #     f.writelines(str(i.get_all_path_length()))
    
    # ave_fitness = sum_fitness/100
    # ave_collision = sum_collision/100
    # ave_all_path_length = sum_all_path_length/100
    # ave_total_num_obstacles = sum_total_num_obstacles/100
    # ave_time = sum_time/100
    # f = open('data_test_ga.txt', 'a', encoding='UTF-8')
    # f.writelines('\n')
    # f.writelines("ave_fitness::"+str(ave_fitness))
    # f.writelines('\n')
    # f.writelines("ave_collision::"+str(ave_collision))
    # f.writelines('\n')
    # f.writelines("ave_path_length::"+str(ave_all_path_length))
    # f.writelines('\n')
    # f.writelines("ave_total_num_obstacles::"+str(ave_total_num_obstacles))
    # f.writelines('\n')
    # f.writelines("ave_time::" + str(ave_time))