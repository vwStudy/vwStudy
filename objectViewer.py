import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class AnimationVisualizer:
    def __init__(self, trajectory_file='trajectory_test.npy', obstacles_file='obstacles_test.npy', end_positions_file='end_positions_test.npy', line_length=1.0):
        self.trajectory = np.load(trajectory_file)
        self.obstacles = np.load(obstacles_file, allow_pickle=True)
        self.end_positions = np.load(end_positions_file)
        self.line_length = line_length
        self.path_lines = []

    def init(self):
        for car_circle in self.car_circles:
            car_circle.set_center((0, 0))
        for end_circle in self.end_circles:
            end_circle.set_center((0, 0))
        for obstacle_circle in self.obstacle_circles:
            obstacle_circle.set_center((0, 0))
        self.lines.set_data([], [])
        
        for path_line in self.path_lines:
            path_line.set_data([], [])
        
        return self.car_circles + self.end_circles + self.obstacle_circles + [self.lines] + self.path_lines

    def update(self, frame):
        positions = self.trajectory[frame]
        next_positions = self.trajectory[frame + 1] if frame + 1 < len(self.trajectory) else positions

        # 車の位置を更新
        for i, car_circle in enumerate(self.car_circles):
            car_circle.set_center(positions[i])
            # 軌跡の更新
            self.path_lines[i].set_data(
                np.append(self.path_lines[i].get_xdata(), positions[i][0]),
                np.append(self.path_lines[i].get_ydata(), positions[i][1])
            )

        # 障害物の位置を生成
        for i, obstacle_circle in enumerate(self.obstacle_circles):
            obstacle_circle.set_center(self.obstacles[i])

        # 終点の位置を更新
        for i, end_circle in enumerate(self.end_circles):
            end_circle.set_center(self.end_positions[i])

        # 次のステップの方向を示す実線を更新
        lines_x = []
        lines_y = []
        for i in range(len(positions)):
            direction = next_positions[i] - positions[i]
            norm_direction = direction / (np.linalg.norm(direction) + 1e-10)
            future_position = positions[i] + norm_direction * self.line_length
            lines_x.extend([positions[i][0], future_position[0], None])
            lines_y.extend([positions[i][1], future_position[1], None])

        self.lines.set_data(lines_x, lines_y)
        return self.car_circles + self.end_circles + self.obstacle_circles + [self.lines] + self.path_lines

    def animate(self):
        fig, self.ax = plt.subplots()
        self.ax.set_xlim(0, 30)
        self.ax.set_ylim(0, 30)
        self.ax.set_aspect('equal')
        
        # 円の大きさを指定
        car_radius = 1
        end_point_radius = 0.2
        obstacle_radius = 3

        self.car_circles = [plt.Circle((0, 0), car_radius, color='blue', fill=True) for _ in range(len(self.trajectory[0]))]
        self.end_circles = [plt.Circle((0, 0), end_point_radius, color='green', fill=True) for _ in range(len(self.trajectory[0]))]
        self.obstacle_circles = [plt.Circle((0, 0), obstacle_radius, color='red', fill=True) for _ in range(len(self.obstacles))]

        for circle in self.car_circles + self.end_circles + self.obstacle_circles:
            self.ax.add_patch(circle)
        
        self.lines, = self.ax.plot([], [], 'k-')  # 実線を設定
        
        # 各車の軌跡用のラインを追加
        self.path_lines = [self.ax.plot([], [], lw=2, color='blue')[0] for _ in range(len(self.trajectory[0]))]
        
        ani = FuncAnimation(fig, self.update, frames=len(self.trajectory), init_func=self.init, blit=True, repeat=False)
        plt.show()

if __name__ == "__main__":
    # for _ in range(100000000):
    #     continue
    visualizer = AnimationVisualizer('trajectory_test.npy', 'obstacles_test.npy', 'end_positions_test.npy')
    visualizer.animate()
