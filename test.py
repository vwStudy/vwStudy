import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter
import numpy as np
import random

class RRTVisualizerWithRecording:
    def __init__(self, space, start, goal, step_size, max_iter, obstacles):
        self.space = space
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.step_size = step_size
        self.max_iter = max_iter
        self.tree = [self.start]
        self.edges = []
        self.obstacles = obstacles
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(space[0])
        self.ax.set_ylim(space[1])
        self.ax.set_aspect('equal')
        self.init_plot()

    def init_plot(self):
        self.ax.plot(self.start[0], self.start[1], "go", label="Start")  # Start point
        self.ax.plot(self.goal[0], self.goal[1], "ro", label="Goal")  # Goal point
        self.ax.legend()
        for obs in self.obstacles:
            rect = plt.Rectangle((obs[0], obs[1]), obs[2], obs[3], color="gray", alpha=0.7)
            self.ax.add_patch(rect)

    def random_sample(self):
        return np.array([random.uniform(*self.space[0]), random.uniform(*self.space[1])])

    def nearest(self, x_rand):
        distances = [np.linalg.norm(node - x_rand) for node in self.tree]
        return self.tree[np.argmin(distances)]

    def steer(self, x_near, x_rand):
        direction = x_rand - x_near
        norm = np.linalg.norm(direction)
        if norm > self.step_size:
            direction = direction / norm * self.step_size
        return x_near + direction

    def collision(self, x_near, x_new):
        for obs in self.obstacles:
            if self.line_intersects_rect(x_near, x_new, obs):
                return True
        return False

    def line_intersects_rect(self, p1, p2, rect):
        rect_lines = [
            ((rect[0], rect[1]), (rect[0] + rect[2], rect[1])),
            ((rect[0], rect[1]), (rect[0], rect[1] + rect[3])),
            ((rect[0] + rect[2], rect[1]), (rect[0] + rect[2], rect[1] + rect[3])),
            ((rect[0], rect[1] + rect[3]), (rect[0] + rect[2], rect[1] + rect[3]))
        ]
        for line in rect_lines:
            if self.line_intersection(p1, p2, line[0], line[1]):
                return True
        return False

    def line_intersection(self, p1, p2, q1, q2):
        def ccw(a, b, c):
            return (c[1] - a[1]) * (b[0] - a[0]) > (b[1] - a[1]) * (c[0] - a[0])
        return ccw(p1, q1, q2) != ccw(p2, q1, q2) and ccw(p1, p2, q1) != ccw(p1, p2, q2)

    def grow_tree(self):
        for _ in range(self.max_iter):
            x_rand = self.random_sample()
            x_near = self.nearest(x_rand)
            x_new = self.steer(x_near, x_rand)
            if not self.collision(x_near, x_new):
                self.tree.append(x_new)
                self.edges.append((x_near, x_new))
                if np.linalg.norm(x_new - self.goal) < self.step_size:
                    self.tree.append(self.goal)
                    self.edges.append((x_new, self.goal))
                    break

    def update_plot(self, frame):
        if frame < len(self.edges):
            edge = self.edges[frame]
            self.ax.plot([edge[0][0], edge[1][0]], [edge[0][1], edge[1][1]], "b-")

    def animate(self):
        ani = FuncAnimation(self.fig, self.update_plot, frames=len(self.edges), interval=50, repeat=False)
        plt.show()

    def record_simulation(self, filename="rrt_simulation.mp4"):
        ani = FuncAnimation(self.fig, self.update_plot, frames=len(self.edges), interval=50, repeat=False)
        writer = FFMpegWriter(fps=20, metadata={"artist": "RRT Simulation"})
        ani.save(filename, writer=writer)
        print(f"Simulation recorded as {filename}")


# Parameters
space = [(0, 100), (0, 100)]
start = (10, 10)
goal = (90, 90)
step_size = 5
max_iter = 500
obstacles = [(30, 30, 20, 10), (60, 40, 15, 30), (40, 70, 20, 10)]

# Execution
visualizer = RRTVisualizerWithRecording(space, start, goal, step_size, max_iter, obstacles)
visualizer.grow_tree()
visualizer.record_simulation("rrt_with_obstacles.mp4")
