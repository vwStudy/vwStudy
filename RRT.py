import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import random


class RRTVisualizerWithObstacles:
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
        """Initialize the plot with start, goal, and obstacles."""
        self.ax.plot(self.start[0], self.start[1], "go", label="Start")  # Start point
        self.ax.plot(self.goal[0], self.goal[1], "ro", label="Goal")  # Goal point
        self.ax.legend()
        self.draw_obstacles()

    def draw_obstacles(self):
        """Draw rectangular obstacles on the plot."""
        for obs in self.obstacles:
            rect = plt.Rectangle((obs[0], obs[1]), obs[2], obs[3], color="gray", alpha=0.7)
            self.ax.add_patch(rect)

    def random_sample(self):
        """Generate a random sample point within the space."""
        x = random.uniform(*self.space[0])
        y = random.uniform(*self.space[1])
        return np.array([x, y])

    def nearest(self, x_rand):
        """Find the nearest node in the tree to the random point."""
        distances = [np.linalg.norm(node - x_rand) for node in self.tree]
        nearest_index = np.argmin(distances)
        return self.tree[nearest_index]

    def steer(self, x_near, x_rand):
        """Move a step from x_near towards x_rand."""
        direction = x_rand - x_near
        norm = np.linalg.norm(direction)
        if norm > self.step_size:
            direction = direction / norm * self.step_size
        return x_near + direction

    def collision(self, x_near, x_new):
        """Check if the path between x_near and x_new collides with any obstacle."""
        for obs in self.obstacles:
            if self.line_intersects_rect(x_near, x_new, obs):
                return True
        return False

    def line_intersects_rect(self, p1, p2, rect):
        """Check if a line segment intersects a rectangle."""
        rect_lines = [
            ((rect[0], rect[1]), (rect[0] + rect[2], rect[1])),  # Bottom edge
            ((rect[0], rect[1]), (rect[0], rect[1] + rect[3])),  # Left edge
            ((rect[0] + rect[2], rect[1]), (rect[0] + rect[2], rect[1] + rect[3])),  # Right edge
            ((rect[0], rect[1] + rect[3]), (rect[0] + rect[2], rect[1] + rect[3]))  # Top edge
        ]
        for line in rect_lines:
            if self.line_intersection(p1, p2, line[0], line[1]):
                return True
        return False

    def line_intersection(self, p1, p2, q1, q2):
        """Check if two line segments (p1-p2 and q1-q2) intersect."""
        def ccw(a, b, c):
            return (c[1] - a[1]) * (b[0] - a[0]) > (b[1] - a[1]) * (c[0] - a[0])

        return ccw(p1, q1, q2) != ccw(p2, q1, q2) and ccw(p1, p2, q1) != ccw(p1, p2, q2)

    def grow_tree(self):
        """Grow the RRT tree."""
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
        """Update the plot for animation."""
        if frame < len(self.edges):
            edge = self.edges[frame]
            self.ax.plot([edge[0][0], edge[1][0]], [edge[0][1], edge[1][1]], "b-")

    def animate(self):
        """Animate the RRT tree growth."""
        ani = FuncAnimation(self.fig, self.update_plot, frames=len(self.edges), interval=50, repeat=False)
        plt.show()


# Parameters for RRT with obstacles
space = [(0, 100), (0, 100)]  # X and Y bounds
start = (10, 10)
goal = (90, 90)
step_size = 5
max_iter = 500

# Define obstacles as a list of rectangles [x, y, width, height]
obstacles = [
    (30, 30, 20, 10),
    (60, 40, 15, 30),
    (40, 70, 20, 10)
]

# Run the RRT with obstacles and visualize
visualizer = RRTVisualizerWithObstacles(space, start, goal, step_size, max_iter, obstacles)
visualizer.grow_tree()
visualizer.animate()
