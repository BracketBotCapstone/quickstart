from queue import PriorityQueue

import matplotlib.pyplot as plt
import numpy as np


class DStar:
    def __init__(self):
        """Initialize the D* pathfinding algorithm with default values."""
        self.grid = None
        self.start = None
        self.goal = None
        self.path = []

    def initialize(self, grid, start, goal):
        """
        Initialize the grid, start, and goal for the pathfinding algorithm.

        :param grid: 2D numpy array representing the grid (True for obstacles, False for free space)
        :param start: Tuple representing the starting coordinates (x, y)
        :param goal: Tuple representing the goal coordinates (x, y)
        """
        self.grid = grid
        self.start = start
        self.goal = goal
        self.path = []

    def update(self, new_grid, new_start, new_goal):
        """
        Update the grid, start, and goal for the pathfinding algorithm.

        :param new_grid: 2D numpy array representing the new grid
        :param new_start: Tuple representing the new starting coordinates
        :param new_goal: Tuple representing the new goal coordinates
        """
        self.update_grid(new_grid)
        self.update_start(new_start)
        self.update_goal(new_goal)
        self.path = []

    def update_grid(self, new_grid):
        """
        Update the grid for the pathfinding algorithm.

        :param new_grid: 2D numpy array representing the new grid
        """
        self.grid = new_grid

    def update_start(self, new_start):
        """
        Update the starting point for the pathfinding algorithm.

        :param new_start: Tuple representing the new starting coordinates
        """
        self.start = new_start

    def update_goal(self, new_goal):
        """
        Update the goal point for the pathfinding algorithm.

        :param new_goal: Tuple representing the new goal coordinates
        """
        self.goal = new_goal

    def heuristic(self, a, b):
        """
        Calculate the heuristic (Euclidean distance) between two points.

        :param a: Tuple representing the first point (x, y)
        :param b: Tuple representing the second point (x, y)
        :return: Euclidean distance between point a and b
        """
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def neighbors(self, node):
        """
        Get all valid neighboring nodes for a given node.

        :param node: Tuple representing the current node (x, y)
        :return: List of tuples representing valid neighboring nodes
        """
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (-1, -1), (1, -1), (-1, 1)]
        result = []
        for d in directions:
            neighbor = (node[0] + d[0], node[1] + d[1])
            if 0 <= neighbor[0] < self.grid.shape[0] and 0 <= neighbor[1] < self.grid.shape[1]:
                result.append(neighbor)
        return result

    def run(self):
        """
        Execute the D* pathfinding algorithm to find a path from start to goal.

        :return: List of tuples representing the path from start to goal, or an empty list if no path is found
        """
        open_set = PriorityQueue()
        open_set.put((0, self.start))
        came_from = {}
        g_score = {self.start: 0}
        f_score = {self.start: self.heuristic(self.start, self.goal)}

        while not open_set.empty():
            current = open_set.get()[1]

            if current == self.goal:
                self.reconstruct_path(came_from, current)
                return self.path

            for neighbor in self.neighbors(current):
                if self.grid[neighbor[0], neighbor[1]]:
                    continue

                tentative_g_score = g_score[current] + 1

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, self.goal)
                    open_set.put((f_score[neighbor], neighbor))

        return []

    def reconstruct_path(self, came_from, current):
        """
        Reconstruct the path from start to goal using the came_from map.

        :param came_from: Dictionary mapping each node to its predecessor
        :param current: Tuple representing the current node (x, y)
        """
        while current in came_from:
            self.path.append(current)
            current = came_from[current]
        self.path.append(self.start)
        self.path.reverse()

    def plot_path(self):
        """Plot the grid and the path found by the D* algorithm."""
        plt.imshow(self.grid, cmap='gray')
        if self.path:
            path_x, path_y = zip(*self.path)
            plt.plot(path_y, path_x, color='red')
        plt.scatter(self.start[1], self.start[0], color='green', marker='o', label='Start')
        plt.scatter(self.goal[1], self.goal[0], color='blue', marker='x', label='Goal')
        plt.legend()
        plt.title('D* Pathfinding')
        plt.savefig('dstar_path.png')
        # clear the plot
        plt.clf()

if __name__ == "__main__":
    # Example usage
    grid = np.array([
        [False, False, False, False, False],
        [False, True, True, True, False],
        [False, False, False, True, False],
        [False, True, False, False, False],
        [False, False, False, False, False]
    ])

    start = (0, 0)
    goal = (4, 4)

    dstar = DStar()
    dstar.initialize(grid, start, goal)
    path = dstar.run()

    if path:
        print("Path found:", path)
        dstar.plot_path()
    else:
        print("No path found.")
