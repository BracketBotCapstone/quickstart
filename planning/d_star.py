import numpy as np
import matplotlib.pyplot as plt
from queue import PriorityQueue

class DStar:
    def __init__(self):
        self.grid = None
        self.start = None
        self.goal = None
        self.path = []

    def initialize(self, grid, start, goal):
        self.grid = grid
        self.start = start
        self.goal = goal
        self.path = []

    def update(self, new_grid, new_start, new_goal):
        self.update_grid(new_grid)
        self.update_start(new_start)
        self.update_goal(new_goal)
        self.path = []

    def update_grid(self, new_grid):
        self.grid = new_grid
     
    def update_start(self, new_start):
        self.start = new_start

    def update_goal(self, new_goal):
        self.goal = new_goal

    def heuristic(self, a, b):
        # euclidean distance
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def neighbors(self, node):
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (-1, -1), (1, -1), (-1, 1)]
        result = []
        for d in directions:
            neighbor = (node[0] + d[0], node[1] + d[1])
            if 0 <= neighbor[0] < self.grid.shape[0] and 0 <= neighbor[1] < self.grid.shape[1]:
                result.append(neighbor)
        return result

    def run(self):
        open_set = PriorityQueue()
        open_set.put((0, self.start))
        came_from = {}
        g_score = {self.start: 0}
        f_score = {self.start: self.heuristic(self.start, self.goal)}

        while not open_set.empty():
            # print("open set size: ", open_set.qsize())
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
        while current in came_from:
            self.path.append(current)
            current = came_from[current]
        self.path.append(self.start)
        self.path.reverse()

    def plot_path(self):
        plt.imshow(self.grid, cmap='gray')
        if self.path:
            path_x, path_y = zip(*self.path)
            plt.plot(path_y, path_x, color='red')
        plt.scatter(self.start[1], self.start[0], color='green', marker='o', label='Start')
        plt.scatter(self.goal[1], self.goal[0], color='blue', marker='x', label='Goal')
        plt.legend()
        plt.title('D* Pathfinding')
        plt.savefig('dstar_path.png')

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