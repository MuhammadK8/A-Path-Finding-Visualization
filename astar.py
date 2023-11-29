"""
Author: Mohammad Shike
Project: A* Pathfinding Visualization
Purpose: This program implements the A* pathfinding algorithm and visualizes the process using Pygame. 
         It is designed to demonstrate how the algorithm searches for the shortest path between two points on a grid.
"""

# Import pygame for graphical interface creation
import pygame
# Import math for mathematical operations and calculations
import math
# Import PriorityQueue from the queue module for efficient queue operations, used in the A* algorithm
from queue import PriorityQueue


# Set the width of the window for the graphical display
WIDTH = 800

# Create a pygame window with the defined width and height
WINDOW = pygame.display.set_mode((WIDTH, WIDTH))

# Set the caption of the pygame window to "A* Path Finding Algorithm"
pygame.display.set_caption("A* Path Finding Algorithm")


FIREBRICK = (178, 34, 34)
LIMEGREEN = (50, 205, 50)
BLUE = (0, 0, 255)
GOLD = (255, 215, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
PURPLE = (128, 0, 128)
PINK = (255, 192, 203)
GREY = (128, 128, 128)
TURQUOISE = (64, 224, 208)


# Define the Node class to represent each cell or node in the pathfinding grid
class Node:
    def __init__(self, row, col, width, total_num_rows):
        # Initialize a node with position, color, neighbors, and dimensions
        self.row = row
        self.col = col
        self.x = row * width  # x-coordinate on the grid

        self.y = col * width  # y-coordinate on the grid

        self.color = WHITE  # Initial color of the node

        self.neighbors = []  # List to store neighboring nodes
        self.width = width
        self.total_num_rows = total_num_rows

    def get_pos(self):
        # Return the grid position of the node
        return self.row, self.col

    def was_visited(self):
        # Check if the node was visited (based on its color)
        return self.color == FIREBRICK

    def not_visited(self):
        # Check if the node has not been visited (based on its color)
        return self.color == LIMEGREEN

    def is_obstacle(self):
        # Check if the node is an obstacle (based on its color)
        return self.color == BLACK

    def is_start(self):
        # Check if the node is the start node (based on its color)
        return self.color == PINK

    def is_end(self):
        # Check if the node is the end node (based on its color)
        return self.color == TURQUOISE

    def reset(self):
        # Reset the node's color to white (default)
        self.color = WHITE

    def make_start(self):
        # Mark the node as the start node
        self.color = PINK

    def make_visited(self):
        # Mark the node as visited
        self.color = FIREBRICK

    def make_not_visited(self):
        # Mark the node as not visited
        self.color = LIMEGREEN

    def make_obstacle(self):
        # Mark the node as an obstacle
        self.color = BLACK

    def make_end(self):
        # Mark the node as the end node
        self.color = TURQUOISE

    def make_path(self):
        # Mark the node as part of the path
        self.color = PURPLE

    def draw(self, window):
        # Draw the node on the screen
        pygame.draw.rect(window, self.color, (self.x, self.y, self.width, self.width))

    def update_neighbors(self, grid):
        # Update the list of neighbors for the node, considering obstacles
        self.neighbors = []
        # Check for neighbors in all four directions (down, up, right, left) and add them if they are not obstacles       
        if self.row < self.total_num_rows - 1 and not grid[self.row + 1][self.col].is_obstacle(): # DOWN
            self.neighbors.append(grid[self.row + 1][self.col])
        if self.row > 0 and not grid[self.row - 1][self.col].is_obstacle(): # UP
            self.neighbors.append(grid[self.row - 1][self.col])
        if self.col < self.total_num_rows - 1 and not grid[self.row][self.col + 1].is_obstacle(): # RIGHT
            self.neighbors.append(grid[self.row][self.col + 1])
        if self.col > 0 and not grid[self.row][self.col - 1].is_obstacle(): # LEFT
            self.neighbors.append(grid[self.row][self.col - 1])
    # Less than comparison override for nodes, used in priority queue
    def __lt__(self, other):
        return False



def h(p1, p2):
    """
    Heuristic function for A* pathfinding.
    
    Calculates the Manhattan distance (L1 norm) between two points, p1 and p2.
    
    Args:
    p1 (tuple): The (x, y) coordinates of the first point.
    p2 (tuple): The (x, y) coordinates of the second point.
    
    Returns:
    int: Manhattan distance between p1 and p2.
    """
    x1, y1 = p1
    x2, y2 = p2
    return abs(x1 - x2) + abs(y1 - y2)


def reconstruct_path(came_from, current, draw):
    """
    Reconstructs the path from start to end node after the A* algorithm execution.
    
    It backtracks from the end node to the start node using the `came_from` dictionary and updates the GUI to display the path.
    
    Args:
    came_from (dict): A dictionary storing where each node came from for path reconstruction.
    current (Node): The current node, typically the end node when called.
    draw (function): A function to update the GUI.
    """
    while current in came_from:
        current = came_from[current]
        current.make_path()
        draw()


def algorithm(draw, grid, start, end):
    """
    The A* algorithm implementation for pathfinding.
    
    Args:
    draw (function): Function to update the GUI.
    grid (list): 2D list representing the grid of nodes.
    start (Node): The starting node.
    end (Node): The end node or goal.
    
    Returns:
    bool: True if a path is found, False otherwise.
    """
    count = 0
    open_set = PriorityQueue()
    open_set.put((0, count, start))
    came_from = {}
    g_score = {node: float("inf") for row in grid for node in row}
    g_score[start] = 0
    f_score = {node: float("inf") for row in grid for node in row}
    f_score[start] = h(start.get_pos(), end.get_pos())
    
    open_set_hash = {start}

    while not open_set.empty():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

        current = open_set.get()[2]
        open_set_hash.remove(current)

        if current == end:
            reconstruct_path(came_from, end, draw)
            end.make_end()
            return True

        for neighbor in current.neighbors:
            temp_g_score = g_score[current] + 1

            if temp_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = temp_g_score
                f_score[neighbor] = temp_g_score + h(neighbor.get_pos(), end.get_pos())
                if neighbor not in open_set_hash:
                    count += 1
                    open_set.put((f_score[neighbor], count, neighbor))
                    open_set_hash.add(neighbor)
                    neighbor.make_not_visited()
        draw()

        if current != start:
            current.make_visited()

    return False



def make_grid(rows, width):
    grid = []
    gap = width // rows
    for i in range(rows):
        grid.append([])
        for j in range(rows):
            node = Node(i, j, gap, rows)
            grid[i].append(node)

    return grid


def draw_grid(window, rows, width):
    gap = width // rows
    for i in range(rows):
        pygame.draw.line(window, GREY, (0, i * gap), (width, i * gap))
    for j in range(rows):
        pygame.draw.line(window, GREY, (j * gap, 0), (j * gap, width))


def draw(window, grid, rows, width):
    window.fill(WHITE)

    for row in grid:
        for node in row:
            node.draw(window)

    draw_grid(window, rows, width)
    pygame.display.update()


def get_clicked_pos(pos, rows, width):
    gap = width // rows
    y, x = pos

    row = y // gap
    col = x // gap

    return row, col


def main(window, width):
    ROWS = 50
    grid = make_grid(ROWS, width)

    start = None
    end = None

    run = True

    while run:
        draw(window, grid, ROWS, width)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

            if pygame.mouse.get_pressed()[0]:  # LEFT MOUSE BUTTON
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, width)
                node = grid[row][col]
                if not start and node != end:
                    start = node
                    start.make_start()

                elif not end and node != start:
                    end = node
                    end.make_end()

                elif node != end and node != start:
                    node.make_obstacle()

            elif pygame.mouse.get_pressed()[2]:  # RIGHT MOUSE BUTTON
                pos = pygame.mouse.get_pos()
                row, col = get_clicked_pos(pos, ROWS, width)
                node = grid[row][col]
                node.reset()
                if node == start:
                    start = None
                elif node == end:
                    end = None

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE and start and end:
                    for row in grid:
                        for node in row:
                            node.update_neighbors(grid)

                    algorithm(
                        lambda: draw(
                            window,
                            grid,
                            ROWS,
                            width),
                        grid,
                        start,
                        end)

                if event.key == pygame.K_c:
                    start = None
                    end = None
                    grid = make_grid(ROWS, width)

    pygame.quit()


main(WINDOW, WIDTH)
