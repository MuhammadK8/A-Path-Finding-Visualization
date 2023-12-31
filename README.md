# A* Pathfinding Visualization

## Project Overview
The A* Pathfinding Visualization is a Python project that vividly demonstrates the A* pathfinding algorithm using Pygame. This application serves as a practical example of how advanced algorithms can be applied in real-world scenarios, particularly in game development and robotics. A* algorithm is an informed search algorithm, known for its efficiency and effectiveness in pathfinding and graph traversal. It's highly regarded in various domains for its ability to find the shortest path in a cost-effective manner.

## Preview
![Pathfinding Visualization](Visualization.gif)

## Features

### Interactive Grid
- Users can interact with the grid, creating and removing obstacles and endpoints to test the algorithm's adaptability in different scenarios.

### Color-Coded Nodes
- **Start Node:** Pink, marking the starting point of the search.
- **End Node:** Turquoise, representing the goal or destination.
- **Visited Nodes:** Red, indicating nodes that have been evaluated by the algorithm.
- **Unvisited Nodes:** Green, showing nodes that are yet to be considered.
- **Obstacles:** Black, representing impassable barriers on the grid.
- **Path:** Purple, highlighting the optimal path discovered from start to end.
- **Default Node:** White, denoting a node that is not yet assigned a specific role.

## Technical Details

## Mathematical Concept
The A* algorithm combines features of uniform-cost search and pure heuristic search to efficiently compute optimal paths. It calculates the cost (f) of a path from the start node to a target node using the formula: `f = g + h`, where:
- `g` is the actual cost from the start node to the current node.
- `h` is a heuristic estimated cost from the current node to the target node.
This project uses the Manhattan distance as the heuristic, which is suitable for grid-based maps.

### Manhattan Distance vs. Euclidean Distance
The project uses Manhattan Distance as the heuristic for its suitability in grid-based maps. This heuristic is beneficial in scenarios where diagonal movement is either not allowed or not preferable. In contrast, the Euclidean Distance, while representing the shortest straight-line distance between two points, is less practical in grid environments where movement is typically limited to orthogonal directions.


## Programming Skills Showcase

### Data Structures and Algorithms
- **Priority Queues:** The A* algorithm in this project utilizes priority queues to manage the open set of nodes, ensuring efficient selection of the next node to visit.
- **Path Reconstruction:** Once the target node is reached, the path is reconstructed by tracing back from the end node to the start node, exemplifying my skills in algorithmic thinking and data structure manipulation.

### Application in AI
- **Heuristic Function:** The use of the Manhattan Distance as a heuristic reflects an intelligent estimation of distance to the goal, showcasing an application of problem-solving strategies in AI.
- **Optimization and Efficiency:** The project underlines the importance of optimization and efficiency in AI, with the A* algorithm efficiently finding the shortest path, balancing between shortest path estimation and actual path cost.

## Code Snippets and Examples
Below is an example snippet from the `astar.py` script, focusing on the functionality of the A* algorithm. This snippet showcases how the algorithm processes each node and finds the shortest path. This snippet also demonstrates the use of priority queues in the A* algorithm implementation. It exemplifies how the algorithm efficiently selects the next node to visit based on the calculated cost.

In this snippet, the `PriorityQueue` is used to store nodes along with their priority values. The A* algorithm uses this queue to keep track of nodes that need to be evaluated, selecting the one with the lowest cost (`priority`) first. This ensures that the path found is not only valid but also the most cost-effective one, considering both the distance traveled and the estimated distance to the target.

```python
def algorithm(draw, grid, start, end):
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
```
---
### Explanation of the Snippet

1. **Initialization**: The `open_set` is a priority queue that holds nodes to be evaluated, starting with the `start` node. The `came_from` dictionary tracks where each node was reached from, and `cost_so_far` keeps the cost of the shortest path found to each node, initially only containing the start node with a cost of 0.

2. **Main Loop**: The algorithm repeatedly checks the node in the `open_set` with the lowest cost (`priority`).

3. **Path Finding**: For each neighbor of the current node, the algorithm calculates the cost (`new_cost`) of reaching that neighbor. If this neighbor hasn't been reached before or if this new path to it is shorter, it updates `cost_so_far` and adds the neighbor to the `open_set` with the priority based on the cost and heuristic.

4. **Heuristic Function**: The `heuristic` function estimates the cost from the current node to the end node. This heuristic is key to the efficiency of A*, guiding the algorithm towards the goal.

5. **Path Reconstruction**: Once the end node is reached, `reconstruct_path` uses `came_from` to backtrack from the end node to the start node, thereby reconstructing the shortest path found.

6. **Assumption**: This snippet assumes that moving from one node to a neighboring node costs 1 unit. This value can be adjusted based on the specific requirements of your grid.
   
## Challenges and Learning
Throughout this project, I faced challenges such as optimizing the algorithm for better performance and handling edge cases in grid navigation. Overcoming these hurdles not only improved my problem-solving skills but also deepened my understanding of algorithm optimization and Python programming.

## Conclusion and Future Scope
This project stands as a testament to my skills in software development, showcasing my ability to apply complex algorithms in practical scenarios. Future enhancements can include implementing different heuristics, allowing diagonal movements, and exploring the algorithm's application in other contexts like 3D terrain navigation.
