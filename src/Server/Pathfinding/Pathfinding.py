from queue import PriorityQueue
import numpy as np

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1]) #Manhattan distance - heuristic between two points in a grid, x1-x2 + y1-y2. 

def a_star_search(grid, start, goal):
    # A* search algorithm to find the shortest path between two points in a grid.
    # The grid should be a 2D list of 0s and 1s, where 0 is a valid path and 1 is an obstacle. (The binary grid)
    rows, cols = len(grid), len(grid[0])
    open_set = PriorityQueue()
    open_set.put((0, start))
    came_from = {}
    cost_so_far = {start: 0}

    while not open_set.empty():
        _, current = open_set.get()

        if current == goal:
            break

        for dx, dy in [(-1, 0), (1,0), (0,-1), (0,1)]: 
            neighbor = (current[0] + dx, current[1] + dy)  
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols: 
                if grid[neighbor[0]][neighbor[1]] != 1: 
                    new_cost = cost_so_far[current] + 1 
                    if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:  
                        cost_so_far[neighbor] = new_cost
                        priority = new_cost + heuristic(goal, neighbor)
                        open_set.put((priority, neighbor))
                        came_from[neighbor] = current
    
    current = goal
    path = []
    while current != start:
        path.append(current)
        current = came_from[current]
        if current is None: 
            return []  
    path.append(start)
    path.reverse()
    return path 

def a_star_fms_search(grid, clearance_grid, start, goal, robot_size, goal_clearance):
    #Same as function above but we're now using the clearance grid to check if the robot can fit through the path.
    rows, cols = grid.shape
    open_set = PriorityQueue()
    open_set.put((0, start))
    came_from = {}
    cost_so_far = {start: 0}

    while not open_set.empty():
        _, current = open_set.get()

        if current == goal:
            break

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            neighbor = (current[0] + dx, current[1] + dy)
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:
                required_clearance = goal_clearance if neighbor == goal else robot_size
                if clearance_grid[neighbor[0]][neighbor[1]] >= required_clearance:
                    new_cost = cost_so_far[current] + 1
                    if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                        cost_so_far[neighbor] = new_cost
                        priority = new_cost + heuristic(goal, neighbor)
                        open_set.put((priority, neighbor))
                        came_from[neighbor] = current

    current = goal
    path = []
    while current != start:
        path.append(current)
        current = came_from.get(current)
        if current is None:
            return []
    path.append(start)
    path.reverse()
    return path

def calculate_clearance_grid(grid, max_clearance):
    # Calculate the clearance grid for a given binary grid, where each cell contains the distance to the nearest obstacle.
    rows, cols = grid.shape
    clearance_grid = np.full((rows, cols), max_clearance, dtype=int)
    pq = PriorityQueue()

    for y in range(rows):
        for x in range(cols):
            if grid[y, x] == 1:
                clearance_grid[y, x] = 0
                pq.put((0, y, x))
    
    while not pq.empty():
        clearance, y, x = pq.get()
        
        for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            ny, nx = y + dy, x + dx
            if 0 <= ny < rows and 0 <= nx < cols:
                new_clearance = clearance + 1
                if new_clearance < clearance_grid[ny, nx]:
                    clearance_grid[ny, nx] = new_clearance
                    pq.put((new_clearance, ny, nx))

    return clearance_grid