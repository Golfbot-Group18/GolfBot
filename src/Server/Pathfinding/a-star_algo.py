from queue import PriorityQueue

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1]) #Manhattan distance - heuristic between two points in a grid, x1-x2 + y1-y2. 

def a_star_search(grid, start, goal):
    # A* search algorithm to find the shortest path between two points in a grid.
    # The grid should be a 2D list of 0s and 1s, where 0 is a valid path and 1 is an obstacle. (The binary grid)
    rows, cols = len(grid), len(grid[0])
    open_set = PriorityQueue()
    open_set.put(start, 0)
    came_from = {}
    cost_so_far = {start: 0}

    while not open_set.empty():
        _, current = open_set.get()

        if current == goal:
            break

        for dx, dy in [(-1, 0), (1,0), (0,-1), (0,1)]: # for all the neighbors of the current node.
            neighbor = (current[0] + dx, current[1] + dy)  # neighbor is the current node + dx, dy.
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols: # if the neighbor is within the grid.
                if grid[neighbor[0]][neighbor[1]] != 1: # if 0 then it is a valid path. 
                    new_cost = cost_so_far[current] + 1 # new cost is the cost so far + 1.
                    if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]: # if the neighbor is not in the cost_so_far or the new cost is less than the cost so far.   
                        cost_so_far[neighbor] = new_cost
                        priority = new_cost + heuristic(goal, neighbor)
                        open_set.put((priority, neighbor))
                        came_from[neighbor] = current
    
    current = goal
    path = []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()
    return path # retuns the path from start to goal in form of a list of tuples - coordinates.