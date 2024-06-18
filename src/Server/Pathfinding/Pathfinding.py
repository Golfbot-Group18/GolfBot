from queue import PriorityQueue, Queue
import numpy as np
from shapely.geometry import Point
import math

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1]) #Manhattan distance - heuristic between two points in a grid, x1-x2 + y1-y2. 

'''This is currently not used in the code. It does not use the clearance grid.'''
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

'''At the moment this function calculates shortest path from the center point of the robot to the goal point. We might need to change this to calculate the path from the tip of the robot to the goal point. '''
def a_star_fms_search(grid, clearance_grid, start, goal, min_clearance):
    rows, cols = grid.shape
    open_set = PriorityQueue()
    open_set.put((0, start))
    came_from = {}
    cost_so_far = {start: 0}

    print(f"Starting A* search from {start} to {goal} with minimum clearance {min_clearance}")

    while not open_set.empty():
        _, current = open_set.get()
        #print(f"Current node: {current}")

        if current == goal:
            print("Goal reached!")
            break

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            neighbor = (current[0] + dx, current[1] + dy)
            
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:
                neighbor_clearance = clearance_grid[neighbor[0], neighbor[1]]
                #print(f"Checking neighbor: {neighbor}, Clearance: {neighbor_clearance}")

                if neighbor_clearance >= min_clearance:
                    new_cost = cost_so_far[current] + 1
                    if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                        cost_so_far[neighbor] = new_cost
                        priority = new_cost + heuristic(goal, neighbor)
                        open_set.put((priority, neighbor))
                        came_from[neighbor] = current
                        #print(f"Adding neighbor: {neighbor} with priority: {priority}")

    current = goal
    path = []
    while current != start:
        path.append(current)
        current = came_from.get(current)
        if current is None:
            print("Path not found")
            return []
    path.append(start)
    path.reverse()
    return path

'''Also not currently used, but this function should be used to figure out if a ball is in proximity of an obstacle'''
def is_goal_in_proximity(point, clearance_grid, threshold):
    x, y = point
    if 0 <= y < clearance_grid.shape[0] and 0 <= x < clearance_grid.shape[1]:
        if(clearance_grid[y, x] <= threshold):
            print(f"Clearance at {point} is {clearance_grid[y, x]}")
            return True
    return False

'''Not used as of now, but should be used to shift the virtual ball position to a safe location'''
def is_ball_shiftable(ball_position, clearance_grid, robot_width, buffer=10, tolerance=5):
    x, y = ball_position
    shift_distance = robot_width // 2 + buffer

    def check_clearance(axis):
        if(axis == "x"):
            for i in range(-int(shift_distance), int(shift_distance) + 1):
                if clearance_grid[y, x + i] < clearance_value - tolerance:
                    return False
            return True
        else:
            for i in range(-int(shift_distance), int(shift_distance) + 1):
                if clearance_grid[y + i, x] < clearance_value - tolerance:
                    return False
            return True

    # check x-axis
    clearance_value = clearance_grid[y, x]
    print(f"Clearance Value: {clearance_value}")
    print(f"We will check this if-statement for x-axis: {clearance_grid[y][x + int(shift_distance)]} >= {clearance_value} - {tolerance} and {clearance_grid[y][x - int(shift_distance)]} >= {clearance_value} - {tolerance}")
    print(f"We will check this if-statement for y-axis: {clearance_grid[y + int(shift_distance)][x]} >= {clearance_value} - {tolerance} and {clearance_grid[y - int(shift_distance)][x]} >= {clearance_value} - {tolerance}")
    if (check_clearance("x")):
        print("Can shift along y-axis")
        print(f"Clearance Value at target: {clearance_grid[y][x+int(shift_distance)]}")
        # we can shift along the y-axis
        if clearance_grid[y + int(shift_distance)][x] >= clearance_value - tolerance:
            print("Can shift up")
            print(f"Clearance Value at target: {clearance_grid[y+int(shift_distance)][x]}")
            shift_goal = (x, y + int(shift_distance))
        elif clearance_grid[x][y - int(shift_distance)] >= clearance_value - tolerance:
            print("Can shift down")
            print(f"Clearance Value at target: {clearance_grid[y-int(shift_distance)][x]}")
            shift_goal = (x, y - int(shift_distance))
        else:
            return False, None
        
    elif (check_clearance("y")):
        print("Can shift along x-axis")
        print(f"Clearance Value at target: {clearance_grid[y+int(shift_distance)][x]}")
        # we can shift along the x-axis
        if clearance_grid[y][x + int(shift_distance)] >= clearance_value - tolerance:
            print("Can shift right")
            print(f"Clearance Value at target: {clearance_grid[y][x+int(shift_distance)]}")
            shift_goal = (x + int(shift_distance), y)
        elif clearance_grid[y][x - int(shift_distance)] >= clearance_value - tolerance:
            print("Can shift left")
            print(f"Clearance Value at target: {clearance_grid[y][x-int(shift_distance)]}")
            shift_goal = (x - int(shift_distance), y)
        else:
            return False, None
    else:
        return False, None

    return True, shift_goal
