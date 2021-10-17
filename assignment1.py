# Name this file to assignment1.py when you submit
import csv
import math


def csv_to_array(filepath):
    results = []
    with open(filepath) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        for row in csv_reader:
            results.append(row)

    return results


def a_star_search(grid, start_node, goal_node, obstacles):
    frontier = [grid[start_node[0]][start_node[1]]]
    explored = []
    path = []

    loop = True
    while loop:
        if not frontier:
            break

        frontier = sorted(frontier, key=lambda x: x['h'])
        leaf = frontier.pop(0)

        if leaf['location'] == goal_node:
            explored.append(leaf)
            next = leaf
            path.append(next['parent'])
            while next['location'] != start_node:
                path.append(next['parent'])
                next = grid[next['parent'][0]][next['parent'][1]]
            break

        explored.append(leaf)
        for adjacent in adjacent_nodes(grid, leaf['location']):
            if adjacent in obstacles:
                continue

            curr_path_cost = leaf['cost'] + 1

            if adjacent not in frontier and adjacent not in explored or adjacent in frontier and curr_path_cost < \
                    adjacent['cost']:
                adjacent['parent'] = leaf['location']
                adjacent['cost'] = curr_path_cost
                frontier.append(adjacent)

    path.reverse()
    cost = grid[goal_node[0]][goal_node[1]]['cost']
    new_explored_list = list(map(simplify_explored, explored))

    return path, new_explored_list, cost


def simplify_explored(explored):
    return explored['location']


def adjacent_nodes(grid, node):
    col, row = node

    adjacent = []
    if row + 1 < len(grid[0]):
        adjacent.append(grid[col][row + 1])
    if row - 1 >= 0:
        adjacent.append(grid[col][row - 1])
    if col + 1 < len(grid):
        adjacent.append(grid[col + 1][row])
    if col - 1 >= 0:
        adjacent.append(grid[col - 1][row])

    return adjacent


def heuristic(node, goal):
    y = node[0] - goal[0]
    x = node[1] - goal[1]
    r = round(math.hypot(x, y), 1)
    return r


def pathfinding(input_filepath):
    # input_filepath contains the full path to a CSV file with the input grid
    # optimal_path is a list of tuples indicated the optimal path from start to goal
    # explored_list is the list of nodes explored during search
    # optimal_path_cost is the cost of the optimal path from the start state to the goal state

    # get grid from csv file in array format
    grid = csv_to_array(input_filepath)

    start_node = (0, 0)
    goal_node = (0, 0)
    walls = []
    hazards = []
    obstacles = []

    # iterate the data and find the starting node, goal node, walls, and hazards
    for row, squares in enumerate(grid):
        for col, square in enumerate(squares):
            if square == "S":
                start_node = (row, col)
            if square == "G":
                goal_node = (row, col)
            if square == "X":
                walls.append((row, col))
            if square == "H":
                hazards.append((row, col))
            # each node is a dictionary with the keys, location, cost, parent, and h (heuristic)
            node = {'location': (row, col), 'cost': 0 if (row, col) == start_node else 1}
            grid[row][col] = node

    for row, squares in enumerate(grid):
        for col, square in enumerate(squares):
            grid[row][col]['h'] = heuristic(grid[row][col]['location'], goal_node)

    for hazard in hazards:
        adjacent_hazard_nodes = adjacent_nodes(grid, hazard)
        for adjacent_hazards in adjacent_hazard_nodes:
            obstacles.append(adjacent_hazards)
        obstacles.append(grid[hazard[0]][hazard[1]])

    for index, wall in enumerate(walls):
        walls[index] = grid[wall[0]][wall[1]]

    obstacles += walls

    optimal_path, explored_list, optimal_path_cost = (a_star_search(grid, start_node, goal_node, obstacles))

    return optimal_path, explored_list, optimal_path_cost


file_path = 'C:/Users/lasit/Desktop/Carleton_year_4/fall_2021/COMP_3016A_Intro_to_AI/A1/Examples/Examples/Example1/input.txt'
result = pathfinding(file_path)

optimal_path, explored_list, optimal_path_cost = result
print('Optimal Path: ', optimal_path)
print('Explored List: ', explored_list)
print('Optimal Path Cost: ', optimal_path_cost)
