import csv


# Lasitha Amuwala M.
# 101114683
# Comp 3106A
# Oct 17 2021


# converts a csv file into an array
def csv_to_array(filepath):
    results = []

    with open(filepath) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        for row in csv_reader:
            results.append(row)

    return results


# returns the path property of a node
def simplify_explored(explored):
    return explored['location']


# input:
# gird: 2d array of string
# node: tuple
# output: list of adjacent nodes that are within the bounds of grid
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


# input:
#   grid: 2d array of stings
#   start_node: tuple
#   goal_node: tuple
#   obstacles: list of tuples
# output:
#   path: list of tuples, optimal path from start_node to goal_node
#   explored: list of tuples, path of explored nodes from start to goal
#   cost: integer, optimal cost of path from start to goal
def a_star_search(grid, start_node, goal_node, obstacles):
    # create frontier and explore lists
    frontier = []
    explored = []

    # add the start node to frontier
    frontier.append(grid[start_node[0]][start_node[1]])

    # loop until optimal path to goal node is found
    while len(frontier) > 0:

        # sort frontier based on f
        frontier = sorted(frontier, key=lambda x: x['f'])

        # remove the priority item and add it to the explored list
        curr_node = frontier.pop(0)
        explored.append(curr_node)

        # if the goal node is reached return the path, explored lit, and cost
        if curr_node['location'] == goal_node:
            path = []
            curr = curr_node
            path.append(goal_node)

            # find the path from start node to goal node by backtracking the node parent
            while curr['location'] != start_node:
                path.append(curr['parent'])
                curr = grid[curr['parent'][0]][curr['parent'][1]]

            # return path, explored list, and cost in tuple list format
            path.reverse()
            explored = list(map(simplify_explored, explored))
            return path, explored, curr_node['cost']

        # iterate through the adjacent nodes
        for adjacent in adjacent_nodes(grid, curr_node['location']):

            # check if current node is already in the explored list or if it is an obstacle
            if adjacent in obstacles or adjacent in explored:
                continue

            # update the adjacent node properties
            adjacent['cost'] = curr_node['cost'] + 1
            adjacent['h'] = abs(adjacent['location'][0] - goal_node[0]) + abs(adjacent['location'][1] - goal_node[1])
            adjacent['f'] = adjacent['cost'] + adjacent['h']
            adjacent['parent'] = curr_node['location']

            # check if adjacent node is already in the frontier
            append = True
            for node in frontier:
                # check if adjacent has a lower f value than the node in frontier
                if node['location'] == adjacent['location'] and node['f'] >= adjacent['f']:
                    append = False

            # append adjacent to frontier
            if append:
                frontier.append(adjacent)
    return


# finds the optimal path from a start node to a goal node of a given 2d grid environment
# input: file name of csv file
# output: optimal_path, optimal_path_cost, explored_list
def pathfinding(input_filepath):
    # input_filepath contains the full path to a CSV file with the input grid
    # optimal_path is a list of tuples indicated the optimal path from start to goal
    # explored_list is the list of nodes explored during search
    # optimal_path_cost is the cost of the optimal path from the start state to the goal state

    # get grid from csv file in array format
    grid = csv_to_array(input_filepath)

    if grid is None:
        return -1

    start_node = None
    goal_node = None
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

    # if no start node or goal node is found return -1
    if start_node is None or goal_node is None:
        return -1

    # add properties for each square in grid
    for row, squares in enumerate(grid):
        for col, square in enumerate(squares):
            grid[row][col] = {'location': (row, col), 'h': 0, 'cost': 0, 'f': 0, 'parent': None}

    # create a list with all hazards and squares adjacent to hazards
    for hazard in hazards:
        adjacent_hazard_nodes = adjacent_nodes(grid, hazard)
        for adjacent_hazards in adjacent_hazard_nodes:
            obstacles.append(adjacent_hazards)
        obstacles.append(grid[hazard[0]][hazard[1]])

    # add all wall locations to obstacles list
    for index, wall in enumerate(walls):
        obstacles.append(grid[wall[0]][wall[1]])

    # use a star path finding algorithm to find the optimal path, cost, and nodes explored
    optimal_path, explored_list, optimal_path_cost = (a_star_search(grid, start_node, goal_node, obstacles))

    return optimal_path, explored_list, optimal_path_cost
