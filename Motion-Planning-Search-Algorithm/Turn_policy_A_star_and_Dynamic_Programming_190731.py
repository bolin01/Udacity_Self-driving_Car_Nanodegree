#-----------
# Discrete path planning with A*
# and policy planning with dynamic programming
# on grid with different penalty for right/no/left turns
#
# created by Bo Lin on Aug. 1, 2019
#
# DETAILED DESCRIPTION:
# Given a car in grid (binary matrix with 1 representng unnavigable space)
# with initial state "init".
# Compute and return the car's optimal path to the position specified in "goal"; 
# the costs for each motion are as defined in "cost".
#-----------

# There are four motion directions: go up, left, down, and right.
# Increasing the index in this array corresponds to making a
# a left turn, and decreasing the index corresponds to making a 
# right turn.
forward = [[-1,  0], # go up
           [ 0, -1], # go left
           [ 1,  0], # go down
           [ 0,  1]] # go right
forward_sign = ['^','<','v','>']
#forward_name = ['up', 'left', 'down', 'right']

# There are 3 possible actions: right turn, no turn, left turn
# the value in "action" corresponds to move up/down in "forward"
action = [-1, 0, 1]
action_name = ['R', '#', 'L']

# cost has 3 values, corresponding to making 
# a right turn, no turn, and a left turn
cost = [2, 1, 20] 
                  
# EXAMPLE INPUTS:
# grid: binary matrix 
# 0 = navigable space
# 1 = unnavigable space 
"""grid = [[1, 1, 1, 0, 0, 0],
        [1, 1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 0],
        [1, 1, 1, 0, 1, 1],
        [1, 1, 1, 0, 1, 1]]

init = [4, 3, 0] # given in the form [row,col,direction]
                 # direction = 0: up
                 #             1: left
                 #             2: down
                 #             3: right
                
goal = [2, 0] # given in the form [row,col]"""

grid = [[0, 0, 0, 0, 0, 0, 1, 1],
        [0, 1, 1, 0, 0, 0, 0, 1],
        [0, 0, 0, 0, 1, 0, 1, 1],
        [0, 1, 0, 0, 1, 0, 1, 1],
        [0, 1, 1, 1, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 1, 0],
        [1, 1, 1, 1, 1, 0, 1, 0],
        [1, 1, 1, 1, 1, 0, 0, 0],
        [1, 1, 1, 1, 1, 0, 1, 1]]

init = [8, 5, 0] # given in the form [row,col,direction]
                 # direction = 0: up
                 #             1: left
                 #             2: down
                 #             3: right
                
goal = [2, 2] # given in the form [row,col]

# EXAMPLE OUTPUT:
# calling optimum_policy2D with the given parameters should return 
# [['x', 'x', 'x', 'R', '#', 'R'],
#  ['x', 'x', 'x', '#', 'x', '#'],
#  ['*', '#', '#', '#', '#', 'R'],
#  ['x', 'x', 'x', '#', 'x', 'x'],
#  ['x', 'x', 'x', '#', 'x', 'x']]
# ----------

# Plot map at the initial state:
# 'o': unnavigable space
# '*': target
# '^','<','v','>': heading direction at the start location
map_init = [[' ' for col in range(len(grid[0]))] for row in range(len(grid))]
map_init[goal[0]][goal[1]] = '*'
map_init[init[0]][init[1]] = forward_sign[init[2]]
print("----- Initial state: -----")
for row in range(len(map_init)):
    for col in range(len(map_init[0])):
        if grid[row][col] == 1:
            map_init[row][col] = 'o'
    print(map_init[row])
print("--------------------------")

# main function to conduct discrete path planning 
# with A* and policy planning with dynamic programming

def optimum_policy2D(grid,init,goal,cost):

    ##### Chose types of search algorithm #####
    # 1 - A*
    # 0 - Dynamic programming
    Is_A_star = 0;

    # Initialize policy grid
    policy2D = [[' ' for row in range(len(grid[0]))] for col in range(len(grid))]
    for row in range(len(policy2D)):
        for col in range(len(policy2D[0])):
            if grid[row][col] == 1:
                policy2D[row][col] = 'o'
    policy2D[goal[0]][goal[1]] = '*'

    ############################ A* ############################ 
    if Is_A_star:
        # Generate Heuristic using the distance to the goal as measure
        heuristic = [[0 for col in range(len(grid[0]))] for row in range(len(grid))]
        for row in range(len(grid)):
            for col in range(len(grid[0])):
                heuristic[row][col] = abs(row-goal[0])+abs(col-goal[1]) 

        # "closed" state indicates whether the location and heading direction has been checked or not:
        # 0 - not checked
        # 1 - checked
        closed=[[[[0 for act in range(len(action))] for dir in range(len(forward))] for col in range(len(grid[0]))] for row in range(len(grid))]
        # "action_all" state indicates the right/no/left turn action taken in the location
        # initalization with 0.1 to avoid confusion when the action is added/subtracted
        action_all=[[[0.1 for dir in range(len(forward))] for col in range(len(grid[0]))] for row in range(len(grid))]
        # "total_cost" is 3-dimensional state determining the 
        total_cost=[[[999 for dir in range(len(forward))] for col in range(len(grid[0]))] for row in range(len(grid))]
    
        # Order of search
        expand=[[[-1 for dir in range(len(forward))] for col in range(len(grid[0]))] for row in range(len(grid))]
        
        x = init[0]
        y = init[1]
        heading_dir = init[2]
        g = 0 # cost of path from start node 
        h = heuristic[x][y] #  heuristic function that estimates the cost of the cheapest path to the goal
        f = g + h

        open = [[f, g, h, x, y, heading_dir]]

        found = False  # flag that is set when search is complete
        resign = False # flag set if we can't find expand
        count = 0 # Number of search conducted

        print("----- State searched: -----")
        while not found and not resign:
            if len(open) == 0:
                resign = True
                return "Fail"
            else:
                # Get the next search by finding the smallest cost item
                open.sort()
                open.reverse()
                next = open.pop()
                f=next[0]
                g = next[1]
                x = next[3]
                y = next[4]
                heading_dir = next[5]
                # consider introducing new state act becaues four states consist a unique state,
                # and check the value
                # Assign the order of search to "expand"
                expand[x][y][heading_dir] = count
                count += 1


                if x == goal[0] and y == goal[1]:
                    found = True
                    print("Goal reached!")
                else:
                    # Go through every possible action (right/no/left turn) at the current location
                    for i in range(len(action)):
                        # Calculate new location and heading direction
                        heading_dir2=(heading_dir+action[i])%len(forward)
                        x2 = x + forward[heading_dir2][0]
                        y2 = y + forward[heading_dir2][1]
                        # Check if the new location is feasible
                        if x2 >= 0 and x2 < len(grid) and y2 >=0 and y2 < len(grid[0]) and \
                          closed[x2][y2][heading_dir2][i] == 0 and grid[x2][y2] == 0:
                            g2 = g + cost[i] 
                            h2 = heuristic[x2][y2]
                            f2 = g2 + h2
                            open.append([f2, g2, h2, x2, y2, heading_dir2])
                            closed[x2][y2][heading_dir2][i] = 1
                            # If the new location has been reached before,
                            # check if the new search has lower cost.
                            # If yes, update the action
                            if g2<total_cost[x2][y2][heading_dir2]:
                                total_cost[x2][y2][heading_dir2]=g2
                                action_all[x2][y2][heading_dir2]=i
                                print(x2,y2,heading_dir2,i)
                                
        print("----- Optimum path from goal to initial: -----")
        x=goal[0]
        y=goal[1]
        iter = 0
        while x!=init[0] or y!=init[1]:
            act = action_all[x][y][heading_dir]
            x2=x-forward[heading_dir][0]
            y2=y-forward[heading_dir][1]
            heading_dir2=(heading_dir-action[act])%len(forward)
            policy2D[x2][y2]=action_name[act]
            print(x,y,heading_dir,act)
            x=x2
            y=y2
            heading_dir=heading_dir2
            if iter>20:
                break
            iter+=1
        
    ############################ Dynamic Programming ############################ 
    else:
        value = [[[999 for dir in range(len(forward))] for row in range(len(grid[0]))] for col in range(len(grid))]
        action_all = [[[-1 for dir in range(len(forward))] for row in range(len(grid[0]))] for col in range(len(grid))]
        change = True

        while change:
            change = False
            # Go through every grid location
            for x in range(len(grid)):
                for y in range(len(grid[0])):
                    # Start searching from the goal
                    if goal[0] == x and goal[1] == y:
                        for heading_dir in range(len(forward)):
                            if value[x][y][heading_dir] > 0:
                                value[x][y][heading_dir] = 0
                                change = True

                    elif grid[x][y] == 0:
                        # Go through every heading direction
                        for heading_dir in range(len(forward)):
                            # Go through every motion
                            for a in range(len(action)):
                                heading_dir2 = (heading_dir+action[a])%len(forward)
                                x2 = x + forward[heading_dir2][0]
                                y2 = y + forward[heading_dir2][1]
                                if x2 >= 0 and x2 < len(grid) and y2 >= 0 and y2 < len(grid[0]) and grid[x2][y2] == 0:
                                    v2 = value[x2][y2][heading_dir2] + cost[a]
                                    # The searched (x2,y2,heading_dir2) has lower value,
                                    # if value(x2,y2,headin_dir2) + motion is smaller than value(x,y,heading_dir),
                                    # it means (x2,y2,heading_dir2) can be reached from (x,y,heading_dir) + motion at lower cost
                                    # update the action needed at (x,y,heading_dir)
                                    if v2 < value[x][y][heading_dir]:
                                        change = True
                                        value[x][y][heading_dir] = v2
                                        action_all[x][y][heading_dir] = a

        print("----- Optimum path from initial to goal: -----")
        x=init[0]
        y=init[1]
        heading_dir=init[2]
        while x!=goal[0] or y!=goal[1]:
            act=action_all[x][y][heading_dir]
            print(x,y,heading_dir,act)
            heading_dir2 = (heading_dir+action[act])%len(forward)
            x2=x+forward[heading_dir2][0]
            y2=y+forward[heading_dir2][1]
            policy2D[x][y]=action_name[act]
            x=x2
            y=y2
            heading_dir=heading_dir2
    
    return policy2D

policy2D=optimum_policy2D(grid,init,goal,cost)
print("----- Optimum policy: -----")
for row in range(len(policy2D)):
    print(policy2D[row])
print("---------------------------")
