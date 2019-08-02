# Motion-Planning-Search-Algorithm
![Turn Policy on Grid](Turn_policy_on_grid.png)

# A* 
The key logic of implementing the turn policy using A* is shown in:
![A* detail](A_star_detail.png) 

The following code shows the key steps:  
```python
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
```

# Dynamic Programming
![Dynamic programming detail](Dynamic_programming_detail.png) 

```python
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
```
