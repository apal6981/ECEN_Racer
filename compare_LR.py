import numpy as np

def find_center(grid, shape):
    print('')
    x,y = shape
    starting_y = y-5
    obj_flag = False
    left_bound = 0
    right_bound = 0
    for i in range(2, x-2):
        val = np.average(grid[starting_y, i-2:i+2])
        if val >= 0 and obj_flag == False and left_bound == 0:
            obj_flag = True
            left_bound = i
            if left_bound >= x/2:
                left_bound = 0
                right_bound = left_bound
                break
        if val >= 0 and obj_flag == False and left_bound != 0 and right_bound == 0:
            obj_flag = True
            right_bound = i

        if val == 0 and obj_flag == True:
            obj_flag = False
    if right_bound == 0:
        right_bound = x
    center_x = (right_bound - left_bound)/2 + left_bound
    return int(center_x), left_bound, right_bound

def direction(grid):
    x,y = grid.shape()
    center_x, left_bound, right_bound = find_center(grid, (x,y))
    
    # Positive number means turn right, negative means turn left
    return center_x - int(x/2) 