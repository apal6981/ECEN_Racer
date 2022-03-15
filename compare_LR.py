import numpy as np

def find_center(grid):
    y, x = grid.shape
    starting_y = y-15
    # print("Y", starting_y)
    # print(grid[starting_y, :])
    obj_flag = False
    center_x_avg = 0
    left_bound = 0
    right_bound = 0
    y_spacing = 1
    # for j in range(starting_y, starting_y+y_spacing):
    for i in range(2, x-2):
        val = np.average(grid[starting_y, i-1:i+1])
        if val >= 100 and obj_flag == False and left_bound == 0:
            obj_flag = True
            left_bound = i
            if left_bound >= x/2 - 10:
                right_bound = left_bound
                left_bound = 0
                break
        if val >= 100 and obj_flag == False and left_bound != 0 and right_bound == 0:
            obj_flag = True
            right_bound = i

        if val <= 100 and obj_flag == True:
            obj_flag = False
    if right_bound == 0:
        right_bound = x
    # center_x_avg += (right_bound - left_bound)/2 + left_bound
    # center_x = center_x_avg/y_spacing
    center_x = (right_bound - left_bound)/2 + left_bound
    # print("left bound: ", left_bound)
    # print("right bound: ", right_bound)
    return int(center_x)

max_angle = 20
max_offset = 20
def direction(grid, counter):
    y,x = grid.shape
    center_x = find_center(grid)
    if counter % 10 == 0:
        print("Center x:", center_x)
    
    # Positive number means turn right, negative means turn left
    # Mapping direction to steering angle
    spacing = center_x - int(x/2)+10
    step_size = max_angle/max_offset

    steering = spacing*step_size
    if steering >= max_angle:
        steering = max_angle
    if steering <= -max_angle:
        steering = -max_angle
    if counter % 10 == 0:
        print("Steering: ", steering)
    return steering
