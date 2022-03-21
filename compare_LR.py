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
    sum_center_x = 0
    num_iter = 0
    # for j in range(starting_y, starting_y+y_spacing):
    for j in range(y-5, y-10, -1):
        num_iter += 1
        for i in range(2, x-2):
            val = np.average(grid[j, i-1:i+1])
            if val >= x/2 and obj_flag == False and left_bound == 0:
                obj_flag = True
                left_bound = i
                if left_bound >= x/2 - 10:
                    right_bound = left_bound
                    left_bound = 0
                    break
            if val >= x/2 and obj_flag == False and left_bound != 0 and right_bound == 0:
                obj_flag = True
                right_bound = i

            if val <= x/2 and obj_flag == True:
                obj_flag = False
        if right_bound == 0:
            right_bound = x
        
    # center_x_avg += (right_bound - left_bound)/2 + left_bound
    # center_x = center_x_avg/y_spacing
        sum_center_x += (right_bound - left_bound)/2 + left_bound
    center_x = sum_center_x/num_iter
    # print("Center x", center_x)
    # print("left bound: ", left_bound)
    # print("right bound: ", right_bound)
    return int(center_x)

max_angle = 25
max_offset = 20
def direction(grid, counter):
    y,x = grid.shape
    center_x = find_center(grid)
    # if counter % 10 == 0:
    #     print("Center x:", center_x)
    
    # Positive number means turn right, negative means turn left
    # Mapping direction to steering angle
    spacing = center_x - int(x/2)+10
    step_size = max_angle/max_offset

    steering = spacing*step_size
    if steering >= max_angle:
        steering = max_angle
    if steering <= -max_angle:
        steering = -max_angle
    # if counter % 10 == 0:
    #     print("Steering: ", steering)
    return steering


def process_lines(lines):
    max_angle = 25
    max_offset = 20
    slopes_pos = []
    slopes_neg = []
    slopes = []
    steering = 0
    slopes_avg = 0
    if(lines is not None):
        for line in lines:
            x1,y1,x2,y2 = line[0]
            m = ((y1-y2)/(x2-x1))
            if m > 0:
                slopes_pos.append(m)
            else:
                slopes_neg.append(m)
            slopes.append(m)
        pass
        slopes = np.array(slopes)
        slopes_neg = np.array(slopes_neg)
        slopes_pos = np.array(slopes_pos)
        slopes_avg = np.average(slopes)
        
        
        step_size = max_angle/2

        steering = slopes_avg*step_size
        if steering >= max_angle:
            steering = max_angle
        if steering <= -max_angle:
            steering = -max_angle

    else:
        steering = 0
    print("slopes: ", slopes_avg, " Steering: ", steering)
    # print(steering)
    # print(slopes_neg)
    return steering