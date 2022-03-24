import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import camera_processing

# Time to write greedy algorithm, find max value within preset bounds,
# then update bounds for the next search, use np.max to find the index
# of max bounds,then offset to the correct location
# Might need to implement method to pull greedy algorithm towards top to
# avoid drift, or instead, calculate slope of line to determine steering angle

def _upscan(f):
    for i, fi in enumerate(f):
        if fi == np.inf: continue
        for j in range(1,i+1):
            x = fi+j*j
            if f[i-j] < x: break
            f[i-j] = x

def distance_transform(bitmap):
    f = np.where(bitmap, 0.0, np.inf)
    for i in range(f.shape[0]):
        _upscan(f[i,:])
        _upscan(f[i,::-1])
    for i in range(f.shape[1]):
        _upscan(f[:,i])
        _upscan(f[::-1,i])
    np.sqrt(f,f)
    return f

def greedy_global(grid):
    camera_middle_offset = 0
    height, width = grid.shape
    y_step = int(height*0.06)
    num_steps = 14
    mid = int(width/2)
    
    paths = []
    
    # consider setting first val to the middle
    for n in range(2):
        if n == 0:
            max_lat_step = int(width * 0.04) # Local
        else:
            max_lat_step = int(width) # Global

        x0 = mid - int(width * 0.1)
        path = [x0]
        for i in range(1, num_steps):
            left_bound = x0 - max_lat_step
            right_bound = x0 + round(max_lat_step*1.5)
            if left_bound < 0:
                left_bound = 0
            if right_bound > (width-1):
                right_bound = width - 1
            sub_array = grid[height-1-(i*y_step), left_bound:right_bound]
            
            b = sub_array[::-1]
            right_idx = len(b) - np.argmax(b) - 1
            left_idx = np.argmax(sub_array)
        
            if right_idx != left_idx:
                max_idx = int(round((right_idx + left_idx)/2,0))
            else:
                max_idx = right_idx
     
            if n == 0:
                shifted_max_idx = x0-max_lat_step + max_idx
                if shifted_max_idx < 0:
                    shifted_max_idx = 0 
                x0 = shifted_max_idx
            else:
                x0 = max_idx # shifted_max_idx
            path.append(x0)
        path = np.array(path)
        paths.append(path)

    paths = np.array(paths)
    y_vals = []
    
    for i in range(path.shape[0]):
        y_vals.append(height-y_step*(i+2))

    y_vals = np.array(y_vals)
    grid_vals_array = []
    
    local_path = paths[0]
    t = 8 # number of steps ahead we're looking
    grid_vals = grid[y_vals[0:t], local_path[0:t]]
    
    return paths, y_vals, grid_vals, y_step

# Add white borders to left and ride sides
def border(img):
    img[:,0] = 255      # Left
    img[:,-1] = 255     # Right
    # img[0,:] = 255    # Top
    # img[-1,:] = 255   # Bottom
    return img

colors = ['b', 'g', 'r', 'c', 'y', 'm', 'k', 'w']

def get_slope_global(img):
    hsv_img = img[:, 160:480]
    
    blurred = cv.GaussianBlur(hsv_img, (11,11), 0)
    ret, blurred = cv.threshold(blurred, 40, 255,cv.THRESH_BINARY)
    
    bins = camera_processing.binner(blurred)
    bins = bins[:, 13:-1]
    bins = border(bins)
    # inverted = np.invert(blurred)
    # dmap = mh.distance(inverted)
    dmap = distance_transform(bins)
        
    # path, y_step = optimizer.find_path(dmap)
    paths, y_vals, grid_vals, y_step = greedy_global(dmap)
    grid_avg = grid_vals.mean(axis=0)
    b_idx = np.argmax(grid_avg)

    # percent_grid_diff = np.abs(best_path-worst_path)/((best_path+worst_path)/2)
    
    num_steps = np.shape(paths)[1]
    upperbound = int(num_steps * 0.5)

    # consider calculating the slope from the middle
    slopes = []
    # Constrained Greedy
    left_x = paths[0][upperbound]
    slopes.append((left_x-paths[0][1])/(y_vals[upperbound]-y_vals[1]))
    
    # Global

    def reject_outliers(data, m=2):
        return data[abs(data - np.mean(data)) < m * np.std(data)]

    # reject outliers:
    ending_idx = np.shape(paths[1])[0] - 1 
    outlier_idx = ending_idx
    for i in range(ending_idx, 0, -1):
        if abs(paths[1][i] - np.mean(paths[1][1:-1])) < 1*np.std(paths[1][1:-1]):
            outlier_idx = i

            break
    global_path = paths[1][0:outlier_idx]
    global_y_vals = y_vals[0:outlier_idx]

    ones = np.ones_like(global_path)
    psi = np.array([global_y_vals, ones]).T
    w = np.linalg.inv(psi.T@psi)@psi.T@global_path
    
    top_y_val = 15
    top_x_val = w[0]*top_y_val+w[1]
    global_slope = (top_x_val-paths[1][0])/(top_y_val-y_vals[0])    

    #'''
    plt.imshow(dmap)
    
    plt.scatter(paths[0,:], y_vals, color=colors[0])         # Local points
    plt.scatter(global_path, global_y_vals, color=colors[1]) # Global points
    
    # Local dashed line
    point1 = [paths[1][0], y_vals[0]]
    point2 = [top_x_val, top_y_val]
    x_values = [point1[0], point2[0]]
    y_values = [point1[1], point2[1]]
    plt.plot(x_values, y_values, c='r', linestyle="--")
    
    # Line of best fit
    x_vals = np.arange(start=0, stop=30, step=1)
    y_vals1 = w[0]*x_vals+w[1]
    plt.plot(y_vals1, x_vals, c='r')
    
    cv.imshow("hsv", hsv_img)
    cv.imshow("Bins", bins)

    plt.xlim([0, 65])
    plt.ylim([30, 0])
    
    plt.pause(0.1)
    plt.clf()
    cv.waitKey(0)
    # plt.show()

    #'''    
    
    return global_slope, grid_avg


# Average is less than 5: turn 20
# less than 3: turn 30
# if slope is vertical: turn right based on grid_avg
# if slope is
max_steering = 25
max_slope = 2
max_grid = 20
slope_step = max_steering/max_slope
grid_step = max_steering/max_grid
k_p = 0.8
k_d = 0.1
old_steering = 0

def get_steering(slope, grid_avg, old_steering, counter):
    max_speed = 2
    steering = 0
    speed = 0
    direction = ''
    if slope > 0.1:
        direction = 'L'
    elif slope < -0.1:
        direction = 'R'
    else:
        direction = 'R'
        slope = -2.0

    steering_slope = slope * slope_step * 1.5
    if np.isposinf(grid_avg):
        grid_avg = 100
    steering_grid = 1/grid_avg * grid_step
    steering = steering_slope
    # steering = (steering_slope + steering_grid)/2
    if grid_avg > 12 and (slope) > 1.8: # 20
        steering = steering / 5
    elif grid_avg > 12:
        steering = steering / 7
    elif grid_avg > 8:  # 13
        steering = steering / 4
    elif grid_avg > 4: # 7
        steering = steering / 2
    elif grid_avg > 2: # 5
        steering = steering / 1.5
    else:
        if steering < 0:
            steering = max_steering*-1
        else:
            steering = max_steering
    
    steering *= -1
    steering = int(steering)
    print("Steering:", round(steering,2), "Grid avg:", grid_avg)

    # PD Implementation
    if counter == 1:
        old_steering = steering
    steering = k_p * steering + k_d * (steering - old_steering)
    old_steering = steering

    # Handling speed
    speed = 2 - abs(steering) / 20
    if abs(steering) < 5:
        speed = 3

    # Handling backup case
    if (abs(steering) < 6 and grid_avg < 8) or grid_avg < 4:
        print("#### BACKUP! ####")
        speed = 0

    return steering, speed, old_steering # backup flag


