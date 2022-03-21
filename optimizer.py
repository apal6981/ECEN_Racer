# Not implemented yet
# from scipy.optimize import minimize, Bounds
import mahotas as mh
import cv2 as cv

import numpy as np
import matplotlib.pyplot as plt

'''
# New objective: maxamize point value
# minimize distance to y=0, x=50%
# minimize distance traveled?
# cons: 
# Implementation: Each step is z pixels ahead
# ineq: new x must be within x_dist
def obj(X,*args):

    grid, mid, iteration, height, y_step = args[0][0], args[0][1], args[0][2], args[0][3], args[0][4]
    # X = X.reshape((int(len(X)/2),2))
    
    # Distance from y=0, x=mid
    dist_from_y0 = 0
    for i in range(np.size(X)):
        dist_from_y0 += np.sqrt((X[i]-mid)**2 + (height-iteration*y_step)**2) 

    # Sum of points in grid (we want to minimize)
    grid_sum = 0
    for i in range(np.size(X)):
        grid_sum += grid[int(height-(iteration+i)*y_step), int(X[i])]
    grid_sum *= -1

    # dist_sum = 0
    # for i in range(np.size(X)):
    #     dist_sum += (X[i,1])
    # return (dist_from_y0*0.5) + grid_sum
    return grid_sum

# ineq: new x must be within x_dist
def anchor_con(X,*args):
    # X = X.reshape((int(len(X)/2),2))
    max_lateral_step, x0 = args[0], args[1]
    constraint = []
    for i in range(np.size(X)):
        if i==0:
            constraint.append(np.abs(x0-X[i])-max_lateral_step)
        else:    
            constraint.append(np.abs(X[i-1]-X[i])-max_lateral_step)
    return np.array(constraint)



# New objective: maxamize point value
# minimize distance to y=0, x=50%
# minimize distance traveled?
# cons: 
# Implementation: Each step is z pixels ahead
# ineq: new x must be within x_dist
def find_path(grid):
    # find starting position
    height, width = grid.shape
    y_step = int(height*0.02)
    mid = int(width/2)
    
    x0 = mid
    num_steps=3
    # X = np.array([
    #     [height-y_step*2, mid],
    #     [height-y_step*3, mid],
    #     [height-y_step*4, mid]
    # ])
    X = np.full(num_steps, mid)
    i = 0
    args = [grid, mid, i, height, y_step]
    max_lateral_step = int(width * 0.05)
    stepsize_con_args = [max_lateral_step, x0]

    con = (
        {'type':'ineq',
        'fun':anchor_con,
        'args':stepsize_con_args},
    )

    path = np.array([x0])
    
    num_iter = 0
    
    lb = np.full(num_steps, 0)
    up = np.full(num_steps, width-1)
    bounds = Bounds(lb, up)

    # while num_iter < 10:
    for i in range(2, 10):
        num_iter += 1
        args = [grid, mid, i, height, y_step]
        x = minimize(obj,X,args=args,constraints=con, bounds=bounds) #, method='trust-constr')
        # X = x.x.reshape((int(len(x.x)/2),2))
        X=x.x
        X = np.append(X[1:], mid)
        # X = np.vstack([X[1:],mid])
        path = np.vstack([path,x.x[0]])


        # if is1stIteration:
        #     true_optimum = step_size * np.array([
        #         [np.sqrt(2)/2,np.sqrt(2)/2],
        #         [2*np.sqrt(2)/2,2*np.sqrt(2)/2],
        #         [3*np.sqrt(2)/2,3*np.sqrt(2)/2]
        #     ])
            # print("Accuracy:",np.linalg.norm(X-true_optimum),"meters")
            # print("Number of iterations until convergence:",x.nit)
            # is1stIteration = False

        # Update variables (i.e. move along path)
        x0 = path[-1][0]
        stepsize_con_args = [max_lateral_step, x0]
        con = (
            {'type':'ineq',
            'fun':anchor_con,
            'args':stepsize_con_args},
        )
    print(path)
    return(path, y_step)
'''
# Time to write greedy algorithm, find max value within preset bounds,
# then update bounds for the next search, use np.max to find the index
# of max bounds,then offset to the correct location
# Might need to implement method to pull greedy algorithm towards top to
# avoid drift, or instead, calculate slope of line to determine steering angle

def greedy(grid):
    camera_middle_offset = 15
    height, width = grid.shape
    y_step = int(height*0.02)
    mid = int(width/2)+camera_middle_offset
    max_lat_step = int(width * 0.02)
    x0 = mid
    num_steps = 30
    path = []
    for i in range(1, num_steps):
        left_bound = x0 - max_lat_step
        right_bound = x0 + max_lat_step
        if left_bound < 0:
            left_bound = 0
        if right_bound > (width-1):
            right_bound = width - 1
        sub_array = grid[height-(i*y_step), left_bound:right_bound]
        max_idx = np.argmax(sub_array)
        shifted_max_idx = x0-max_lat_step + max_idx
        x0 = shifted_max_idx
        path.append(x0)
    path = np.array(path)
    # print(path)
    y_vals = []
    for i in range(path.shape[0]):
        y_vals.append(height-y_step*(i+2))
    y_vals = np.array(y_vals)

    grid_vals = grid[y_vals, path]
    
    return path, y_vals, grid_vals, y_step

# Add white borders to left and ride sides
def border(img):
    img[:,0] = 255
    img[:,-1] = 255
    # img[0,:] = 255
    # img[-1,:] = 255
    return img

# Pass in hsv_img
# Slope ranges from left(1.2) to right(-1.0) counter intuitive
# birds eye views:
# camera_points_orig = np.array([[20,50],[640-20, 50],[80, 250], [640-80, 250]])
def get_slope(img):
    hsv_img = img[:, 160:480]
    
    
    blurred = cv.GaussianBlur(hsv_img, (11,11), 0)
    ret, blurred = cv.threshold(blurred, 40, 255,cv.THRESH_BINARY)
    
    inverted = np.invert(blurred)
    dmap = mh.distance(inverted)
        
    # path, y_step = optimizer.find_path(dmap)
    path, y_vals, grid_vals, y_step = greedy(dmap)
    num_steps = np.shape(path)[0]
    upperbound = int(num_steps * 0.4)
    slope = ((path[upperbound]-path[1])/(y_vals[upperbound]-y_vals[1]))
    
    plt.imshow(dmap)
    plt.scatter(path, y_vals, color='r')
    
    cv.imshow("hsv", hsv_img)
    cv.imshow("Bins", blurred)
    cv.waitKey(2)
    plt.show()
    return slope