# Not implemented yet
# from scipy.optimize import minimize, Bounds
# import mahotas as mh
import cv2 as cv

import numpy as np
import matplotlib.pyplot as plt
import camera_processing

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


def greedy(grid):
    camera_middle_offset = 0
    height, width = grid.shape
    y_step = int(height*0.06)
    max_lat_step = int(width * 0.04)
    num_steps = 14
    num_routes = 5
    mid = int(width/2)
    
    paths = []
    # mid_start = [int(mid-0.2*width), mid, int(mid+0.2*width)]
    mid_start = [int(mid-0.25*width), int(mid-0.1*width), mid, int(mid+0.1*width), int(mid+0.25*width),]
    for n in range(num_routes):
        
        x0 = mid_start[n]
        # consider setting first val to the middle
        path = [mid, x0]
        for i in range(1, num_steps):
            left_bound = x0 - max_lat_step
            right_bound = x0 + max_lat_step
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
            
            shifted_max_idx = x0-max_lat_step + max_idx
            
            if shifted_max_idx < 0:
                shifted_max_idx = 0 

            x0 = shifted_max_idx
            path.append(x0)
        path = np.array(path)
        paths.append(path)
    paths = np.array(paths)
    # print(path)
    y_vals = []
    for i in range(path.shape[0]):
        y_vals.append(height-y_step*(i+2))
    y_vals = np.array(y_vals)
    grid_vals = []
    for i in range(np.shape(paths)[0]):
        grid_vals.append(grid[y_vals, paths[i]])
    
    grid_vals = np.array(grid_vals)
    
    return paths, y_vals, grid_vals, y_step

# Add white borders to left and ride sides
def border(img):
    img[:,0:10] = 255
    # img[:,-1] = 255
    # img[0,:] = 255
    # img[-1,:] = 255
    return img

colors = ['b', 'g', 'r', 'c', 'y', 'm', 'k', 'w']
# Pass in hsv_img
# Slope ranges from left(1.2) to right(-1.0) counter intuitive
# birds eye views:
# camera_points_orig = np.array([[20,50],[640-20, 50],[80, 250], [640-80, 250]])
def get_slope(img):
    hsv_img = img[:, 160:480]
    
    blurred = cv.GaussianBlur(hsv_img, (11,11), 0)
    ret, blurred = cv.threshold(blurred, 40, 255,cv.THRESH_BINARY)
    
    bins = camera_processing.binner(blurred)
    bins = bins[:, 13:-1]
    # bins = border(bins)
    # inverted = np.invert(blurred)
    # dmap = mh.distance(inverted)
    dmap = distance_transform(bins)
        
    # path, y_step = optimizer.find_path(dmap)
    paths, y_vals, grid_vals, y_step = greedy(dmap)
    grid_avg = grid_vals.mean(axis=1)
    b_idx = np.argmax(grid_avg)
    best_path = grid_avg[b_idx]
    worst_path = np.min(grid_avg)

    percent_grid_diff = np.abs(best_path-worst_path)/((best_path+worst_path)/2)
    
    num_steps = np.shape(paths)[1]
    upperbound = int(num_steps * 0.8)

    # consider calculating the slope from the middle
    slopes = []
    for i in range(np.shape(paths)[0]):
        left_x = paths[i][upperbound]
        # if left_x < 0:
        #     left_x = 0
        slopes.append((left_x-paths[i][1])/(y_vals[upperbound]-y_vals[1]))
    
    slopes = np.array(slopes)
    slope = slopes[b_idx]   
    
    best_slope = np.max(slopes)
    worst_slope = np.min(slopes)
    percent_slope_diff = np.abs(best_slope-worst_slope)/((best_slope+worst_slope)/2)

    if percent_grid_diff < 0.1 and np.average(slopes) < 0.2:
        print("####### slope criteria met! #######")
        slope = -10
    '''
    print("Best path:", b_idx, "Slope:", slope)
    print("Grid avg: ", grid_avg)
    print("Slopes:", slopes)
    print("%_grid diff:", percent_grid_diff)
    print("%_slope diff:", np.average(slopes))
    plt.imshow(dmap)
    
    for i in range(np.shape(paths)[0]):
        plt.scatter(paths[i,:], y_vals, color=colors[i%np.shape(colors)[0]])

    
<<<<<<< HEAD
    cv.imshow("hsv", hsv_img)
    cv.imshow("Bins", bins)
    plt.pause(0.01)
    plt.clf()
    cv.waitKey(0)
=======
    # cv.imshow("hsv", hsv_img)
    # cv.imshow("Bins", bins)
    # cv.waitKey(2)
    # plt.pause(0.1)
    # plt.clf()
>>>>>>> 87c0cdd657ba923d2d906608f9b2a1aa7845260e
    # plt.show()
    # print("Slope: ", slope)
    '''
    
    return slope
    # return 0