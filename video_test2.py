# python3 ECEnRacer.py

# import the necessary packages
import numpy as np
import matplotlib.pyplot as plt
# import imutils
import cv2 as cv
import optimizer
import compare_LR # Trying to stay between left and right lines

import mahotas as mh
from pylab import imshow, show
import matplotlib.pyplot as plt


from camera_processing import *

frames_per_steering = 5
old_steering = 0
steering_array = np.empty(frames_per_steering)
# try:
cap = cv.VideoCapture('video_ashton.avi')
print("Driving Car")
counter = 0     
k_p = 0.8   # increase until oscillation, then half
k_d = 0.4   # increase until minimal oscillation

# fig = plt.figure()

while(cap.isOpened()):
    counter += 1
    ret, rgb = cap.read()
    hsv_img = hsv_processing(rgb)
    hsv_img = transform_birds_eye(hsv_img)

    bins = binner(hsv_img)
    bins = chop_binner(bins, 0.25) # chop off the top 25%
    img8 = (bins).astype('uint8')
    
    # hsv_img = hsv_img[:, 160:480]

    def border(img):
        img[:,0] = 255
        img[:,-1] = 255
        # img[0,:] = 255
        # img[-1,:] = 255
        return img
    
    slope = optimizer.get_slope(hsv_img)
    print("Slope: ", slope)

    # hsv_img = border(hsv_img)
    
    blurred = cv.GaussianBlur(hsv_img, (11,11), 0)
    ret, blurred = cv.threshold(blurred, 40, 255,cv.THRESH_BINARY)
    cv.imshow("hsv", hsv_img)
    cv.imshow("rgb", rgb)
    cv.imshow("Bins", blurred)
    
    '''
    #########################################################
    # creating region
    # numpy.ndarray
    # regions = np.zeros((10, 10), bool)
    
    # # setting 1 value to the region
    # regions[:3, :3] = 1
    # regions[6:, 6:] = 1
    
    # getting labeled function
    norm_val = np.linalg.norm(blurred)
    if norm_val != 0:
        
        norm1 = np.invert(blurred)
        # super_threshold_indices = norm1 > 1
        # norm1[super_threshold_indices] = 1
        # norm1 = norm1 / norm_val
        # cv.imshow("invert", norm1)
        cv.waitKey(0)

        # setting gaussian filter
        # gaussian = mh.gaussian_filter(norm1, 15)
        
        # setting threshold value
        # gaussian = (gaussian > gaussian.mean())
        
        # creating a labeled image
        # labeled, n_nucleus = mh.label(gaussian)
        
        # labeled, nr_objects = mh.label(norm1)
        
        # showing the image with interpolation = 'nearest'
        # print("Image")
        # imshow(norm1, interpolation ='nearest')
        # show()    
        # getting distance map
        dmap = mh.distance(norm1)
        
        # path, y_step = optimizer.find_path(dmap)
        path, y_vals, grid_vals, y_step = optimizer.greedy(dmap)
        num_steps = np.shape(path)[0]
        upperbound = int(num_steps * 0.3)
        slope1 = ((path[upperbound]-path[1])/(y_vals[upperbound]-y_vals[1]))
        
        upperbound = int(num_steps * 0.6)
        slope2 = ((path[upperbound]-path[1])/(y_vals[upperbound]-y_vals[1]))
        
        upperbound = int(num_steps-1)
        slope3 = ((path[upperbound]-path[1])/(y_vals[upperbound]-y_vals[1]))
        
        # current slope values: 1.2 far left, -1 far right
        print("Slope 1: ", slope1)
        print("Slope 2: ", slope2)
        print("Slope 3: ", slope3)
        
        # calculate slope changes
        init_slope = np.inf
        change_point = 0
        for i in range(np.shape(path)[0]-1):
            slope = ((path[i]-path[i+1])/(y_vals[i]-y_vals[i+1]))
            if i == 0:
                init_slope = slope
                continue
            if slope != init_slope:
                change_point = i
                break
        print("Change point: ", change_point)


        # value ideas
        # if F3 > 15000 steering max = 5
        # if F3 < 5000 steering max = 15
        #
        #
        #

        u_bound = int(num_steps*0.9)
        l_bound = int(num_steps*0.5)
        
        further_avg = np.average(grid_vals[l_bound:u_bound])
        print("L5: ", further_avg)
        print("F3: ", np.average(grid_vals[0:3]))
        print("F5: ", np.average(grid_vals[0:5]))
        print("AVG ", np.average(grid_vals))
        # print("Max Grid = ", np.max(grid_vals), " at ", np.argmax(grid_vals))
        # print("Min Grid = ", np.min(grid_vals), " at ", np.argmin(grid_vals))
        # print(grid_vals)      
        # showing image
        # print("Distance Map")
        # plt.imshow(dmap, cmap='binary_r', interpolation='nearest', vmax=2000)
        
        ########## uncomment
        # plt.imshow(dmap)
        # plt.scatter(path, y_vals, color='r')

        # # convert canvas to image
        # img = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8,sep='')
        # img  = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))

        # # img is rgb, convert to opencv's default bgr
        # img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)


        # display image with opencv or any operation you like
        # cv2.imshow("plot",img)
        plt.pause(0.2)
        plt.clf()
        # plt.show()
'''
    ################################################################

    # blurred = cv.GaussianBlur(img8, (19,9), 0)
	# cv.imshow("blur", blurred)
    
    steering_angle = compare_LR.direction(blurred, counter)
    if (counter == 0):
        old_steering = steering_angle

    # PD implementation
    steering_angle = k_p * steering_angle - k_d * (steering_angle - old_steering)
    old_steering = steering_angle

    print("Steering: ", steering_angle)

'''
def chop_binner(bins, row_percent):
    height, width = np.shape(bins)
    chop_height = int(row_percent*height)
    bins[0:chop_height, :] = 0
    return bins
    '''
