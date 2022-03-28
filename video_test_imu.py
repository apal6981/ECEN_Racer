# python3 ECEnRacer.py

# import the necessary packages
import numpy as np
import matplotlib.pyplot as plt
# import imutils
import cv2 as cv
import optimizer_clean as optimizer
import compare_LR # Trying to stay between left and right lines

import mahotas as mh
from pylab import imshow, show
import matplotlib.pyplot as plt


from camera_processing import *

frames_per_steering = 5
old_steering = 0
steering_array = np.empty(frames_per_steering)
# try:
cap = cv.VideoCapture('videos/derek_pd_greedy.avi')
# cap = cv.VideoCapture('videos/ashton_derek_joint_cones.avi')
# cap = cv.VideoCapture('videos/ashton_derek_joint.avi')
imu_data = np.load('videos/derek_pd_greedy.npy')

# imu = imu_data[list(range(1,len(imu_data)+1,2))]
# gyro = imu_data[list(range(0,len(imu_data),2))]

print("Driving Car")
counter = 0     
# k_p = 0.8   # increase until oscillation, then half
# k_d = 0.4   # increase until minimal oscillation
k_p = 0.8
k_d = 0.1
# fig = plt.figure()


while(cap.isOpened()):
    counter += 1
    print("Frame: ", counter)
    ret, rgb = cap.read()
    cv.imshow("rgb", rgb)
    
    #frame 385-400: cone crash, should back up, check max val at that time to determin back up
    # if counter <= 11:
        # continue
    
    hsv_img = hsv_processing(rgb)
    hsv_img = transform_birds_eye(hsv_img)

    bins = binner(hsv_img)
    # bins = chop_binner(bins, 0.25) # chop off the top 25%
    img8 = (bins).astype('uint8')
    
    
    # slope, grid_avg = optimizer.get_slope_single(hsv_img)
    slope, grid_avg = optimizer.get_slope_global(hsv_img)
    steering_angle, speed, old_steering = optimizer.get_steering(slope, grid_avg, old_steering, counter)
    print("Steering:", steering_angle)


    
