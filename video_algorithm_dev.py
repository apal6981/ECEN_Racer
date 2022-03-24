import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
from camera_processing import *

# print(create_turn_matrix(20,10))
# git test
vidcap = cv.VideoCapture('ashton_derek_joint_cones.avi')
count = 0

IMAGE_H = 250
IMAGE_W = 640
OUT_H_FACTOR = 1

writer = cv.VideoWriter('test.mp4', cv.VideoWriter_fourcc(*'mp4v'), 15, (640, 170), True)
while True:

    success, frame = vidcap.read()
    if not success:
        break
    # Get HSV image of rgb image
    # hsv_img = hsv_processing(frame)
    # get the min and max values of the bins of the hsv image, chop off the top of the hsv image
    # bin_matrix = binner2(hsv_img[80:, :])
    # matrix = turn_matrix_calc(bin_matrix)
    # turn_values = get_min_max(matrix)
    # print(turn_values)
    # priority = column_matrix(bin_matrix)
    # print("\n",priority)
    # max_c = max_columns(priority)
    # print(max_c)
    # min_of_max_c = np.min(max_c)
    # max_c_indices = np.where(max_c == min_of_max_c)[0]
    # print(len(max_c_indices))
    # while len(max_c_indices) < 2:
    #     min_of_max_c += 1
    #     print("increase")
    #     max_c_indices = np.where(max_c == min_of_max_c)[0]
    # print(max_c_indices)
    # consec = get_consecutive_arrays(max_c_indices)
    # print("conseq",consec)
    #
    # sharp = check_sharp_corners(max_c,consec)
    # print("sharp corners:",sharp)
    # print("turn", average_turn_value(sharp))

    line_thresh, obs_thresh = hsv_line_obs_processing(frame[80:,:])
    line_bin = binner3(line_thresh, BIN_WIDTH,BIN_HEIGHT,pixel_count=100)
    obs_bin = binner3(obs_thresh,BIN_WIDTH, BIN_HEIGHT,pixel_count=200)
    line_matrix = mask_bins(line_bin,"line")
    left_matrix = mask_bins(obs_bin,"left")
    right_matrix = mask_bins(obs_bin,"right")
    print("\n\nline\n",line_matrix)
    print("left\n",left_matrix)
    print("right\n",right_matrix)
    # decision = left_right_line_decision(left_matrix,right_matrix,line_matrix)
    # print("Decision:",decision)
    # find_distances(line_bin,obs_bin)
    decision = turn_decision(line_bin,obs_bin)
    if decision is None:
        decision = 0
    print("Decision:",decision)
    line_img = draw_bins2(line_matrix,cv.cvtColor(line_thresh[:,:],cv.COLOR_GRAY2BGR))
    cv.imshow("lines",line_img)
    left_img = draw_bins2(left_matrix, cv.cvtColor(obs_thresh[:, :], cv.COLOR_GRAY2BGR))
    cv.imshow("left", left_img)
    right_img = draw_bins2(right_matrix, cv.cvtColor(obs_thresh[:, :], cv.COLOR_GRAY2BGR))
    cv.imshow("right", right_img)
    print("turning angle:",PARABOLIC_TURN_VALUES[decision-21])
    cv.waitKey()