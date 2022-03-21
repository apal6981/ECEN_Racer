import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
from camera_processing import *

# print(create_turn_matrix(20,10))
# git test
vidcap = cv.VideoCapture('video_ashton.avi')
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
    hsv_img = hsv_processing(frame)
    # get the min and max values of the bins of the hsv image, chop off the top of the hsv image
    bin_matrix = binner2(hsv_img[80:, :])
    matrix = turn_matrix_calc(bin_matrix)
    priority = column_matrix(bin_matrix)
    print("\n",priority)
    max_c = max_columns(priority)
    print(max_c)
    min_of_max_c = np.min(max_c)
    max_c_indices = np.where(max_c == min_of_max_c)[0]
    print(len(max_c_indices))
    while len(max_c_indices) < 2:
        min_of_max_c += 1
        print("increase")
        max_c_indices = np.where(max_c == min_of_max_c)[0]
    print(max_c_indices)
    consec = get_consecutive_arrays(max_c_indices)
    print("conseq",consec)

    sharp = check_sharp_corners(max_c,consec)
    print("sharp corners:",sharp)
    print("turn", average_turn_value(sharp))
    print_img = draw_bins(priority,cv.cvtColor(hsv_img[80:,:],cv.COLOR_GRAY2BGR))
    # print(print_img.shape)
    # writer.write(print_img)
    cv.imshow("frame",print_img)
    cv.waitKey()