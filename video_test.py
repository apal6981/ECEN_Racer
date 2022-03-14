# python3 ECEnRacer.py

# import the necessary packages
import numpy as np
# import imutils
import cv2 as cv
import compare_LR	# Trying to stay between left and right lines
from camera_processing import *
# import optimizer 

try:
	cap = cv.VideoCapture('Video200.avi')

	print("Driving Car")
	counter = 0
	while(cap.isOpened()):
		ret, rgb = cap.read()
		counter += 1
        #rgb = # video writer
		# (time, rgb, depth, accel, gyro) = rs.getData()
		
        # print("image received")
		hsv_img = hsv_processing(rgb)
		# print("HSV Processed")
		top_down_img = transform_birds_eye(hsv_img)
		# print("Perspective Transformed")
		bins = binner(top_down_img)
		# print("Bins calculated")
		# path = optimizer.find_path(bins)
		steering_angle = compare_LR.direction(bins, counter)
		print("Steering angle: ", steering_angle)
		# print("Steering angle calculated")
	    # controller.steering(Car, steering_angle)
		# print("Sending steering angle")

		
except Exception as e:
	print(e)

finally:
	print("Deleting Car and Camera")

