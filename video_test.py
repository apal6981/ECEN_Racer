# python3 ECEnRacer.py

# import the necessary packages
import numpy as np
# import imutils
import cv2 as cv
import compare_LR	# Trying to stay between left and right lines
from camera_processing import *
# import optimizer 

frames_per_steering = 5
steering_array = np.empty(frames_per_steering)
try:
	cap = cv.VideoCapture('Video200.avi')

	print("Driving Car")
	counter = 0
	while(cap.isOpened()):
		counter += 1
		ret, rgb = cap.read()
        #rgb = # video writer
		# (time, rgb, depth, accel, gyro) = rs.getData()
		view_rgb = draw_points(rgb)
        # print("image received")
		hsv_img = hsv_processing(rgb)
		# print("HSV Processed")
		# top_down_img = (hsv_img)
		top_down_img = transform_birds_eye(hsv_img)
		# print("Perspective Transformed")
		bins = binner(top_down_img)
		# print("Bins calculated")
		# path = optimizer.find_path(bins)
		steering_angle = compare_LR.direction(bins, counter)
		if counter == 1:
			steering_array = np.full(frames_per_steering, steering_angle) # fill array with first value
		steering_array[counter % frames_per_steering] = steering_angle
		if counter % frames_per_steering == 0:
			print("Steering angle: ", np.average(steering_array))

		# cv.imshow("RGB", view_rgb)
		# cv.imshow("HSV", hsv_img)
		# cv.imshow("Top down", top_down_img)
		cv.imshow("Bins", bins)
		cv.waitKey()
		# print("Steering angle calculated")
	    # controller.steering(Car, steering_angle)
		# print("Sending steering angle")

		
except Exception as e:
	print(e)

finally:
	print("Deleting Car and Camera")

