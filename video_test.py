# python3 ECEnRacer.py

# import the necessary packages
import numpy as np
import matplotlib.pyplot as plt
# import imutils
import cv2 as cv
import compare_LR # Trying to stay between left and right lines
from camera_processing import *
# import optimizer 

frames_per_steering = 5
steering_array = np.empty(frames_per_steering)
# try:
cap = cv.VideoCapture('video_derek.avi')
print("Driving Car")
counter = 0
while(cap.isOpened()):
	counter += 1
	ret, rgb = cap.read()
	#rgb = # video writer
	# (time, rgb, depth, accel, gyro) = rs.getData()
	view_rgb = draw_points(rgb)
	# print("image received")
	hsv_img, noodles, cones = hsv_processing(rgb)
	# print("HSV Processed")
	top_down_img = (hsv_img)
	# top_down_img = transform_birds_eye(hsv_img)
	# print("Perspective Transformed")

	bins = binner(top_down_img)
	# bins = chop_binner(bins, 0.25) # chop off the top 25%
	img8 = (bins).astype('uint8')
	# points = draw_points(rgb)
	# cv.imshow("Points", points)
	# kernel = np.ones((4,4),np.uint8)

	# img8 = cv.morphologyEx(img8, cv.MORPH_OPEN, kernel)
	# bins1 = cv.cvtColor(bins, cv.CV_8U)
	blurred = cv.GaussianBlur(img8, (11,11), 0)
	ret, blurred = cv.threshold(blurred, 40, 255,cv.THRESH_BINARY)
	cv.imshow("blur", blurred)
	# hist = np.histogram(blurred[5 : int(np.shape(blurred)[0]/2), :], bins=10)
	# print(hist)
	# plt.hist(hist)

	#fitting a line:
	# contours,hierarchy = cv.findContours(blurred, 1, 2)
	# if len(contours) != 0:
	# 	cnt = contours[0]
	# 	rows,cols = blurred.shape[:2]
	# 	[vx,vy,x,y] = cv.fitLine(cnt, cv.DIST_L2,0,0.01,0.01)
	# 	lefty = int((-x*vy/vx) + y)
	# 	righty = int(((cols-x)*vy/vx)+y)
	# 	cv.line(rgb,((cols-1)*2,(righty*2)),(0,lefty*2),(255,0,0),2)



	# blurred = blurred.astype('float32')    
	edges = cv.Canny(blurred, 50, 200)
	minLineLength = 5
	maxLineGap = 10
	lines = cv.HoughLinesP(edges,rho=2,theta=np.pi/180,threshold=20, minLineLength=minLineLength,maxLineGap=maxLineGap)
	cv.imshow("edges", edges)
	# backtorgb = cv.cvtColor(bins,cv.COLOR_GRAY2RGB)

	# Sum lines and infer steering
	if(lines is not None):
		for line in lines:
			for x1,y1,x2,y2 in line:
				cv.line(rgb,(x1*2,y1*2),(x2*2,y2*2),(0,255,0),2)
	cv.imshow("lines", rgb)
	
	steering_angle = compare_LR.process_lines(lines)






	# print("Bins calculated")
	# path = optimizer.find_path(bins)
	# steering_angle = compare_LR.direction(bins, counter)
	
	# if counter == 1:
	# 	steering_array = np.full(frames_per_steering, steering_angle) # fill array with first value
	# steering_array[counter % frames_per_steering] = steering_angle
	# if counter % frames_per_steering == 0:
	# 	print("Steering angle: ", np.average(steering_array))

	# cv.imshow("RGB", view_rgb)
	# cv.imshow("HSV", hsv_img)
	# cv.imshow("Top down", top_down_img)
	cv.imshow("Bins", img8)
	cv.waitKey()
	# print("Steering angle calculated")
	# controller.steering(Car, steering_angle)
	# print("Sending steering angle")

		
# except Exception as e:
# 	print(e)

# finally:
# 	print("Deleting Car and Camera")

