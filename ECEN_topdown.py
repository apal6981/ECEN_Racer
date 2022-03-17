# python3 ECEnRacer.py
''' 
This program is for ECEN-631 BYU Race
*************** RealSense Package ***************
From the Realsense camera:
	RGB Data
	Depth Data
	Gyroscope Data
	Accelerometer Data
*************** Arduino Package ****************
	Steer(int degree) : -30 (left) to +30 (right) degrees
	Drive(float speed) : -3.0 to 3.0 meters/second
	Zero(int PWM) : Sets front wheels going straight around 1500
	Encoder() : Returns current encoder count.  Reset to zero when stop
	Pid(int flag) : 0 to disable PID control, 1 to enable PID control
	KP(float p) : Proporation control 0 ~ 1.0 : how fast to reach the desired speed.
	KD(float d) : How smoothly to reach the desired speed.

    EXTREMELY IMPORTANT: Read the user manual carefully before operate the car
**************************************
'''

# import the necessary packages
from Arduino import Arduino
from RealSense import *
import numpy as np
# import imutils
import cv2 as cv
import compare_LR	# Trying to stay between left and right lines
import controller	# For driving and steering
# import optimizer 
from camera_processing import *

frames_per_steering = 2
steering_array = np.empty(frames_per_steering)
driving_size = 4
driving_array = np.ones(driving_size)
old_turn = 0
try:
	print("Init Camera")
	rs = RealSense("/dev/video2", RS_VGA)		# RS_VGA, RS_720P, or RS_1080P
	fps = 30
	# writer_depth = cv.VideoWriter('Video_derek_D.avi', cv.VideoWriter_fourcc(*'MJPG'), fps, (depth.shape[1], depth.shape[0]), True)

	print("Init Car")
	# Use $ ls /dev/tty* to find the serial port connected to Arduino
	Car = Arduino("/dev/ttyUSB0", 115200)                # Linux
	#Car = Arduino("/dev/tty.usbserial-2140", 115200)    # Mac

	Car.zero(1500)      # Set car to go straight. Change this for your car.
	Car.pid(1)          # Use PID control
	# You can use kd and kp commands to change KP and KD values.  Default values are good.
	# loop over frames from Realsense
	# print("Driving Car")
	# controller.start_driving(Car)
	# print("Car started")
	counter = 0
	writer = None
	speed = 1
	while True:
		counter += 1
		(time, rgb, depth, accel, gyro) = rs.getData()

		if writer is None:
			writer = cv.VideoWriter('Video_derek.avi', cv.VideoWriter_fourcc(*'MJPG'), fps, (rgb.shape[1], rgb.shape[0]), True)
			writer_depth = cv.VideoWriter('Video_derek_D.avi', cv.VideoWriter_fourcc(*'MJPG'), fps, (depth.shape[1], depth.shape[0]), True)
			cv.imwrite("first_depth.jpg", depth)

		writer.write(rgb)
		writer_depth.write(depth)
		# print("image received")
		hsv_img = hsv_processing(rgb)
		# print("HSV Processed")
		top_down_img = transform_birds_eye(hsv_img)
		# print("Perspective Transformed")
		bins = binner(top_down_img)
		# print("Bins calculated")
		# path = optimizer.find_path(bins)
		steering_angle = compare_LR.direction(bins, counter)
		if counter == 1:
			steering_array = np.full(frames_per_steering, steering_angle) # fill array with first value
			old_turn = steering_angle
		steering_array[counter % frames_per_steering] = steering_angle
		# print("Steering Array: ", steering_array)
		steering_command = np.average(steering_array)
		# if counter % frames_per_steering == 0:
			# print("Steering angle: ", steering_angle)
			# print("Steering angle calculated")
		if np.abs(steering_angle) < 8 and np.abs(old_turn) < 8:
			steering_angle = (steering_angle + old_turn)/2
		
		controller.steering(Car, steering_angle)
		old_turn = steering_angle
			# Enable for speed testing!
		if counter % 3 == 0:
			if np.abs(steering_angle) < 5:
				speed = 5
			elif np.abs(steering_angle) < 10:
				speed = 3
			elif np.abs(steering_angle) < 16.5:
				speed = 1.5
			else:
				speed = 1
		if speed == 1:
			avg_speed = 0.8
		else:
			driving_array[counter % driving_size] = speed
			avg_speed = np.average(driving_array)
		Car.drive(avg_speed)
		# writer.write(rgb)
		# writer_depth.write(depth)
		# print("Sending steering angle")
		
		
		# if counter % 50 == 0:
		# Car.drive(0.8)
		# controller.go_forward(Car, 1.6)
			# print("Driving command sent")
		
except Exception as e:
	print(e.with_traceback())

finally:
	print("Deleting Car and Camera")
	if Car is not None:
		Car.drive(0)
		del Car
	if rs is not None:
		del rs
	if writer is not None:
		writer.release()
		writer_depth.release()

