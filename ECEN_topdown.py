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
driving_array = np.ones(3)

try:
	print("Init Camera")
	rs = RealSense("/dev/video2", RS_VGA)		# RS_VGA, RS_720P, or RS_1080P
	writer = None

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
	speed = 1
	while True:
		counter += 1
		(time, rgb, depth, accel, gyro) = rs.getData()
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
		steering_array[counter % frames_per_steering] = steering_angle
		# print("Steering Array: ", steering_array)
		steering_command = np.average(steering_array)
		# if counter % frames_per_steering == 0:
			# print("Steering angle: ", steering_angle)
			# print("Steering angle calculated")
		controller.steering(Car, steering_angle)
			# Enable for speed testing!
		if counter % 3 == 0:
			if np.abs(steering_angle) < 5:
				speed = 2.5
			elif np.abs(steering_angle) < 10:
				speed = 1.5
			elif np.abs(steering_angle) < 16.5:
				speed = 1.3
			else:
				speed = 0.8
		driving_array[counter % 3] = speed
		avg_speed = np.average(driving_array)
		Car.drive(avg_speed)
		# print("Sending steering angle")
		
		
		# if counter % 50 == 0:
		# Car.drive(0.8)
		# controller.go_forward(Car, 1.6)
			# print("Driving command sent")
		
except Exception as e:
	print(e)

finally:
	print("Deleting Car and Camera")
	if Car is not None:
		Car.drive(0)
		del Car
	if rs is not None:
		del rs

