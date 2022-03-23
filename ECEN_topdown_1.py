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
import optimizer 
from camera_processing import *

frames_per_steering = 2
steering_array = np.empty(frames_per_steering)
driving_size = 4
driving_array = np.ones(driving_size)
old_steering = 0
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
    # Speed 1 values
    # k_p = 0.8   # increase until oscillation, then half
    # k_d = 0.4   # increase until minimal oscillation
    k_p = 0.8
    k_d = 0.1

    while True:
        counter += 1
        (time, rgb, depth, accel, gyro) = rs.getData()

        # if writer is None:
        #     writer = cv.VideoWriter('Video_derek_pd_greedy.avi', cv.VideoWriter_fourcc(*'MJPG'), fps, (rgb.shape[1], rgb.shape[0]), True)
		# 	# writer_depth = cv.VideoWriter('Video_derek_D.avi', cv.VideoWriter_fourcc(*'MJPG'), fps, (depth.shape[1], depth.shape[0]), True)
		# 	# cv.imwrite("first_depth.jpg", depth)

        # writer.write(rgb)
		# writer_depth.write(depth)

        hsv_img = hsv_processing(rgb)
        hsv_img = transform_birds_eye(hsv_img)
        # bins = binner(hsv_img)
        # img8 = (bins).astype('uint8')
        # blurred = cv.GaussianBlur(img8, (11,11), 0)
        # ret, blurred = cv.threshold(blurred, 40, 255,cv.THRESH_BINARY)

        slope, grid_avg = optimizer.get_slope_single(hsv_img)
        steering_angle, speed = optimizer.get_steering(slope, grid_avg)
        # steering_angle = compare_LR.direction(bins, counter)
        if (counter == 0):
            old_steering = steering_angle

        # print("line 91")
        steering_angle = k_p * steering_angle + k_d * (steering_angle - old_steering)
        old_steering = steering_angle
        # print("line 94")
        if counter % 10 == 0:
            print("Steering: ", steering_angle)

        controller.steering(Car, steering_angle)

        # if steering_angle > 7:
            # speed = 2
        # else:
            # speed = 2
        Car.drive(2 - abs(steering_angle) / 20)
        # Car.drive(speed)
		# writer.write(rgb)
		# writer_depth.write(depth)
		# print("Sending steering angle")
		
except Exception as e:
	print(e.with_traceback())
    # print(e)

finally:
	print("Deleting Car and Camera")
	if Car is not None:
		Car.drive(0)
		del Car
	if rs is not None:
		del rs
	if writer is not None:
		writer.release()
		# writer_depth.release()

