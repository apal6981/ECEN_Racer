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
from camera_processing import *
from Arduino import Arduino
from RealSense import *
import numpy as np
from optimizer import *
import time as t
# import imutils
import cv2

rs = None
Car = None
try:
    rs = RealSense("/dev/video2", RS_VGA)  # RS_VGA, RS_720P, or RS_1080P
    writer = None

    # Use $ ls /dev/tty* to find the serial port connected to Arduino
    Car = Arduino("/dev/ttyUSB0", 115200)  # Linux
    # Car = Arduino("/dev/tty.usbserial-2140", 115200)    # Mac

    Car.zero(1500)  # Set car to go straight.  Change this for your car.
    Car.pid(1)  # Use PID control
    # You can use kd and kp commands to change KP and KD values.  Default values are good.
    # loop over frames from Realsense

    # tell car to go the lowest speed and just stay at that speed
    Car.drive(1.5)
    while True:
        (time, rgb, depth, accel, gyro) = rs.getData()

        line_thresh, obs_thresh = hsv_line_obs_processing(rgb[80:, :])
        line_bin = binner3(line_thresh, BIN_WIDTH, BIN_HEIGHT, pixel_count=50)
        obs_bin = binner3(obs_thresh, BIN_WIDTH, BIN_HEIGHT, pixel_count=100)
        # print("\n\nline\n", line_matrix)
        # print("left\n", left_matrix)
        # print("right\n", right_matrix)

        # slope = get_slope(transform_birds_eye(cv.bitwise_or(line_thresh,obs_thresh)))

        decision = turn_decision(line_bin, obs_bin)
        if decision > 20:
            print("crash!!!!!!!!!!!!!")
            Car.drive(-3)
            Car.steer(0)
            t.sleep(1)
            continue
        print("Decision:", decision, "speed:",1/700*(abs(LARGER_SIN_TURN_VALUES[decision-21])-31)**2+2.75)
        Car.steer(PARABOLIC_TURN_VALUES[decision-21])
        Car.drive(1/700*(abs(LARGER_SIN_TURN_VALUES[decision-21])-31)**2+2.75)
        # Car.drive(4 - abs(LARGER_SIN_TURN_VALUES[decision-21]) / 10)
        # if turn_values[1] > abs(turn_values[0]):
        #     Car.steer(turn_values[1])
        #     Car.drive(2 - turn_values[1] / 20)
        # else:
        #     Car.steer(turn_values[0])
        #     Car.drive(2 - abs(turn_values[0]) / 20)
except Exception as e:
    print("Something went wrong brother:", e.with_traceback())
finally:
    if rs is not None:
        del rs
    if Car is not None:
        del Car

