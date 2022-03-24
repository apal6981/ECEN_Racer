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
from sebastian_steering import *
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
    rs = RealSense("/dev/video2", RS_VGA)		# RS_VGA, RS_720P, or RS_1080P
    writer = None

    # Use $ ls /dev/tty* to find the serial port connected to Arduino
    Car = Arduino("/dev/ttyUSB0", 115200)                # Linux
    #Car = Arduino("/dev/tty.usbserial-2140", 115200)    # Mac

    Car.zero(1500)      # Set car to go straight.  Change this for your car.
    Car.pid(1)          # Use PID control
    # You can use kd and kp commands to change KP and KD values.  Default values are good.
    # loop over frames from Realsense

    while True:
        (time, rgb, depth, accel, gyro) = rs.getData()
        # if writer is None:
        # # initialize our video writer
        #     writer = None #cv2.VideoWriter('ashton_derek_joint_cones.avi', cv2.VideoWriter_fourcc(*'MJPG'), 15, (rgb.shape[1], rgb.shape[0]), True)
        depth = cv.cvtColor(depth, cv.COLOR_BGR2GRAY)
        steering_angle = steering(rgb, depth)
        curve = curve_level(rgb, depth)
        speed = [2, 1, 0.8, 0.5]
        
        if steering_angle == BACK_UP:
            Car.steer(0)
            Car.drive(-1)
            t.sleep(2.5)
        else:
            Car.steer(steering_angle)
            Car.drive(speed[curve])

except Exception as e:
    print("Something went wrong brother:",e.with_traceback())
finally:
    if rs is not None:
        del rs
    if Car is not None:
        Car.drive(0)
        del Car
    if writer is not None:
        writer.release()

