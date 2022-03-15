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

    # tell car to go the lowest speed and just stay at that speed
    Car.drive(1.3)
    while True:
        (time, rgb, depth, accel, gyro) = rs.getData()

        # Get HSV image of rgb image
        hsv_img = hsv_processing(rgb)
        # get the min and max values of the bins of the hsv image, chop off the top of the hsv image
        turn_values = get_min_max(turn_matrix_calc(binner2(hsv_img[130:, :])))
        # chose to go left over going right
        if turn_values[0] > abs([turn_values[1]]):
            Car.steer(turn_values[0]/20*30)
        else:
            Car.steer(turn_values[1]/20*30)
except Exception as e:
    print("Something went wrong brother:",e.with_traceback())
finally:
    if rs is not None:
        del rs
    if Car is not None:
        del Car
