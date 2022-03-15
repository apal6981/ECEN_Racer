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
from itertools import count
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
    backup = False
    encoder_prev = -10000000
    # Use $ ls /dev/tty* to find the serial port connected to Arduino
    Car = Arduino("/dev/ttyUSB0", 115200)                # Linux
    #Car = Arduino("/dev/tty.usbserial-2140", 115200)    # Mac

    Car.zero(1500)      # Set car to go straight.  Change this for your car.
    Car.pid(1)          # Use PID control
    # You can use kd and kp commands to change KP and KD values.  Default values are good.
    # loop over frames from Realsense

    # tell car to go the lowest speed and just stay at that speed
    Car.drive(1.5)
    while True:
        (time, rgb, depth, accel, gyro) = rs.getData()
        new_encoder = Car.encoder()

        # Get HSV image of rgb image
        hsv_img = hsv_processing(rgb)
        # get the min and max values of the bins of the hsv image, chop off the top of the hsv image
        turn_values = get_min_max(turn_matrix_calc(binner2(hsv_img[130:, :])))
        if backup:
            counter += 1
            if counter < 60:
                backup = False
            else:
                continue

        
        print(turn_values)
        if turn_values[0] == -30 and turn_values[1] == 30 or abs(new_encoder-encoder_prev) < 50:
            Car.steer(0.0)
            Car.drive(-1.5)
            backup = True
            counter = 0
            continue
        # chose to go left over going right
        if turn_values[1] > abs(turn_values[0]):
            Car.steer(turn_values[1])
            Car.drive(2-turn_values[1]/20)
        else:
            Car.steer(turn_values[0])
            Car.drive(2-abs(turn_values[0])/20)
        encoder_prev = new_encoder
except Exception as e:
    print("Something went wrong brother:",e.with_traceback())
finally:
    if rs is not None:
        del rs
    if Car is not None:
        Car.drive(0)
        del Car

