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
writer = None
try:
    rs = RealSense("/dev/video2", RS_VGA)  # RS_VGA, RS_720P, or RS_1080P
    backup = False
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
        if writer is None:
            writer = cv.VideoWriter('Video_ashton.avi', cv.VideoWriter_fourcc(*'MJPG'), 30,
                                    (rgb.shape[1], rgb.shape[0]), True)
        writer.write(rgb)

        # Get HSV image of rgb image
        hsv_img = hsv_processing(rgb)
        # get the min and max values of the bins of the hsv image, chop off the top of the hsv image
        bin_matrix = binner2(hsv_img[80:, :])
        matrix = turn_matrix_calc(bin_matrix)
        priority = column_matrix(bin_matrix)
        max_c = max_columns(priority)
        min_of_max_c = np.min(max_c)
        max_c_indices = np.where(max_c == min_of_max_c)[0]
        while len(max_c_indices) < 2:
            min_of_max_c += 1
            max_c_indices = np.where(max_c == min_of_max_c)[0]

        consec = get_consecutive_arrays(max_c_indices)
        sharp = check_sharp_corners(max_c, consec)
        print("\n\nturn columns:",max_c)
        turn = average_turn_value(get_optimal_column(max_c,sharp))
        print("turning:",turn)
        Car.steer(turn)
        Car.drive(2 - abs(turn) / 20)
except Exception as e:
    print("Something went wrong brother:", e.with_traceback())
finally:
    if writer is not None:
        writer.release()
    if rs is not None:
        del rs
    if Car is not None:
        Car.drive(0)
        del Car

