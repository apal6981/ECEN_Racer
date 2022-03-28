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
from Realsense_derek import *
# from RealSense import *
import numpy as np
# import imutils
import cv2 as cv
import compare_LR	# Trying to stay between left and right lines
import controller	# For driving and steering
# from optimizer_clean import *
from optimizer_testing import *
from camera_processing import *

frames_per_steering = 2
steering_array = np.empty(frames_per_steering)
driving_size = 4
driving_array = np.ones(driving_size)
old_steering = 0
try:
    k_p = 0.8
    k_d = 0.1

    accel_array = []
    gyro_array = []

    record = True # Change to record video
    vid_name = "derek_cone_gap"
    video_name = str(vid_name + ".avi")
    np_name = str(vid_name + ".npy")
    f = open(np_name, 'wb')
    backup_flag = False
    num_frames_stopped = 0
    num_frames_backup = 0
    fps = 15

	# writer_depth = cv.VideoWriter('Video_derek_D.avi', cv.VideoWriter_fourcc(*'MJPG'), fps, (depth.shape[1], depth.shape[0]), True)

    print("Init Car")
	# Use $ ls /dev/tty* to find the serial port connected to Arduino
    Car = Arduino("/dev/ttyUSB0", 115200)                # Linux
	#Car = Arduino("/dev/tty.usbserial-2140", 115200)    # Mac

    my_opt = Optimizer()

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


    start_key = ''
    print("### Press spacebar then enter to start! ###")
    while start_key != ' ':
        start_key = input()
    
    print("Init Camera")
    rs = RealSense("/dev/video2", RS_VGA)		# RS_VGA, RS_720P, or RS_1080P

    while True:
        counter += 1
        (time, rgb, depth, accel, gyro) = rs.getData()
        

        if writer is None and record == True:
            writer = cv.VideoWriter(video_name, cv.VideoWriter_fourcc(*'MJPG'), fps, (rgb.shape[1], rgb.shape[0]), True)
			# writer_depth = cv.VideoWriter('Video_derek_D.avi', cv.VideoWriter_fourcc(*'MJPG'), fps, (depth.shape[1], depth.shape[0]), True)
			# cv.imwrite("first_depth.jpg", depth)

        if record == True:
            writer.write(rgb)
            # writer_depth.write(depth)
            np.save(f, accel)
            # np.save(f, gyro)
        
        hsv_img, obstacles = hsv_processing(rgb)
        hsv_img = transform_birds_eye(hsv_img)
        obstacles = transform_birds_eye(obstacles)
        # bins = binner(hsv_img)
        # img8 = (bins).astype('uint8')
        # blurred = cv.GaussianBlur(img8, (11,11), 0)
        # ret, blurred = cv.threshold(blurred, 40, 255,cv.THRESH_BINARY)

        if backup_flag == True:
            num_frames_backup += 1
            # How long to backup
            if num_frames_backup > 12:
                backup_flag = False
                num_frames_backup = 0
        else:
            # Enough time to get going after initialization
            if counter > 50:
                if np.all(accel < 0.08):
                    # How long to be stuck before moving
                    if num_frames_stopped > 5:
                        backup_flag = True
                        print("#### Accel Backup! ####")
                        num_frames_stopped = 0
                    else:
                        num_frames_stopped += 1
                else:
                    backup_flag = False
                    num_frames_stopped = 0

        slope, grid_avg = my_opt.get_slope_global(hsv_img, obstacles)
        steering_angle, speed = my_opt.get_steering(slope, grid_avg, counter, backup_flag)
        
        
        # steering_angle = compare_LR.direction(bins, counter)
        
        # if (counter == 1):
        #     old_steering = steering_angle

        # print("line 91")
        # steering_angle = k_p * steering_angle + k_d * (steering_angle - old_steering)
        # old_steering = steering_angle
        # print("line 94")
        # if counter % 5 == 0:
            # print("Accel:", accel)
            # print("Steering: ", steering_angle)

        controller.steering(Car, steering_angle)

        Car.drive(speed)
		# writer.write(rgb)
		# writer_depth.write(depth)
		
except Exception as e:
	print(e.with_traceback())
    # print(e)

finally:
    if record == True:
        f.close()
        # accel_array = np.array(accel_array)
        # gyro_array = np.array(gyro_array)
        
    print("Deleting Car and Camera")
    if Car is not None:
    	Car.drive(0)
    	del Car
    if rs is not None:
    	del rs
    if writer is not None:
    	writer.release()
		# writer_depth.release()

