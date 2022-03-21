
from RealSense import *
import numpy as np
import cv2 as cv
from Arduino import Arduino


rs = None
Car = None
writer = None

try:
    rs = RealSense("/dev/video2", RS_VGA)		# RS_VGA, RS_720P, or RS_1080P
    (time, rgb, depth, accel, gyro) = rs.getData()
    depth_writer = cv.VideoWriter('depth_video_seba.avi', cv.VideoWriter_fourcc(*'MJPG'), 30, (rgb.shape[1], rgb.shape[0]), True)
    rgb_writer = cv.VideoWriter('rgb_video_seba.avi', cv.VideoWriter_fourcc(*'MJPG'), 30, (rgb.shape[1], rgb.shape[0]), True)
    while True:
        (time, rgb, depth, accel, gyro) = rs.getData()
        depth_writer.write(depth)
        rgb_writer.write(rgb)
        cv.waitKey(1)

except Exception as ex:
    print(f"mal")