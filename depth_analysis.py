import numpy as np
import cv2 as cv


depth_video = cv.VideoCapture("depth_video_seba.avi", )
# rgb_video = cv.VideoCapture("rgb_video_seba.avi")

while True:
    _, depth_frame = depth_video.read()
    # _, rgb_frame = rgb_video.read()
    print(f"depth_frame.shape: {depth_frame.shape}")
    cv.imshow("frame", depth_frame)
    cv.waitKey(0)

    