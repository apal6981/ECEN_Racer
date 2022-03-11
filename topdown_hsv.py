import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt

vidcap = cv.VideoCapture('Video200.avi')
success,frame = vidcap.read()
count = 0

camera_points_orig = np.array([[150,150],[640-150, 150],[50, 250], [640-50, 250]])
camera_points = camera_points_orig.copy()
camera_points = camera_points.astype(np.float32)
# birds_eye_points = np.array([[0,0],[500,0],[0,500],[500,500]],dtype=np.float32)
IMAGE_H = 250
IMAGE_W = 640
OUT_H_FACTOR = 1
# src = np.float32([[0, IMAGE_H], [IMAGE_W, IMAGE_H], [848, 189], [927, 189]])
output_size = 200
dst = np.float32([[output_size, 0], [IMAGE_W - output_size, 0],[output_size, IMAGE_H * OUT_H_FACTOR], [IMAGE_W - output_size, IMAGE_H * OUT_H_FACTOR], ])
frame_number = 0
while success:
    frame_number += 1
    success,frame = vidcap.read()
    # if frame_number != 172:
    # if frame_number != 100:
    #     continue
    frame = frame[50:400,0:-1]

    width = 640
    height = 250
    dim = (width, height)

    frame = cv.resize(frame, dim, interpolation = cv.INTER_AREA)

    hsvImage = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    # cv.imshow("HSI", hsvImage)
    h, hsv_s, v = cv.split(hsvImage)
    # hsv_split = np.concatenate((h,s,v),axis=1)
    ret,thresh1 = cv.threshold(hsv_s,75,255,cv.THRESH_BINARY)

    kernel = np.ones((5,5),np.uint8)
    saturation = cv.morphologyEx(thresh1, cv.MORPH_OPEN, kernel)

    lines_b = np.bitwise_and(saturation, frame[:,:,0]) # Blue
    lines_g = np.bitwise_and(saturation, frame[:,:,1]) # Green
    lines = np.bitwise_or(lines_b, lines_g)
    obstacles = np.bitwise_and(saturation, frame[:,:,2]) # Red

    ret,thresh_lines = cv.threshold(lines,175,255,cv.THRESH_BINARY)
    ret,thresh_obstacles = cv.threshold(obstacles,175,255,cv.THRESH_BINARY)

    # Chop off top of cones
    
    for i in range((IMAGE_H-25), 100, -5):
        for j in range(50, IMAGE_W-50, 5):
            if thresh_obstacles[i,j] == 255 and thresh_obstacles[i-20, j] == 255:
                thresh_obstacles[0:i-20, j:j+75] = 0
                j += 75
    # cv.imshow("Obstacles", thresh_obstacles)
            
    combined = np.bitwise_or(thresh_lines, thresh_obstacles)

    concat = np.concatenate((thresh_lines, thresh_obstacles), axis=1)
    # cv.imshow("Split HSV", concat)

    # gray = cv.cvtColor(combined,cv.COLOR_BGR2GRAY)
    for point in camera_points_orig:
        cv.circle(combined, point,4,(0,255,0),cv.FILLED)
        cv.circle(frame,point,4,(0,255,0),cv.FILLED)
    M = cv.getPerspectiveTransform(camera_points, dst)
    warped_img = cv.warpPerspective(combined, M, (IMAGE_W, IMAGE_H))
    # warped_img = cv.warpPerspective(frame, M, (IMAGE_W, IMAGE_H))
    
    threshold = 50
    res = 5
    shape = (int(IMAGE_H/res), int(IMAGE_W/res))
    warped_img = warped_img[0:shape[0]*res, 0:shape[1]*res]   
    sh = shape[0],warped_img.shape[0]//shape[0],shape[1],warped_img.shape[1]//shape[1]
    grid = warped_img.reshape(sh).mean(-1).mean(1)

    # grid = grid[grid >= threshold] = 255
    # grid = grid[grid <= threshold] = 0
    # h, w = warped_img.shape
    # res = 5
    # h_res = int(h/res)
    # w_res = int(w/res)
    # grid = np.zeros((h_res, w_res))
    # grid_thresh = 50
    # for i in range (h_res-1):
    #     for j in range (w_res-1):
    #         val = np.average(warped_img[(i*h_res):((i+1)*h_res), (j*w_res):((j+1)*w_res)])
    #         if val > grid_thresh:
    #             grid[i,j] = 1

    # plt.imshow(grid, origin='upper', cmap='binary')
    cv.imshow("grid", grid)
    cv.imshow("RGB", combined)
    cv.imshow("warped",warped_img)

    cv.imshow("frame",frame)
    # plt.show()
    cv.waitKey(0)
    count += 1
