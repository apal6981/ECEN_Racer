import cv2
import numpy as np

vidcap = cv2.VideoCapture('Monster Truck Sample Code/Video200.avi')
success,frame = vidcap.read()
count = 0

camera_points_orig = np.array([[200,200],[640-200, 230],[0, 400], [640, 400]])
camera_points = camera_points_orig.copy()
camera_points = camera_points.astype(np.float32)
# birds_eye_points = np.array([[0,0],[500,0],[0,500],[500,500]],dtype=np.float32)
IMAGE_H = 480
IMAGE_W = 640
OUT_H_FACTOR = 1
# src = np.float32([[0, IMAGE_H], [IMAGE_W, IMAGE_H], [848, 189], [927, 189]])
output_size = 200
dst = np.float32([[output_size, 0], [IMAGE_W - output_size, 0],[output_size, IMAGE_H * OUT_H_FACTOR], [IMAGE_W - output_size, IMAGE_H * OUT_H_FACTOR], ])

while success:
    success,frame = vidcap.read()
    width = 640
    height = 480
    dim = (width, height)

    frame = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA)

    # frame = frame[100:625,0:-1]
    hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # cv2.imshow("HSI", hsvImage)
    h, hsv_s, v = cv2.split(hsvImage)
    # hsv_split = np.concatenate((h,s,v),axis=1)
    ret,thresh1 = cv2.threshold(hsv_s,75,255,cv2.THRESH_BINARY)

    kernel = np.ones((5,5),np.uint8)
    saturation = cv2.morphologyEx(thresh1, cv2.MORPH_OPEN, kernel)

    lines_b = np.bitwise_and(saturation, frame[:,:,0]) # Blue
    lines_g = np.bitwise_and(saturation, frame[:,:,1]) # Green
    lines = np.bitwise_or(lines_b, lines_g)
    obstacles = np.bitwise_and(saturation, frame[:,:,2]) # Red

    ret,thresh_lines = cv2.threshold(lines,175,255,cv2.THRESH_BINARY)
    ret,thresh_obstacles = cv2.threshold(obstacles,175,255,cv2.THRESH_BINARY)

    # Chop off top of cones
    cv2.imshow("Obstacles", thresh_obstacles)

    combined = np.bitwise_or(thresh_lines, thresh_obstacles)

    concat = np.concatenate((thresh_lines, thresh_obstacles), axis=1)
    # cv2.imshow("Split HSV", concat)

    # gray = cv2.cvtColor(combined,cv2.COLOR_BGR2GRAY)
    for point in camera_points_orig:
        cv2.circle(combined, point,4,(0,255,0),cv2.FILLED)
        # cv2.circle(frame,point,4,(0,255,0),cv2.FILLED)
    M = cv2.getPerspectiveTransform(camera_points, dst)
    warped_img = cv2.warpPerspective(combined, M, (IMAGE_W, IMAGE_H))
    # warped_img = cv2.warpPerspective(frame, M, (IMAGE_W, IMAGE_H))
    
    # h, w = warped_img.shape
    # res = 50
    # h_res = int(h/res)
    # w_res = int(w/res)
    # grid = np.zeros((h_res, w_res))
    # grid_thresh = 100
    # for i in range (h_res-1):
    #     for j in range (w_res-1):
    #         val = np.average(warped_img[(i*h_res):((i+1)*h_res), (j*w_res):((j+1)*w_res)])
    #         if val > grid_thresh:
    #             grid[i,j] = 1

    # cv2.imshow("Grid", grid)

    # cv2.imshow("RGB", combined)
    cv2.imshow("warped",warped_img)
    cv2.imshow("frame",frame)

    cv2.waitKey(0)
    count += 1
