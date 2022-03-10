import cv2 as cv
import numpy as np

print("this is a test")
camera_points_orig = np.array([[400,300],[1280-400,360],[0,600],[1280,600]])
camera_points = camera_points_orig.copy()
camera_points = camera_points.astype(np.float32)
# birds_eye_points = np.array([[0,0],[500,0],[0,500],[500,500]],dtype=np.float32)
IMAGE_H = 720
IMAGE_W =1280
OUT_H_FACTOR = 1
# src = np.float32([[0, IMAGE_H], [IMAGE_W, IMAGE_H], [848, 189], [927, 189]])
output_size = 200
dst = np.float32([[output_size, 0], [IMAGE_W - output_size, 0],[output_size, IMAGE_H * OUT_H_FACTOR], [IMAGE_W - output_size, IMAGE_H * OUT_H_FACTOR], ])

vid = cv.VideoCapture("Video200.avi")

while True:
    ret, frame = vid.read()
    if not ret:
        break
    print(frame.shape)
    gray = cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
    for point in camera_points_orig:
        cv.circle(frame,point,4,(0,255,0),cv.FILLED)
    M = cv.getPerspectiveTransform(camera_points, dst)
    warped_img = cv.warpPerspective(gray, M, (IMAGE_W, IMAGE_H))
    cv.imshow("RGB", frame)
    cv.imshow("warped",warped_img)
    cv.waitKey()