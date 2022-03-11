import cv2 as cv
import numpy as np

# Constants
IMAGE_H = 250
IMAGE_W = 640
OUT_H_FACTOR = 1
output_size = 200
dst = np.float32([[output_size, 0], [IMAGE_W - output_size, 0],[output_size, IMAGE_H * OUT_H_FACTOR], [IMAGE_W - output_size, IMAGE_H * OUT_H_FACTOR], ])
camera_points_orig = np.array([[150,150],[640-150, 150],[50, 250], [640-50, 250]])
camera_points = camera_points_orig.copy()
camera_points = camera_points.astype(np.float32)
M = cv.getPerspectiveTransform(camera_points, dst)

width = 640
height = 250
dim = (width, height)
kernel = np.ones((5,5),np.uint8)
threshold = 50
res = 5
shape = (int(IMAGE_H/res), int(IMAGE_W/res))

binner2_width_res = 20
binner2_height_res = 10


# Process image to HSV and binarize it, returns combined lines and obstacles
def hsv_processing(img):
    hsvImage = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    h, hsv_s, v = cv.split(hsvImage)
    ret,thresh1 = cv.threshold(hsv_s,75,255,cv.THRESH_BINARY)
    saturation = cv.morphologyEx(thresh1, cv.MORPH_OPEN, kernel)
    lines_b = np.bitwise_and(saturation, img[:,:,0]) # Blue
    lines_g = np.bitwise_and(saturation, img[:,:,1]) # Green
    lines = np.bitwise_or(lines_b, lines_g)
    obstacles = np.bitwise_and(saturation, img[:,:,2]) # Red
    ret,thresh_lines = cv.threshold(lines,175,255,cv.THRESH_BINARY)
    ret,thresh_obstacles = cv.threshold(obstacles,175,255,cv.THRESH_BINARY)
    thresh_obstacles = cone_chopper(thresh_obstacles)
    return np.bitwise_or(thresh_lines, thresh_obstacles)

# Chop off the top of the cone to clean up the image
def cone_chopper(img):
    for i in range((IMAGE_H-25), 100, -5):
        for j in range(50, IMAGE_W-50, 5):
            if img[i,j] == 255 and img[i-20, j] == 255:
                img[0:i-20, j:j+75] = 0
                j += 75
    return img

# Top down view creater
def transform_birds_eye(img):
    return cv.warpPerspective(img, M, (IMAGE_W, IMAGE_H))

# Bin the image
def binner(img):
    warped_img = warped_img[0:shape[0]*res, 0:shape[1]*res]
    sh = shape[0],warped_img.shape[0]//shape[0],shape[1],warped_img.shape[1]//shape[1]
    return warped_img.reshape(sh).mean(-1).mean(1)

# Bin the image using pixel counter
def binner2(img):
    bin_width = img.shape[1]//binner2_width_res
    bin_height = img.shape[0]//binner2_height_res
    output_array = np.zeros((binner2_height_res,binner2_width_res))
    for row in range(binner2_height_res):
        for column in range(binner2_width_res):
            output_array[row][column] = cv.countNonZero(
                img[row*bin_height:row*bin_height+bin_height,column*bin_height:column*bin_width+bin_width])
    return np.where(output_array > 1000, 1, 0)



