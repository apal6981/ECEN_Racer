import cv2 as cv
import numpy as np

# rgb_sequence1 = cv.VideoCapture("depth_videos/rgb_video_seba.avi")
# depth_sequence1 = cv.VideoCapture("depth_videos/depth_video_seba.avi")

# rgb_sequence2 = cv.VideoCapture("depth_videos/rgb_video_seba_D.avi") # Recorded using Derek's code
# depth_sequence2 = cv.VideoCapture("depth_videos/depth_video_seba_D.avi") # Recorded using Derek's code

IMAGE_X_CENTER = 310
X_RANGES = [0, 150, 200, 250, IMAGE_X_CENTER, 388, 438, 488, 640]
STEERING_ANGLES = [-30, -20, -12, -5, 5, 12, 20, 30]
BACK_UP = "BACK_UP" 
SMALL_APERTURE = 30 # If the aperture is smaller than this it is considered too small
DISTANCES_TO_EVALUATE = [100, 170, 220, 280] # y distance of frame from the camera
DISTANCE_INDEX_STEERING = 0

# def main():
#     while True:
#         _, rgb_frame = rgb_sequence1.read()
#         _, depth_frame = depth_sequence1.read()
#         depth_frame = cv.cvtColor(depth_frame, cv.COLOR_BGR2GRAY)
#         steering_angle = steering(rgb_frame, depth_frame)
#         c_level = curve_level(rgb_frame, depth_frame)
        # print(f"steering_angle: {steering_angle}")
        # print(f"curve_level: {c_level}")

def curve_level(rgb_frame, depth_frame):
    bottom_center= find_where_to_go(rgb_frame, depth_frame)[0][0]
    middle_center = find_where_to_go(rgb_frame, depth_frame)[1][0]
    if ( (type(bottom_center) == str) or (type(middle_center) == str) ):
        return BACK_UP
    # print(f"bottom_center: {bottom_center}")
    # print(f"middle_center: {middle_center}")
    abs_diff = abs(bottom_center - middle_center)
    # print(f"abs_diff: {abs_diff}")
    if abs_diff > 120: # Hard curve coming
        return 3
    elif abs_diff > 80: # Slight curve coming
        return 2
    elif abs_diff > 30: # Slight curve coming
        return 1
    else:
        return 0

def steering(rgb_frame, depth_frame):
    circle_centers = find_where_to_go(rgb_frame, depth_frame)
    
    for circle in circle_centers:
        if ( (circle[0] == None) or (circle[1] == None) or (circle == BACK_UP)):
            continue 
        # print("circle: ", circle)
        # direction = cv.circle(rgb_frame, tuple(circle), 5, (0, 0, 255), -1)
    # cv.imshow("DIRECTION TO GO", direction)
    # cv.waitKey(0)

    where_to_go = circle_centers[DISTANCE_INDEX_STEERING] # Select which band to use as direction
    if where_to_go == BACK_UP:
        return BACK_UP
    where_to_go_x = where_to_go[0]
    print("where to go in X: ", where_to_go_x)
    steering_angle = 0
    if where_to_go_x != IMAGE_X_CENTER:
        selected_index = map_target_to_steering_angle(where_to_go_x)
        steering_angle = STEERING_ANGLES[selected_index]
    return steering_angle

def map_target_to_steering_angle(where_to_go):
    steering_angle_index = 0
    for index, range_limit in enumerate(X_RANGES):
        if where_to_go > range_limit:
            steering_angle_index = index
    return steering_angle_index
  
def find_where_to_go(rgb_frame, depth_frame):
    """ Determines the point on which the center of the camera should be facing
    """
    hsv_image = cv.cvtColor(rgb_frame, cv.COLOR_BGR2HSV)
    h, hsv_s, v = cv.split(hsv_image)
    ret, thresh1 = cv.threshold(hsv_s, 75, 255, cv.THRESH_BINARY)
    kernel = np.ones((5, 5), np.uint8)
    saturation = cv.morphologyEx(thresh1, cv.MORPH_OPEN, kernel)
    lines_b = np.bitwise_and(saturation, rgb_frame[:, :, 0])  # Blue
    lines_g = np.bitwise_and(saturation, rgb_frame[:, :, 1])  # Green
    lines_r = np.bitwise_and(saturation, rgb_frame[:, :, 2])  # Red
    lines = np.bitwise_or(lines_b, lines_g)
    lines = np.bitwise_or(lines, lines_r)
    # Find max value of green and blue in the frame and establish a threshold a bit below it to filter ouout walls and obbjects outside of the noodles
    max_frame_value = lines[:, :].max()
    # print(f"max_frame_value:{max_frame_value}")
    threshold_tolerance = 10
    _, lines = cv.threshold(lines, max_frame_value - threshold_tolerance, 255, cv.THRESH_BINARY)
    # cv.imshow("lines", lines)
    # cv.waitKey(0)

    # print(f"depth_frame.shape: {depth_frame.shape}")
    # print(f"lines.shape: {lines.shape}")
    # Mask the depth frame with the lines frame
    masked_depth = np.bitwise_and(depth_frame, lines)
    masked_depth = cv.dilate(masked_depth, kernel)
    # cv.imshow("masked_depth", masked_depth)

    circle_centers = list()
    for index, distance in enumerate(DISTANCES_TO_EVALUATE):
        circle_center = get_lane_center_at_distance(distance, masked_depth)
        # print(f"center of circle for decision bar at {distance}: {circle_center}")
        if ( index == (len(DISTANCES_TO_EVALUATE) - 1) ):
            circle_center = get_lane_center_at_distance(distance, masked_depth, 10, True)
        circle_centers.append(circle_center)
    print("circle_centers: ", circle_centers)
    return circle_centers

def get_lane_center_at_distance(px_distance_from_camera, masked_depth, decision_bar_height = 10, long_distance_depth_evaluation = False):
    center_y = masked_depth.shape[0] - px_distance_from_camera
    decision_bar = masked_depth[center_y:center_y + decision_bar_height, :]
    kernel = np.ones((5, 5), np.uint8)
    decision_bar = cv.dilate(decision_bar, kernel)
    decision_bar = cv.dilate(decision_bar, kernel)
    decision_bar = cv.dilate(decision_bar, kernel)
    # decision_bar = cv.dilate(decision_bar, kernel)
    # print(f"decision_bar array: {decision_bar}. shape: {decision_bar.shape}")
    # cv.imshow(f"decision_bar @ {center_y}", decision_bar)
    # cv.waitKey(0)
    
    apertures = detect_apertures(decision_bar)
    # print("apertures @", px_distance_from_camera, " px from camera: \n", apertures)
    largest_aperture_index = find_biggest_aperture(apertures)

    if (not apertures): # List is empty -> back up 
        print("NO APERTURES AVAILABLE")
        return BACK_UP

    largest_aperture = apertures[largest_aperture_index]

    aperture_length = largest_aperture[1] - largest_aperture[0]
    if (aperture_length < SMALL_APERTURE): # List is empty -> back up 
        print("APERTURE TOO SMALL: ", aperture_length)
        return BACK_UP

    center_of_aperture = int((largest_aperture[1] - largest_aperture[0])/2 + largest_aperture[0])
    if ( (largest_aperture[0] == 0) and (largest_aperture[1] < 639) ):
        center_of_aperture = int(largest_aperture[0] + (largest_aperture[1] - largest_aperture[0])/4 )
    if ( (largest_aperture[0] > 0) and (largest_aperture[1] >= 639) ):
        center_of_aperture = int(largest_aperture[1] - (largest_aperture[1] - largest_aperture[0])/4 )
    if long_distance_depth_evaluation:
        # This point is to show the spot with the longest distance according to the gray scale of depth on the given distance from the camera
        center_of_aperture = find_farthest_x(decision_bar)
    return [center_of_aperture, center_y]
    
def find_farthest_x(decision_field_img):
    row = decision_field_img[0] # Grab first row of the image
    x_coordinate = None
    max_depth = 255 # Init value is set to be the closest (whitest) it can be
    farthest_recognizable_depth = 10
    for pixel_index, pixel_value in enumerate(row):
        # print(f"pixel_value: {pixel_value}")
        if ( (pixel_value < max_depth) and (pixel_value > farthest_recognizable_depth) ):
            max_depth = pixel_value
            x_coordinate = pixel_index
    return x_coordinate

def detect_apertures(decision_field_img):
    # print(f"decision_field_img array: \n{decision_field_img}")
    # Iterate through the top row and record the ranges over which the pixels are black
    apertures = []
    row = decision_field_img[0] # Grab first row of the image
    aperture_start = None
    aperture_end = None
    for pixel_index, pixel_value in enumerate(row):
        # print(f"pixel_value: {pixel_value}")
        # We are in the middle of white area (not an aperture)
        if ( (pixel_value != 0) and (aperture_start == None) and (aperture_end == None) ):
            continue

         # We just hit the first black pixel in a new aperture
        if ( (pixel_value == 0) and (aperture_start == None) and (aperture_end == None) ):
            aperture_start = pixel_index
            continue

        # We are in the middle of an aperture
        elif ( (pixel_value == 0) and (aperture_start != None) ): 
            if (pixel_index == len(row)-1):
                # We hit the last index and there was no end to the aperture so we arbitrarily close the aperture at the last index
                aperture_end = pixel_index
                # Append range to apertures
                apertures.append([aperture_start, aperture_end])
            continue

        # We just hit the end of an aperture
        elif ( (pixel_value != 0) and (aperture_start != None) and (aperture_end == None) ): 
            aperture_end = pixel_index
            # Append range to apertures
            apertures.append([aperture_start, aperture_end])

            # Reset start and end
            aperture_start = None
            aperture_end = None
            continue

    # print(f"aperture_start: {aperture_start}. aperture_end: {aperture_end}")

    return apertures

def find_biggest_aperture(apertures):
    largest_index = 0
    largest_length = 0
    for i, aperture in enumerate(apertures):
        aperture_length = aperture[1] - aperture[0]
        if aperture_length > largest_length:
            largest_length = aperture_length
            largest_index = i
    return largest_index


# if __name__ == "__main__":
#     main()