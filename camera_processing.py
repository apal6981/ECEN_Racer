import cv2 as cv
import numpy as np

# Constants
IMAGE_H = 250
IMAGE_W = 640
OUT_H_FACTOR = 1
output_size = 200
dst = np.float32([[output_size, 0], [IMAGE_W - output_size, 0], [output_size, IMAGE_H * OUT_H_FACTOR],
                  [IMAGE_W - output_size, IMAGE_H * OUT_H_FACTOR], ])
# camera_points_orig = np.array([[150, 150], [640 - 150, 150], [50, 250], [640 - 50, 250]])
camera_points_orig = np.array([[20, 50], [640 - 20, 50], [80, 250], [640 - 80, 250]])

camera_points = camera_points_orig.copy()
camera_points = camera_points.astype(np.float32)
M = cv.getPerspectiveTransform(camera_points, dst)

width = 640
height = 250
dim = (width, height)
kernel = np.ones((5, 5), np.uint8)
threshold = 50
res = 5
shape = (int(IMAGE_H / res), int(IMAGE_W / res))

binner2_width_res = 20
binner2_height_res = 10


def draw_points(img):
    img = img[50:400, 0:-1]
    img = cv.resize(img, dim, interpolation=cv.INTER_AREA)
    for point in camera_points_orig:
        img = cv.circle(img, point, 4, (0, 255, 0), cv.FILLED)
    return img


# Process image to HSV and binarize it, returns combined lines and obstacles
def hsv_processing(img):
    img = img[50:400, 0:-1]
    img = cv.resize(img, dim, interpolation=cv.INTER_AREA)
    hsvImage = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    h, hsv_s, v = cv.split(hsvImage)
    ret, thresh1 = cv.threshold(hsv_s, 75, 255, cv.THRESH_BINARY)
    saturation = cv.morphologyEx(thresh1, cv.MORPH_OPEN, kernel)
    lines_b = np.bitwise_and(saturation, img[:, :, 0])  # Blue
    lines_g = np.bitwise_and(saturation, img[:, :, 1])  # Green
    lines = np.bitwise_or(lines_b, lines_g)
    obstacles = np.bitwise_and(saturation, img[:, :, 2])  # Red
    ret, thresh_lines = cv.threshold(lines, 175, 255, cv.THRESH_BINARY)
    ret, thresh_obstacles = cv.threshold(obstacles, 175, 255, cv.THRESH_BINARY)
    # thresh_obstacles = cone_chopper(thresh_obstacles)
    return np.bitwise_or(thresh_lines, thresh_obstacles), thresh_obstacles


# Chop off the top of the cone to clean up the image
def cone_chopper(img):
    for i in range((IMAGE_H - 25), 100, -5):
        for j in range(50, IMAGE_W - 50, 5):
            if img[i, j] == 255 and img[i - 20, j] == 255:
                img[0:i - 20, j:j + 75] = 0
                j += 75
    return img


# Top down view creater
def transform_birds_eye(img):
    M = cv.getPerspectiveTransform(camera_points, dst)
    return cv.warpPerspective(img, M, (IMAGE_W, IMAGE_H))


# Bin the image
def binner(img):
    res = 8
    shape = (int(IMAGE_H / res), int(IMAGE_W / res))
    warped_img = img[0:shape[0] * res, 0:shape[1] * res]
    sh = shape[0], warped_img.shape[0] // shape[0], shape[1], warped_img.shape[1] // shape[1]
    return warped_img.reshape(sh).mean(-1).mean(1)


# Bin the image using pixel counter
def binner2(img):
    bin_width = img.shape[1] // binner2_width_res
    bin_height = img.shape[0] // binner2_height_res
    output_array = np.zeros((binner2_height_res, binner2_width_res))
    for row in range(binner2_height_res):
        for column in range(binner2_width_res):
            output_array[row][column] = cv.countNonZero(
                img[row * bin_height:row * bin_height + bin_height, column * bin_width:column * bin_width + bin_width])
    return np.where(output_array > 170, 1, 0)


# create the turn matrix
def create_turn_matrix(w, h):
    max_value = 30
    matrix = np.zeros((h, w), dtype=int)
    mid = w // 2
    for i in range(mid):
        for j in range(h):
            matrix[j * -1 - 1][(i - mid + 1) * -1] = max_value - j * 3 - i if i < 2 else max_value - j * 3 - i * 3 + 1

    matrix = np.where(matrix < 0, 0, matrix)
    matrix[:, mid:] = np.fliplr(matrix[:, 0:mid] * -1)
    return matrix


# Another constant
turn_matrix = create_turn_matrix(binner2_width_res, binner2_height_res)


# Turn Matrix calculator (int)
def turn_matrix_calc(grid):
    return (grid * turn_matrix).astype(int)


# min_max values (Get the min and max values)
def get_min_max(matrix):
    return np.min(matrix), np.max(matrix)


def draw_bins(matrix, image):
    bin_width = image.shape[1] // binner2_width_res
    bin_height = image.shape[0] // binner2_height_res
    # draw the lines first
    for i in range(binner2_height_res):
        for j in range(binner2_width_res):
            cv.putText(image, str(matrix[i][j]), (j * bin_width + 2, i * bin_height + bin_height - 5),
                       cv.FONT_HERSHEY_SIMPLEX, .35, (0, 0, 255), 1)
    return image


######## horizontal bins processing code ##############

# create priority matrix
def create_priority_maxtrix():
    return np.repeat(np.arange(0, binner2_height_res), binner2_width_res, 0).reshape(
        (binner2_height_res, binner2_width_res))


priority_matrix = create_priority_maxtrix()


# multiply drive matrix and priority matrix
def column_matrix(matrix):
    return (matrix * priority_matrix).astype(int)


# return max values for each column
def max_columns(matrix):
    return np.max(matrix, axis=0)


turn_values = np.arange(-29, 31, 2)


# turn_values = np.array(
#     [-30, -25, -20, -17, -14, -11, -9, -7, -5, -3, -2.5, -2, -1.5, -1, -.5, .5, 1, 1.5, 2, 2.5, 3, 5, 7, 9, 11, 14, 17,
#      20, 25, 30])


def average_turn_value(indices):
    return np.average(turn_values[indices])


def get_consecutive_arrays(array):
    split_indices = np.where(np.diff(array) != 1)[0] + 1
    if len(split_indices) < 1:
        return np.array([array])
    # elif len(split_indices) == 1:
    #     # return array[:split_indices[0]] if split_indices[0] > len(array) // 2 else array[split_indices[0]:]
    #     return array
    else:
        split_indices = np.insert(split_indices, 0, 0)
        split_indices = np.append(split_indices, [len(array)])
        # max_diff = np.argmax(np.diff(split_indices))
        # return array[split_indices[max_diff]:split_indices[max_diff + 1]]
        return [array[split_indices[i]:split_indices[i + 1]] for i in range(len(split_indices) - 1)]


def check_sharp_corners(max_columns_array, array):
    if array[-1] != binner2_width_res - 1 and len(array) > 1 and max_columns_array[array[-1] + 1] > max_columns_array[
        array[-1]] * 2:
        array = np.delete(array, -1)
    if array[0] != 0 and len(array) > 1 and max_columns_array[array[0] - 1] > max_columns_array[array[0]] * 2:
        array = np.delete(array, 0)
    return array


def get_optimal_column(max_column_array, array):
    if max_column_array[array[0]] < 3:
        return 14
    return array


################# left right line detection #############
BIN_WIDTH = 20
BIN_HEIGHT = 16


# need slightly different hsv functions
def hsv_line_obs_processing(img):
    img = img[80:300, :]
    # img = cv.resize(img, dim, interpolation=cv.INTER_AREA)
    hsvImage = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    h, hsv_s, v = cv.split(hsvImage)
    ret, thresh1 = cv.threshold(hsv_s, 75, 255, cv.THRESH_BINARY)
    saturation = cv.morphologyEx(thresh1, cv.MORPH_OPEN, kernel)
    lines_b = np.bitwise_and(saturation, img[:, :, 0])  # Blue
    lines_g = np.bitwise_and(saturation, img[:, :, 1])  # Green
    lines = np.bitwise_or(lines_b, lines_g)
    obstacles = np.bitwise_and(saturation, img[:, :, 2])  # Red
    ret, thresh_lines = cv.threshold(lines, 175 , 255, cv.THRESH_BINARY)
    ret, thresh_obstacles = cv.threshold(obstacles, 225, 255, cv.THRESH_BINARY)
    # thresh_obstacles = cone_chopper(thresh_obstacles)
    return thresh_lines, thresh_obstacles


# Bin the image using pixel counter
def binner3(img, w, h, pixel_count=150):
    bin_width = img.shape[1] // w
    bin_height = img.shape[0] // h
    output_array = np.zeros((h, w))
    for row in range(h):
        for column in range(w):
            output_array[row][column] = cv.countNonZero(
                img[row * bin_height:row * bin_height + bin_height, column * bin_width:column * bin_width + bin_width])
    return np.where(output_array > pixel_count, 1, 0)


def draw_bins2(matrix, image):
    bin_width = image.shape[1] // BIN_WIDTH
    bin_height = image.shape[0] // BIN_HEIGHT
    # draw the lines first
    for i in range(BIN_HEIGHT):
        for j in range(BIN_WIDTH):
            cv.putText(image, str(matrix[i][j]), (j * bin_width + 2, i * bin_height + bin_height - 5),
                       cv.FONT_HERSHEY_SIMPLEX, .35, (0, 0, 255), 1)
    return image


# create the line turn matrix
def create_line_turn_matrix(w, h):
    max_value = 20
    matrix = np.zeros((h, w), dtype=int)
    mid = w // 2
    for i in range(mid):
        for j in range(h):
            matrix[j * -1 - 1][(i - mid + 1) * -1] = max_value - j * 2.75 - i if i < 1 else max_value - j * 1.25 - i * 1.0 + 1 # comment this one out and uncomment the one below for better cone avoidance, both lines
            # matrix[j * -1 - 1][
            #     (i - mid + 1) * -1] = max_value - j * 3 - i if i < 1 else max_value - j * 2 - i * 2 + 1 # less aggresive turning for which helps with cones
    matrix = np.where(matrix < 0, 0, matrix)
    matrix[:, mid:] = np.fliplr(matrix[:, 0:mid] * -1)
    matrix[BIN_HEIGHT-1][BIN_WIDTH//2-2] = 19
    matrix[BIN_HEIGHT-1][BIN_WIDTH//2+1] = -19
    return matrix


# needs to be created first
def create_left_turn_matrix(w, h):
    max_value = 19
    matrix = np.zeros((h, w), dtype=int)
    for i in range(w):
        for j in range(h):
            matrix[j * -1 - 1][i] = max_value - j * 1 if i < 2 else max_value - j * 1 - i * .75 + 1

    matrix[0:h // 3 + 1, 1:] -= 1
    matrix = np.where(matrix < 0, 0, matrix)
    return matrix * -1


def create_right_turn_matrix(left_matrix):
    return np.fliplr(left_matrix) * -1


LINE_MATRIX = create_line_turn_matrix(BIN_WIDTH, BIN_HEIGHT)
LEFT_OBS_MATRIX = create_left_turn_matrix(BIN_WIDTH, BIN_HEIGHT)
# print("line matrix:",LINE_MATRIX)
# print("left matrix:",LEFT_OBS_MATRIX)
RIGHT_OBS_MATRIX = create_right_turn_matrix(LEFT_OBS_MATRIX)


def mask_bins(matrix, style="line"):
    if style == "line":
        return matrix * LINE_MATRIX
    elif style == "left":
        return matrix * LEFT_OBS_MATRIX
    elif style == "right":
        return matrix * RIGHT_OBS_MATRIX
    else:
        return None


def bin_value(matrix, mode="line"):
    if mode == "line":
        return np.min(matrix), np.max(matrix)
    elif mode == "left":
        return np.min(matrix)
    elif mode == "right":
        return np.max(matrix)
    else:
        print("ERRRRRRROOOOORRRRRR ")
        return None


def find_distances(line_matrix, cone_matrix, cones):
    # if cones == [0]:
    #     return None
    # print("cones len",len(cones))
    if len(cones[0]) == 0:
        return None
    # line_coordinates = tuple(zip(np.where(line_matrix ==1)))
    cone_decision = []
    for cone in cones:

        # print("cone",cone )
        # Looking left
        # get first closes row where cone is
        cone_bottom_row = np.where(cone_matrix[:, cone[0]])[0]
        # print("cone bottom row",cone_bottom_row)
        left_values = []
        if len(cone_bottom_row) == 0:
            # print("no cones")
            continue
        elif len(cone_bottom_row) > 4:
            cone_bottom_row = cone_bottom_row[-3:-1]
        for left in cone_bottom_row:
            line_row_indices = np.where(line_matrix[left, :])[0]
            less_indices = np.where(line_row_indices < cone[0])[0]
            # print("line row indices:",line_row_indices,"less indices:",less_indices)
            left_values.append(
                abs(cone[0] - line_row_indices[less_indices[-1]]) if len(less_indices) > 0 else abs(cone[0] - 0))
        left_distance = min(left_values)
        # Looking right

        right_values = []
        cone_bottom_row = np.where(cone_matrix[:, cone[-1]])[0]
        # print("right cone bottom row:",cone_bottom_row)
        if len(cone_bottom_row) > 4:
            cone_bottom_row = cone_bottom_row[-3:-1]
        for right in cone_bottom_row:
            line_row_indices = np.where(line_matrix[right, :])[0]
            greater_indices = np.where(line_row_indices > cone[0])[0]
            # print("right line row indices:", line_row_indices, "greater indices:", greater_indices)
            right_values.append(
                abs(cone[-1] - line_row_indices[greater_indices[0]]) if len(greater_indices) > 0 else abs(
                    cone[-1] - BIN_WIDTH))
        right_distance = min(right_values)
        cone_decision.append([left_distance, right_distance])
        # print("left_distance:",left_distance, "right_distances:",right_distance)

    return cone_decision


def turn_decision(line_bin, obs_bin):
    line_turn_matrix = mask_bins(line_bin, "line")
    left_turn_matrix = mask_bins(obs_bin, "left")
    right_turn_matrix = mask_bins(obs_bin, "right")

    column_indices = np.where(max_columns(obs_bin) == 1)[0]
    # print("column indices", column_indices)
    cones = get_consecutive_arrays(column_indices)
    # print("cones:", cones)
    cone_decisions = find_distances(line_bin, obs_bin, cones)
    if cone_decisions is None:
        line_turn = bin_value(line_turn_matrix, "line")
        together_check = bin_value(mask_bins(obs_bin,"line"),"line")
        if (int(line_turn[0]) == -20 and int(line_turn[1]) == 20) or (int(together_check[0]) == -20 and int(together_check[1]) == 20):
            # print("about to crash:")
            return 100
        if abs(line_turn[0]) == line_turn[1]:
            # print("driving panic")
            return line_turn[1]
        return line_turn[0] if abs(line_turn[0]) > line_turn[1] else line_turn[1]
    else:
        # print("Cone Distances:", cone_decisions)
        # print("line decision:", bin_value(line_turn_matrix, "line"))
        canidates = []
        for index, individual in enumerate(cone_decisions):
            if individual[0] == individual[1] and individual[0] > 1 and individual[1] > 1:
                canidates.append([index,3,individual[0]])
            if individual[0] > 1:
                canidates.append([index, 0, individual[0]])
            if individual[1] > 1:
                canidates.append([index, 1, individual[1]])
        # print("Canidates:",canidates)
        if len(canidates) == 0:  # Just do normal noodle following stuff
            line_turn = bin_value(line_turn_matrix, "line")
            together_check = bin_value(mask_bins(obs_bin,"line"),"line")
            if (int(line_turn[0]) == -20 and int(line_turn[1]) == 20) or (int(together_check[0]) == -20 and int(together_check[1]) == 20):
                # print("about to crash:")
                return 100
            if abs(line_turn[0]) == line_turn[1]:
                # print("driving panic")
                return line_turn[1]
            # print("driving normal")
            return line_turn[0] if abs(line_turn[0]) > line_turn[1] else line_turn[1]
        else:

            # first find largest gap
            line_turn = bin_value(line_turn_matrix, "line")
            together_check = bin_value(mask_bins(obs_bin,"line"),"line")
            # Check to see if we are about to run into something
            if (int(line_turn[0]) == -20 and int(line_turn[1]) == 20) or (int(together_check[0]) == -20 and int(together_check[1]) == 20):
                # print("about to crash:")
                return 100
            
            greatest_distance = np.argmax(np.array(canidates)[:, 2])
            best_canidate = canidates[greatest_distance]
            greatest_distance = best_canidate[0]
            # print("cone decision:",cone_decisions, canidates, best_canidate)
            if best_canidate[1] == 0:  # go left
                if greatest_distance == 0:  # left cone
                    left_cone_turn = bin_value(left_turn_matrix[:, cones[canidates[greatest_distance][0]]], "left")
                    line_turn_max = line_turn[0] if abs(line_turn[0]) > line_turn[1] else line_turn[1]
                    if left_cone_turn == 0:
                        # print("left follow the line:", left_cone_turn,line_turn_max)
                        return line_turn_max
                    if line_turn_max < 0:  # same direction
                        # print("left same sign:", left_cone_turn, line_turn_max)
                        return np.min([left_cone_turn, line_turn_max])
                    else:
                        # print("left diff sign:", left_cone_turn,line_turn_max)
                        return left_cone_turn  # differing directions so cone gets preference
                else:
                    left_cone_turn = bin_value(
                        left_turn_matrix[:, cones[greatest_distance]], "left")  # this is a negative number
                    right_cone_turn = bin_value(right_turn_matrix[:, cones[greatest_distance-1]], "right")
                    cone_choice = [left_cone_turn, right_cone_turn][np.argmax([abs(left_cone_turn), right_cone_turn])]
                    line_turn_max = line_turn[0] if abs(line_turn[0]) > line_turn[1] else line_turn[1]
                    if cone_choice == 0:
                        # print("multiple cones left same sign:", cone_choice, line_turn_max)
                        return line_turn_max
                    if np.sign(cone_choice) == np.sign(line_turn_max):
                        # print("multiple cones left same sign:", cone_choice, line_turn_max)
                        return line_turn_max if abs(line_turn_max) > abs(cone_choice) else cone_choice
                    else:
                        # print("mutiple cones diff sign:", cone_choice)
                        return cone_choice
            elif best_canidate[1] == 1:  # go right
                if greatest_distance - len(cone_decisions) == -1 or len(cone_decisions) == 1:  # farthest right cone
                    right_cone_turn = bin_value(right_turn_matrix[:, cones[greatest_distance]],
                                                "right")  # this is a positive number
                    line_turn_max = line_turn[0] if abs(line_turn[0]) > line_turn[1] else line_turn[1]
                    if right_cone_turn == 0:
                        # print("right we are going to follow line:",right_cone_turn,line_turn_max)
                        return line_turn_max
                    if line_turn_max > 0:  # same direction
                        # print("right single same sign:", right_cone_turn, line_turn_max)
                        return np.max([right_cone_turn, line_turn_max])
                    else:
                        # print("right single diff sign:", right_cone_turn, line_turn_max)
                        return right_cone_turn  # differing directions so cone gets preference
                else:
                    left_cone_turn = bin_value(
                        left_turn_matrix[:, cones[greatest_distance+1]], "left")  # this is a negative number
                    right_cone_turn = bin_value(left_turn_matrix[:, cones[greatest_distance]], "right")
                    cone_choice = [left_cone_turn, right_cone_turn][np.argmin([abs(left_cone_turn), right_cone_turn])]
                    line_turn_max = line_turn[0] if abs(line_turn[0]) > line_turn[1] else line_turn[1]
                    if cone_choice == 0:
                        # print("right mulitple same sign:", cone_choice, line_turn_max)
                        return line_turn_max
                    if np.sign(cone_choice) == np.sign(line_turn_max):
                        # print("right mulitple same sign:", cone_choice, line_turn_max)
                        return line_turn_max if abs(line_turn_max) > abs(cone_choice) else cone_choice
                    else:
                        # print("right mulitiple diff sign:", cone_choice, line_turn_max)
                        return cone_choice
            else: # turn directions are the same so choose the smallest turn angle
                # if len(cones) == 1: # only one cone
                right_cone_turn = bin_value(right_turn_matrix[:, cones[greatest_distance]],
                                            "right")  # this is a positive number
                left_cone_turn = bin_value(left_turn_matrix[:, cones[greatest_distance]], "left")
                cone_choice = [left_cone_turn, right_cone_turn][np.argmin([abs(left_cone_turn),right_cone_turn])]
                # print("cone decision:",cone_decisions, canidates, best_canidate)
                if cone_choice < 0: # going left, choose max of cone or line left turning
                    # print("same turn going left")
                    if abs(cone_choice) < line_turn[1]: # i actually need to go right
                        # print("jk, going right")
                        return line_turn[1]
                    return np.min([cone_choice,line_turn[0]])
                else:
                    # print("same_turn, going right")
                    if cone_choice < abs(line_turn[0]):
                        # print("jk, going left")
                        return line_turn[0]
                    return np.max([cone_choice,line_turn[1]])
                # if greatest_distance == 0: # far left cone


        # print(canidates)


def left_right_line_decision(left, right, line):
    obs_value = bin_value(left, "left"), bin_value(right, "right")
    # right_value = bin_value(right, "right")
    line_values = bin_value(line, "line")
    # print("obs values:", obs_value, "line values:", line_values)

    obs_decision_index = np.argmin([abs(obs_value[0]), obs_value[1]])
    line_decision_index = np.argmin([abs(line_values[0]), line_values[1]])
    if obs_value == (0, 0):
        # print("no objects")
        return line_values[line_decision_index - 1]
    if obs_decision_index < line_decision_index:
        return obs_value[obs_decision_index] if np.max([abs(line_values[0]), line_values[1]]) < 10 else line_values[
            line_decision_index - 1]
    elif line_decision_index < obs_decision_index:
        return obs_value[obs_decision_index] if np.max([abs(line_values[0]), line_values[1]]) < 10 else line_values[
            line_decision_index - 1]
    else:
        return obs_value[obs_decision_index] if abs(obs_value[obs_decision_index]) > abs(
            line_values[line_decision_index]) else line_values[line_decision_index]


def create_sin_turn_values():
    return np.array([30 * np.sin(i / (2 * np.pi)) for i in range(-10, 11)])


def create_larger_sin_turn_values():
    return np.array([30 * np.sin(i / (4 * np.pi)) for i in range(-20, 21)])


def create_parabolic_turn_values():
    return np.array([-1 / 13.35 * i ** 2 if i < 0 else 1 / 13.35 * i ** 2 for i in range(-20, 21)])


SIN_TURN_VALUES = create_sin_turn_values()
LARGER_SIN_TURN_VALUES = create_larger_sin_turn_values()
PARABOLIC_TURN_VALUES = create_parabolic_turn_values()
# print(LARGER_SIN_TURN_VALUES)
