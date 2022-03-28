import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import camera_processing


class Optimizer:
    def __init__(self):
        self.counter = 0
        self.old_steering = 0
        self.backup_counter = 0
        self.further_is_good = False
        self.grid_avg_cones = 0
        self.running_avg_size = 4
        self.num_zeros_inFront = 0
        self.right_mean = 0
        self.left_mean = 0
        self.grid_running_avg = np.zeros(self.running_avg_size)
        
    def _upscan(self, f):
        for i, fi in enumerate(f):
            if fi == np.inf: continue
            for j in range(1,i+1):
                x = fi+j*j
                if f[i-j] < x: break
                f[i-j] = x

    def distance_transform(self, bitmap):
        f = np.where(bitmap, 0.0, np.inf)
        for i in range(f.shape[0]):
            self._upscan(f[i,:])
            self._upscan(f[i,::-1])
        for i in range(f.shape[1]):
            self._upscan(f[:,i])
            self._upscan(f[::-1,i])
        np.sqrt(f,f)
        return f

    def greedy_global(self, grid):
        camera_middle_offset = 0
        height, width = grid.shape
        y_step = int(height*0.06)
        num_steps = 14
        mid = int(width/2)
        
        paths = []
        
        # consider setting first val to the middle
        for n in range(2):
            if n == 0:
                max_lat_step = int(width * 0.04) # Local
            else:
                max_lat_step = int(width) # Global

            x0 = mid - int(width*0.1) #0.15
            path = [x0]
            for i in range(1, num_steps):
                left_bound = x0 - max_lat_step
                right_bound = x0 + round(max_lat_step*1.5)
                if left_bound < 0:
                    left_bound = 0
                if right_bound > (width-1):
                    right_bound = width - 1
                sub_array = grid[height-1-(i*y_step), left_bound:right_bound]
                
                b = sub_array[::-1]
                right_idx = len(b) - np.argmax(b) - 1
                left_idx = np.argmax(sub_array)
            
                if right_idx != left_idx:
                    max_idx = int(round((right_idx + left_idx)/2,0))
                else:
                    max_idx = right_idx
        
                if n == 0:
                    shifted_max_idx = x0-max_lat_step + max_idx
                    if shifted_max_idx < 0:
                        shifted_max_idx = 0 
                    x0 = shifted_max_idx
                else:
                    x0 = max_idx # shifted_max_idx
                path.append(x0)
            path = np.array(path)
            paths.append(path)

        paths = np.array(paths)
        y_vals = []
        
        for i in range(path.shape[0]):
            y_vals.append(height-y_step*(i+2))

        y_vals = np.array(y_vals)
        grid_vals_array = []
        
        local_path = paths[0]
        t = 8 # number of steps ahead we're looking
        grid_vals = grid[y_vals[0:t], local_path[0:t]]
        
        return paths, y_vals, grid_vals, y_step

    # Add white borders to left and ride sides
    def border(self, img):
        img[:,0] = 255      # Left
        img[:,-1] = 255     # Right
        # img[0,:] = 255    # Top
        # img[-1,:] = 255   # Bottom
        return img


    def get_slope_global(self, img, obstacles):
        hsv_img = img[:, 160:480]
        obstacles = obstacles[:, 160:480]
        
        blurred = cv.GaussianBlur(hsv_img, (11,11), 0)
        ret, blurred = cv.threshold(blurred, 40, 255,cv.THRESH_BINARY)
        blurred_cones = cv.GaussianBlur(obstacles, (11,11), 0)
        ret, blurred_cones = cv.threshold(blurred_cones, 40, 255,cv.THRESH_BINARY)


        bins = camera_processing.binner(blurred)
        bins = bins[:, 13:-1]
        bins = self.border(bins)

        bins_cones = camera_processing.binner(blurred_cones)
        bins_cones = bins_cones[:, 13:-1]
        bins_cones = self.border(bins_cones)
        # inverted = np.invert(blurred)
        # dmap = mh.distance(inverted)
        dmap = self.distance_transform(bins)
        # map_cones = self.distance_transform(bins)
            
        # path, y_step = optimizer.find_path(dmap)
        paths, y_vals, grid_vals, y_step = self.greedy_global(dmap)

        # paths_c, y_vals_c, grid_vals_c, y_step_c = self.greedy_global(map_cones)
        # self.grid_avg_cones = grid_vals_c.mean(axis=0)
        grid_avg = grid_vals.mean(axis=0)

        height, width = bins.shape
        front_of_vehicle= dmap[18:24, 15:50]
        self.num_zeros_inFront = front_of_vehicle.size - np.count_nonzero(front_of_vehicle)

        left_obstacles = dmap[15:29, 10:30]
        self.left_mean = left_obstacles.mean()

        right_obstacles = dmap[15:29, 30:50]
        self.right_mean = right_obstacles.mean()


        self.grid_avg_cones = grid_avg
        if grid_vals[4:-1].mean() > grid_vals[0:4].mean():
            self.further_is_good = True
        else:
            self.further_is_good = False
        b_idx = np.argmax(grid_avg)

        # print("grid avg:", self.grid_avg_cones)

        # percent_grid_diff = np.abs(best_path-worst_path)/((best_path+worst_path)/2)
        
        num_steps = np.shape(paths)[1]
        upperbound = int(num_steps * 0.5)

        # consider calculating the slope from the middle
        slopes = []
        # Constrained Greedy
        left_x = paths[0][upperbound]
        slopes.append((left_x-paths[0][1])/(y_vals[upperbound]-y_vals[1]))
        
        # Global

        def reject_outliers(data, m=2):
            return data[abs(data - np.mean(data)) < m * np.std(data)]

        # reject outliers:
        ending_idx = np.shape(paths[1])[0] - 1 
        outlier_idx = ending_idx
        for i in range(ending_idx, 0, -1):
            if abs(paths[1][i] - np.mean(paths[1][1:-1])) < 1*np.std(paths[1][1:-1]):
                outlier_idx = i

                break
        
        # if outlier_idx - 3 > 3:
            # outlier_idx = outlier_idx-3

        global_path = paths[1][0:outlier_idx]
        global_y_vals = y_vals[0:outlier_idx]

        ones = np.ones_like(global_path)
        psi = np.array([global_y_vals, ones]).T
        w = np.linalg.inv(psi.T@psi)@psi.T@global_path
        
        top_y_val = 15
        top_x_val = w[0]*top_y_val+w[1]
        global_slope = (top_x_val-paths[1][0])/(top_y_val-y_vals[0])    

        # Turn of comments to view plots and print statements
        '''
        colors = ['b', 'g', 'r', 'c', 'y', 'm', 'k', 'w']

        plt.imshow(dmap)
        
        plt.scatter(paths[0,:], y_vals, color=colors[0])         # Local points
        plt.scatter(global_path, global_y_vals, color=colors[1]) # Global points
        
        # Local dashed line
        point1 = [paths[1][0], y_vals[0]]
        point2 = [top_x_val, top_y_val]
        x_values = [point1[0], point2[0]]
        y_values = [point1[1], point2[1]]
        plt.plot(x_values, y_values, c='r', linestyle="--")
        
        # Line of best fit
        x_vals = np.arange(start=0, stop=30, step=1)
        y_vals1 = w[0]*x_vals+w[1]
        plt.plot(y_vals1, x_vals, c='r')
        
        cv.imshow("hsv", hsv_img)
        cv.imshow("Bins", bins)

        plt.xlim([0, 65])
        plt.ylim([30, 0])
        

        # plt.show()

        '''    
        
        return global_slope, grid_avg

    def get_steering(self, slope, grid_avg, counter, backup_flag):
        # Average is less than 5: turn 20
        # less than 3: turn 30
        # if slope is vertical: turn right based on grid_avg
        # if slope is
        self.grid_running_avg[counter % self.running_avg_size] = grid_avg
        running_avg = self.grid_running_avg.mean()
        
        max_steering = 25
        max_slope = 2.5
        max_grid = 20
        slope_step = max_steering/max_slope
        grid_step = max_steering/max_grid
        k_p = 0.6
        k_d = 0.3

        max_speed = 2
        steering = 0
        speed = 0
        direction = ''


        if slope > 0:
            direction = 'L'
        else:
            direction = 'R'


        steering = slope * slope_step * 1.5
        steering_1 = steering*-1
        # check if grid has garbage values (happens during initializing)
        if np.isposinf(grid_avg):
            grid_avg = 100

        # Tuning values, depending on distance (urgency), turns will be stronger or weaker
        # if grid_avg > 12 and (slope) > 1.8: # 20
        #     steering = steering / 2
        if grid_avg > 18:
            steering = steering / 8
        elif grid_avg > 16 and abs(steering) < 5:
            steering = steering / 2
        # elif grid_avg > 16:
            # steering = steering / 4
        elif grid_avg > 14:  # 13
            steering = steering / 4 #1
        elif grid_avg > 12: # 7
            steering = steering / 2.5
        elif grid_avg > 10: # 5
            steering = steering / 2
        elif grid_avg > 4:
            steering = steering / 1.5
        else:
            steering = steering
        
        # if grid_avg > 12:
        #     steering = steering / 4 # 3
        # elif grid_avg > 8:  # 13
        #     steering = steering / 2 #1
        # elif grid_avg > 4: # 7
        #     steering = steering / 0.75
        # elif grid_avg > 2: # 5
        #     steering = steering / 0.75
        # else:
        #     if steering < 0:
        #         steering = max_steering*-1
        #     else:
        #         steering = max_steering
        
        # steering = int(steering)

        if abs(steering) > max_steering:    
            if steering < 0:
                steering = max_steering*-1
            else:
                steering = max_steering
        steering *= -1

        # PD Implementation
        # if counter == 1:
        #     self.old_steering = steering
        # # steering = k_p * steering + k_d * (steering - self.old_steering)
        # self.old_steering = steering

        # Letting camera noise settle before backing up
        if counter > 50:
            # Handling backup case
            # if self.further_is_good == False:
            # print("Running avg:", running_avg)
            if (abs(steering) < 4 and running_avg < 2) or running_avg < 2:
                print("#### Object Backup #### ", running_avg)
                self.backup_counter = 1
                speed = -3
            elif self.right_mean < 1.0 and self.left_mean > 2 and self.left_mean > self.right_mean*3 and abs(steering<5):
                print("Hard left")
                steering = steering*1.1
            elif self.left_mean < 1.0 and self.right_mean > 2 and self.right_mean > self.left_mean*3 and abs(steering<5):
                print("Hard right")
                steering = steering*1.1
            elif self.left_mean < 1.5 and self.right_mean < 1.5:
                print("#### Object Backup LR bounds #### ")
                self.backup_counter = 1
                speed = -3

            if self.backup_counter != 0 or backup_flag == True:
                steering = (steering*-1)/8
                speed = -3
                self.backup_counter += 1
                if self.backup_counter >= 3:
                    self.backup_counter = 0
        
        if abs(steering) > max_steering:    
            if steering < 0:
                steering = max_steering*-1
            else:
                steering = max_steering

        # Handling speed
        if speed != -3:
            speed = 2.5 - abs(steering) / 20
            if abs(steering) < 5:
                speed = 4

            if speed < 0.8:
                speed = 0.8


        if counter % 5 == 0:
            # print("Speed:", speed)
            print("#", counter, "Slope:", round(slope,3), "OG_S:", round(steering_1,2), "Steering:", round(steering,2), "Grid avg:", round(grid_avg,2), "#0's:", self.num_zeros_inFront, "LM:",round(self.left_mean,2), "RM:",round(self.right_mean,2))
        # plt.pause(0.1)
        # plt.clf()
        # cv.waitKey(0)
        return steering, speed # backup flag


