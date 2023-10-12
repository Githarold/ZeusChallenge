import cv2
import time
# import matplotlib
# matplotlib.use('TkAgg')  # or 'Qt5Agg', 'WXAgg', etc.
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt

class Palletizing:
    
    def __init__(self):
        
        # Size of cell(grid)
        self.cell_size = 1
        self.z_cell_size = 1
        
        # Size of bucket
        self.h_max = 4
        self.real_x, self.real_y, self.real_z = 36, 25, 14
        
        # Initializing other values
        self.weight_map = np.zeros((self.real_y, self.real_x), dtype=int)
        self.vis_depth_map = np.zeros((self.real_y, self.real_x), dtype=int)
        self.after_depth_map = np.zeros((self.real_y, self.real_x), dtype=int)
        self.before_depth_map = np.zeros((self.real_y, self.real_x), dtype=int)
        self.min_idx = (0, 0)
        self.place_h = 0
        self.x_min = -0.213
        self.x_max = 0.184
        self.y_min = -0.148
        self.y_max = 0.145
        self.z_min = 0.0
        self.z_max = 0.0
        self.res_x = 0.0
        self.res_y = 0.0
        self.res_z = 0.0
        
        # Initializing figure
        self.fig, self.axes = plt.subplots(2, 1, figsize=(10, 5))
        # plt.close(self.fig)
        self.before_img = None
        self.after_img = None

        cv2.namedWindow('Depth Maps', cv2.WINDOW_NORMAL)
               
    def make_before_depth_map(self, pcd):
        
        points = np.asarray(pcd.points)

        z_max = 0.5
        z_max = np.min(points[:, 2]), np.max(points[:, 2])
        
        # Set origin
        points[:, 0] -= self.x_min
        points[:, 1] -= self.y_min
        points[:, 2] = z_max - points[:, 2]

        # Update min, max
        self.x_min, self.x_max = np.min(points[:, 0]), np.max(points[:, 0])
        self.y_min, self.y_max = np.min(points[:, 1]), np.max(points[:, 1])
        # self.z_min, self.z_max = np.min(points[:, 2]), np.max(points[:, 2])
        self.z_min, self.z_max = 0.0, 0.505 - 0.374
        
        self.res_x = self.real_x/(self.x_max - self.x_min)/self.cell_size
        self.res_y = self.real_y/(self.y_max - self.y_min)/self.cell_size
        self.res_z = self.real_z/(self.z_max - self.z_min)/self.z_cell_size
        
        width = int(np.ceil((self.x_max - self.x_min) * self.res_x))
        length = int(np.ceil((self.y_max - self.y_min) * self.res_y))
        
        # Make depth map
        self.before_depth_map = np.full((length, width), self.z_min)
        
        # Pointcloud to Matrix mapping
        for point in points:
            x, y, z = point
            i = int(y * self.res_y - 1)
            j = int(x * self.res_x - 1)
            self.before_depth_map[i, j] = max(self.before_depth_map[i, j], np.round(z*self.res_z))

        self.vis_depth_map = self.before_depth_map.copy()

    def make_after_depth_map(self, len1, len2, offset_y):
        start_time = time.time()
        
        self.len1 = len1
        self.len2 = len2
        
        depth_map = self.before_depth_map.copy()

        find_flag = False
        h, w = depth_map.shape
        min_height = float('inf')
        std_map = np.ones((h-len1, w-len2))
        
        # Find plane with size of object
        for i in range (h - len1):
            
            for j in range (w - len2):
                tmp_map = depth_map[i:i+len1, j:j+len2]
                std_map[i, j] = np.std(tmp_map)
                
                # Find plane
                if len(np.unique(tmp_map)) == 1:
                    find_flag = True
                    
                    if depth_map[i, j] <= min_height:
                        min_height = depth_map[i, j]
                        self.min_idx = (i, j)
                        self.place_h = depth_map[i, j]
        
        # Not found
        if not find_flag:
            min_std_idx = np.argmin(std_map)
            self.min_idx = np.unravel_index(min_std_idx, std_map.shape)          
            print('perfect plane is not found, alternative position find')        

        depth_map[self.min_idx[0]:self.min_idx[0]+len1, self.min_idx[1]:self.min_idx[1]+len2] += self.h_max
        self.after_depth_map = depth_map

        end_time = time.time()
        print(self.min_idx)
        print(f"Calculate time : {end_time - start_time} seconds")

        # x_min, y_min = -0.220, -0.142   # (m)

        # rel_x = x_min * 1000.0 + (j + len2 / 2.0) * 10.0
        # rel_y = y_min * 1000.0 + (i + len1 / 2.0 - offset_y) * 10.0

        rel_x = self.x_min * 1000.0 + (j + len2 / 2.0) * 10.0
        rel_y = self.y_min * 1000.0 + (i + len1 / 2.0 - offset_y) * 10.0

        return rel_x, rel_y

    def visualization_depth_map(self, flag):
            
        # Set the size of the plotting window
        plt.figure(figsize=(10, 5))

        # Visualizing Before Depth Map
        if hasattr(self, 'before_img') and self.before_img:  # Check if before_img exists
            self.before_img.remove()
        self.before_img = self.axes[0].imshow(self.vis_depth_map, cmap='jet')
        
        if flag == 1:
            self.axes[0].set_title("Before Place Depth Map")
        elif flag == 2:
            self.axes[0].set_title("Result Depth Map + Weight Masking")
        
        # Adding the colorbar for Before Depth Map
        if not hasattr(self, 'color_bar_before'):  # Add colorbar only on the first run
            self.color_bar_before = plt.colorbar(self.before_img, ax=self.axes[0], orientation='vertical')

        # Visualizing After Depth Map
        if hasattr(self, 'after_img') and self.after_img:  # Check if after_img exists
            self.after_img.remove()
        self.after_img = self.axes[1].imshow(self.after_depth_map, cmap='jet')
        self.axes[1].set_title("After Place Depth Map")

        # Adding the colorbar for After Depth Map
        if not hasattr(self, 'color_bar_after'):  # Add colorbar only on the first run
            self.color_bar_after = plt.colorbar(self.after_img, ax=self.axes[1], orientation='vertical')

        plt.draw()
        plt.pause(0.01)  # Gives a moment for the plots to update

        plt.show(block=False)

    def add_title_to_image(self, image, title):
        # Choose a font type and size
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_size = 0.5
        font_color = (0, 0, 255)  # Red color in BGR format
        thickness = 1

        # Define the position for the text
        position = (10, 20)  # 10 pixels from the left, 20 pixels from the top

        # Add the text to the image
        cv2.putText(image, title, position, font, font_size, font_color, thickness, cv2.LINE_AA)

        return image

    def add_colorbar_to_image(self, image):
        # Create a gradient of values ranging from 0 to 255 with the same height as the image
        gradient = np.linspace(0, 255, image.shape[0]).astype(np.uint8)
        gradient = np.repeat(gradient[:, np.newaxis], 20, axis=1)  # Width of the colorbar

        # Apply the colormap to the gradient
        colorbar = cv2.applyColorMap(gradient, cv2.COLORMAP_JET)

        # Stack the colorbar to the right of the image
        return np.hstack((image, colorbar))

    def visualization_opencv(self, flag):
        # Normalize depth maps to range [0, 255]
        before_normalized = cv2.normalize(self.vis_depth_map, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        after_normalized = cv2.normalize(self.after_depth_map, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

        # Apply colormap
        before_colored = cv2.applyColorMap(before_normalized, cv2.COLORMAP_JET)
        after_colored = cv2.applyColorMap(after_normalized, cv2.COLORMAP_JET)

        # Add titles
        if flag == 1:  
            before_colored = self.add_title_to_image(before_colored, "Before Depth Map")
            after_colored = self.add_title_to_image(after_colored, "After Depth Map")
        elif flag == 2:
            before_colored = self.add_title_to_image(before_colored, "Result Depth Map + Order Masking")
            after_colored = self.add_title_to_image(after_colored, "Estimate Depth Map")            

        # Add colorbars
        before_colored = self.add_colorbar_to_image(before_colored)
        after_colored = self.add_colorbar_to_image(after_colored)

        # Stack images vertically
        stacked_image = np.vstack((before_colored, after_colored))

        # Display the image
        cv2.imshow("Depth Maps", stacked_image)
        cv2.moveWindow("Depth Maps", 0, 0)        

        key = cv2.waitKey(0)
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()

    def make_weight_map(self, list_from_robot_cam, list_from_fixed_cam):

        # list_from_robot_cam = [[x, y, width, height, detected obj weight], ...]
        # list_from_fixed_cam = [grasped obj weight]
        
        # set zero point
        
        list_from_robot_cam[:, 0] -= -26
        list_from_robot_cam[:, 1] -= -16

        res_x, res_y = self.real_x / 52 / self.cell_size, self.real_y / 32 / self.cell_size
        
        for item in list_from_robot_cam:
            x, y, w, h, weight = item
            x_min, x_max = x, x + w
            y_min, y_max = y, y + h

            for y_coord in np.arange(y_min, y_max + 1, 0.1):
                for x_coord in np.arange(x_min, x_max + 1, 0.1):
                    i = int(y_coord * res_y - 1)
                    j = int(x_coord * res_x - 1)
                    i = min(int(y_coord * res_y), self.weight_map.shape[0] - 1)
                    j = min(int(x_coord * res_x), self.weight_map.shape[1] - 1)
                    self.weight_map[i, j] = weight
        
        self.weightmap2mask(list_from_fixed_cam)


    def weightmap2mask(self, list_from_fixed_cam):

        list_from_fixed_cam = np.array(list_from_fixed_cam).copy()
        grasp_obj_weight = list_from_fixed_cam[0]

        for i in range(self.real_y):
            for j in range(self.real_x):
                if self.weight_map[i][j] < grasp_obj_weight:
                    self.before_depth_map[i][j] *= 100
                else:
                    self.before_depth_map[i][j] *= 1