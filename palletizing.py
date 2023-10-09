import time
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt

class Palletizing:
    
    def __init__(self):
        
        # Size of cell(grid)
        self.cell_size = 1
        self.z_cell_size = 3
        
        # Size of bucket
        self.h_max = 16/3
        self.real_x, self.real_y, self.real_z = 36, 25, 14
        
        # Initializing other values
        self.before_depth_map = None
        self.after_depth_map = None
        self.weight_map = np.zeros((self.real_y, self.real_x), dtype=int)
        self.min_idx = (0, 0)
        self.place_h = 0
        self.x_min = 0.0
        self.x_max = 0.0
        self.y_min = 0.0
        self.y_max = 0.0
        self.z_min = 0.0
        self.z_max = 0.0
        self.res_x = 0.0
        self.res_y = 0.0
        self.res_z = 0.0
        
        # Initializing figure
        self.fig, self.axes = plt.subplots(2, 1, figsize=(10, 5))
        plt.close(self.fig)
        self.before_img = None
        self.after_img = None

        
        
    # Optimize function        
    def make_before_depth_map(self, pcd):
        points = np.asarray(pcd.points)

        x_min = np.min(points[:, 0])
        y_min = np.min(points[:, 1])
        z_max = np.max(points[:, 2])

        points[:, 0] -= x_min
        points[:, 1] -= y_min
        points[:, 2] = z_max - points[:, 2]

        # Update min, max
        self.x_min, self.x_max = np.min(points[:, 0]), np.max(points[:, 0])
        self.y_min, self.y_max = np.min(points[:, 1]), np.max(points[:, 1])
        self.z_min, self.z_max = np.min(points[:, 2]), np.max(points[:, 2])

        self.res_x = self.real_x / (self.x_max - self.x_min) / self.cell_size
        self.res_y = self.real_y / (self.y_max - self.y_min) / self.cell_size
        self.res_z = self.real_z / (self.z_max - self.z_min) / self.z_cell_size

        i_values = (points[:, 1] * self.res_y - 1).astype(int)
        j_values = (points[:, 0] * self.res_x - 1).astype(int)
        z_values = np.round(points[:, 2] * self.res_z)

        # Assuming the depth map has already been initialized to the proper size and filled with z_min
        for i, j, z in zip(i_values, j_values, z_values):
            self.before_depth_map[i, j] = max(self.before_depth_map[i, j], z)

        self.vis_depth_map = self.before_depth_map.copy()
        
    # def make_before_depth_map(self, pcd):
        
    #     points = np.asarray(pcd.points)

    #     x_min = np.min(points[:, 0])
    #     y_min = np.min(points[:, 1])
    #     z_min, z_max = np.min(points[:, 2]), np.max(points[:, 2])
        
    #     # Set origin
    #     points[:, 0] -= x_min
    #     points[:, 1] -= y_min
    #     points[:, 2] = z_max - points[:, 2]

    #     # Update min, max
    #     self.x_min, self.x_max = np.min(points[:, 0]), np.max(points[:, 0])
    #     self.y_min, self.y_max = np.min(points[:, 1]), np.max(points[:, 1])
    #     self.z_min, self.z_max = np.min(points[:, 2]), np.max(points[:, 2])
        
    #     self.res_x = self.real_x/(self.x_max - self.x_min)/self.cell_size
    #     self.res_y = self.real_y/(self.y_max - self.y_min)/self.cell_size
    #     self.res_z = self.real_z/(self.z_max - self.z_min)/self.z_cell_size
        
    #     width = int(np.ceil((self.x_max - self.x_min) * self.res_x))
    #     length = int(np.ceil((self.y_max - self.y_min) * self.res_y))
        
    #     # Make depth map
    #     self.before_depth_map = np.full((length, width), self.z_min)
        
    #     # Pointcloud to Matrix mapping
    #     for point in points:
    #         x, y, z = point
    #         i = int(y * self.res_y - 1)
    #         j = int(x * self.res_x - 1)
    #         self.before_depth_map[i, j] = max(self.before_depth_map[i, j], np.round(z*self.res_z))
                    
    def make_after_depth_map(self, len1, len2):
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
        print(f"Calculate time : {end_time - start_time} seconds")
    
    def matrix_to_absolute_coordinate(self):
        
        i = self.min_idx[0] + self.len1/2.0
        j = self.min_idx[1] + self.len2/2.0
        
        # Convert matrix 'i' index to actual 'y' coordinate
        y = (i + 1) / self.res_y + self.y_min
        
        # Convert matrix 'j' index to actual 'x' coordinate
        x = (j + 1) / self.res_x + self.x_min
        
        # Convert the depth_value to 'z' coordinate considering the resolution
        z = self.place_h / self.res_z
        
        # Restore the original z coordinate by subtracting from z_max
        z = self.z_max - z
        
        return x, y, z
    
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

            for y_coord in np.arrange(y_min, y_max + 1, 0.1):
                for x_coord in np.arrange(x_min, x_max + 1, 0.1):
                    i = int(y_coord * res_y - 1)
                    j = int(x_coord * res_x - 1)
                    i = min(int(y_coord * res_y), self.weight_map.shape[0] - 1)
                    j = min(int(x_coord * res_x), self.weight_map.shape[1] - 1)
                    self.weight_map[i, j] = weight
        
        self.weightmap2mask(list_from_fixed_cam)


    def weightmap2mask(self, list_from_fixed_cam):

        list_from_fixed_cam = np.array[list_from_fixed_cam].copy()
        grasp_obj_weight = list_from_fixed_cam[0,0]

        for i in self.real_y:
            for j in self.real_x:
                if self.weight_map[i][j] < grasp_obj_weight:
                    self.before_depth_map[i][j] *= 100
                else:
                    self.before_depth_map[i][j] *= 1


