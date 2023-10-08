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
        self.real_x, self.real_y, self.real_z = 41, 29, 16
        
        # Initializing depth maps
        self.before_depth_map = None
        self.after_depth_map = None
        
    def before_depth_map(self, pcd):
        
        points = np.asarray(pcd.points)

        x_min, x_max = np.min(points[:, 0]), np.max(points[:, 0])
        y_min, y_max = np.min(points[:, 1]), np.max(points[:, 1])
        z_min, z_max = np.min(points[:, 2]), np.max(points[:, 2])
        
        # Set origin        
        points[:, 0] -= x_min
        points[:, 1] -= y_min
        points[:, 2] = z_max - points[:, 2]

        # Update min, max
        x_min, x_max = np.min(points[:, 0]), np.max(points[:, 0])
        y_min, y_max = np.min(points[:, 1]), np.max(points[:, 1])
        z_min, z_max = np.min(points[:, 2]), np.max(points[:, 2])
        
        res_x = self.real_x/(x_max - x_min)/self.cell_size
        res_y = self.real_y/(y_max - y_min)/self.cell_size
        res_z = self.real_z/(z_max - z_min)/self.z_cell_size
        
        width = int(np.ceil((x_max - x_min) * res_x))
        length = int(np.ceil((y_max - y_min) * res_y))
        
        # Make depth map
        self.before_depth_map = np.full((length, width), z_min) 
        
        # Pointcloud to Matrix mapping
        for point in points:
            x, y, z = point
            i = int(y * res_y - 1)
            j = int(x * res_x - 1)
            self.before_depth_map[i, j] = max(self.before_depth_map[i, j], np.round(z*res_z))
                    
    def after_depth_map(self, len1, len2):
        start_time = time.time()
        
        depth_map = self.before_depth_map
        
        max_len = max(len1, len2)
        min_len = min(len1, len2)   

        find_flag = False
        h, w = depth_map.shape
        min_height = float('inf')
        std_map = np.ones((h-min_len, w-max_len))
        
        # Find plane with size of object
        for i in range (h - min_len):
            
            for j in range (w - max_len):
                tmp_map = depth_map[i:i+min_len, j:j+max_len]
                std_map[i, j] = np.std(tmp_map)
                
                # Find plane
                if len(np.unique(tmp_map)) == 1:
                    find_flag = True
                    
                    if depth_map[i, j] <= min_height:
                        min_height = depth_map[i, j]
                        min_idx = (i, j)
        
        # Not found
        if not find_flag:
            min_std_idx = np.argmin(std_map)
            min_idx = np.unravel_index(min_std_idx, std_map.shape)          
            print('perfect plane is not found, alternative idx find')        

        depth_map[min_idx[0]:min_idx[0]+min_len, min_idx[1]:min_idx[1]+max_len] = self.h_max
        self.after_depth_map = depth_map

        end_time = time.time()
        print(f"Calculate time : {end_time - start_time} seconds")
        
        #
        # need to make min_idx to position (x, y) & return position
        #
    
    def visualization_depth_map(self):
        
        # Set the size of the plotting window
        plt.figure(figsize=(10, 5))

        # Visualizing Before Depth Map
        if hasattr(self, 'before_img') and self.before_img:  # Check if before_img exists
            self.before_img.remove()
        self.before_img = self.axes[0].imshow(self.before_depth_map, cmap='jet')
        self.axes[0].set_title("Before Place Depth Map")
        
        # Adding the colorbar only once
        if not hasattr(self, 'color_bar'):  # Add colorbar only on the first run
            self.color_bar = plt.colorbar(self.before_img, ax=self.axes[0], orientation='vertical')

        # Visualizing After Depth Map
        if hasattr(self, 'after_img') and self.after_img:  # Check if after_img exists
            self.after_img.remove()
        self.after_img = self.axes[1].imshow(self.after_depth_map, cmap='jet')
        self.axes[1].set_title("After Place Depth Map")

        plt.draw()
        plt.pause(0.01)  # Gives a moment for the plots to update