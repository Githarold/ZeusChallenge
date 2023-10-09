import numpy as np
import open3d as o3d
import pyrealsense2 as rs

class FrameCapture:

    def __init__(self, serial):
        
        ctx = rs.context()
        devices = ctx.query_devices()

        if len(devices) < 2:
            print("Two cameras are required")
            exit()
        
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_device(serial)

        self.pc = rs.pointcloud()
    
        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = self.config.resolve(pipeline_wrapper)

        # device = pipeline_profile.get_device()

        # found_rgb = False
        # for s in device.sensors:
        #     if s.get_info(rs.camera_info.name) == 'RGB Camera':
        #         found_rgb = True
        #         break
            
        # if not found_rgb:
        #     print("The project requires Depth camera with Color sensor")
        #     exit(0)

        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        profile = self.pipeline.start(self.config)
        
        # Post processing with realsense-viewer default setting
        with open("D435i.json", 'r') as file:
            json_str = file.read().strip()
        device = profile.get_device()
        advanced_mode = rs.rs400_advanced_mode(device)
        advanced_mode.load_json(json_str)        
        
        # Make align
        align_to = rs.stream.color
        self.align = rs.align(align_to)

        # Get intrinsics
        depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
        self.intrinsics = depth_profile.get_intrinsics()

        # Make depth_scale
        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()


    def get_frame(self):
        
        while True:
            try:
                # Get camera frame
                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(frames)

                # Align depth frame to color frame        
                aligned_depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()

                if not aligned_depth_frame or not color_frame:
                    continue
                
                # Return data
                return aligned_depth_frame, color_frame, self.pc

            except Exception as e:
                print(f"Error getting frames: {e}. Retrying...")
                continue
    
    def get_pcd(self, depth_frame, color_frame, flag):
        
        depths = np.asanyarray(depth_frame.get_data())
        colors = np.asanyarray(color_frame.get_data())

        points = self.pc.calculate(depth_frame)

        v = points.get_vertices()
        verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)

        # get point cloud
        xmap, ymap = np.arange(640), np.arange(480)
        xmap, ymap = np.meshgrid(xmap, ymap)
        points_z = depths / 1000.0
        
        if (flag == 'object'):
            zmin, zmax = 0.36, 0.56
        elif (flag == 'depth'):
            zmin, zmax = 0.32, 0.54
        
        # remove outlier
        mask = (points_z > zmin) & (points_z < zmax)
        points = verts.reshape(480, 640, 3)
        points = points[mask].astype(np.float32)
        colors = colors[mask].astype(np.float32)
        
        rot_mat = np.array([[1,0,0],[0,1,0],[0,0,-1]])
        pointss = points@rot_mat        
        colors = colors / 255        
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pointss)
        pcd.colors = o3d.utility.Vector3dVector(colors)        
        
        return pcd
    

    def free(self):

        self.pipeline.stop()


    def get_scale_intrinsics(self):

        return self.depth_scale, self.intrinsics
