import numpy as np
import open3d as o3d
import pyrealsense2 as rs

class FrameCapture:

    def __init__(self, serial):

        self.coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05, origin=[0, 0, 0])

        self.serial = serial
        
        ctx = rs.context()
        devices = ctx.query_devices()

        if len(devices) < 2:
            print("Two cameras are required")
            exit()
        
        while True:

            try:  # Start of try-except block

                # Configure depth and color streams
                self.pipeline = rs.pipeline()
                self.config = rs.config()
                self.config.enable_device(serial)

                self.pc = rs.pointcloud()

                # Get device product line for setting a supporting resolution
                pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
                pipeline_profile = self.config.resolve(pipeline_wrapper)

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

                break            

            except RuntimeError as e2:  # Handling exceptions
                print(f"RuntimeError !!! : {e2}")
                self.pipeline.stop()  # Stop the pipeline
                # raise  # Re-raise the exception for further handling or to inform the caller

            except Exception as e:  # Handling exceptions
                print(f"Error during initialization: {e}")
                self.pipeline.stop()  # Stop the pipeline
                # raise  # Re-raise the exception for further handling or to inform the caller

        print("Camera allocate done")

    def get_frame(self, timeout_ms=6000):  # Default timeout set to 6000ms (6 seconds)
        
        while True:
            try:
                # Get camera frame with a timeout
                frames = self.pipeline.wait_for_frames(timeout_ms)
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
                self.pipeline.stop()
                self.__init__(self.serial)
                continue
    
    def get_pcd(self, depth_frame, color_frame, flag):
        self.clear_buffer()

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
            zmin, zmax = -100.0, 100.0
        elif (flag == 'depth'):
            zmin, zmax = 0.36, 0.58
        
        # remove outlier
        mask = (points_z > zmin) & (points_z < zmax)
        points = verts.reshape(480, 640, 3)
        points = points[mask].astype(np.float32)
        colors = colors[mask].astype(np.float32)
        
        colors = colors / 255
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)

        o3d.visualization.draw_geometries([pcd, self.coordinate_frame])
        
        return pcd
    

    def free(self):

        self.pipeline.stop()


    def get_scale_intrinsics(self):

        return self.depth_scale, self.intrinsics

    def clear_buffer(self, num_frames=10):
        """
        clear realSense buffer
        """
        last_frame = None
        for _ in range(num_frames):
            last_frame = self.pipeline.wait_for_frames()
        return last_frame