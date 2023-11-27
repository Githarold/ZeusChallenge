import os
import argparse
import torch
import numpy as np
import open3d as o3d
import pyrealsense2 as rs
import time



import cv2

class rbiz:

    def __init__(self):

        
       

        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.pc = rs.pointcloud()
        

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = self.config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()

        found_rgb = False
        for s in device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            print("The demo requires Depth camera with Color sensor")
            exit(0)

        # self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        # self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        profile = self.pipeline.start(self.config)
        with open("D435i.json", 'r') as file:
            json_str = file.read().strip()
        device = profile.get_device()
        advanced_mode = rs.rs400_advanced_mode(device)
        advanced_mode.load_json(json_str)
        
        
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        

        clipping_distance_in_meters = 1
        self.clipping_distance = clipping_distance_in_meters / depth_scale

        align_to = rs.stream.color
        self.align = rs.align(align_to)
        time.sleep(10)
        # main
        self.capture_flag = 1
        self.Capture()
        
        

    def Capture(self):
        # cv2.namedWindow('img', cv2.WINDOW_NORMAL)
        # cv2.namedWindow('depth', cv2.WINDOW_NORMAL)
        # Wait for a coherent pair of frames: depth and color
        while self.capture_flag:

            # align을 맞춰 준다.
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)

            
            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            
            
            if not aligned_depth_frame or not color_frame:
                continue
            else:
                self.capture_flag = 0

            

            # get data
            self.depths = np.asanyarray(aligned_depth_frame.get_data())
            self.colors = np.asanyarray(color_frame.get_data())
            print(f"color size!!{self.colors.shape}")
            cv2.imwrite('/home/robotics/handtoeyecali_umeyama/data/cali_rgb.png', self.colors)

            # cv2.imshow('img',self.colors)
            # cv2.imshow('depth',self.depths)
            # for test
            self.points = self.pc.calculate(aligned_depth_frame)
            v, t = self.points.get_vertices(), self.points.get_texture_coordinates()
            self.verts = np.asanyarray(v).view(np.float32).reshape(720,1280, 3)  # xyz
            

            np.save('/home/robotics/handtoeyecali_umeyama/data/xyz_image.npy', self.verts)

            # key = cv2.waitKey(0)
        
            # if key & 0xFF == ord('q') or key == 27:
            #     cv2.destroyAllWindows()
            #     break
if __name__ == '__main__':
    rbiz()