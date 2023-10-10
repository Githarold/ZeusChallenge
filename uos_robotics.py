import atexit
import time

from get_grasp import GraspDetector
from palletizing import Palletizing
from client_api import JeusController
from camera_utils import FrameCapture
from yolo_detector import YoloDetector
from pose_estimation import PoseDetector

class UOSRobotics:

    def __init__(self):
        
        self.miss_stack = 0
        self.pack = Palletizing()
        self.yolo = YoloDetector()
        self.zeus = JeusController()
        self.grasp = GraspDetector()
        self.object = PoseDetector()
        self.camera1 = FrameCapture(serial='047322070277')   # mounted camera
        # self.camera2 = FrameCapture(serial='139522076807')   # fixed camera
        self.depth_scale, self.intrinsics = self.camera1.get_scale_intrinsics()

        print("Init done")

    def run(self):
        atexit.register(self.camera1.free)
        # atexit.register(self.camera2.free)
        
        self.zeus.move_depth()
        self.zeus.gripper_open()

        # Make Initial depth map
        depth_frame, color_frame, _ = self.camera1.get_frame()
        depth_pcd = self.camera1.get_pcd(depth_frame, color_frame, flag = 'depth')
        detected_obj_property = self.yolo.detect_obj_property(color_frame, depth_frame, self.depth_scale, self.intrinsics)
        
        self.pack.make_before_depth_map(depth_pcd)
        
        while True:
            
            # Move to place to grasp object detect
            if self.miss_stack == 0:
                self.zeus.move_pick()
            else:
                pass
            
            # Predict grasp pose
            while True:
                    depth_frame, color_frame, pc = self.camera1.get_frame()
                    grasp_pose, score, any_flag = self.grasp.predict_grasp(depth_frame, color_frame, pc)

                    if score > 0.2:
                        break
            
            # Yolo detect to know object is exist
            detected_obj_list, yolo_flag = self.yolo.detect_obj_list(color_frame)
            
            # If no detection, move pick
            if any_flag * yolo_flag == 0:
                self.miss_stack += 1                
                if self.miss_stack > 3:
                    break                
                else:
                    continue            
            else:
                self.miss_stack = 0

            # Move to object & Pick
            self.zeus.move_object(grasp_pose)
            self.zeus.pick()
            
            # Move to fixed camera
            self.zeus.move_pose_estimation()
                        
            # Object 2D pose, shape detect
            depth_frame, color_frame, _ = self.camera2.get_frame()
            
            detected_obj_list, yolo_flag = self.yolo.detect_obj_list(color_frame)

            # Use YOLO, Check object. If there's not object, Move to first place
            if yolo_flag == 0:
                print("NO OBJECT DETECTED")
                self.zeus.gripper_open()
                continue
                    
            # Make weight map
            self.pack.make_weight_map(detected_obj_list, detected_obj_property) ## ????

            obj_pcd = self.camera2.get_pcd(depth_frame, color_frame, flag = 'object')

            # Change object orientation
            self.object.estimate_plane(obj_pcd)
            base2fixed = self.object.estimate_pose()
            ## 로봇 base2fixed HTM으로 ori 변환 
            angle = self.object.check_theta()
            if angle < 10 :
                len1, len2 = self.object.estimate_length()
            else:
                self.object.estimate_plane()
                base2fixed = self.object.estimate_pose()
                ## 로봇 base2fixed HTM으로 ori 변환 
                angle = self.object.check_theta()
                len1, len2 = self.object.estimate_length()


            # Make depth map & visualization
            self.pack.make_after_depth_map(len1, len2)
            self.pack.visualization_depth_map(flag = 1)
            
            # Depth map to absolute x, y, z
            place_x, place_y, place_z = self.pack.matrix_to_absolute_coordinate()
            
            # Move object to box & Place
            self.zeus.move_only_position(place_x, place_y, place_z)
            self.zeus.place()
            
            # Move to above the box
            self.zeus.move_depth()
            
            # Update Box state
            depth_frame, color_frame, _ = self.camera1.get_frame()
            depth_pcd = self.camera1.get_pcd(depth_frame, color_frame, flag = 'depth')
            detected_obj_property = self.yolo.detect_obj_property(color_frame, depth_frame, self.depth_scale, self.intrinsics)

            # Make depth map & visualization
            self.pack.make_before_depth_map(depth_pcd)
            self.pack.visualization_depth_map(flag = 2)

        # Close connection
        self.zeus.send_data(-1, "finish")
        self.zeus.close()

        # Camera pipline stop
        self.camera1.free()
        self.camera2.free()

               
if __name__ == "__main__":
    rbiz = UOSRobotics()
    rbiz.run()