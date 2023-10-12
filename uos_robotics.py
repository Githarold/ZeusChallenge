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
        self.camera2 = FrameCapture(serial='139522076807')   # fixed camera
        self.depth_scale, self.intrinsics = self.camera1.get_scale_intrinsics()
        atexit.register(self.camera1.free)
        atexit.register(self.camera2.free)        
        self.plane_stack = 0
        print("Init done")

    def run(self):
        init_pos = self.zeus.get_current_pos()

        if init_pos[2] < 313.53:
            self.zeus.move([init_pos[0], init_pos[1], 313.53, -90, 0.0, -180])

        else:
            self.zeus.move([init_pos[0], init_pos[1], init_pos[2], -90, 0.0, -180])

        self.zeus.move_depth()
        self.zeus.gripper_open()

        # Make Initial depth map
        depth_frame, color_frame, _ = self.camera1.get_frame()
        depth_pcd = self.camera1.get_pcd(depth_frame, color_frame, flag = 'depth')
        detected_obj_property = self.yolo.detect_obj_property(color_frame, depth_frame, self.depth_scale, self.intrinsics)
        
        self.pack.make_before_depth_map(depth_pcd)
        
        while True:
            print("Start")
            # Move to place to grasp object detect
            if self.miss_stack == 0:
                self.zeus.move_pick()
            else:
                pass
            
            # Predict grasp pose
            while True:
                    depth_frame, color_frame, pc = self.camera1.get_frame()
                    grasp_pose, score, any_flag = self.grasp.predict_grasp(depth_frame, color_frame, pc)

                    #########################
                    if score > 0.18:    #kym : original = 0.2
                        break
                    #########################
            # Yolo detect to know object is exist
            detected_obj_list, yolo_flag = self.yolo.detect_obj_list(color_frame)
            
            if any_flag * yolo_flag == 0:
                self.miss_stack += 1
                if self.miss_stack > 3:
                    
                    break                
                else:
                    continue            
            else:
                self.miss_stack = 0
            ###############################################################################################################################
            success_flag = self.zeus.move_object(grasp_pose)                        # kym

            if self.plane_stack > 3:
                   
                    break

            if success_flag == -1:
                self.plane_stack += 1
                continue
                
            else:
                self.plane_stack = 0
                # print(f"predict anygrasp z {grasp_pose[2]}")
                ###########################################

                
                # fpr not to crash with ground
                before_pick = self.zeus.get_current_pos()
                ##########################################
                # -17.7 : bottom z poistion, get by teach mode
                if before_pick[2] -35.0 < -18.21:            # -17.7
                    before_pick[2] = -17.7

                else:
                    before_pick[2] -= 35.0

                print(f"predict pick {before_pick[2]}")
                ###########################################     

                self.zeus.move([before_pick[0],before_pick[1],before_pick[2], before_pick[3],before_pick[4],before_pick[5]])
                
                
                    # self.zeus.move([before_pick[0],before_pick[1],before_pick[2] -30, before_pick[3],before_pick[4],before_pick[5]])

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
            ###############################################################################################################################
            # Make weight map
            self.pack.make_weight_map(detected_obj_property, detected_obj_list)

            # Change object orientation
            obj_pcd = self.camera2.get_pcd(depth_frame, color_frame, flag = 'object')
            self.object.estimate_plane(obj_pcd)
            self.object.estimate_pose()
            rx, ry, rz = self.object.compute_euler_difference()
            self.object.visualization()
            self.zeus.move_only_orientation(0.0, 0.0, ry)
            angle = self.object.check_theta()

            if angle > 10 : 
                depth_frame, color_frame, _ = self.camera2.get_frame()
                obj_pcd = self.camera2.get_pcd(depth_frame, color_frame, flag = 'object')
                self.object.estimate_plane(obj_pcd)
                self.object.estimate_pose()
                rz, ry, rx = self.object.compute_euler_difference()
                self.object.visualization()
                self.zeus.move_only_orientation(0.0, 0.0, ry)


            object_pose = self.zeus.get_current_pos()
            

            len1, len2, offset_y = self.object.estimate_length()
            
            # Make depth map & visualization
            rel_x, rel_y, rotate_flag = self.pack.make_after_depth_map(len1, len2, offset_y)
            abs_x, abs_y = self.zeus.depth_pos_to_abs(rel_x, rel_y)

            # self.pack.visualization_depth_map(flag = 1)
            # self.pack.visualization_opencv(flag = 1)

            # Move object to box & Place

            ###################################################################################################
            self.zeus.move_pose_estimation()
            print(f"after object pose {object_pose}")
            self.zeus.move([abs_x, abs_y, 332.80, object_pose[3], object_pose[4], object_pose[5]])
            self.zeus.place(-50.0)
            ###################################################################################################            
            
            # Move to above the box
            self.zeus.move_depth()
            
            # Update Box state
            depth_frame, color_frame, _ = self.camera1.get_frame()
            depth_pcd = self.camera1.get_pcd(depth_frame, color_frame, flag = 'depth')
            detected_obj_property = self.yolo.detect_obj_property(color_frame, depth_frame, self.depth_scale, self.intrinsics)

            # Make depth map & visualization
            self.pack.make_before_depth_map(depth_pcd)
            # self.pack.visualization_depth_map(flag = 2)
            # self.pack.visualization_opencv(flag = 2)

        # Close connection
        self.zeus.send_data(-1, "finish")
        self.zeus.close()

        # Camera pipline stop
        self.camera1.free()
        self.camera2.free()
               
if __name__ == "__main__":
    rbiz = UOSRobotics()
    rbiz.run()