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
        self.camera1 = FrameCapture(serial = '047322070277')   # mounted camera
        self.camera2 = FrameCapture(serial = '139522076807')   # fixed camera

    def run(self):
        
        # Move to place to grasp object detect
        self.zeus.move_pick()
        
        while True:
                        
            # Predict grasp pose
            depth_frame, color_frame, pc = self.camera1.get_frame()
            grasp_pose = self.grasp.predict_grasp(depth_frame, color_frame, pc)
            
            #
            # grasp_pose to robot base code
            #
            
            # Yolo detect to know object is exist
            detected_obj = self.yolo.detect(color_frame)
            
            if not grasp_pose or not detected_obj:
                self.miss_stack += 1
                
                if self.miss_stack > 3:
                    print('Program done')
                    break
                
                else:
                    continue
            
            else:
                self.miss_stack = 0
                
            # Move to object & Pick
            self.zeus.move(grasp_pose)      # need to change
            self.zeus.pick()
            
            # Move to fixed camera
            self.zeus.move_pose_estimation()
                        
            # Use YOLO, Check object. If there's not object, Move to first place
            _, color_frame, _ = self.camera2.get_frame()
            detected_obj = self.yolo.detect(color_frame)
            
            if not detected_obj:
                # Move to first place
                self.zeus.move_pick()
                continue
                        
            # Object 2D pose, shape detect
            depth_frame, color_frame, _ = self.camera2.get_frame()
            pose_pcd = self.camera2.get_pcd(depth_frame, color_frame, flag = 'object')

            self.object.estimate_plane(pose_pcd)
            obj_pose = self.object.estimate_pose()
            len1, len2 = self.object.estimate_length()
            
            # Move to above the box
            self.zeus.move_place()
            
            depth_frame, color_frame, _ = self.camera1.get_frame()
            depth_pcd = self.camera1.get_pcd(depth_frame, color_frame, flag = 'depth')
            
            # Make depth map & visualization
            self.pack.before_depth_map(depth_pcd)
            self.pack.after_depth_map(len1, len2)
            self.pack.visualization_depth_map()

            #
            # object_pose to place pose code
            #
            
            # Move object to box & Place
            self.zeus.move(obj_pose)
            self.zeus.place()
            
            #
            # Smoothly move to first place code
            #            
            
        # Close connection
        self.zeus.close()

        # Camera pipline stop
        self.camera1.free()
        self.camera2.free()
               
if __name__ == "__main__":
    rbiz = UOSRobotics()
    rbiz.run()