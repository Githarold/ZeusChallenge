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
        
        self.zeus.move_place()
        
        # Make Initial depth map
        depth_frame, color_frame, _ = self.camera1.get_frame()
        depth_pcd = self.camera1.get_pcd(depth_frame, color_frame, flag = 'depth')
        self.pack.before_depth_map(depth_pcd)
                
        while True:
            
            # Move to place to grasp object detect
            if self.miss_stack == 0:
                self.zeus.move_pick()
            else:
                pass
            
            # Predict grasp pose
            depth_frame, color_frame, pc = self.camera1.get_frame()
            grasp_pose = self.grasp.predict_grasp(depth_frame, color_frame, pc)
            
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
                
            #
            # grasp_pose to robot base code
            #

            # Move to object & Pick
            self.zeus.move(grasp_pose)      # need to change
            self.zeus.pick()
            
            # Move to fixed camera
            self.zeus.move_pose_estimation()
                        
            # Use YOLO, Check object. If there's not object, Move to first place
            _, color_frame, _ = self.camera2.get_frame()
            detected_obj = self.yolo.detect(color_frame)
            
            if not detected_obj:
                # Restart loop
                continue
                        
            # Object 2D pose, shape detect
            depth_frame, color_frame, _ = self.camera2.get_frame()
            obj_pcd = self.camera2.get_pcd(depth_frame, color_frame, flag = 'object')

            self.object.estimate_plane(obj_pcd)
            obj_pose = self.object.estimate_pose()
            len1, len2 = self.object.estimate_length()

            # Make depth map & visualization
            self.pack.after_depth_map(len1, len2)
            self.pack.visualization_depth_map(flag = 1)

            #
            # object_pose to place pose code
            #
            
            # Move object to box & Place
            self.zeus.move(obj_pose)        # need to change
            self.zeus.place()
            
            # Move to above the box
            self.zeus.move_place()          # need to change
            
            depth_frame, color_frame, _ = self.camera1.get_frame()
            depth_pcd = self.camera1.get_pcd(depth_frame, color_frame, flag = 'depth')       # Multi threding?     
            
            # Make depth map & visualization
            self.pack.before_depth_map(depth_pcd)                                            #
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