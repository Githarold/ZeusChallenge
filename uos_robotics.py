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

        self.depth_scale, self.intrinsics = self.camera1.get_scale_intrinsics()

    def run(self):
        
        self.zeus.move_depth()
        
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
            depth_frame, color_frame, pc = self.camera1.get_frame()
            grasp_pose = self.grasp.predict_grasp(depth_frame, color_frame, pc)
            
            # Yolo detect to know object is exist
            detected_obj_list = self.yolo.detect_obj_list(color_frame)
            
            if not grasp_pose or not detected_obj_list:
                self.miss_stack += 1
                
                if self.miss_stack > 3:
                    break
                
                else:
                    continue
            
            else:
                self.miss_stack = 0

            # Move to object & Pick
            self.zeus.move(grasp_pose)
            self.zeus.pick()
            
            # Move to fixed camera
            self.zeus.move_pose_estimation()
                        
            # Object 2D pose, shape detect
            depth_frame, color_frame, _ = self.camera2.get_frame()
            
            detected_obj_list = self.yolo.detect_obj_list(color_frame)            
            # Use YOLO, Check object. If there's not object, Move to first place            
            if not detected_obj_list:
                # Restart loop
                continue
            
            # Make weight map
            self.pack.make_weight_map(detected_obj_list, detected_obj_property)

            obj_pcd = self.camera2.get_pcd(depth_frame, color_frame, flag = 'object')

            
            # 변환행렬로 로봇 오리 변경 -> 평면 z랑 카메라 z비교 해서 일정치 이하로 내려가면 밑면길이도출
            self.object.estimate_plane(obj_pcd)
            rot_matrix = self.object.estimate_pose()
            # 로봇오리엔테이션변경 동작
            len1, len2 = self.object.estimate_length()

            # Make depth map & visualization
            self.pack.make_after_depth_map(len1, len2)
            self.pack.visualization_depth_map(flag = 1)
            
            # Depth map to absolute x, y, z
            place_x, place_y, place_z = self.pack.matrix_to_absolute_coordinate()
            
            # Move object to box & Place
            self.zeus.move_only_position(place_x, place_y)
            self.zeus.place(place_z)
            
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