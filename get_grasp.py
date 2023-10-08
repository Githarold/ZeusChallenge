import argparse
import numpy as np
import open3d as o3d
from gsnet import AnyGrasp

class GraspDetector:
    
    def __init__(self):

        # Anygrasp configs
        parser = argparse.ArgumentParser()
        parser.add_argument('--checkpoint_path', default="/home/robotics/rbiz/src/anygrasp_sdk/grasp_detection/log/checkpoint_detection.tar", help='Model checkpoint path')
        parser.add_argument('--max_gripper_width', type=float, default=0.1, help='Maximum gripper width (<=0.1m)')
        parser.add_argument('--gripper_height', type=float, default=0.11, help='Gripper height')
        parser.add_argument('--top_down_grasp', action='store_true', help='Output top-down grasps')
        parser.add_argument('--debug', action='store_true', help='Enable visualization')

        cfgs = parser.parse_args()

        self.anygrasp = AnyGrasp(cfgs)
        self.anygrasp.load_net()

        xmin, xmax = -0.1, 0.1
        ymin, ymax = -0.1, 0.1
        zmin, zmax = 0.0, 1.0
        self.lims = [xmin, xmax, ymin, ymax, zmin, zmax]

    def predict_grasp(self, depth_frame, color_frame, pc):

        depths = np.asanyarray(depth_frame.get_data())
        colors = np.asanyarray(color_frame.get_data())

        points = pc.calculate(depth_frame)

        v = points.get_vertices()
        verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)

        # get point cloud
        xmap, ymap = np.arange(640), np.arange(480)
        xmap, ymap = np.meshgrid(xmap, ymap)
        points_z = depths / 1000.0

        # remove outlier
        mask = (points_z > 0.15) & (points_z < 0.4)
        points = verts.reshape(480, 640, 3)
        points = points[mask].astype(np.float32)
        colors = colors[mask].astype(np.float32)

        # generate prediction
        gg, cloud = self.anygrasp.get_grasp(points, colors, self.lims)

        if len(gg) == 0:
            print('No grasp detect!')
            return None
        
        gg = gg.nms().sort_by_score()
        gg_pick = gg[0:20]
        print('grasp score:', gg_pick[0].score)
        
        # visualization
        trans_mat = np.array([[1,0,0,0],[0,1,0,0],[0,0,-1,0],[0,0,0,1]])
        grippers = gg.to_open3d_geometry_list()
        for gripper in grippers:
            gripper.transform(trans_mat)

        rot_mat = np.array([[1,0,0],[0,1,0],[0,0,-1]])
        pointss = points@rot_mat

        colors = colors / 255
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pointss)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        o3d.visualization.draw_geometries([pcd])
        print("get pcd complete")
        
        return gg_pick[0]