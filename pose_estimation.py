import open3d as o3d
import numpy as np

class PoseDetector:
    def __init__(self):

        self.coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05, origin=[0, 0, 0])


    def estimate_plane(self, pcd):

        points = np.asarray(pcd.points)
        colors = np.asarray(pcd.colors)
        z_min, z_max = np.min(points[:, 2]), np.max(points[:, 2])
        indices = np.where((points[:, 2] >= 0) & (points[:, 2] <= (z_min + z_max) / 2))[0]
        filtered_points = points[indices]
        filtered_colors = colors[indices]
        self.filtered_pcd = o3d.geometry.PointCloud()
        self.filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)
        self.filtered_pcd.colors = o3d.utility.Vector3dVector(filtered_colors)

        self.filtered_pcd = self.filtered_pcd.voxel_down_sample(voxel_size=0.005)

        labels = np.array(self.filtered_pcd.cluster_dbscan(eps=0.01, min_points=10, print_progress=True))

        # 각 클러스터의 최소 z 값 계산
        unique_labels = np.unique(labels)
        min_z_per_cluster = {}
        for label in unique_labels:
            if label != -1:  # 노이즈를 제외
                cluster_points = np.asarray(self.filtered_pcd.points)[labels == label]
                min_z_per_cluster[label] = np.min(cluster_points[:, 2])

        # 가장 작은 z 값을 가진 클러스터의 레이블 찾기
        label_with_min_z = min(min_z_per_cluster, key=min_z_per_cluster.get)

        # 해당 클러스터만 추출
        cluster_points = np.asarray(self.filtered_pcd.points)[labels == label_with_min_z]
        cluster_colors = np.asarray(self.filtered_pcd.colors)[labels == label_with_min_z]

        self.cluster_pcd = o3d.geometry.PointCloud()
        self.cluster_pcd.points = o3d.utility.Vector3dVector(cluster_points)
        self.cluster_pcd.colors = o3d.utility.Vector3dVector(cluster_colors)

        # 평면 검출
        plane_model, inliers = self.cluster_pcd.segment_plane(distance_threshold=0.001,
                                                        ransac_n=3,
                                                        num_iterations=1000)
        [a, b, c, d] = plane_model

        # 평면의 법선 벡터
        self.normal_vector = np.array([a, b, c])
        # print("Normal vector of the plane:", normal_vector)

        # 평면 위의 포인트들 추출
        self.plane_points = np.asarray(self.cluster_pcd.points)[inliers]
        plane_colors = np.asarray(self.cluster_pcd.colors)[inliers]

        # 평면 위의 포인트들을 붉은색으로 표시
        plane_colors[:] = [1, 0, 0]  # red color

        self.plane_pcd = o3d.geometry.PointCloud()
        self.plane_pcd.points = o3d.utility.Vector3dVector(self.plane_points)
        self.plane_pcd.colors = o3d.utility.Vector3dVector(plane_colors)


        self.plane_center = np.mean(self.plane_points, axis=0)


    def estimate_pose(self):
        
        # 법선벡터와 [0 0 1] 사이의 각도 계산 

        # dot_product = np.dot(self.normal_vector, [0, 0, 1])
        # norm_normal_vector = np.linalg.norm(self.normal_vector)
        # cosine = dot_product / norm_normal_vector * 1
        # angle_rad = np.arccos(np.clip(cosine, -1.0, 1.0))
        # self.angle_deg = np.degrees(angle_rad)

        # 법선 벡터의 크기와 방향 조절
        arrow_length = 0.05
        arrow_origin = self.plane_center
        arrow_end = self.plane_center + self.normal_vector * arrow_length

        lines = [[self.plane_center, arrow_end]]
        self.line_set = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector([self.plane_center, arrow_end]),
            lines=o3d.utility.Vector2iVector([[0, 1]])
        )
        self.line_set.colors = o3d.utility.Vector3dVector([[0, 1, 0]])


        z_vector_origin = self.plane_center
        z_vector_end = self.plane_center + np.array([0, 0, 1]) * arrow_length

        z_lines = [[z_vector_origin, z_vector_end]]
        self.z_line_set = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector([z_vector_origin, z_vector_end]),
            lines=o3d.utility.Vector2iVector([[0, 1]])
        )
        self.z_line_set.colors = o3d.utility.Vector3dVector([[0, 0, 1]])  # 파란색으로 표시

        #### ROT_M 계산 ####
        z_0, z_1 = self.normal_vector, [0, 0, 1]
        y_0, y_1 = np.cross(z_0, z_1), np.cross(z_0, z_1) 
        x_0, x_1 = np.cross(z_0, y_0), np.cross(z_1, y_1)

        # 각 축 정규화
        x_0, y_0, z_0 = x_0/np.linalg.norm(x_0), y_0/np.linalg.norm(y_0), z_0/np.linalg.norm(z_0)
        x_1, y_1, z_1 = x_1/np.linalg.norm(x_1), y_1/np.linalg.norm(y_1), z_1/np.linalg.norm(z_1)

        # 두 좌표계의 회전행렬 구성
        self.R_0 = np.column_stack([x_0, y_0, z_0])
        R_1 = np.column_stack([x_1, y_1, z_1])

        # 두 좌표계 사이의 회전행렬 계산
        self.R = np.linalg.inv(self.R_0) @ R_1

        T = np.eye(4)
        T[:3, :3] = self.R

        return T
    

    def estimate_length(self):

        # if self.angle_deg < 10:
        #     length_1 = np.max(self.plane_points[:, 0]) - np.min(self.plane_points[:, 0])
        #     length_2 = np.max(self.plane_points[:, 1]) - np.min(self.plane_points[:, 1])
        #     length_1, length_2 = np.ceil(length_1*102), np.ceil(length_2*102)
        #     print(length_1, length_2)
        #     return length_1, length_2

        length_1 = np.max(self.plane_points[:, 0]) - np.min(self.plane_points[:, 0])
        length_2 = np.max(self.plane_points[:, 1]) - np.min(self.plane_points[:, 1])
        length_1, length_2 = np.ceil(length_1*102), np.ceil(length_2*102)
        print(length_1, length_2)

        return length_1, length_2


    def check_theta(self):

        dot_product = np.dot(self.normal_vector, [0, 0, 1])
        norm_normal_vector = np.linalg.norm(self.normal_vector)
        cosine = dot_product / norm_normal_vector * 1
        angle_rad = np.arccos(np.clip(cosine, -1.0, 1.0))
        angle_deg = np.degrees(angle_rad)

        return angle_deg
    

    def visualization(self):

        print("z cut")
        o3d.visualization.draw_geometries([self.filtered_pcd, self.coordinate_frame])
        
        print("downsampling")
        o3d.visualization.draw_geometries([self.filtered_pcd, self.coordinate_frame])

        print("clustering")
        o3d.visualization.draw_geometries([self.cluster_pcd, self.coordinate_frame])

        print("plane with vector")
        o3d.visualization.draw_geometries([self.plane_pcd, self.coordinate_frame, self.line_set, self.z_line_set])

        base_axis = self.R_0.T

        after_axis = self.R_0 @ self.R

        def create_lineset(axis, origin, colors):
            points = [origin + axis[i] for i in range(3)]
            lines = [[origin, p] for p in points]
            lineset = o3d.geometry.LineSet(
                points=o3d.utility.Vector3dVector([origin] + points),
                lines=o3d.utility.Vector2iVector([[0, i+1] for i in range(3)])
            )
            lineset.colors = o3d.utility.Vector3dVector(colors)

            return lineset

        origin = self.plane_center
        base_colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]  # RGB for base_axis
        after_colors = [[1, 0, 0], [0, 2, 0], [0, 0, 2]]  # colors for after_axis

        base_lineset = create_lineset(base_axis*0.1, origin, base_colors)
        after_lineset = create_lineset(after_axis*0.1, origin, after_colors)

        o3d.visualization.draw_geometries([self.plane_pcd, base_lineset, after_lineset, self.coordinate_frame])