import open3d as o3d
import numpy as np

# .npy 파일에서 데이터를 불러옴
xyz_data = np.load("./data/xyz_image_1.npy")
print(xyz_data)
xyz_data_reshaped = xyz_data.reshape(-1, 3)
xyz_data_filtered = xyz_data_reshaped[~np.all(xyz_data_reshaped == 0, axis=1)]

# numpy 배열을 Open3D의 PointCloud 객체로 변환
point_cloud = o3d.geometry.PointCloud()
point_cloud.points = o3d.utility.Vector3dVector(xyz_data_reshaped)

# 포인트 클라우드 시각화
o3d.visualization.draw_geometries([point_cloud])