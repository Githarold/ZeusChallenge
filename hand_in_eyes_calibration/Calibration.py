import numpy as np
from cmath import nan, sqrt
import numpy.linalg
from scipy.spatial.transform import Rotation as Rot
import open3d as o3d 
##Least-Squares Rigid Motion Using SVD 논문 참고!
def umeyama(P, Q):
    assert P.shape == Q.shape
    n, dim = P.shape

    centeredP = P - P.mean(axis=0)
    centeredQ = Q - Q.mean(axis=0)

    C = np.dot(np.transpose(centeredP), centeredQ) / n

    V, S, W = np.linalg.svd(C)
    # print("V: {}".format(V))
    # print("W: {}".format(W))
    d = (np.linalg.det(V) * np.linalg.det(W)) < 0.0  # det는 orthogonal이라 1 or -1

    if d:
        S[-1] = -S[-1]
        V[:, -1] = -V[:, -1]

    R = np.dot(V, W)  # 방향성.
    varP = np.var(a1, axis=0).sum()
    c = 1 / varP * np.sum(S)  # scale factor

    t = Q.mean(axis=0) - P.mean(axis=0).dot(c * R)

    return c, R, t

# Testing
np.set_printoptions(precision=3)

x = 0.0
y = 0.0
z = 0.0  # 툴 사이즈.

a1 = np.array([  
        
[ 0.0175020 , 0.0192009 , 0.3260000 ],
####################################
[ -0.0221525 , 0.0219102 , 0.3720000 ],
#####################################
[ -0.0258913 , 0.0429563 , 0.4790000 ],
#####################################
[ 0.0418519 , 0.0637820 , 0.5020000 ],
#####################################
[ 0.0541009 , 0.0090676 , 0.4440000 ],
#####################################
[ -0.0013075 , -0.0032725 , 0.5490000 ],
#####################################
[ 0.0113466 , -0.0020617 , 0.5480000 ],
#####################################
[ -0.0130400 , 0.0121449 , 0.5120000 ],
#####################################
[ 0.0322885 , 0.0475965 , 0.4530000 ],
#####################################
[ 0.0177426 , 0.0259971 , 0.4770000 ],
#####################################
[ 0.1936524 , 0.0269324 , 0.4750000 ],
#####################################
[ 0.0123624 , -0.0028523 , 0.4040000 ],
#####################################
[ -0.0994204 , 0.0038402 , 0.4610000 ],
#####################################
[ -0.1568074 , -0.0244206 , 0.4320000 ],
#####################################
[ -0.0634473 , -0.0055965 , 0.3100000 ],
#####################################
[ 0.2544123 , -0.0194607 , 0.5460000 ],
#####################################
[ -0.0501188 , 0.0198800 , 0.5230000 ],
#####################################
[ -0.0311446 , 0.0255730 , 0.3250000 ],
#####################################
[ 0.0183357 , 0.0423574 , 0.3720000 ],
#####################################
[ -0.0592729 , 0.0795498 , 0.3520000 ],
#####################################
[ 0.0420416 , 0.0146144 , 0.4210000 ],
####################################
[ 0.0686175 , -0.0012115 , 0.3220000 ],
#####################################
[ 0.0622937 , -0.0238459 , 0.3590000 ],
#####################################
[ 0.1054679 , -0.0248103 , 0.4000000 ],
#####################################
[ 0.1080289 , -0.0011690 , 0.4390000 ],
#####################################
[ 0.0801068 , 0.0037272 , 0.3540000 ],
#####################################
[ 0.0970999 , -0.0092505 , 0.3310000 ],
#####################################
[ 0.0519485 , -0.0051572 , 0.5570000 ],
#####################################
[ 0.0860783 , 0.0378029 , 0.3320000 ],
####################################
[ 0.0547125 , 0.0283665 , 0.3460000 ],
####################################
        ])

        # gripper to object
a2 = np.array([  # robot position
        # # 순 - 2



[-86.5094364  , 22.337069  , 147.18608176],
[-83.04730088,  60.8748803 , 193.6840057 ],
[-62.84915981,  62.19380698, 294.99169206],
[-42.73925463,  -7.57558187, 321.40728301],
[-99.25074779, -21.57214549 ,263.40541567],
[-114.91784918,   34.15678474 , 365.07589114],
[-115.16787766,   21.89976935 , 364.86096113],
[-102.39645112,   47.86442561 , 329.48738134],
[-66.94411639,   4.43714119 ,273.20759572],
[-88.7220767,   21.20324992 ,295.44882805],
[ -86.69431142, -150.56380826 , 294.96250102],
[-113.29445062 ,  29.37244621 , 223.45792951],
[-105.12350975 , 141.33852706 , 280.1114408 ],
[-131.76746425,  197.26082514 , 251.24238582],
[-111.40716143  ,103.5286468  , 131.43113092],
[-127.4496408 , -209.06244712 , 365.43174679],
[-86.01575593 , 94.41632296 ,340.09912465],
[-83.51306669 , 69.12734647 ,150.23111083],
[-68.02008873  ,12.60488784 ,196.07552456],
[-35.43799765 , 93.01814633 ,175.43930739],
[-87.09930489 , -7.15318072 ,234.28600192],
[-110.7069926  , -30.27309025 , 146.02195416],
[-125.21336867 , -25.50708175 , 175.94449377],
[-131.89799761,  -70.65783538 , 221.7247648 ],
[-104.9292294  , -72.45914289 , 255.56411706],
[-103.05273462 , -43.99804049 , 176.73136796],
[-115.8493799   ,-54.1628933  , 149.38098227],
[-114.50737671  , -8.23179899 , 371.55696422],
[-74.76544726, -46.2862368  ,155.46625608],
[-80.30923548 ,-12.08293456, 166.80806422],
])

a2 = a2/1000      
a3 = a1.copy()
a4 = a2.copy()

a2[:, 0] = a2[:, 0] - x
a2[:, 1] = a2[:, 1] - y
a2[:, 2] = a2[:, 2] - z

# a3 = np.array([  # 검증용
#     #   [-48.19801986, 72.07606182, 815.],
#      [-0.02971538 ,-0.0469541  , 0.32500002],
# [-0.08386305 ,-0.02145464 , 0.32300001],
# [-0.13983552 , 0.00379076 , 0.32600001],
# [ 0.06205218 , 0.02878702 , 0.32100001],
# [ 0.11953172 , 0.05420733 , 0.31900001],
# [-0.15475786 ,-0.04758261 , 0.32200003],
# [-0.         ,-0.         , 0.        ],
# [ 0.18674083 , 0.00229218 , 0.317     ],
# [-0.06271729 , 0.02781477 , 0.32200003],
# [ 0.19066802 , 0.05230801 , 0.31600001],
# [ 0.11568479 ,-0.04904597 , 0.32000002],
# [-0.13684043 ,-0.0240143  , 0.324     ],
# [-0.         , 0.         , 0.        ],
# [-0.         , 0.         , 0.        ],
# [-0.12780952 , 0.05242991 , 0.32100001]
# ])

# a4 = np.array([
#     #    [678.87, 181.60, 111.78],
#     [73.78,-556.81,486.92],
# [116.24,-544.12,486.92],
# [158.94,-530.87,486.92],
# [201.88,-517.18,486.92],
# [245.61,-502.81,486.59],
# [87.81,-599.58,486.59],
# [129.48,-586.84,486.36],
# [172.81,-574.08,486.17],
# [216.36,-560.35,486.17],
# [259.74,-547.39,486.17],
# [102.12,-643.04,486.14],
# [143.60,-630.02,486.13],
# [187.00,-616.30,486.13],
# [230.83,-602.72,486.13],
# [274.78,-590.01,486.13]

# ])


a4[:, 0] = a4[:, 0] - x
a4[:, 1] = a4[:, 1] - y
a4[:, 2] = a4[:, 2] - z

c, R, t = umeyama(a1, a2)
print("R =\n", R)
print("c =", c)
print("t =\n", t)
T = np.zeros((4, 4))
T[0:3, 0:3] = np.copy(c * R)
T[0:3, 3] = np.copy(t.transpose())
T[3, 3] = 1

print("T = \n{}".format(T))

# T : Tool to camera
np.savez("data/T_b2c_tool.npz", t=T)

# num = 1

# print("Check:  a1*cR + t = a2  is", np.allclose(a1.dot(c * R) + t, a2))
# err = sqrt(((a1.dot(c * R) + t - a2) ** 2).sum())
# print("Residual error", err)
# # err2 = sqrt(((a3[num].dot(c * R) + t - a4[num]) ** 2).sum())
# # # print("Residual error", err)
# # print("Residual errorr2", err2)
sum_of_error = 0
# # 검증..
print(a1.shape[0])
for num in range(0, a3.shape[0]):
    err2 = sqrt(((a3[num].dot(c * R) + t - a4[num]) ** 2).sum())
    # print("Test Residual error", num + 1, err2)
    sum_of_error += err2

    # print("-"*10)
    # print(a3[num].dot(c * R) + t) ##a3: camera
    # print(a4[num]) #a4: base to tool
    # print("-"*10)
print("mean of error ", sum_of_error/a1.shape[0] *1000 , "mm")

# print(T)
# print(np.linalg.pinv(T))
# print(" ")


c, R, t = umeyama(a1, a2)
# print(a1.shape[0])
# transformed_a1 = np.zeros((int(a1.shape[0]), int(a1.shape[1])), dtype=int)
transformed_a1 = np.empty((a1.shape[0], a1.shape[1]))
for num in range(0, a3.shape[0]):
    
    temp = a1[num].dot(c * R) + t ##a3: camera
    transformed_a1[num] = temp
# print(transformed_a1)
# transformed_a1 = c * a1.dot(R) + t

# Create point clouds
pcd1 = o3d.geometry.PointCloud()
pcd1.points = o3d.utility.Vector3dVector(a1)

pcd2 = o3d.geometry.PointCloud()
pcd2.points = o3d.utility.Vector3dVector(a2)

pcd3 = o3d.geometry.PointCloud()
pcd3.points = o3d.utility.Vector3dVector(transformed_a1)

# print(a1)
pcd1.paint_uniform_color([1, 0.706, 0])
pcd2.paint_uniform_color([0, 0.651, 0.929])
pcd3.paint_uniform_color([0, 0.651, 0.929])
# Visualize point clouds
mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
# o3d.visualization.draw_geometries([pcd1, pcd2, mesh_frame], window_name='Before Calibration')
# o3d.visualization.draw_geometries([pcd1, pcd3, mesh_frame], window_name='After Calibration')

############################ 

# print(T) #base to camera??
# print(a3[3].dot(c*R) + t)
# print(np.dot(a3[3], c*R) + t)#+t

# print(np.dot((c*R).T, a)) #행렬에 transpose를 왜 해야할까??;;
# T[0:3, 0:3] = T[0:3, 0:3].T

# a = np.append(a3[2], [1])
# print(np.dot(T,a))
# print(T @ a)

# np.savez("data/T_b2c_tool.npz", t=T)


######### 회전각 고려하기

def robot_rotation(end_anlges, end_positions, tool_posi):  # numpy; (n,3),ex)x:0,y:0,z:70mm; euler: zyx transform
    result_position = np.empty((0, 3))

    # 로봇 엔드의 euler angle
    r = Rot.from_euler('zyx', end_anlges, degrees=True)
    r_matrix = r.as_matrix()  # 2,3,3

    # 로봇 엔드의 위치 x,y,z
    for idx in range(r_matrix.shape[0]):
        r_mat = r_matrix[idx]
        # tool_posi
        tool_posi = r_mat @ tool_posi
        end_position = end_positions[idx]
        end_position[0] = end_position[0] + tool_posi[0]  # 부호; 로봇의 베이스 x보고 정할것.
        end_position[1] = end_position[1] + tool_posi[1]  # 부호; 로봇의 베이스 y보고 정할것.
        end_position[2] = end_position[2] + tool_posi[2]  # 부호; 로봇의 베이스 z보고 정할것?? 확인필요..

        result_position = np.vstack((result_position, r_mat @ end_position))  # 회전이 고려된 로봇 엔드의 위치..

    return result_position


#######################

##아래는 위에 함수 만드는 과정

result_position = np.empty((0, 3))
robot_angle = np.array([[1, 2, 3], [4, 5, 6]])  ##  로봇 엔드의 euler angle
r = Rot.from_euler('zyx', robot_angle, degrees=True)
r_matrix = r.as_matrix()  # 2,3,3

tool = np.array([[5, 3, 2], [4, 5, 15]])  # 로봇 엔드의 위치 x,y,z

for idx in range(r_matrix.shape[0]):
    r_mat = r_matrix[idx]
    tool_ = tool[idx]
    result_position = np.vstack((result_position, r_mat @ tool_))  # 회전이 고려된 로봇 엔드의 위치.

# print(result_position)
#
# ### 회전각 추가하기.
# r = Rot.from_euler('zyx', [7.57, -173.2, 9.62], degrees=True)  # 로봇 엔드의 euler angle 넣기.
# r_matrix = r.as_matrix()
# print(np.dot(r_matrix, np.array([0, 0, 75]))) #75mm : 툴길이; [0, 0, 75] -> 툴 길이 고려된 robot position