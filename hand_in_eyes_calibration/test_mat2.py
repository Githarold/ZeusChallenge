from scipy.spatial.transform import Rotation as Rot
import numpy as np


def transformation_matrix(x, y, z, rz, ry, rx):
        """Generate the homogeneous transformation matrix."""
        # R = self.euler_to_rotation_matrix(rx, ry, rz)
        # r = Rot.from_euler('xyz', [rx,ry,rz], degrees=True)

        # ZYX : 오일러 기준 ZYX 좌표계
        # zyx : fixed ZYX 좌표계
        r = Rot.from_euler('ZYX', [rz,ry,rx], degrees=True)
        R = r.as_matrix()
        T = np.array([
            [R[0, 0], R[0, 1], R[0, 2], x],
            [R[1, 0], R[1, 1], R[1, 2], y],
            [R[2, 0], R[2, 1], R[2, 2], z],
            [0, 0, 0, 1]
        ])
        return T




# 로봇 base에서 체커 보드의 한 점을 찍었을 때 로봇의 자세(base to object)
# 숫자는 각각 x,y,z, rz, ry, rx
btw = transformation_matrix(431.29,436.96,-22.63,-90.08,-0.20,-179.79)


# camera - robot pose
# 카메라를 이용해 체커 보드를 찍었을 때 로봇의 자세(camera to object)
btg1= transformation_matrix(452.84,350.53,124.72,-90.13,-0.05,-179.77)
btg2= transformation_matrix(444.81,335.16,171.20,-117.95,0.16,-179.64)
btg3= transformation_matrix(402.58,351.34,271.79,-152.02,0.52,-179.99)
btg4= transformation_matrix(385.97,442.21,298.47,-179.40,0.45,179.67)
btg5= transformation_matrix(347.94,496.06,240.54,155.77,0.22,179.59)

#########################################################################

btg6= transformation_matrix(344.76,520.49,342.32,118.05,-0.08,179.54)
btg7= transformation_matrix(388.57,545.16,342.52,99.35,-0.23,179.58)
btg8= transformation_matrix(416.22,547.88,307.23,71.54,-0.34,179.73)
btg9= transformation_matrix(474.65,486.35,250.91,44.14,-0.30,179.83)
btg10= transformation_matrix(511.54,483.35,272.36,16.48,0.27,179.90)


#########################################################################
btg11= transformation_matrix(480.29,270.89,272.68,-14.24,0.32,179.68)
btg12= transformation_matrix(538.65,387.89,200.30,-39.97,0.13,179.49)
btg13= transformation_matrix(605.15,397.52,256.13,-66.88,-0.12,179.37)
btg14= transformation_matrix(647.70,336.02,227.12,-81.74,-0.18,179.45)
btg15= transformation_matrix(534.47,324.90,108.52,-90.85,-0.39,179.43)

#########################################################################

btg16= transformation_matrix(258.47,269.58,345.62,-77.58,-0.18,179.33)


###############rx,ry,rz start!!##########
# 17th


btg17= transformation_matrix(242.81,435.50,287.94,-119.86,1.67,-137.03)
btg18= transformation_matrix(464.12,407.96,157.38,-149.83,-31.94,-169.85)
btg19= transformation_matrix(483.35,365.93,165.72,-173.77,-33.75,160.84)
btg20= transformation_matrix(456.78,304.35,127.21,122.81,-50.35,-166.78)

#########################################################################
btg21= transformation_matrix(338.05,250.96,116.07,-143.03,31.32,155.00)
btg22= transformation_matrix(431.38,386.92,156.23,-86.52,-20.99,169.42)
btg23= transformation_matrix(384.39,247.43,73.10,-115.23,27.63,159.29)
btg24= transformation_matrix(495.07,298.34,197.07,-96.74,-0.45,142.32)
btg25= transformation_matrix(469.06,234.46,175.21,-111.50,17.99,139.86)

#########################################################################

btg26= transformation_matrix(456.03,341.12,161.74,-95.81,-3.22,155.16)
btg27= transformation_matrix(311.81,339.78,99.64,-95.69,-3.52,-156.49)
btg28= transformation_matrix(344.24,407.45,355.24,-94.97,-11.97,-168.18)
btg29= transformation_matrix(387.90,486.02,143.54,-90.11,-41.99,178.97)
btg30= transformation_matrix(337.63,476.98,132.44,-123.05,-24.52,-147.28)



# gripper to camera  = (base to gripper)^(-1) @ base to object 
result1 = (np.linalg.inv(btg1)) @ btw
result2 = (np.linalg.inv(btg2)) @ btw
result3 = (np.linalg.inv(btg3)) @ btw
result4 = (np.linalg.inv(btg4)) @ btw
result5 = (np.linalg.inv(btg5)) @ btw
result6 = (np.linalg.inv(btg6)) @ btw
result7 = (np.linalg.inv(btg7)) @ btw
result8 = (np.linalg.inv(btg8)) @ btw
result9 = (np.linalg.inv(btg9)) @ btw
result10 = (np.linalg.inv(btg10)) @ btw
result11 = (np.linalg.inv(btg11)) @ btw
result12 = (np.linalg.inv(btg12)) @ btw
result13 = (np.linalg.inv(btg13)) @ btw
result14 = (np.linalg.inv(btg14)) @ btw
result15 = (np.linalg.inv(btg15)) @ btw
result16 = (np.linalg.inv(btg16)) @ btw
result17 = (np.linalg.inv(btg17)) @ btw
result18 = (np.linalg.inv(btg18)) @ btw
result19 = (np.linalg.inv(btg19)) @ btw
result20 = (np.linalg.inv(btg20)) @ btw
result21 = (np.linalg.inv(btg21)) @ btw
result22 = (np.linalg.inv(btg22)) @ btw
result23 = (np.linalg.inv(btg23)) @ btw
result24 = (np.linalg.inv(btg24)) @ btw
result25 = (np.linalg.inv(btg25)) @ btw
result26 = (np.linalg.inv(btg26)) @ btw
result27 = (np.linalg.inv(btg27)) @ btw
result28 = (np.linalg.inv(btg28)) @ btw
result29 = (np.linalg.inv(btg29)) @ btw
result30 = (np.linalg.inv(btg30)) @ btw


print(result1[:3,3])
print(result2[:3,3])
print(result3[:3,3])
print(result4[:3,3])
print(result5[:3,3])
print(result6[:3,3])
print(result7[:3,3])
print(result8[:3,3])
print(result9[:3,3])
print(result10[:3,3])
print(result11[:3,3])
print(result12[:3,3])
print(result13[:3,3])
print(result14[:3,3])
print(result15[:3,3])
print(result16[:3,3])
print(result17[:3,3])
print(result18[:3,3])
print(result19[:3,3])
print(result20[:3,3])
print(result21[:3,3])
print(result22[:3,3])
print(result23[:3,3])
print(result24[:3,3])
print(result25[:3,3])
print(result26[:3,3])
print(result27[:3,3])
print(result28[:3,3])
print(result29[:3,3])
print(result30[:3,3])



