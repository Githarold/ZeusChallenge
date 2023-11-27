from client_api import JeusController
import time
import numpy as np
from scipy.spatial.transform import Rotation as Rot

def transform_vector_by_matrix(vector, matrix):
    """Transforms a 3D vector using a 4x4 transformation matrix."""
    # Extend the vector to 4D
    vector = np.append(vector, 1)
    # Multiply the vector by the matrix
    transformed_vector = np.dot(matrix, vector)
    # Return the first 3 elements of the result (discard the last element)
    return transformed_vector[:-1]

def tran(rz):
        rz = abs(rz) + 90
        if rz > 180:
            rz -= 180

        return -abs(rz) 

def transformation_matrix(pose_list):
    """Generate the homogeneous transformation matrix."""
    # R = self.euler_to_rotation_matrix(rx, ry, rz)
    x = pose_list[0]
    y = pose_list[1]
    z = pose_list[2]
    rz = pose_list[3]
    ry = pose_list[4]
    rx = pose_list[5]

    r = Rot.from_euler('zyx', [rz,ry,rx], degrees=True)

    R = r.as_matrix()
    T = np.array([
        [R[0, 0], R[0, 1], R[0, 2], x],
        [R[1, 0], R[1, 1], R[1, 2], y],
        [R[2, 0], R[2, 1], R[2, 2], z],
        [0, 0, 0, 1]
    ])

    return T

if __name__ == '__main__':
    
    try:
    
        robot = JeusController()
        # robot.move_pick()
        # robot.move_only_orientation(0, 0, 10)
        # T_cal = [0., 1.  , 0. ,  90.,
        #          -1.,  0.  , 0. , 0.,
        #          0.,  0. , 1., -345.,
        #          0.,  0.  , 0.  , 1.]
        p = robot.get_current_pos()
        print(p)
        # T_cal_np = np.array(T_cal).reshape(-1,4)

        # vector = np.array([-36.04001639778728, -72.77943103067527, 145.20099])
        # transformed_vector = transform_vector_by_matrix(vector, T_cal_np)
            
        # robot.transformation_matrix(p)s
        # robot.move_depth()
        # time.sleep(1)
        # print(1)
        # robot.close()
        # robot.move_pick()
        # print(2)
        # time.sleep(1)
        # robot.move_object(pose_list=[453.44,349.26,56.6,-121.01,-0.0,-180.0])
        # time.sleep(3)
        # robot.pick(offset= -20)
        # time.sleep(1)
        # robot.move_pose_estimation()
        # time.sleep(1)
        # robot.move_place()
        # robot.gripper_open()
        # time.sleep(1)
        # robot.place(z_pos = 200)
        # time.sleep(1)
        # robot.move_depth()
        # robot.move_only_position(453.44,349, 1)
        # print()
        # robot.move(pos_list=[446.799, 269.6,300.6, transformed_vector[0], transformed_vector[1], transformed_vector[2]])
        # robot.move(pos_list=[505.45041140718536, 324.11007259170555, 313.531, 1.0452857787863887, -5.526844329427708, 176.18728777624625])
        # p = robot.get_current_pos()
        # print(p)
        
        # rz = -23
        # rz = tran(rz)
        # robot.move_only_orientation(rx = 20)
        
        # robot.move(pos_list=[531.6574930990802, 303.71921775063896, 313.531, -0.07355184734438766, 0.8461386592787524, -179.96105324007158])
        # robot.move(pos_list=[476.5987528320121, 369.8875661314016, 313.906670436852494, 178.98238543062305, -6.738316193084422, -173.69397044368966])
        # robot.move(pos_list=[437.3886981995866, 446.9729326254111, 19.639748859115116, ( 147.60713354505208 - 180), 0, -173.94548916895624])
        curr = robot.get_current_pos()
        print(curr)
        # curr[3] = 0.0
        # curr[4] = 0.0
        # curr[5] = 0.0
        # robot.move(pos_list=[508.56570042584065, 410.1949950029693, 84.92631385237155, 85.028734575630042, -0.9502187478471951, -168.77751825905364])
        # robot.move(pos_list=curr)
        robot.send_data(-1, "finish")
        robot.close()
    
    except KeyboardInterrupt:
        robot.close()
    
    except Exception:
        robot.close()