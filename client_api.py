import numpy as np

from socket import *
from scipy.spatial.transform import Rotation as Rot

class JeusController():

    def __init__(self):
        self.client = socket(AF_INET, SOCK_STREAM)
        print('connecting...')
        self.client.connect(('192.168.0.23', 9000))
        self.client.setblocking(False)
        print('connected!')

        T_cal = [0.  , 1.  , 0. , -105.,
                 -1. ,  0.  , 0. , 0.,
                 0. ,  0.  , 1., -345.,
                 0.  , 0.  , 0.  , 1.]
        
        # T_cal = [0.  , -1.  , 0. ,  90.,
        #          1. ,  0.  , 0. , 0.,
        #          0. ,  0.  , 1., -345.,
        #          0.  , 0.  , 0.  , 1.]

        # T_cal = [ 1.  , 0.  , 0. ,  0.,
        #          0. ,  1.  , 0. , 90.,
        #           0. ,  0.  , 1., 345.,
        #           0.  , 0.  , 0.  , 1.]
                
        self.T_cal = np.array(T_cal).reshape(4, 4)

    def get_current_pos(self):
        '''
        Get current pose of the robot.
        '''
        list_data = [[1], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]

        send_flag = str(list_data).encode()
        self.client.send(send_flag)
        close_flag = 0
        
        while True:
            try:
                recived_data = self.client.recv(1024)
                curr_pos = eval(recived_data)
                if recived_data:
                    print('received current position')
                    # print(curr_pos)
                    break

            except KeyboardInterrupt:
                # ctrl-c
                close_flag = 1
                print("socket closed")
                self.client.close()
                break

            except BlockingIOError:
                # when data hasn't arrived yet
                pass

            if close_flag == 1:
                break

        return curr_pos

    def send_data(self, data, index):
        '''
        Send data and wait for completion signal.
        '''
        close_flag = 0
        send_flag = str(data).encode()
        self.client.send(send_flag)
        flag = "complete " + index

        while True:
            try:
                recived_data = self.client.recv(1024)
                error_flag = eval(recived_data)

                if error_flag == 1:
                    print(flag)
                    break
                elif error_flag == -1:
                    print('invalid index')
                    break

            except KeyboardInterrupt:
                # ctrl-c
                close_flag = 1
                print("socket closed")
                self.client.close()
                break

            except BlockingIOError:
                # when data hasn't arrived yet
                pass

            if close_flag == 1:
                break

    def move(self, pos_list):
        '''
        Custom move command.
        pose_list = [x,y,z,rz,ry,rx]
        '''
        list_data = [[0], pos_list]
        
        self.send_data(list_data, "custom move")

    def move_pick(self):
        '''
        Move robot to pick object position.
        '''
        list_data = [[2], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]

        self.send_data(list_data, "moving to pick position")

    def move_place(self):
        '''
        Move robot to place object position.
        '''
        list_data = [[3], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]

        self.send_data(list_data, "moving to place position")

    def move_pose_estimation(self):
        '''
        Move robot to position for pose estimation.
        '''
        list_data = [[4], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]

        self.send_data(list_data, "moving to pose estimation position")

    def move_depth(self):
        '''
        Move robot to position for depth map creation.
        '''
        list_data = [[5], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]

        self.send_data(list_data, "moving to depth position")

    def move_object(self, T):
        '''
        Move robot to XY axis position of the object.
        pose = [x, y, z, rz, ry, rx]
        '''
        curr_pose = self.get_current_pos()
        print(f"curr {curr_pose}")
        curr_T = self.transformation_matrix(curr_pose)
        htm = curr_T @ self.T_cal
        T = htm @ T
        pose_list = self.HTM2PL(T)
        print(f"go pose !!{pose_list}")

        #####################################################
        # pose_list[3] = abs(pose_list[3]) - 90
        pose_list[3] = -abs(pose_list[3])
        # print(f"go pose after !!{pose_list}")
        # pose_list[3] = self.tran(pose_list[3])

        # print(f"go pose after !!{pose_list}")
        #####################################################
        list_data = [[6], pose_list]

        self.pick_z = pose_list[2]


        self.send_data(list_data, "moving object")
                
    def move_only_position(self, x, y, z):
        '''
        Move Zeus only position
        '''
        curr_pose = self.get_current_pos()
        curr_T = self.transformation_matrix(curr_pose)
        htm = curr_T @ self.T_cal

        relative_coords = np.array([x, y, z, 1])
        absolute_coords =  htm @ relative_coords 
        x_abs = absolute_coords[0]
        y_abs = absolute_coords[1]
        self.z_abs = absolute_coords[2]
        
        list_data = [[9], [x_abs, y_abs]]
        
        self.send_data(list_data, "moving only translations")


    def move_only_orientation(self, rz = 0.0, ry = 0.0, rx = 0.0):
        '''
        Move Zeus oreientation offset
        rz, ry rx 인자에 순서로 넣어주기
        
        '''
    
        list_data = [[12], [rz, ry, rx]]
        
        self.send_data(list_data, "moving only translations")

    def pick(self):
        '''
        Move robot in Z-axis direction to pick up the object.
        '''
        # offset = -60.0
        offset = 0.0
        list_data = [[7], [offset, 0.0, 0.0, 0.0, 0.0, 0.0]]

        self.send_data(list_data, "pick")

    def place(self):
        '''
        Move robot in Z-axis direction to place the object.
        z_pos : position in Z-axis direction of the object
        '''
        list_data = [[8], [self.z_abs, 0.0 , 0.0 , 0.0 , 0.0 , 0.0]]       
        
        self.send_data(list_data, "place")

    def gripper_open(self):
        '''
        open gripper
        '''
        list_data = [[10], [0.0, 0.0 , 0.0 , 0.0 , 0.0 , 0.0]]       
        
        self.send_data(list_data, "gripper open")

    def gripper_close(self):
        '''
        Move robot in Z-axis direction to place the object.
        z_pos : position in Z-axis direction of the object
        '''
        list_data = [[11], [0.0, 0.0 , 0.0 , 0.0 , 0.0 , 0.0]]       
        
        self.send_data(list_data, "gripper close")
    
    def close(self):
        
        print("close client")
        self.client.close()

    def HTM2PL(self, HTM):
        '''
        Generate the homogeneous transformation matrix.
        '''
        if not isinstance(HTM, np.ndarray):
            HTM = np.array(HTM).reshape(4, 4)

        P = HTM[:3, 3].tolist()
        R = HTM[:3, :3]
        euler_angles = Rot.from_matrix(R).as_euler('zyx', degrees=True).tolist()

        # for i in range(3):
        #     if (euler_angles[i] < 0.0 and abs(euler_angles[i]) < 1e6):
        #         euler_angles[i] = -180.0

        #         if i == 1:
        #             euler_angles[i] = 0.0
  
        return P + euler_angles
    
    def transformation_matrix(self, pose_list):
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
    
    
    
    def tran(self,rz):
  
        # 0 ~ 45
        
        if rz > 0.0:
        
            if rz < 45.0 and 0.0 <= rz :
            
                return -45.0 - abs(abs(45) - abs(rz))
                
            # 45 ~ 90
            elif rz < 90.0 and 45.0 <= rz :
            
                return -0.0 - abs(abs(90.0) - abs(rz))
            
            
            elif rz < 135.0 and 90.0 <= rz:
                return (-135  - abs(abs (135) - abs(rz)))
            
            elif rz < 180.0 and 135.0 <= rz:
                return -90  - (abs(180) - abs(rz))
            
            elif rz == 180.0:
                return -180.0
            
        else:
            if rz > -45.0 and -0.0 >= rz :
            
                return -135 + (abs(-45) - abs(rz))
                
            # 45 ~ 90
            elif rz > -90.0 and -45.0 >= rz :
            
                return -180 + (abs(-90) - abs(rz))
            
            
            elif rz > -135.0 and -90.0 >= rz:
                return (-45  + (abs(-135) - abs(rz)))
            
            elif rz > -180.0 and 135.0 >= rz:
                return -90.0 + (abs(-180.0) - abs(rz))
            
            elif rz == -180.0:
                return -180.0