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
        

        # 1
        # T_cal = [-0.00314022,  0.99994476 , 0.01003077 ,-0.11278258*1000.0,
        #         -0.99999424, -0.00312717 ,-0.00131649,  0.04079831*1000.0,
        #         -0.00128504 ,-0.01003485 , 0.99994882, -0.18338595*1000.0,
        #         0.        ,  0.    ,      0.       , 1.        ]
        

        #2
        T_cal = [-1.78418058e-03,  9.99903899e-01, -1.37480769e-02, -1.02841690e-01*1000,
 -9.99998347e-01 ,-1.77919711e-03 , 3.74706623e-04 , 3.79794325e-02*1000,
  3.50210075e-04  ,1.37487227e-02  ,9.99905421e-01, -1.80563825e-01*1000,
 0.00000000e+00 , 0.00000000e+00  ,0.00000000e+00,  1.00000000e+00]
        
        # T_cal = [0 ]

        self.rot = np.array([-1.  , 0.  , 0. ,  0.,
                 0. ,  -1.  , 0. , 0.,
                 0. ,  0.  , 1., 0.,
                 0.  , 0.  , 0.  , 1.]).reshape(-1,4)
                
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
        curr_T = self.transformation_matrix(curr_pose)
        htm = curr_T @ self.T_cal

        T = htm @ T

        pose_list = self.HTM2PL(T)

        if (pose_list[3] > 0):
            pose_list[3] = -180 + pose_list[3]

        print(f"go pose !!{pose_list}")

        ########################################
        # pose_list[2] += 100
        ########################################

        list_data = [[6], pose_list]

        self.pick_z = pose_list[2] 

        #####################################

        if self.pick_z < -18.21:
            print("detect plane!!")
            return -1

        #####################################
        self.send_data(list_data, "moving object")

        return 1
                
    def move_only_position(self, x, y, z = 400):
        '''
        Move Zeus only position
        '''
        
        list_data = [[9], [x, y, z]]
        
        self.send_data(list_data, "moving only translations")

    def move_only_orientation(self, rz = 0.0, ry = 0.0, rx = 0.0):
        '''
        Move Zeus oreientation offset
        rz, ry rx 인자에 순서로 넣어주기
        
        '''
    
        list_data = [[12], [rz, ry, rx]]
        
        self.send_data(list_data, "moving only translations")

    def pick(self, offset = 0.0):
        '''
        Move robot in Z-axis direction to pick up the object.
        '''
        # offset = -60.0
        # offset = -20.0
        list_data = [[7], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]

        self.send_data(list_data, "pick")

    def place(self, z_abs = 0.0):
        '''
        Move robot in Z-axis direction to place the object.
        z_pos : position in Z-axis direction of the object
        '''
        list_data = [[8], [z_abs, 0.0 , 0.0 , 0.0 , 0.0 , 0.0]]       
        
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
        euler_angles = Rot.from_matrix(R).as_euler('ZYX', degrees=True).tolist()
  
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

        r = Rot.from_euler('ZYX', [rz,ry,rx], degrees=True)

        R = r.as_matrix()
        T = np.array([
            [R[0, 0], R[0, 1], R[0, 2], x],
            [R[1, 0], R[1, 1], R[1, 2], y],
            [R[2, 0], R[2, 1], R[2, 2], z],
            [0, 0, 0, 1]
        ])

        return T
    
    def depth_pos_to_abs(self, rel_x, rel_y):

        depth_pose = [-70.87, 333.76, 354.93, -90.0 , -0.0, -180.0]
        depth_T = self.transformation_matrix(depth_pose)

        T_btoc = depth_T @ self.T_cal

        rel_V = np.array([rel_x, rel_y, 0.0, 1.0]).T

        abs_T = T_btoc @ rel_V
        
        return abs_T[0], abs_T[1]