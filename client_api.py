import time

from socket import *
from scipy.spatial.transform import Rotation as Rot

class JeusController():

    def __init__(self):
        self.client = socket(AF_INET, SOCK_STREAM)
        print('connecting...')
        self.client.connect(('192.168.0.23', 9000))
        self.client.setblocking(False)
        print('connected!')

        self.T_cal = [ 1.  , 0.  , 0. ,  0.,
                 0. ,  1.  , 0. , 90.,
                 0. ,  0.  , 1., 345.,
                 0.  , 0.  , 0.  , 1.]

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
                    print(curr_pos)
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

    def move(self, T):
        '''
        Custom move command.
        pose_list = [x,y,z,rz,ry,rx]
        '''
        pose_list = self.HTM2PL(T)

        list_data = [[0], pose_list]

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
        pose_list = self.HTM2PL(T)
        
        list_data = [[6], pose_list]

        self.send_data(list_data, "moving object")
                
    def move_only_position(self, x, y):
        '''
        Move Zeus only position
        '''
        list_data = [[9], [x, y]]
        
        self.send_data(list_data, "moving only translations")

    def pick(self):
        '''
        Move robot in Z-axis direction to pick up the object.
        '''
        offset = -50.0
        list_data = [[7], [self.z_pick + offset, 0.0, 0.0, 0.0, 0.0, 0.0]]

        self.send_data(list_data, "pick")

    def place(self, z_pos):
        '''
        Move robot in Z-axis direction to place the object.
        z_pos : position in Z-axis direction of the object
        '''
        list_data = [[8], [z_pos , 0.0 , 0.0 , 0.0 , 0.0 , 0.0]]       
        
        self.send_data(list_data, "place")
    
    def close(self):
        
        print("close client")
        self.client.close()

    def HTM2PL(self, T):
        '''
        Generate the homogeneous transformation matrix.
        '''
        HTM = self.T_cal @ T

        P = HTM[:3, 3].tolist()
        R = HTM[:3, :3]
        euler_angles = Rot.from_matrix(R).as_euler('zyx', degrees=True).tolist()
        
        return P + euler_angles